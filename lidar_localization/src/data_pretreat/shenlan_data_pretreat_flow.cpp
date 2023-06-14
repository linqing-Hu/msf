/*
 * @Description: data preprocess for shenlan dataset
 * @Author: ZiJieChen
 * @Date: 2022-10-03 14:28:23
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-11-03 21:50:31
 */
#include "lidar_localization/data_pretreat/shenlan_data_pretreat_flow.h"

namespace lidar_localization {
ShenLanDataPretreatFlow::ShenLanDataPretreatFlow(
    const ros::NodeHandle& nh, const std::string cloud_topic) {
  // subscribers:
  // a. velodyne measurement:
  cloud_sub_ptr_ =
      std::make_shared<CloudSubscriber>(nh, "/lidar/PointCloud2", 100000);
  // b. G410 IMU:
  imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/camera/imu", 1000000);
  // c. G410 INS:
  gnss_sub_ptr_ =
      std::make_shared<OdometrySubscriber>(nh, "/gnss/odometry", 1000000);
  // d. encoder velocity
  velocity_sub_ptr_ =
      std::make_shared<VelocitySubscriber>(nh, "/canbus/chassis", 100000);

  // publishers:
  cloud_pub_ptr_ =
      std::make_shared<CloudPublisher>(nh, cloud_topic, "/velo_link", 100);
  imu_pub_ptr_ =
      std::make_shared<IMUPublisher>(nh, "/synced_imu", "/imu_link", 100);
  pos_vel_pub_ptr_ = std::make_shared<PosVelPublisher>(
      nh, "/synced_pos_vel", "/map", "/imu_link", 100);
  gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(
      nh, "/synced_gnss", "/map", "/map", 100);

  // motion compensation for lidar measurement:
  distortion_adjust_ptr_ = std::make_shared<DistortionAdjust>();

  // loading config params
  const std::string config_file_path =
      WORK_SPACE_PATH + "/config/mapping/shenlan_data_pretreat.yaml";
  YAML::Node config_node = YAML::LoadFile(config_file_path);

  float array[16];
  for (size_t i = 0; i < 16; i++) {
    array[i] = config_node["T_imu_lidar"][i].as<float>();
  }
  T_imu_lidar_ = Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>>(array);

  is_mapping_ = config_node["is_mapping"].as<bool>();

  LOG(INFO) << std::endl
            << "-----------------Init Data Pretreat-------------------"
            << std::endl
            << "is_mapping: " << is_mapping_ << std::endl
            << "T_imu_lidar: " << std::endl
            << T_imu_lidar_ << std::endl;
}

bool ShenLanDataPretreatFlow::Run() {
  if (!ReadData()) {
    return false;
  }

  if (!InitGNSS()) {
    return false;
  }

  while (HasData()) {
    if (!ValidData()) {
      continue;
    }

    TransformData();
    PublishData();
  }

  return true;
}

bool ShenLanDataPretreatFlow::ReadData() {
  static std::deque<IMUData> unsynced_imu_;
  static std::deque<VelocityData> unsynced_velocity_;
  static std::deque<PoseData> unsynced_gnss_;

  // fetch lidar measurements from buffer:
  cloud_sub_ptr_->ParseData(cloud_data_buff_);
  imu_sub_ptr_->ParseData(unsynced_imu_);
  velocity_sub_ptr_->ParseData(unsynced_velocity_);
  gnss_sub_ptr_->ParseData(unsynced_gnss_);

  if (cloud_data_buff_.size() == 0) {
    return false;
  }

  // use timestamp of lidar measurement as reference:
  const double cloud_time = cloud_data_buff_.front().time_;
  // sync IMU, velocity and GNSS with lidar measurement:
  // find the two closest measurement around lidar measurement time
  // then use linear interpolation to generate synced measurement:
  const bool valid_imu =
      IMUData::SyncData(cloud_time, unsynced_imu_, imu_data_buff_);
  const bool valid_velocity = VelocityData::SyncData(
      cloud_time, unsynced_velocity_, velocity_data_buff_);
  const bool valid_gnss =
      PoseData::SyncData(cloud_time, unsynced_gnss_, gnss_data_buff_);

  if (!valid_imu || !valid_gnss || !valid_velocity) {
    if (unsynced_gnss_.front().time_ > cloud_time) {
      cloud_data_buff_.pop_front();
    } else if (unsynced_imu_.front().time_ > cloud_time) {
      cloud_data_buff_.pop_front();
    } else if (unsynced_velocity_.front().time_ > cloud_time) {
      cloud_data_buff_.pop_front();
    }

    return false;
  }

  return true;
}

bool ShenLanDataPretreatFlow::InitGNSS() {
  static bool gnss_inited = false;
  if (gnss_data_buff_.empty()) {
    return false;
  }

  if (!gnss_inited) {
    // system is mapping mode
    if (is_mapping_) {
      gnss_origin_ = gnss_data_buff_.front().pose_.block<3, 1>(0, 3);
      gnss_inited = true;
      LOG(INFO) << "System is mapping mode, gnss origin inited." << std::endl;
    }
    // system is localization mode
    else {
      const std::string gnss_origin_path =
          WORK_SPACE_PATH + "/slam_data/save_map/gnss_origin.txt";
      // const std::string gnss_origin_path =
      //     WORK_SPACE_PATH + "/slam_data/map/gnss_origin.txt";
      if (!FileManager::IsValidDirectory(gnss_origin_path)) {
        LOG(ERROR)
            << "System is localization mode, can not find gnss origin point."
            << std::endl;
        return false;
      }

      std::ifstream ifs(gnss_origin_path);

      double x = 0.0;
      double y = 0.0;
      double z = 0.0;

      ifs >> x;
      ifs >> y;
      ifs >> z;

      gnss_origin_.x() = x;
      gnss_origin_.y() = y;
      gnss_origin_.z() = z;

      // gnss_init_data_.latitude_ = lat;
      // gnss_init_data_.longitude_ = lon;
      // gnss_init_data_.altitude_ = alt;
      // gnss_init_data_.InitOriginPosition();

      gnss_inited = true;
    }
  }

  return gnss_inited;
}

bool ShenLanDataPretreatFlow::SaveGnssOrigin() {
  const std::string gnss_origin_path =
      WORK_SPACE_PATH + "/slam_data/map/gnss_origin.txt";

  std::ofstream ofs(gnss_origin_path);

  ofs << std::setprecision(16) << gnss_origin_.x() << " " << gnss_origin_.y()
      << " " << gnss_origin_.z();
  ofs.close();

  LOG(INFO) << "save gnss_origin in: " << gnss_origin_path << std::endl
            << " x: " << gnss_origin_.x() << " y: " << gnss_origin_.y()
            << " z: " << gnss_origin_.z() << std::endl;
  return true;
}

bool ShenLanDataPretreatFlow::HasData() {
  if (cloud_data_buff_.size() == 0) {
    return false;
  }
  if (imu_data_buff_.size() == 0) {
    return false;
  }
  if (velocity_data_buff_.size() == 0) {
    return false;
  }
  if (gnss_data_buff_.size() == 0) {
    return false;
  }
  return true;
}

bool ShenLanDataPretreatFlow::ValidData() {
  current_cloud_data_ = cloud_data_buff_.front();
  current_imu_data_ = imu_data_buff_.front();
  current_velocity_data_ = velocity_data_buff_.front();

  current_gnss_data_ = gnss_data_buff_.front();

  const double diff_imu_time =
      current_cloud_data_.time_ - current_imu_data_.time_;
  const double diff_velocity_time =
      current_cloud_data_.time_ - current_velocity_data_.time_;
  const double diff_gnss_time =
      current_cloud_data_.time_ - current_gnss_data_.time_;
  //
  // this check assumes the frequency of lidar is 10Hz:
  //
  const double half_sampling_time = 0.05;
  if (diff_imu_time < -half_sampling_time ||
      diff_gnss_time < -half_sampling_time) {
    cloud_data_buff_.pop_front();
    return false;
  }

  if (diff_imu_time > half_sampling_time) {
    imu_data_buff_.pop_front();
    return false;
  }

  if (diff_velocity_time > half_sampling_time) {
    velocity_data_buff_.pop_front();
    return false;
  }

  if (diff_gnss_time > half_sampling_time) {
    gnss_data_buff_.pop_front();
    return false;
  }

  cloud_data_buff_.pop_front();
  imu_data_buff_.pop_front();
  velocity_data_buff_.pop_front();
  gnss_data_buff_.pop_front();

  return true;
}

bool ShenLanDataPretreatFlow::TransformData() {
  // static Eigen::Vector3f gnss_origin = Eigen::Vector3f::Zero();
  // static bool gnss_init = false;

  // a. init gnss origin
  // to make sure the gnss origin is [0,0,0]
  // if (!gnss_init) {
  //   gnss_origin = current_gnss_data_.pose_.block<3, 1>(0, 3);
  //   gnss_init = true;
  // }
  current_gnss_data_.pose_.block<3, 1>(0, 3) =
      current_gnss_data_.pose_.block<3, 1>(0, 3) - gnss_origin_;

  // b. motion compensation for lidar measurements:
  // convert the laser points to imu frame
  CloudData::CloudTypePtr temp_cloud(new CloudData::CloudType());
  pcl::transformPointCloud(
      *current_cloud_data_.cloud_ptr_, *temp_cloud, T_imu_lidar_);

  // // convert the gnss velocity from ENU frame to imu frame
  // Eigen::Vector3d v_b = (current_gnss_data_.pose_.block<3, 3>(0, 0).inverse()
  // *
  //                        current_gnss_data_.vel_.v)
  //                           .cast<double>();

  // current_velocity_data_.linear_velocity_.x = v_b.x();
  // current_velocity_data_.linear_velocity_.y = v_b.y();
  // current_velocity_data_.linear_velocity_.z = v_b.z();

  // current_velocity_data_.angular_velocity_.x =
  //     current_imu_data_.angular_velocity_.x;
  // current_velocity_data_.angular_velocity_.y =
  //     current_imu_data_.angular_velocity_.y;
  // current_velocity_data_.angular_velocity_.z =
  //     current_imu_data_.angular_velocity_.z;

  // distortion_adjust_ptr_->SetMotionInfo(current_velocity_data_, 0.1);
  // CloudData::CloudTypePtr adjusted_cloud(new CloudData::CloudType());
  // distortion_adjust_ptr_->AdjustCloud(temp_cloud, adjusted_cloud);
  // *current_cloud_data_.cloud_ptr_ = *adjusted_cloud;
  *current_cloud_data_.cloud_ptr_ = *temp_cloud;

  // set synced pos vel
  pos_vel_.pos_.x() = current_gnss_data_.pose_(0, 3);
  pos_vel_.pos_.y() = current_gnss_data_.pose_(1, 3);
  pos_vel_.pos_.z() = current_gnss_data_.pose_(2, 3);
  // linear velocity in body frame
  pos_vel_.vel_.x() = current_velocity_data_.linear_velocity_.x;
  pos_vel_.vel_.y() = current_velocity_data_.linear_velocity_.y;
  pos_vel_.vel_.z() = current_velocity_data_.linear_velocity_.z;

  Eigen::Quaterniond q = Eigen::Quaterniond(
      current_gnss_data_.pose_.block<3, 3>(0, 0).cast<double>());
  current_imu_data_.orientation_.w = q.w();
  current_imu_data_.orientation_.x = q.x();
  current_imu_data_.orientation_.y = q.y();
  current_imu_data_.orientation_.z = q.z();

  return true;
}

bool ShenLanDataPretreatFlow::PublishData() const {
  // take lidar measurement time as synced timestamp:
  const double timestamp_synced = current_cloud_data_.time_;

  cloud_pub_ptr_->Publish(current_cloud_data_.cloud_ptr_, timestamp_synced);

  imu_pub_ptr_->Publish(current_imu_data_, timestamp_synced);

  pos_vel_pub_ptr_->Publish(pos_vel_, timestamp_synced);

  // a. gnss measurement in ENU frame
  // b. encoder linear/angular velocity in body frame
  gnss_pub_ptr_->Publish(current_gnss_data_.pose_,
                         current_velocity_data_,
                         current_gnss_data_.cov_,
                         timestamp_synced);

  return true;
}

}  // namespace lidar_localization