/*
 * @Description: IMU-lidar fusion for localization workflow
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-11-05 23:25:09
 */
#include "lidar_localization/filtering/shenlan_filtering_flow.h"

namespace lidar_localization {

ShenLanFilteringFlow::ShenLanFilteringFlow(ros::NodeHandle& nh) {
  // subscriber:
  // a. IMU raw measurement:
  imu_raw_sub_ptr_ =
      std::make_shared<IMUSubscriber>(nh, "/camera/imu", 1000000);
  // b. undistorted Velodyne measurement:
  cloud_sub_ptr_ =
      std::make_shared<CloudSubscriber>(nh, "/synced_cloud", 100000);
  // c. IMU synced measurement:
  imu_synced_sub_ptr_ =
      std::make_shared<IMUSubscriber>(nh, "/synced_imu", 100000);
  // d. synced GNSS-odo measurement:
  pos_vel_sub_ptr_ =
      std::make_shared<PosVelSubscriber>(nh, "/synced_pos_vel", 100000);
  // e. lidar pose in map frame:
  gnss_sub_ptr_ =
      std::make_shared<OdometrySubscriber>(nh, "/synced_gnss", 100000);
  // // f. lidar to imu tf:
  // lidar_to_imu_ptr_ =
  //     std::make_shared<TFListener>(nh, "/imu_link", "/velo_link");

  // publisher:
  // a. global point cloud map:
  global_map_pub_ptr_ =
      std::make_shared<CloudPublisher>(nh, "/global_map", "/map", 100);
  // b. local point cloud map:
  local_map_pub_ptr_ =
      std::make_shared<CloudPublisher>(nh, "/local_map", "/map", 100);
  // c. current scan:
  current_scan_pub_ptr_ =
      std::make_shared<CloudPublisher>(nh, "/current_scan", "/map", 100);
  // d. estimated lidar pose in map frame:
  laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(
      nh, "/laser_localization", "/map", "/lidar", 100);
  // e. fused pose in map frame:
  fused_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(
      nh, "/fused_localization", "/map", "/lidar", 100);

  // f. tf:
  laser_tf_pub_ptr_ = std::make_shared<TFBroadCaster>("/map", "/vehicle_link");

  filtering_ptr_ = std::make_shared<ShenLanFiltering>();
}

bool ShenLanFilteringFlow::Run() {
  // if (!InitCalibration()) {
  //   return false;
  // }

  // if new global map is available, publish it:
  PublishGlobalMap();
  // if new local map is available, publish it:
  PublishLocalMap();

  ReadData();

  while (HasData()) {
    if (!HasInited()) {
      if (ValidLidarData()) {
        InitLocalization();
      }
    } else {
      // TODO: handle timestamp chaos in an more elegant way
      if (HasLidarData() && ValidLidarData()) {
        if (HasIMUData()) {
          while (HasIMUData() && ValidIMUData() &&
                 current_imu_raw_data_.time_ < current_cloud_data_.time_) {
            UpdateLocalization();
          }

          if (current_imu_raw_data_.time_ >= current_cloud_data_.time_) {
            imu_raw_data_buff_.push_back(current_imu_raw_data_);
          }
        }

        CorrectLocalization();
      }

      if (HasIMUData() && ValidIMUData()) {
        UpdateLocalization();
      }
    }
  }

  return true;
}

bool ShenLanFilteringFlow::SaveOdometry() {
  if (0 == trajectory_.N) {
    return false;
  }

  // init output files:
  std::ofstream fused_odom_ofs;
  std::ofstream laser_odom_ofs;
  std::ofstream ref_odom_ofs;
  if (!FileManager::CreateFile(
          WORK_SPACE_PATH + "/slam_data/trajectory/fused.txt",
          fused_odom_ofs) ||
      !FileManager::CreateFile(
          WORK_SPACE_PATH + "/slam_data/trajectory/laser.txt",
          laser_odom_ofs) ||
      !FileManager::CreateFile(
          WORK_SPACE_PATH + "/slam_data/trajectory/ground_truth.txt",
          ref_odom_ofs)) {
    return false;
  }

  // write outputs:
  const double half_sampling_time = 0.05;  // LiDAR is 10hz
  for (size_t i = 0; i < trajectory_.N; ++i) {
    // sync ref pose with gnss measurement:
    while (!gnss_data_buff_.empty() &&
           (gnss_data_buff_.front().time_ - trajectory_.time.at(i) <=
            -half_sampling_time)) {
      gnss_data_buff_.pop_front();
    }

    if (gnss_data_buff_.empty()) {
      break;
    }

    current_gnss_data_ = gnss_data_buff_.front();

    const Eigen::Vector3f& position_ref =
        current_gnss_data_.pose_.block<3, 1>(0, 3);
    const Eigen::Vector3f& position_lidar =
        trajectory_.lidar.at(i).block<3, 1>(0, 3);

    if ((position_ref - position_lidar).norm() > 3.0) {
      LOG(INFO) << "(position_ref - position_lidar).norm() > 3.0";
      continue;
    }

    SavePose(trajectory_.fused.at(i), fused_odom_ofs);
    SavePose(trajectory_.lidar.at(i), laser_odom_ofs);
    SavePose(current_gnss_data_.pose_, ref_odom_ofs);
  }

  return true;
}

bool ShenLanFilteringFlow::ReadData() {
  //
  // pipe raw IMU measurements into buffer:
  //
  imu_raw_sub_ptr_->ParseData(imu_raw_data_buff_);
  while (HasInited() && HasIMUData() &&
         imu_raw_data_buff_.front().time_ < filtering_ptr_->GetTime()) {
    imu_raw_data_buff_.pop_front();
  }

  //
  // pipe synced lidar-GNSS-IMU measurements into buffer:
  //
  cloud_sub_ptr_->ParseData(cloud_data_buff_);
  imu_synced_sub_ptr_->ParseData(imu_synced_data_buff_);
  pos_vel_sub_ptr_->ParseData(pos_vel_data_buff_);
  gnss_sub_ptr_->ParseData(gnss_data_buff_);

  return true;
}

bool ShenLanFilteringFlow::HasInited() const {
  return filtering_ptr_->has_inited();
}

bool ShenLanFilteringFlow::HasData() {
  if (!HasInited()) {
    if (!HasLidarData()) {
      return false;
    }
  } else {
    if (!HasIMUData() && !HasLidarData()) {
      return false;
    }
  }

  return true;
}

bool ShenLanFilteringFlow::ValidIMUData() {
  current_imu_raw_data_ = imu_raw_data_buff_.front();

  imu_raw_data_buff_.pop_front();

  return true;
}

bool ShenLanFilteringFlow::ValidLidarData() {
  current_cloud_data_ = cloud_data_buff_.front();
  current_imu_synced_data_ = imu_synced_data_buff_.front();
  current_pos_vel_data_ = pos_vel_data_buff_.front();

  const double diff_imu_time =
      current_cloud_data_.time_ - current_imu_synced_data_.time_;
  const double diff_pos_vel_time =
      current_cloud_data_.time_ - current_pos_vel_data_.time_;

  const double half_sampling_time = 0.05;  // LiDAR is 10hz
  if (diff_imu_time < -half_sampling_time ||
      diff_pos_vel_time < -half_sampling_time) {
    cloud_data_buff_.pop_front();
    return false;
  }
  // if (diff_imu_time < -half_sampling_time) {
  //   cloud_data_buff_.pop_front();
  //   return false;
  // }

  if (diff_imu_time > half_sampling_time) {
    imu_synced_data_buff_.pop_front();
    return false;
  }

  if (diff_pos_vel_time > half_sampling_time) {
    pos_vel_data_buff_.pop_front();
    return false;
  }

  cloud_data_buff_.pop_front();
  imu_synced_data_buff_.pop_front();
  pos_vel_data_buff_.pop_front();

  return true;
}

bool ShenLanFilteringFlow::InitLocalization() const {
  // ego vehicle velocity in body frame:
  Eigen::Vector3f init_vel = current_pos_vel_data_.vel_;
  Eigen::Matrix3f R = current_imu_synced_data_.GetOrientationMatrix();
  Eigen::Vector3f t = current_pos_vel_data_.pos_;
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = t;

  // using GNSS data for initialization
  // if (filtering_ptr_->Init(T, init_vel, current_imu_synced_data_)) {
  //   LOG(INFO) << "GNSS Localization Init Succeeded." << std::endl;
  // }

  if (filtering_ptr_->Init(
          current_cloud_data_, T, init_vel, current_imu_synced_data_)) {
    LOG(INFO) << "Relocalization Init Succeeded." << std::endl;
  }

  return true;
}

bool ShenLanFilteringFlow::UpdateLocalization() {
  if (filtering_ptr_->Update(current_imu_raw_data_)) {
    PublishFusionOdom();
    return true;
  }

  return false;
}

bool ShenLanFilteringFlow::CorrectLocalization() {
  const bool is_fusion_succeeded =
      filtering_ptr_->Correct(current_imu_synced_data_,
                              current_cloud_data_,
                              current_pos_vel_data_,
                              laser_pose_);
  PublishLidarOdom();

  if (is_fusion_succeeded) {
    PublishFusionOdom();

    // add to odometry output for evo evaluation:
    UpdateOdometry(current_cloud_data_.time_);

    return true;
  }

  return false;
}

bool ShenLanFilteringFlow::PublishGlobalMap() const {
  if (filtering_ptr_->has_new_global_map() &&
      global_map_pub_ptr_->HasSubscribers()) {
    CloudData::CloudTypePtr global_map_ptr(new CloudData::CloudType());
    filtering_ptr_->GetGlobalMap(global_map_ptr);
    global_map_pub_ptr_->Publish(global_map_ptr);

    return true;
  }

  return false;
}

bool ShenLanFilteringFlow::PublishLocalMap() const {
  if (filtering_ptr_->has_new_local_map() &&
      local_map_pub_ptr_->HasSubscribers()) {
    local_map_pub_ptr_->Publish(filtering_ptr_->local_map_ptr());

    return true;
  }

  return false;
}

bool ShenLanFilteringFlow::PublishLidarOdom() const {
  // a. publish lidar odometry
  laser_odom_pub_ptr_->Publish(laser_pose_, current_cloud_data_.time_);
  // b. publish current scan:
  current_scan_pub_ptr_->Publish(filtering_ptr_->current_scan_ptr());

  return true;
}

bool ShenLanFilteringFlow::PublishFusionOdom() {
  // get odometry from Kalman filter:
  filtering_ptr_->GetOdometry(fused_pose_, fused_vel_);
  // a. publish tf:
  laser_tf_pub_ptr_->SendTransform(fused_pose_, current_imu_raw_data_.time_);
  // b. publish fusion odometry:
  fused_odom_pub_ptr_->Publish(
      fused_pose_, fused_vel_, current_imu_raw_data_.time_);

  return true;
}

bool ShenLanFilteringFlow::UpdateOdometry(const double time) {
  trajectory_.time.push_back(time);

  trajectory_.fused.push_back(fused_pose_);
  trajectory_.lidar.push_back(laser_pose_);

  ++trajectory_.N;

  return true;
}

/**
 * @brief  save pose in KITTI format for evo evaluation
 * @param  pose, input pose
 * @param  ofs, output file stream
 * @return true if success otherwise false
 */
bool ShenLanFilteringFlow::SavePose(const Eigen::Matrix4f& pose,
                                    std::ofstream& ofs) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 4; ++j) {
      ofs << pose(i, j);

      if (i == 2 && j == 3) {
        ofs << std::endl;
      } else {
        ofs << " ";
      }
    }
  }

  return true;
}

}  // namespace lidar_localization