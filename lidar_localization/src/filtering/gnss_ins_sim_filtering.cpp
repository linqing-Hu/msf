/*
 * @Description: Kalman filter based localization on GNSS-INS-Sim
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-27 10:07:33
 */
#include "lidar_localization/filtering/gnss_ins_sim_filtering.h"

namespace lidar_localization {

GNSSINSSimFiltering::GNSSINSSimFiltering() {
  // load ROS config:
  InitWithConfig();
}

bool GNSSINSSimFiltering::Init(const Eigen::Matrix4f& init_pose,
                               const Eigen::Vector3f& init_vel,
                               const IMUData& init_imu_data) {
  if (SetInitGNSS(init_pose)) {
    current_vel_ = init_vel;

    kalman_filter_ptr_->Init(
        init_pose.cast<double>(), current_vel_.cast<double>(), init_imu_data);
    std::cout << "finish kalman init" << std::endl;
    return true;
  }

  return false;
}

bool GNSSINSSimFiltering::Update(const IMUData& imu_data) {
  if (kalman_filter_ptr_->Update(imu_data)) {
    kalman_filter_ptr_->GetOdometry(current_pose_, current_vel_);
    return true;
  }

  return false;
}

bool GNSSINSSimFiltering::Correct(const IMUData& imu_data,
                                  const PosVelData& pos_vel_data) {
  static int count = 0;

  // set GNSS-odo measurement:
  current_measurement_.time = pos_vel_data.time_;

  current_measurement_.T_nb = Eigen::Matrix4d::Identity();
  current_measurement_.T_nb(0, 3) = static_cast<double>(pos_vel_data.pos_.x());
  current_measurement_.T_nb(1, 3) = static_cast<double>(pos_vel_data.pos_.y());
  current_measurement_.T_nb(2, 3) = static_cast<double>(pos_vel_data.pos_.z());
  current_measurement_.T_nb =
      init_pose_.inverse().cast<double>() * current_measurement_.T_nb;

  current_measurement_.v_b = pos_vel_data.vel_.cast<double>();

  if (kalman_filter_ptr_->Correct(
          imu_data, config_.fusion_strategy, current_measurement_)) {
    kalman_filter_ptr_->GetOdometry(current_pose_, current_vel_);

    // downsample for observability analysis:
    if (0 == (++count % 10)) {
      // reset downsample counter:
      count = 0;

      // perform observability analysis:
      kalman_filter_ptr_->UpdateObservabilityAnalysis(pos_vel_data.time_,
                                                      config_.fusion_strategy);
    }

    return true;
  }

  return false;
}

void GNSSINSSimFiltering::GetOdometry(Eigen::Matrix4f& pose,
                                      Eigen::Vector3f& vel) {
  pose = init_pose_ * current_pose_;
  vel = init_pose_.block<3, 3>(0, 0) * current_vel_;
}

void GNSSINSSimFiltering::GetStandardDeviation(EKFStd& kf_std_msg) {
  kalman_filter_ptr_->GetCovariance(current_cov_);

  kf_std_msg.header.stamp = ros::Time(kalman_filter_ptr_->time());

  kf_std_msg.pos_x_std = std::sqrt(current_cov_.pos.x);
  kf_std_msg.pos_y_std = std::sqrt(current_cov_.pos.y);
  kf_std_msg.pos_z_std = std::sqrt(current_cov_.pos.z);

  kf_std_msg.vel_x_std = std::sqrt(current_cov_.vel.x);
  kf_std_msg.vel_y_std = std::sqrt(current_cov_.vel.y);
  kf_std_msg.vel_z_std = std::sqrt(current_cov_.vel.z);

  kf_std_msg.ori_w_std = std::sqrt(current_cov_.ori.w);
  kf_std_msg.ori_x_std = std::sqrt(current_cov_.ori.x);
  kf_std_msg.ori_y_std = std::sqrt(current_cov_.ori.y);
  kf_std_msg.ori_z_std = std::sqrt(current_cov_.ori.z);

  kf_std_msg.gyro_bias_x_std = std::sqrt(current_cov_.gyro_bias.x);
  kf_std_msg.gyro_bias_y_std = std::sqrt(current_cov_.gyro_bias.y);
  kf_std_msg.gyro_bias_z_std = std::sqrt(current_cov_.gyro_bias.z);

  kf_std_msg.accel_bias_x_std = std::sqrt(current_cov_.accel_bias.x);
  kf_std_msg.accel_bias_y_std = std::sqrt(current_cov_.accel_bias.y);
  kf_std_msg.accel_bias_z_std = std::sqrt(current_cov_.accel_bias.z);
}

void GNSSINSSimFiltering::SaveObservabilityAnalysis() {
  kalman_filter_ptr_->SaveObservabilityAnalysis(config_.fusion_strategy);
}

bool GNSSINSSimFiltering::InitWithConfig() {
  const std::string config_file_path =
      WORK_SPACE_PATH + "/config/filtering/gnss_ins_sim_filtering.yaml";

  YAML::Node config_node = YAML::LoadFile(config_file_path);

  LOG(INFO) << std::endl
            << "-----------------Init Kalman Filter Fusion for "
               "Localization-------------------"
            << std::endl;

  // a. init fusion:
  InitFusion(config_node);

  return true;
}

bool GNSSINSSimFiltering::InitFusion(const YAML::Node& config_node) {
  // set up fusion strategy:
  config_.fusion_strategy_id["position_velocity"] =
      KalmanFilter::MeasurementType::POSI_VEL;
  config_.fusion_strategy_id["position"] = KalmanFilter::MeasurementType::POSI;

  const std::string fusion_strategy =
      config_node["fusion_strategy"].as<std::string>();

  if (config_.fusion_strategy_id.end() !=
      config_.fusion_strategy_id.find(fusion_strategy)) {
    config_.fusion_strategy = config_.fusion_strategy_id.at(fusion_strategy);
  } else {
    LOG(ERROR) << "Fusion strategy " << fusion_strategy << " NOT FOUND!";
    return false;
  }
  std::cout << "\tGNSS-INS-Sim Localization Fusion Strategy: "
            << fusion_strategy << std::endl;

  // set up fusion method:
  config_.fusion_method = config_node["fusion_method"].as<std::string>();
  if (config_.fusion_method == "error_state_kalman_filter") {
    kalman_filter_ptr_ = std::make_shared<ErrorStateKalmanFilter>(
        config_node[config_.fusion_method]);
  } else {
    LOG(ERROR) << "Fusion method " << config_.fusion_method << " NOT FOUND!";
    return false;
  }
  std::cout << "\tGNSS-INS-Sim Localization Fusion Method: "
            << config_.fusion_method << std::endl;

  return true;
}

/**
 * @brief  set init pose using GNSS measurement
 * @param  init_scan, init key scan
 * @return true if success otherwise false
 */
bool GNSSINSSimFiltering::SetInitGNSS(const Eigen::Matrix4f& gnss_pose) {
  SetInitPose(gnss_pose);
  has_inited_ = true;
  std::cout << "set init gnss" << std::endl;
  return true;
}

bool GNSSINSSimFiltering::SetInitPose(const Eigen::Matrix4f& init_pose) {
  init_pose_ = init_pose;

  return true;
}

}  // namespace lidar_localization