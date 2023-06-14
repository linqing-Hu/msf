/*
 * @Description: LIO localization frontend workflow, implementation
 * @Author: Ge Yao
 * @Date: 2021-01-02 10:47:23
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-11-05 22:43:51
 */
#include "lidar_localization/matching/front_end/shenlan_matching_flow.h"

namespace lidar_localization {

ShenLanMatchingFlow::ShenLanMatchingFlow(const ros::NodeHandle& nh) {
  //
  // subscribers:
  //
  // a. undistorted Velodyne measurement:
  cloud_sub_ptr_ =
      std::make_shared<CloudSubscriber>(nh, "/synced_cloud", 100000);
  // b. lidar pose in map frame:
  gnss_sub_ptr_ =
      std::make_shared<OdometrySubscriber>(nh, "/synced_gnss", 100000);

  //
  // publishers:
  //
  // a. global point cloud map:
  global_map_pub_ptr_ =
      std::make_shared<CloudPublisher>(nh, "/global_map", "/map", 100);
  // b. local point cloud map:
  local_map_pub_ptr_ =
      std::make_shared<CloudPublisher>(nh, "/local_map", "/map", 100);
  // c. current scan:
  current_scan_pub_ptr_ =
      std::make_shared<CloudPublisher>(nh, "/current_scan", "/map", 100);

  // d. estimated lidar pose in map frame, lidar frontend:
  laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(
      nh, "/laser_odometry", "/map", "/lidar", 100);
  // e. estimated lidar pose in map frame, map matching:
  map_matching_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(
      nh, "/map_matching_odometry", "/map", "/lidar", 100);

  matching_ptr_ = std::make_shared<ShenLanMatching>();
}

bool ShenLanMatchingFlow::Run() {
  // update global map if necessary:
  if (matching_ptr_->has_new_global_map() &&
      global_map_pub_ptr_->HasSubscribers()) {
    global_map_pub_ptr_->Publish(matching_ptr_->GetGlobalMapPtr());
  }

  // update local map if necessary:
  if (matching_ptr_->has_new_local_map() &&
      local_map_pub_ptr_->HasSubscribers()) {
    local_map_pub_ptr_->Publish(matching_ptr_->local_map_ptr());
  }

  // read inputs:
  ReadData();

  while (HasData()) {
    if (!ValidData()) {
      LOG(INFO) << "Invalid data. Skip matching" << std::endl;
      continue;
    }

    if (UpdateMatching()) {
      PublishData();
    }
  }

  return true;
}

bool ShenLanMatchingFlow::ReadData() {
  // pipe lidar measurements and reference pose into buffer:
  cloud_sub_ptr_->ParseData(cloud_data_buff_);
  gnss_sub_ptr_->ParseData(gnss_data_buff_);

  return true;
}

bool ShenLanMatchingFlow::HasData() {
  if (cloud_data_buff_.empty()) {
    return false;
  }

  if (matching_ptr_->has_inited()) {
    return true;
  }

  if (gnss_data_buff_.empty()) {
    return false;
  }

  return true;
}

bool ShenLanMatchingFlow::ValidData() {
  current_cloud_data_ = cloud_data_buff_.front();

  if (matching_ptr_->has_inited()) {
    cloud_data_buff_.pop_front();
    gnss_data_buff_.clear();
    return true;
  }

  current_gnss_data_ = gnss_data_buff_.front();

  const double diff_time = current_cloud_data_.time_ - current_gnss_data_.time_;

  //
  // here assumes that the freq. of lidar measurements is 10Hz:
  //
  const double half_sampling_time = 0.05;  // LiDAR is 10hz
  if (diff_time < -half_sampling_time) {
    cloud_data_buff_.pop_front();
    return false;
  }

  if (diff_time > half_sampling_time) {
    gnss_data_buff_.pop_front();
    return false;
  }

  cloud_data_buff_.pop_front();
  gnss_data_buff_.pop_front();

  return true;
}

bool ShenLanMatchingFlow::UpdateMatching() {
  if (!matching_ptr_->has_inited()) {
    matching_ptr_->Init(current_cloud_data_, current_gnss_data_.pose_);
    LOG(INFO) << "Initialization successed." << std::endl;
  }

  return matching_ptr_->Update(
      current_cloud_data_, laser_odometry_, map_matching_odometry_);
}

bool ShenLanMatchingFlow::PublishData() {
  const double timestamp_synced = current_cloud_data_.time_;

  laser_odom_pub_ptr_->Publish(laser_odometry_, timestamp_synced);
  map_matching_odom_pub_ptr_->Publish(map_matching_odometry_, timestamp_synced);

  current_scan_pub_ptr_->Publish(matching_ptr_->current_scan_ptr());

  return true;
}

}  // namespace lidar_localization