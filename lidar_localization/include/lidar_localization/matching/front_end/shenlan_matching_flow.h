/*
 * @Description: LIO localization frontend workflow, interface
 * @Author: Ge Yao
 * @Date: 2021-01-02 10:47:23
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-11-05 19:49:22
 */
#ifndef LIDAR_LOCALIZATION_MATCHING_FRONT_END_SHENLAN_MATCHING_FLOW_H_
#define LIDAR_LOCALIZATION_MATCHING_FRONT_END_SHENLAN_MATCHING_FLOW_H_

#include <glog/logging.h>

#include <ros/ros.h>

// subscriber
#include "lidar_localization/subscriber/cloud_subscriber.h"
#include "lidar_localization/subscriber/odometry_subscriber.h"
// publisher
#include "lidar_localization/publisher/cloud_publisher.h"
#include "lidar_localization/publisher/odometry_publisher.h"
#include "lidar_localization/publisher/tf_broadcaster.h"
// matching
#include "lidar_localization/matching/front_end/shenlan_matching.h"
// global
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {

class ShenLanMatchingFlow {
 public:
  ShenLanMatchingFlow(const ros::NodeHandle& nh);

  bool Run();

 private:
  bool ReadData();
  bool HasData();
  bool ValidData();
  bool UpdateMatching();
  bool PublishData();

  //
  // subscribers:
  //
  std::shared_ptr<CloudSubscriber> cloud_sub_ptr_{};
  std::shared_ptr<OdometrySubscriber> gnss_sub_ptr_{};

  //
  // publishers:
  //
  std::shared_ptr<CloudPublisher> global_map_pub_ptr_{};
  std::shared_ptr<CloudPublisher> local_map_pub_ptr_{};
  std::shared_ptr<CloudPublisher> current_scan_pub_ptr_{};

  std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_{};
  std::shared_ptr<OdometryPublisher> map_matching_odom_pub_ptr_{};

  // matching
  std::shared_ptr<ShenLanMatching> matching_ptr_{};

  std::deque<CloudData> cloud_data_buff_;
  std::deque<PoseData> gnss_data_buff_;

  CloudData current_cloud_data_;
  PoseData current_gnss_data_;

  Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f map_matching_odometry_ = Eigen::Matrix4f::Identity();
};

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_MATCHING_FRONT_END_SHENLAN_MATCHING_FLOW_H_