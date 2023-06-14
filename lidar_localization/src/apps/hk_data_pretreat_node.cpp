/*
 * @Description: ROS node for sensor measurement pre-processing
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:56:27
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-30 15:14:54
 */
#include <glog/logging.h>

#include <ros/ros.h>

#include "lidar_localization/data_pretreat/hk_data_pretreat_flow.h"
#include "lidar_localization/global_defination/global_defination.h"
// service
#include "lidar_localization/saveOrigin.h"

using namespace lidar_localization;

ros::ServiceServer service;
bool need_save_origin = false;

bool save_map_callback(saveOrigin::Request& request,
                       saveOrigin::Response& response) {
  LOG(INFO) << "save_map call in data pretreat" << std::endl;
  need_save_origin = true;
  response.succeed = true;
  return response.succeed;
}

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
  FLAGS_alsologtostderr = 1;

  ros::init(argc, argv, "hk_data_pretreat_node");
  ros::NodeHandle nh;

  std::string cloud_topic;
  nh.param<std::string>("cloud_topic", cloud_topic, "/synced_cloud");
  bool is_mapping;
  nh.param<bool>("mapping", is_mapping, true);

  // subscribe to
  // a. raw Velodyne measurement
  // b. raw GNSS/IMU measurement
  // publish
  // a. undistorted Velodyne measurement
  // b. lidar pose in map frame
  std::shared_ptr<HKDataPretreatFlow> hk_data_pretreat_flow_ptr =
      std::make_shared<HKDataPretreatFlow>(nh, cloud_topic);

  if (is_mapping) {
    service = nh.advertiseService("save_origin", save_map_callback);
  }

  LOG(INFO) << "start hk_data_pretreat_node." << std::endl;

  // pre-process lidar point cloud at 100Hz:
  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();

    hk_data_pretreat_flow_ptr->Run();

    rate.sleep();
  }

  return 0;
}