/*
 * @Description: IMU-lidar fusion for localization
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-25 09:58:40
 */
#include <glog/logging.h>

#include <ros/ros.h>

#include "lidar_localization/filtering/kitti_filtering_flow.h"
// service
#include "lidar_localization/saveOdometry.h"
// global
#include "lidar_localization/global_defination/global_defination.h"

using namespace lidar_localization;

bool need_save_odometry = false;

bool SaveOdometryCB(saveOdometry::Request& request,
                    saveOdometry::Response& response) {
  need_save_odometry = true;
  response.succeed = true;
  return response.succeed;
}

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
  FLAGS_alsologtostderr = 1;

  ros::init(argc, argv, "kitti_filtering_node");
  ros::NodeHandle nh;

  std::shared_ptr<KITTIFilteringFlow> kitti_filtering_flow_ptr =
      std::make_shared<KITTIFilteringFlow>(nh);
  ros::ServiceServer service =
      nh.advertiseService("save_odometry", SaveOdometryCB);

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();

    kitti_filtering_flow_ptr->Run();

    // save odometry estimations for evo evaluation:
    if (need_save_odometry && kitti_filtering_flow_ptr->SaveOdometry()) {
      need_save_odometry = false;
    }

    rate.sleep();
  }

  return 0;
}