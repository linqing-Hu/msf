/*
 * @Description: 闭环检测的node文件
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:56:27
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-25 22:19:34
 */
#include <glog/logging.h>

#include <ros/ros.h>

// mapping
#include "lidar_localization/mapping/loop_closing/loop_closing_flow.h"
// global
#include "lidar_localization/global_defination/global_defination.h"

using namespace lidar_localization;

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
  FLAGS_alsologtostderr = 1;

  ros::init(argc, argv, "loop_closing_node");
  ros::NodeHandle nh;

  // subscribe to:
  // a. key frame pose and corresponding GNSS/IMU pose from backend node
  // publish:
  // a. loop closure detection result for backend node:
  std::shared_ptr<LoopClosingFlow> loop_closing_flow_ptr =
      std::make_shared<LoopClosingFlow>(nh);

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();

    loop_closing_flow_ptr->Run();

    rate.sleep();
  }

  return 0;
}