/*
 * @Description: 订阅imu数据
 * @Author: Ren Qian
 * @Date: 2019-06-14 16:44:18
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-28 22:04:19
 */
#include "lidar_localization/subscriber/velocity_subscriber.h"

namespace lidar_localization {
VelocitySubscriber::VelocitySubscriber(const ros::NodeHandle& nh,
                                       const std::string topic_name,
                                       const size_t buff_size)
    : nh_(nh) {
  subscriber_ = nh_.subscribe(topic_name,
                              buff_size,
                              &VelocitySubscriber::MsgCallback,
                              this,
                              ros::TransportHints().tcpNoDelay());
}

void VelocitySubscriber::MsgCallback(
    const geometry_msgs::TwistStampedConstPtr& twist_msg_ptr) {
  buff_mutex_.lock();
  VelocityData velocity_data;
  velocity_data.time_ = twist_msg_ptr->header.stamp.toSec();

  velocity_data.linear_velocity_.x = twist_msg_ptr->twist.linear.x;
  velocity_data.linear_velocity_.y = twist_msg_ptr->twist.linear.y;
  velocity_data.linear_velocity_.z = twist_msg_ptr->twist.linear.z;

  velocity_data.angular_velocity_.x = twist_msg_ptr->twist.angular.x;
  velocity_data.angular_velocity_.y = twist_msg_ptr->twist.angular.y;
  velocity_data.angular_velocity_.z = twist_msg_ptr->twist.angular.z;

  new_velocity_data_.push_back(velocity_data);
  buff_mutex_.unlock();
}

void VelocitySubscriber::ParseData(
    std::deque<VelocityData>& velocity_data_buff) {
  buff_mutex_.lock();
  if (new_velocity_data_.size() > 0) {
    velocity_data_buff.insert(velocity_data_buff.end(),
                              new_velocity_data_.begin(),
                              new_velocity_data_.end());
    new_velocity_data_.clear();
  }
  buff_mutex_.unlock();
}
}  // namespace lidar_localization