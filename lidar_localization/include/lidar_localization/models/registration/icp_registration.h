/*
 * @Description: ICP 匹配模块
 * @Author: Ge Yao
 * @Date: 2020-10-24 21:46:57
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-11-05 09:59:21
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_REGISTRATION_H_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_REGISTRATION_H_

#include <glog/logging.h>

#include <pcl/registration/icp.h>

#include "lidar_localization/models/registration/registration_interface.h"

namespace lidar_localization {
class ICPRegistration : public RegistrationInterface {
 public:
  ICPRegistration(const YAML::Node& node);
  ICPRegistration(float max_corr_dist,
                  float trans_eps,
                  float euc_fitness_eps,
                  int max_iter);

  bool SetInputTarget(const CloudData::CloudTypePtr& input_target) override;
  bool ScanMatch(const CloudData::CloudTypePtr& input_source,
                 const Eigen::Matrix4f& predict_pose,
                 CloudData::CloudTypePtr& result_cloud_ptr,
                 Eigen::Matrix4f& result_pose) override;

  float GetFitnessScore() override;

  bool IsDegeneracy() override;

 private:
  bool SetRegistrationParam(float max_corr_dist,
                            float trans_eps,
                            float euc_fitness_eps,
                            int max_iter);

 private:
  pcl::IterativeClosestPoint<CloudData::PointType, CloudData::PointType>::Ptr
      icp_ptr_{};
};
}  // namespace lidar_localization

#endif