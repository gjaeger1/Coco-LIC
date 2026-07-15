#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>
#include <algorithm>
#include "../pose_graph.h"

using namespace cocolic;

// Helper to generate ground truth poses of a square trajectory
SE3d getGroundTruthPose(int i)
{
  double step_size = 1.0;
  double x = 0.0, y = 0.0, yaw = 0.0;
  if (i <= 10)
  {
    x = i * step_size;
    y = 0.0;
    yaw = 0.0;
  }
  else if (i <= 20)
  {
    x = 10 * step_size;
    y = (i - 10) * step_size;
    yaw = M_PI / 2.0;
  }
  else if (i <= 30)
  {
    x = (30 - i) * step_size;
    y = 10 * step_size;
    yaw = M_PI;
  }
  else
  {
    x = 0.0;
    y = (40 - i) * step_size;
    yaw = -M_PI / 2.0;
  }
  Eigen::Matrix3d R;
  R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  return SE3d(Eigen::Quaterniond(R).normalized(), Eigen::Vector3d(x, y, 0.0));
}

void runTest(const std::string &robust_kernel)
{
  std::cout << "\nRunning PoseGraph test with robust_kernel: " << robust_kernel << " ..." << std::endl;

  // 1. Generate ground truth and drifted odometry poses
  std::vector<SE3d> gt_poses(40);
  std::vector<SE3d> odom_poses(40);

  double drift_yaw = 0.5 * M_PI / 180.0; // 0.5 deg drift per step
  Eigen::Matrix3d R_drift;
  R_drift = Eigen::AngleAxisd(drift_yaw, Eigen::Vector3d::UnitZ());
  SE3d T_drift(Eigen::Quaterniond(R_drift).normalized(), Eigen::Vector3d(0.02, -0.01, 0.0));

  gt_poses[0] = getGroundTruthPose(0);
  odom_poses[0] = gt_poses[0];

  for (int i = 1; i < 40; ++i)
  {
    gt_poses[i] = getGroundTruthPose(i);
    SE3d T_rel_gt = gt_poses[i - 1].inverse() * gt_poses[i];
    odom_poses[i] = odom_poses[i - 1] * T_rel_gt * T_drift;
  }

  // Calculate uncorrected error at index 39
  SE3d T_err_uncorrected = gt_poses[39].inverse() * odom_poses[39];
  double trans_err_uncorrected = T_err_uncorrected.translation().norm();
  double rot_err_uncorrected = T_err_uncorrected.so3().log().norm();

  std::cout << "Uncorrected error at keyframe 39:" << std::endl;
  std::cout << " - Translation: " << trans_err_uncorrected << " m" << std::endl;
  std::cout << " - Rotation: " << rot_err_uncorrected * 180.0 / M_PI << " deg" << std::endl;

  // 2. Build the pose graph
  LoopClosureConfig config;
  config.robust_kernel = robust_kernel;
  config.odom_rotation_sigma_deg = 0.5;
  config.odom_translation_sigma_m = 0.05;
  config.rp_information_scale = 100.0;

  PoseGraph pg(config);
  for (int i = 0; i < 40; ++i)
  {
    KeyframeSnapshot kf;
    kf.index = i;
    kf.time_ns = i * 1000000000LL;
    kf.T_LtoG_odom = odom_poses[i];
    pg.AddKeyframe(kf);
  }

  // Add loop constraint between 39 and 0
  LoopConstraint constraint;
  constraint.query_index = 39;
  constraint.match_index = 0;
  constraint.T_match_query = gt_poses[0].inverse() * gt_poses[39];
  constraint.information = 400.0 * Eigen::Matrix<double, 6, 6>::Identity();
  pg.AddLoop(constraint);

  // 3. Optimize
  std::vector<SE3d> corrected = pg.Optimize();

  // 4. Verify correction
  SE3d T_err_corrected = gt_poses[39].inverse() * corrected[39];
  double trans_err_corrected = T_err_corrected.translation().norm();
  double rot_err_corrected = T_err_corrected.so3().log().norm();

  std::cout << "Corrected error at keyframe 39:" << std::endl;
  std::cout << " - Translation: " << trans_err_corrected << " m (threshold: " << 0.1 * trans_err_uncorrected << " m)" << std::endl;
  std::cout << " - Rotation: " << rot_err_corrected * 180.0 / M_PI << " deg (threshold: " << 0.1 * rot_err_uncorrected * 180.0 / M_PI << " deg)" << std::endl;

  assert(trans_err_corrected < 0.1 * trans_err_uncorrected && "Translation correction failed!");
  assert(rot_err_corrected < 0.1 * rot_err_uncorrected && "Rotation correction failed!");

  std::cout << "Test passed successfully for robust_kernel: " << robust_kernel << std::endl;
}

int main()
{
  runTest("none");
  runTest("cauchy");
  std::cout << "\nAll PoseGraph tests passed!" << std::endl;
  return 0;
}
