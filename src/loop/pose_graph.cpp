#include "pose_graph.h"

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/linear/NoiseModel.h>  // noiseModel::Robust, mEstimator::*, Diagonal
#include <gtsam/config.h>

#if GTSAM_VERSION_MAJOR > 4 || (GTSAM_VERSION_MAJOR == 4 && GTSAM_VERSION_MINOR >= 1)
#include <gtsam/nonlinear/GncOptimizer.h>
#endif

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace cocolic
{

  static gtsam::Pose3 ToGtsam(const SE3d &T)
  {
    return gtsam::Pose3(gtsam::Rot3(T.unit_quaternion().toRotationMatrix()),
                        gtsam::Point3(T.translation()));
  }

  static SE3d FromGtsam(const gtsam::Pose3 &P)
  {
    return SE3d(Eigen::Quaterniond(P.rotation().matrix()), P.translation());
  }

  PoseGraph::PoseGraph(const LoopClosureConfig &config)
      : config_(config) {}

  void PoseGraph::AddKeyframe(const KeyframeSnapshot &kf)
  {
    initial_.insert(gtsam::Key(kf.index), ToGtsam(kf.T_LtoG_odom));

    if (kf.index == 0)
    {
      // Prior factor on the first keyframe to fix the gauge
      auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4).finished());
      graph_.push_back(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Key(0), ToGtsam(kf.T_LtoG_odom), prior_noise));
    }
    else
    {
      gtsam::Key prev = gtsam::Key(kf.index - 1);
      gtsam::Key cur = gtsam::Key(kf.index);
      SE3d T_prev = odom_poses_.back();
      SE3d T_cur = kf.T_LtoG_odom;

      double sr = config_.odom_rotation_sigma_deg * M_PI / 180.0;
      double st = config_.odom_translation_sigma_m;
      double srp = sr / std::sqrt(config_.rp_information_scale);  // roll/pitch stiffer: gravity is observable via IMU
      auto odom_noise = gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector(6) << srp, srp, sr, st, st, st).finished());

      graph_.push_back(gtsam::BetweenFactor<gtsam::Pose3>(
          prev, cur, ToGtsam(T_prev.inverse() * T_cur), odom_noise));
    }

    odom_poses_.push_back(kf.T_LtoG_odom);
  }

  void PoseGraph::AddLoop(const LoopConstraint &constraint)
  {
    gtsam::Key match_key = gtsam::Key(constraint.match_index);
    gtsam::Key query_key = gtsam::Key(constraint.query_index);
    auto loop_noise = MakeLoopNoise(constraint.information);

    graph_.push_back(gtsam::BetweenFactor<gtsam::Pose3>(
        match_key, query_key, ToGtsam(constraint.T_match_query), loop_noise));

    loop_factor_slots_.push_back(graph_.size() - 1);
    ++num_loops_;
  }

  gtsam::noiseModel::Base::shared_ptr PoseGraph::MakeLoopNoise(
      const Eigen::Matrix<double, 6, 6> &information) const
  {
    using namespace gtsam::noiseModel;
    auto base = Gaussian::Information(information);

    if (config_.robust_kernel == "none")
    {
      return base;
    }
    else if (config_.robust_kernel == "huber")
    {
      return Robust::Create(mEstimator::Huber::Create(1.345), base);
    }
    else if (config_.robust_kernel == "cauchy")
    {
      return Robust::Create(mEstimator::Cauchy::Create(1.0), base);
    }
    else if (config_.robust_kernel == "dcs")
    {
      return Robust::Create(mEstimator::DCS::Create(1.0), base);
    }
    else if (config_.robust_kernel == "gnc")
    {
      return base; // GNC handles robust estimation inside its own optimizer
    }
    else
    {
      throw std::invalid_argument("unknown robust_kernel: " + config_.robust_kernel);
    }
  }

  std::vector<SE3d> PoseGraph::Optimize()
  {
    gtsam::Values result;

    if (config_.robust_kernel == "gnc")
    {
#if GTSAM_VERSION_MAJOR > 4 || (GTSAM_VERSION_MAJOR == 4 && GTSAM_VERSION_MINOR >= 1)
      gtsam::GncParams<gtsam::LevenbergMarquardtParams> gnc_params;
      std::vector<size_t> known;
      for (size_t i = 0; i < graph_.size(); ++i)
      {
        if (std::find(loop_factor_slots_.begin(), loop_factor_slots_.end(), i) == loop_factor_slots_.end())
        {
          known.push_back(i);
        }
      }
      gnc_params.setKnownInliers(known);
      gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams>> opt(graph_, initial_, gnc_params);
      result = opt.optimize();
#else
      throw std::runtime_error("robust_kernel 'gnc' requires GTSAM >= 4.1");
#endif
    }
    else
    {
      gtsam::LevenbergMarquardtOptimizer optimizer(graph_, initial_);
      result = optimizer.optimize();
    }

    initial_ = result;

    std::vector<SE3d> corrected_poses;
    corrected_poses.reserve(odom_poses_.size());
    for (size_t i = 0; i < odom_poses_.size(); ++i)
    {
      gtsam::Key k = gtsam::Key(i);
      if (result.exists(k))
      {
        corrected_poses.push_back(FromGtsam(result.at<gtsam::Pose3>(k)));
      }
      else
      {
        corrected_poses.push_back(odom_poses_[i]);
      }
    }

    return corrected_poses;
  }

} // namespace cocolic
