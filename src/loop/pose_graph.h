#pragma once

#include <vector>

#include "loop_config.h"
#include "loop_types.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

namespace cocolic
{

  // Keyframe pose graph. Deterministic batch backend: same inputs -> same output.
  class PoseGraph
  {
  public:
    explicit PoseGraph(const LoopClosureConfig &config);

    // Must be called once per keyframe in index order. Adds the node's initial
    // value and a BetweenFactor to the previous keyframe (from odometry poses).
    void AddKeyframe(const KeyframeSnapshot &kf);

    void AddLoop(const LoopConstraint &constraint);

    size_t NumLoops() const { return num_loops_; }

    // Batch LM solve. Returns corrected world poses T_LtoG, index-aligned with
    // the keyframes. With zero loop factors this returns the odometry poses
    // (up to solver noise; the caller may skip solving in that case).
    std::vector<SE3d> Optimize();

  private:
    gtsam::noiseModel::Base::shared_ptr MakeLoopNoise(
        const Eigen::Matrix<double, 6, 6> &information) const;

    LoopClosureConfig config_;
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_;
    std::vector<size_t> loop_factor_slots_;  // graph_ indices of loop factors (for GNC known-inliers)
    std::vector<SE3d> odom_poses_;           // per keyframe, for odometry factors
    size_t num_loops_ = 0;
  };

} // namespace cocolic
