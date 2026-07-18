/*
 * Continuous-time loop-closure back-end (offline / finalize).
 *
 * Replaces the old GTSAM pose graph + TrajectoryDeformer. It builds ONE global
 * Ceres problem over the B-spline control points and solves it, deforming the
 * trajectory in place so that:
 *   - each verified loop is satisfied (robust loop edges),
 *   - the online local trajectory shape is preserved (a dense relative-pose
 *     "backbone" whose measurements are the pre-loop online relative poses),
 *   - the start is anchored (gauge: first SplineOrder control points fixed).
 *
 * All edges are the two-time relative-pose factor LoopClosureFactorNURBS. The
 * spline is the IMU trajectory; loop constraints (LiDAR frame) are converted to
 * IMU frame via the LiDAR->IMU extrinsic.
 */
#pragma once

#include <memory>
#include <vector>

#include "loop_config.h"
#include "loop_types.h"
#include <spline/trajectory.h>

namespace cocolic
{

  class LoopBackend
  {
  public:
    LoopBackend(const LoopClosureConfig &config, std::shared_ptr<Trajectory> trajectory)
        : config_(config), trajectory_(trajectory) {}

    // Deforms the spline in place. Returns the number of loop edges added
    // (0 => nothing solved). `snapshots` provides keyframe times; `loops` are the
    // verified relative-pose constraints (LiDAR frame).
    size_t Solve(const std::vector<KeyframeSnapshot> &snapshots,
                 const std::vector<LoopConstraint> &loops);

  private:
    LoopClosureConfig config_;
    std::shared_ptr<Trajectory> trajectory_;
  };

}  // namespace cocolic
