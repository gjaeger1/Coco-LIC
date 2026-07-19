#include "loop_backend.h"

#include <ceres/ceres.h>
#include <cmath>
#include <iostream>

#include <odom/trajectory_estimator.h>
#include <odom/trajectory_estimator_options.h>

#include "information_provider.h"

namespace cocolic
{

  size_t LoopBackend::Solve(const std::vector<KeyframeSnapshot> &snapshots,
                            const std::vector<LoopConstraint> &loops)
  {
    if (loops.empty() || snapshots.empty())
      return 0;

    // Loop constraints are between LiDAR poses; the spline is the IMU trajectory.
    //   T_Lmatch^-1 T_Lquery = T_LtoI^-1 (T_Imatch^-1 T_Iquery) T_LtoI
    // => IMU-frame measurement = T_LtoI * T_match_query * T_LtoI^-1.
    SE3d T_LtoI = trajectory_->GetSensorEP(LiDARSensor).se3;

    TrajectoryEstimatorOptions options;
    options.lock_traj = false;
    options.lock_tran = false;
    options.use_true_lie_manifold = true;  // all loop-problem factors are auto-diff
    TrajectoryEstimator estimator(trajectory_, options, "loop");
    // Gauge: fix the first SplineOrder control points (anchor the start pose).
    estimator.SetFixedIndex(SplineOrder - 1);

    // Factor weights come from the InformationProvider seam (Step-1: static;
    // Step-2 swaps in a covariance-derived provider without touching this code).
    StaticInformationProvider info(config_);

    // ---- dense relative-pose backbone (preserves online local shape) ----
    // One relative-pose edge between every knot group K and K+SplineOrder, i.e.
    // stepping by SplineOrder knots. This is KNOT-INDEX based, not time based:
    // Coco-LIC uses non-uniform knots (as dense as ~0.025 s), so a fixed-time
    // backbone leaves the interior knots of each edge unconstrained in dense
    // regions -> the loop pull makes them oscillate (trajectory blows up). At a
    // stride of SplineOrder, consecutive edges are adjacent (edge K covers knots
    // K..K+3 and K+4..K+7; edge K+4 covers K+4..K+7 and K+8..K+11), so EVERY
    // control point is constrained. Measurement = current (online) relative
    // pose, so the backbone is initially zero-residual (shape-preserving).
    const std::vector<int64_t> &knts = trajectory_->knts;
    const int num_seg = static_cast<int>(trajectory_->blending_mats.size());
    const int64_t t_end = trajectory_->maxTimeNsNURBS();
    // idx_i = K needs GetIdxT(t_i).first = K+SplineOrder-1, i.e. t_i at the start
    // of segment (K+SplineOrder-1); idx_j = K+SplineOrder similarly.
    for (int K = 0; K + 2 * SplineOrder <= num_seg; K += SplineOrder)
    {
      const int seg_i = K + (SplineOrder - 1);
      const int seg_j = K + SplineOrder + (SplineOrder - 1);
      if (seg_j + 1 >= static_cast<int>(knts.size())) break;
      const int64_t t_i = knts[seg_i];
      const int64_t t_j = knts[seg_j];
      if (t_j >= t_end) break;
      SE3d Ti = trajectory_->GetIMUPoseNsNURBS(t_i);
      SE3d Tj = trajectory_->GetIMUPoseNsNURBS(t_j);
      estimator.AddLoopClosureFactorNURBS(t_i, t_j, Ti.inverse() * Tj,
                                          info.BackboneSqrtInfo(t_i, t_j), nullptr);
    }

    // ---- loop edges (robust) ----
    size_t n_loops = 0;
    for (const auto &lc : loops)
    {
      if (lc.match_index < 0 || lc.query_index < 0 ||
          lc.match_index >= static_cast<int>(snapshots.size()) ||
          lc.query_index >= static_cast<int>(snapshots.size()))
        continue;
      int64_t t_match = snapshots[lc.match_index].time_ns;  // earlier
      int64_t t_query = snapshots[lc.query_index].time_ns;  // later
      if (t_match >= t_query)
        continue;  // factor requires i-time < j-time

      SE3d T_meas_imu = T_LtoI * lc.T_match_query * T_LtoI.inverse();
      Eigen::Matrix<double, 6, 1> lp_sqrt = info.LoopSqrtInfo(lc);
      auto *loss = new ceres::CauchyLoss(config_.loop_robust_cauchy);
      estimator.AddLoopClosureFactorNURBS(t_match, t_query, T_meas_imu, lp_sqrt, loss);
      ++n_loops;
    }

    if (n_loops == 0)
      return 0;

    // Pin the knot null space the sparse loop/backbone factors leave open, so
    // the dense trajectory stays smooth under the deformation.
    estimator.AddPositionKnotPriorsNURBS(config_.knot_prior_weight);

    ceres::Solver::Summary summary = estimator.Solve(config_.loop_max_iterations, false);
    std::cout << "🔁 LoopBackend: " << n_loops << " loop edges, spline deformed ("
              << summary.initial_cost << " -> " << summary.final_cost << ")\n";
    return n_loops;
  }

}  // namespace cocolic
