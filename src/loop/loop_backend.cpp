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
    TrajectoryEstimator estimator(trajectory_, options, "loop");
    // Gauge: fix the first SplineOrder control points (anchor the start pose).
    estimator.SetFixedIndex(SplineOrder - 1);

    // Factor weights come from the InformationProvider seam (Step-1: static;
    // Step-2 swaps in a covariance-derived provider without touching this code).
    StaticInformationProvider info(config_);

    // ---- dense relative-pose backbone (preserves online local shape) ----
    // Edges spaced backbone_dt_s apart so knot windows stay disjoint yet cover
    // and link every control point; measurement = current (online) IMU relative
    // pose, so the backbone is initially zero-residual.
    const int64_t t_start = trajectory_->knts[SplineOrder - 1];  // first valid eval time
    const int64_t t_end = trajectory_->maxTimeNsNURBS();
    const int64_t dt = static_cast<int64_t>(config_.backbone_dt_s * S_TO_NS);
    if (dt <= 0)
      return 0;
    for (int64_t t = t_start; t + dt < t_end; t += dt)
    {
      SE3d Ti = trajectory_->GetIMUPoseNsNURBS(t);
      SE3d Tj = trajectory_->GetIMUPoseNsNURBS(t + dt);
      estimator.AddLoopClosureFactorNURBS(t, t + dt, Ti.inverse() * Tj,
                                          info.BackboneSqrtInfo(t, t + dt), nullptr);
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

    ceres::Solver::Summary summary = estimator.Solve(config_.loop_max_iterations, false);
    std::cout << "🔁 LoopBackend: " << n_loops << " loop edges, spline deformed ("
              << summary.initial_cost << " -> " << summary.final_cost << ")\n";
    return n_loops;
  }

}  // namespace cocolic
