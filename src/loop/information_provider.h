#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Core>

#include "loop_config.h"
#include "loop_types.h"

namespace cocolic
{

  // ---------------------------------------------------------------------------
  // Step-2 seam: dynamic uncertainty re-weighting (INTERFACE ONLY for now).
  //
  // Every relative-pose factor in the continuous-time loop back-end
  // (LoopBackend) -- both the odometry backbone edges and the loop-closure
  // edges -- draws its weight from an InformationProvider. Step 1 ships
  // StaticInformationProvider, whose weights are config constants (loop edges)
  // and the verifier's 6x6 (loop constraint). This reproduces the hand-wired
  // behavior that lived inline in LoopBackend::Solve.
  //
  // Step 2 (design only, do NOT implement here) replaces the static weights
  // with runtime, uncertainty-derived ones WITHOUT touching the back-end or the
  // LoopClosureFactorNURBS. Candidate sources of a recovered information matrix,
  // all already present in Coco-LIC, are noted at each hook below:
  //   * the marginalization prior (MarginalizationInfo::marginalize assembles
  //     A = sum(J^T J) and Schur-complements it over the kept control points --
  //     a recovered information over the trajectory);
  //   * a post-solve ceres::Covariance pass over the control points;
  //   * per-sensor verification Hessians (LiDAR GICP information; a future
  //     visual-PnP information), fused per loop edge.
  //
  // Convention: the 6-vector / 6x6 is ordered [translation(3); rotation(3)],
  // matching Sophus::SE3::log() and LoopRelativePoseFunctor's residual layout.
  // Providers return a *sqrt-information diagonal* (the per-residual weight the
  // factor multiplies in), so a diagonal information model needs no factoring
  // and a full-matrix model (Step 2) can supply its Cholesky diagonal or be
  // extended to a full 6x6 sqrt-info without changing call sites.
  // ---------------------------------------------------------------------------
  class InformationProvider
  {
  public:
    virtual ~InformationProvider() = default;

    // sqrt-information (diagonal, [trans; rot]) for one verified loop edge.
    // Step-2 hook: fuse per-sensor verification covariances here (e.g. inflate
    // the LiDAR contribution on low-overlap partial-FOV scans, raise the camera
    // contribution on a strong visual PnP) -> loop info = LiDAR (+) visual.
    virtual Eigen::Matrix<double, 6, 1>
    LoopSqrtInfo(const LoopConstraint &lc) const = 0;

    // sqrt-information (diagonal, [trans; rot]) for one odometry-backbone edge
    // spanning [t_i_ns, t_j_ns] on the trajectory.
    // Step-2 hook: query recovered trajectory covariance over this interval
    // (marginalization Schur complement / ceres::Covariance) so the backbone
    // stiffness tracks how well the online odometry actually constrained the
    // control points, instead of a single global sigma.
    virtual Eigen::Matrix<double, 6, 1>
    BackboneSqrtInfo(int64_t t_i_ns, int64_t t_j_ns) const = 0;
  };

  // Step-1 default: agnostic static weights. Behaviorally identical to the
  // weights LoopBackend::Solve computed inline.
  //   * loop edges  -> the verifier's 6x6 information (its sqrt diagonal);
  //   * backbone    -> fixed config sigmas (odom_translation_sigma_m,
  //                    odom_rotation_sigma_deg).
  class StaticInformationProvider : public InformationProvider
  {
  public:
    explicit StaticInformationProvider(const LoopClosureConfig &cfg)
    {
      const double st = std::max(cfg.odom_translation_sigma_m, 1e-6);
      const double sr = std::max(cfg.odom_rotation_sigma_deg * M_PI / 180.0, 1e-6);
      backbone_sqrt_ << 1.0 / st, 1.0 / st, 1.0 / st,
          1.0 / sr, 1.0 / sr, 1.0 / sr;
    }

    Eigen::Matrix<double, 6, 1>
    LoopSqrtInfo(const LoopConstraint &lc) const override
    {
      return lc.information.diagonal().cwiseMax(1e-8).cwiseSqrt();
    }

    Eigen::Matrix<double, 6, 1>
    BackboneSqrtInfo(int64_t /*t_i_ns*/, int64_t /*t_j_ns*/) const override
    {
      return backbone_sqrt_;
    }

  private:
    Eigen::Matrix<double, 6, 1> backbone_sqrt_;
  };

} // namespace cocolic
