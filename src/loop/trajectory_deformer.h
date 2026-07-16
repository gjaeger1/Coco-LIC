#pragma once

#include <cstdint>
#include <vector>
#include <cmath>
namespace Eigen {
  using std::atan2;
}

#include <utils/sophus_utils.hpp>

namespace cocolic
{

  // Result of a pose-graph solve for one keyframe.
  struct KeyframeCorrection
  {
    int64_t time_ns = -1;
    SE3d delta;                      // left-multiplicative world-frame correction:
                                     // T_corrected = delta * T_odom
  };

  // Correction interpolated at time t: SO(3) slerp + linear translation between
  // the two bracketing keyframe corrections. Outside the keyframe range the
  // nearest correction is used (constant extrapolation).
  // Preconditions: `corrections` sorted by time_ns, non-empty.
  SE3d InterpolateCorrection(const std::vector<KeyframeCorrection> &corrections,
                             int64_t t_ns);

  // In-place warp of spline control points: for each i,
  //   R_i <- delta(t_i).so3() * R_i
  //   p_i <- delta(t_i).so3() * p_i + delta(t_i).translation()
  // with t_i = knot_times[i]. knot_R/knot_p/knot_times must have equal length
  // (callers pass min(numKnots, knts.size()) entries). No-op if `corrections`
  // is empty.
  //
  // First-order approximation: control points of a segment spanning two
  // keyframes receive slightly different corrections; the resulting spline is
  // continuous but its velocity profile is perturbed within loop-corrected
  // segments. Acceptable for mapping/export; not for re-running IMU factors.
  void DeformKnots(const std::vector<int64_t> &knot_times,
                   std::vector<SO3d> &knot_R,
                   std::vector<Eigen::Vector3d> &knot_p,
                   const std::vector<KeyframeCorrection> &corrections);

} // namespace cocolic
