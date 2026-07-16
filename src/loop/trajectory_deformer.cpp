#include "trajectory_deformer.h"

#include <algorithm>
#include <cassert>

namespace cocolic
{

  SE3d InterpolateCorrection(const std::vector<KeyframeCorrection> &corrections,
                             int64_t t_ns)
  {
    assert(!corrections.empty());
    if (t_ns <= corrections.front().time_ns) return corrections.front().delta;
    if (t_ns >= corrections.back().time_ns) return corrections.back().delta;

    auto hi = std::lower_bound(
        corrections.begin(), corrections.end(), t_ns,
        [](const KeyframeCorrection &c, int64_t t) { return c.time_ns < t; });
    auto lo = hi - 1;
    double alpha = double(t_ns - lo->time_ns) / double(hi->time_ns - lo->time_ns);

    // Interpolate in the group: slerp for rotation, lerp for translation.
    Eigen::Quaterniond q = lo->delta.unit_quaternion().slerp(
        alpha, hi->delta.unit_quaternion());
    Eigen::Vector3d p = (1.0 - alpha) * lo->delta.translation() +
                        alpha * hi->delta.translation();
    return SE3d(q, p);
  }

  void DeformKnots(const std::vector<int64_t> &knot_times,
                   std::vector<SO3d> &knot_R,
                   std::vector<Eigen::Vector3d> &knot_p,
                   const std::vector<KeyframeCorrection> &corrections)
  {
    if (corrections.empty()) return;
    assert(knot_times.size() == knot_R.size() && knot_R.size() == knot_p.size());

    for (size_t i = 0; i < knot_times.size(); ++i)
    {
      const SE3d delta = InterpolateCorrection(corrections, knot_times[i]);
      knot_p[i] = delta.so3() * knot_p[i] + delta.translation();
      knot_R[i] = delta.so3() * knot_R[i];
    }
  }

} // namespace cocolic
