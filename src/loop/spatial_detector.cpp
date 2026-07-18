#include "spatial_detector.h"

#include <cmath>

namespace cocolic
{

  SpatialDetector::SpatialDetector(const LoopClosureConfig &config)
      : min_index_gap_(config.min_index_gap),
        search_radius_(config.spatial_search_radius_m) {}

  std::vector<LoopCandidate> SpatialDetector::AddAndQuery(const KeyframeSnapshot &kf)
  {
    const Eigen::Vector3d p = kf.T_LtoG_odom.translation();
    const Eigen::Matrix3d R = kf.T_LtoG_odom.unit_quaternion().toRotationMatrix();

    // Nearest past keyframe within the search ball and index gap.
    std::vector<LoopCandidate> out;
    int best_index = -1;
    double best_dist = search_radius_;
    Eigen::Matrix3d best_rot = Eigen::Matrix3d::Identity();
    for (const auto &e : history_)
    {
      if (kf.index - e.index < min_index_gap_) continue;
      double d = (p - e.pos).norm();
      if (d < best_dist)
      {
        best_dist = d;
        best_index = e.index;
        best_rot = e.rot;
      }
    }

    history_.push_back({kf.index, p, R});

    if (best_index >= 0)
    {
      LoopCandidate c;
      c.query_index = kf.index;
      c.match_index = best_index;
      c.descriptor_score = best_dist;  // metric distance (lower = closer); detector-specific
      // yaw of the relative odometry pose R_match^-1 * R_query, as a GICP init.
      Eigen::Matrix3d R_rel = best_rot.transpose() * R;
      c.yaw_init = std::atan2(R_rel(1, 0), R_rel(0, 0));
      out.push_back(c);
    }
    return out;
  }

} // namespace cocolic
