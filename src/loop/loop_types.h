#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <utils/mypcl_cloud_type.h>   // PosCloud
#include <utils/parameter_struct.h>   // SE3d (Sophus)

#include "trajectory_deformer.h"

namespace cocolic
{

  // One entry per keyframe, index-aligned with LidarHandler::cloud_key_pos_.
  struct KeyframeSnapshot
  {
    int index = -1;          // position in cloud_key_pos_ / the pose graph
    int64_t time_ns = -1;    // keyframe scan time (trajectory time base)
    SE3d T_LtoG_odom;        // LiDAR->world pose from the spline when the keyframe was created
    PosCloud::Ptr scan_ds;   // undistorted, downsampled scan in the LiDAR frame at time_ns
  };

  // Detector output: a hypothesis, unverified.
  struct LoopCandidate
  {
    int query_index = -1;
    int match_index = -1;
    double descriptor_score = -1.0;  // detector-specific; -1 if the detector exposes none
    double yaw_init = 0.0;           // coarse relative yaw [rad] from the detector, 0 if unknown
  };

  // Every field is written to the JSONL log, accepted or not.
  struct VerificationReport
  {
    bool accepted = false;
    double icp_fitness = -1.0;       // PCL getFitnessScore(): mean squared corr. distance
    double overlap_ratio = -1.0;     // fraction of source points with a close target neighbor
    std::string rejected_by;         // empty, or the name of the gate that fired
  };

  // A verified loop, ready for the pose graph.
  struct LoopConstraint
  {
    int query_index = -1;
    int match_index = -1;
    SE3d T_match_query;              // pose of query keyframe expressed in the match keyframe's LiDAR frame
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
    bool injected = false;           // true only for deliberately wrong research-tool loops
  };

} // namespace cocolic
