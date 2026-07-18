#pragma once

#include <vector>

#include <Eigen/Core>

#include "loop_config.h"
#include "loop_detector.h"

namespace cocolic
{

  // Sensor-agnostic proposer: propose the nearest PAST keyframe whose current
  // odometry position is within search_radius of the query and whose index gap
  // exceeds min_index_gap. Uses only KeyframeSnapshot::T_LtoG_odom, so it works
  // for any sensor suite. search_radius must exceed the expected odometry drift
  // at a revisit, or true loops fall outside the ball and are missed.
  //
  // It seeds the candidate yaw_init from the relative odometry yaw, giving the
  // LiDAR verifier a good GICP initial guess (important for partial-FOV LiDAR).
  class SpatialDetector : public LoopDetector
  {
  public:
    explicit SpatialDetector(const LoopClosureConfig &config);

    std::vector<LoopCandidate> AddAndQuery(const KeyframeSnapshot &kf) override;

    std::string Name() const override { return "spatial"; }

  private:
    int min_index_gap_;
    double search_radius_;
    struct Entry
    {
      int index;
      Eigen::Vector3d pos;
      Eigen::Matrix3d rot;
    };
    std::vector<Entry> history_;
  };

} // namespace cocolic
