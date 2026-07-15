#pragma once

#include "loop_config.h"
#include "loop_types.h"

#include <lidar/lidar_handler.h>
#include <spline/trajectory.h>
#include <memory>
#include <vector>

namespace cocolic
{

  // Free function to verify a loop candidate against a pre-loaded submap.
  // This allows unit testing without requiring a fully running LidarHandler.
  bool VerifyAgainstSubmap(const LoopCandidate &candidate,
                           const std::vector<KeyframeSnapshot> &snapshots,
                           const PosCloud::Ptr &submap_in_G,
                           const LoopClosureConfig &config,
                           std::shared_ptr<Trajectory> trajectory,
                           LoopConstraint &constraint,
                           VerificationReport &report);

  // Geometric verification of loop candidates (research seam: every acceptance
  // decision in the pipeline is made in this class and logged by the caller).
  class LoopVerifier
  {
  public:
    LoopVerifier(const LoopClosureConfig &config,
                 std::shared_ptr<LidarHandler> lidar_handler,
                 std::shared_ptr<Trajectory> trajectory);

    // Fills `report` always; fills `constraint` and returns true only when the
    // candidate passes every gate. `snapshots` is the full keyframe list,
    // indexed by KeyframeSnapshot::index.
    bool Verify(const LoopCandidate &candidate,
                const std::vector<KeyframeSnapshot> &snapshots,
                LoopConstraint &constraint,
                VerificationReport &report);

  private:
    LoopClosureConfig config_;
    std::shared_ptr<LidarHandler> lidar_handler_;
    std::shared_ptr<Trajectory> trajectory_;
  };

} // namespace cocolic
