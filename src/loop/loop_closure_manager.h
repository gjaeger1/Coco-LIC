#pragma once

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "loop_config.h"
#include "loop_detector.h"
#include "loop_types.h"
#include "loop_verifier.h"
#include <lidar/lidar_handler.h>
#include <spline/trajectory.h>

namespace cocolic
{
  class PoseGraph;

  // Orchestrates loop closure: keyframe intake -> detection -> verification ->
  // pose graph. Runs synchronously in the odometry loop (deterministic:
  // same bag -> same loops). Owns the JSONL candidate log.
  class LoopClosureManager
  {
  public:
    LoopClosureManager(const LoopClosureConfig &config,
                       std::shared_ptr<Trajectory> trajectory,
                       std::shared_ptr<LidarHandler> lidar_handler,
                       const std::string &default_log_dir);

    ~LoopClosureManager();

    bool Enabled() const { return config_.enable; }

    // Call once per new keyframe time, in creation order.
    void OnKeyframe(int64_t kf_time_ns);

    // After the bag: solve the pose graph (if any loops) and return per-keyframe
    // corrections for the trajectory deformer; empty if nothing to correct.
    // Also writes poses_pgo_tum.txt and flushes the log. Idempotent.
    std::vector<KeyframeCorrection> Finalize();

    // Applies `corrections` to the spline in place (finalize-time deformation).
    // Call exactly once, after the bag ends and before any pose is saved or
    // exported. Returns the number of control points modified.
    size_t ApplyCorrectionsToTrajectory(
        const std::vector<KeyframeCorrection> &corrections);

    // Writes the uncorrected odometry keyframe poses to poses_odom_tum.txt.
    void WriteOdomTum();

    const std::vector<KeyframeSnapshot> &Snapshots() const { return snapshots_; }
    size_t NumAcceptedLoops() const;

  private:
    void LogCandidate(const LoopCandidate &c, const VerificationReport &r, bool injected = false);

    LoopClosureConfig config_;
    std::shared_ptr<Trajectory> trajectory_;
    std::shared_ptr<LidarHandler> lidar_handler_;

    LoopDetector::Ptr detector_;
    std::unique_ptr<LoopVerifier> verifier_;
    std::unique_ptr<PoseGraph> pose_graph_;

    std::vector<KeyframeSnapshot> snapshots_;
    std::string log_dir_;
    std::ofstream log_;         // <log_dir>/loop_candidates.jsonl, opened in ctor
    bool finalized_ = false;
  };

} // namespace cocolic
