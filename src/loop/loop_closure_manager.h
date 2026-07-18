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

  // Orchestrates loop closure: keyframe intake -> detection -> verification ->
  // collect verified constraints. Runs synchronously in the odometry loop
  // (deterministic: same bag -> same loops). Owns the JSONL candidate log. The
  // actual correction is done by the continuous-time LoopBackend (driven by
  // OdometryManager at finalize), which deforms the spline in place.
  class LoopClosureManager
  {
  public:
    LoopClosureManager(const LoopClosureConfig &config,
                       std::shared_ptr<Trajectory> trajectory,
                       std::shared_ptr<LidarHandler> lidar_handler,
                       const std::string &default_log_dir);

    ~LoopClosureManager();

    bool Enabled() const { return config_.enable; }

    // Call once per new keyframe, in creation order. T_LtoG_odom must be the
    // odometry LiDAR pose captured at keyframe-creation time (see
    // LidarHandler::TakeNewKeyframes); it is not re-queried from the spline.
    // `image` is the current LICO-window image at keyframe-creation time (may be
    // empty in LIO mode); the visual detector extracts its descriptor here and
    // the full image is then released.
    void OnKeyframe(int64_t kf_time_ns, const SE3d &T_LtoG_odom,
                    const cv::Mat &image = cv::Mat());

    // After the bag: run the false-loop injection (research adversary) and flush
    // the candidate log. Returns the number of accepted loops (incl. injected).
    // Does NOT touch the spline -- the caller runs LoopBackend with AcceptedLoops().
    // Idempotent.
    size_t Finalize();

    // Writes the uncorrected odometry keyframe poses to poses_odom_tum.txt.
    void WriteOdomTum();
    // Writes the corrected keyframe poses (queried from the deformed spline);
    // call AFTER LoopBackend has run.
    void WritePgoTum();

    const std::vector<KeyframeSnapshot> &Snapshots() const { return snapshots_; }
    const std::vector<LoopConstraint> &AcceptedLoops() const { return accepted_loops_; }
    const LoopClosureConfig &Config() const { return config_; }
    size_t NumAcceptedLoops() const { return accepted_loops_.size(); }

  private:
    void LogCandidate(const LoopCandidate &c, const VerificationReport &r, bool injected = false);

    LoopClosureConfig config_;
    std::shared_ptr<Trajectory> trajectory_;
    std::shared_ptr<LidarHandler> lidar_handler_;

    std::vector<LoopDetector::Ptr> detectors_;  // run in order; candidates unioned
    std::unique_ptr<LoopVerifier> verifier_;

    std::vector<KeyframeSnapshot> snapshots_;
    std::vector<LoopConstraint> accepted_loops_;  // verified + injected loops
    std::string log_dir_;
    std::ofstream log_;         // <log_dir>/loop_candidates.jsonl, opened in ctor
    bool finalized_ = false;
  };

} // namespace cocolic
