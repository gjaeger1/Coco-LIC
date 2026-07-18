#pragma once

#include <memory>
#include <vector>

#include <opencv2/features2d.hpp>
#include <DBoW3/DBoW3.h>

#include "loop_config.h"
#include "loop_detector.h"

namespace cocolic
{

  // Camera place recognition: ORB + DBoW3 bag-of-words. Drift-independent and
  // appearance-based, so it proposes revisits even where the odometry has
  // drifted too far for a spatial ball. Works on the forward-FOV Livox-HAP rig
  // (where Scan Context aliases) because it recognizes the *scene*, not the
  // LiDAR sweep geometry.
  //
  // Like ScanContextDetector, the DBoW3 database is keyed by insertion order, so
  // db_to_snapshot_ maps a returned DB entry id back to the snapshot index
  // (they differ under detection_stride > 1 or when LIO frames have no image).
  class VisualBoWDetector : public LoopDetector
  {
  public:
    explicit VisualBoWDetector(const LoopClosureConfig &config);

    std::vector<LoopCandidate> AddAndQuery(const KeyframeSnapshot &kf) override;

    std::string Name() const override { return "visual_bow"; }

  private:
    std::unique_ptr<DBoW3::Vocabulary> voc_;
    std::unique_ptr<DBoW3::Database> db_;
    cv::Ptr<cv::ORB> orb_;
    int min_index_gap_;
    double score_min_;
    int max_candidates_;
    std::vector<int> db_to_snapshot_;  // DBoW3 DB entry id -> kf.index
  };

} // namespace cocolic
