#include "visual_bow_detector.h"

#include <iostream>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace cocolic
{

  VisualBoWDetector::VisualBoWDetector(const LoopClosureConfig &config)
      : min_index_gap_(config.min_index_gap),
        score_min_(config.bow_score_min),
        max_candidates_(config.bow_max_candidates)
  {
    voc_.reset(new DBoW3::Vocabulary(config.bow_vocab_path));
    if (voc_->empty())
      throw std::runtime_error("VisualBoWDetector: failed to load vocabulary '" +
                               config.bow_vocab_path + "'");
    // useDI=false: no direct index (we do not do feature-level geometric
    // verification here; the LiDAR verifier handles geometry).
    db_.reset(new DBoW3::Database(*voc_, false, 0));
    orb_ = cv::ORB::create(config.bow_num_features);
    std::cout << "[VisualBoWDetector] vocab '" << config.bow_vocab_path
              << "' loaded (" << voc_->size() << " words)." << std::endl;
  }

  std::vector<LoopCandidate> VisualBoWDetector::AddAndQuery(const KeyframeSnapshot &kf)
  {
    std::vector<LoopCandidate> out;
    if (kf.image.empty())
      return out;  // LIO mode or no image in this window: visual detector no-ops.

    cv::Mat gray;
    if (kf.image.channels() == 3)
      cv::cvtColor(kf.image, gray, cv::COLOR_BGR2GRAY);
    else if (kf.image.channels() == 4)
      cv::cvtColor(kf.image, gray, cv::COLOR_BGRA2GRAY);
    else
      gray = kf.image;

    std::vector<cv::KeyPoint> kps;
    cv::Mat desc;
    orb_->detectAndCompute(gray, cv::noArray(), kps, desc);
    if (desc.empty())
      return out;  // textureless frame: skip (do not pollute the DB index map).

    // Query BEFORE adding, so results reference only PAST keyframes; then insert
    // this keyframe and record the DB-id -> snapshot-index mapping.
    DBoW3::QueryResults ret;
    db_->query(desc, ret, max_candidates_ + 1);
    db_->add(desc);
    db_to_snapshot_.push_back(kf.index);

    for (const auto &r : ret)
    {
      if (r.Id >= db_to_snapshot_.size())
        continue;  // safety (should not happen: query ran before add)
      int match_index = db_to_snapshot_[r.Id];
      if (kf.index - match_index < min_index_gap_)
        continue;
      if (r.Score < score_min_)
        continue;
      LoopCandidate c;
      c.query_index = kf.index;
      c.match_index = match_index;
      c.descriptor_score = r.Score;  // DBoW3 similarity (higher = better)
      c.yaw_init = 0.0;              // BoW gives no orientation; verifier/spatial supply it
      out.push_back(c);
    }
    return out;
  }

} // namespace cocolic
