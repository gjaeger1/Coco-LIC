#pragma once

#include "loop_config.h"
#include "loop_detector.h"
#include "Scancontext.h"  // from the fetched Scan Context tree (see CMake)

namespace cocolic
{

  // LoopDetector backed by Scan Context (Kim et al.). The Scan Context ring
  // database is keyed by insertion order, which only equals the keyframe
  // (snapshot) index when every keyframe is inserted. With detection_stride > 1
  // the caller invokes AddAndQuery on a subset, so db_to_snapshot_ maps the
  // returned database index back to the snapshot index.
  class ScanContextDetector : public LoopDetector
  {
  public:
    explicit ScanContextDetector(const LoopClosureConfig &config);

    std::vector<LoopCandidate> AddAndQuery(const KeyframeSnapshot &kf) override;

    std::string Name() const override { return "scan_context"; }

  private:
    SCManager sc_;
    int min_index_gap_;
    std::vector<int> db_to_snapshot_;  // Scan Context DB index -> kf.index
  };

} // namespace cocolic
