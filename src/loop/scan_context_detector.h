#pragma once

#include "loop_config.h"
#include "loop_detector.h"
#include "Scancontext.h"  // from the fetched Scan Context tree (see CMake)

namespace cocolic
{

  // LoopDetector backed by Scan Context (Kim et al.). Descriptor database and
  // keyframe indices stay aligned because AddAndQuery is called once per
  // keyframe in order (interface contract).
  class ScanContextDetector : public LoopDetector
  {
  public:
    explicit ScanContextDetector(const LoopClosureConfig &config);

    std::vector<LoopCandidate> AddAndQuery(const KeyframeSnapshot &kf) override;

    std::string Name() const override { return "scan_context"; }

  private:
    SCManager sc_;
    int min_index_gap_;
  };

} // namespace cocolic
