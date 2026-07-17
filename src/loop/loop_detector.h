#pragma once

#include <memory>
#include <string>
#include <vector>

#include "loop_types.h"

namespace cocolic
{

  // Place-recognition interface. Implementations own their internal descriptor
  // database. Contract:
  //  - AddAndQuery is called on a subset of keyframes in strictly increasing
  //    index order (every keyframe when detection_stride == 1, otherwise every
  //    stride-th one). Implementations must therefore not assume their internal
  //    database index equals kf.index, and must return snapshot indices
  //    (kf.index space) in candidates.
  //  - Returned candidates refer to PAST keyframes only, and never to keyframes
  //    younger than min_index_gap (implementations receive the gap via their
  //    constructor). Empty vector = no candidate this keyframe.
  class LoopDetector
  {
  public:
    using Ptr = std::unique_ptr<LoopDetector>;

    virtual ~LoopDetector() = default;

    virtual std::vector<LoopCandidate> AddAndQuery(const KeyframeSnapshot &kf) = 0;

    virtual std::string Name() const = 0;
  };

} // namespace cocolic
