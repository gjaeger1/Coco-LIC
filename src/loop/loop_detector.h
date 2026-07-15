#pragma once

#include <memory>
#include <string>
#include <vector>

#include "loop_types.h"

namespace cocolic
{

  // Place-recognition interface. Implementations own their internal descriptor
  // database. Contract:
  //  - AddAndQuery is called exactly once per keyframe, in strictly increasing
  //    index order, so detector state stays index-aligned with the pose graph.
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
