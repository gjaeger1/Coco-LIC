#include "scan_context_detector.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace cocolic
{

  ScanContextDetector::ScanContextDetector(const LoopClosureConfig &config)
      : min_index_gap_(config.min_index_gap) {}

  std::vector<LoopCandidate> ScanContextDetector::AddAndQuery(const KeyframeSnapshot &kf)
  {
    // Scan Context wants a single scan in the sensor frame as XYZI.
    pcl::PointCloud<pcl::PointXYZI> scan_xyzi;
    scan_xyzi.reserve(kf.scan_ds->size());
    for (const auto &p : kf.scan_ds->points)
    {
      pcl::PointXYZI q;
      q.x = p.x; q.y = p.y; q.z = p.z; q.intensity = 0.f;
      scan_xyzi.push_back(q);
    }
    sc_.makeAndSaveScancontextAndKeys(scan_xyzi);
    db_to_snapshot_.push_back(kf.index);

    std::vector<LoopCandidate> out;
    auto [db_match_index, yaw] = sc_.detectLoopClosureID();
    // detectLoopClosureID returns an index into the Scan Context ring database
    // (insertion order), which is not the snapshot index when detection_stride
    // > 1. Map it back before applying the gap check or emitting the candidate.
    if (db_match_index >= 0 && (int)db_to_snapshot_.size() > db_match_index)
    {
      int match_index = db_to_snapshot_[db_match_index];
      if (kf.index - match_index >= min_index_gap_)
      {
        LoopCandidate c;
        c.query_index = kf.index;
        c.match_index = match_index;
        c.yaw_init = yaw;
        // SCManager does not expose the descriptor distance through this API;
        // stays at -1 unless the upstream version differs.
        out.push_back(c);
      }
    }
    return out;
  }

} // namespace cocolic
