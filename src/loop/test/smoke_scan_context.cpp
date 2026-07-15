#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <cassert>
#include "../scan_context_detector.h"

using namespace cocolic;

// Helper to generate a synthetic point cloud on a cylinder
PosCloud::Ptr generateCylinderCloud(double radius, double height, int num_points, double noise = 0.0)
{
  PosCloud::Ptr cloud(new PosCloud());
  cloud->points.reserve(num_points);
  for (int i = 0; i < num_points; ++i)
  {
    double theta = (double)rand() / RAND_MAX * 2.0 * M_PI;
    double z = (double)rand() / RAND_MAX * height - height / 2.0;
    PosPoint p;
    p.x = radius * cos(theta) + (noise * ((double)rand() / RAND_MAX - 0.5));
    p.y = radius * sin(theta) + (noise * ((double)rand() / RAND_MAX - 0.5));
    p.z = z;
    p.intensity = 0.f;
    p.timestamp = 0;
    cloud->push_back(p);
  }
  return cloud;
}

int main()
{
  std::cout << "Starting ScanContext smoke test..." << std::endl;

  // Configure loop closure
  LoopClosureConfig config;
  config.enable = true;
  config.min_index_gap = 10; // lower gap for test
  config.detector = "scan_context";

  ScanContextDetector detector(config);

  // Generate distinct point clouds (e.g. cylinder with random noise)
  std::vector<PosCloud::Ptr> clouds;
  for (int i = 0; i < 60; ++i)
  {
    clouds.push_back(generateCylinderCloud(10.0, 5.0, 500, 0.5));
  }

  // We want to test matching. Let's make index 55 have the exact same point cloud as index 0
  clouds[55] = clouds[0];

  // Feed snapshots one by one
  for (int i = 0; i < 60; ++i)
  {
    KeyframeSnapshot kf;
    kf.index = i;
    kf.time_ns = i * 1000000000LL; // 1s spacing
    kf.T_LtoG_odom = SE3d();
    kf.scan_ds = clouds[i];

    std::vector<LoopCandidate> candidates = detector.AddAndQuery(kf);

    if (i == 55)
    {
      assert(!candidates.empty() && "Index 55 should detect a loop closure!");
      assert(candidates[0].match_index == 0 && "Index 55 should match index 0!");
      std::cout << "Successfully matched index 55 to " << candidates[0].match_index << std::endl;
    }
    else
    {
      if (!candidates.empty())
      {
        std::cout << "Unexpected match at index " << i << " with match_index " << candidates[0].match_index << std::endl;
        assert(false && "No early loop closure should be detected!");
      }
    }
  }

  std::cout << "ScanContext smoke test passed successfully!" << std::endl;
  return 0;
}
