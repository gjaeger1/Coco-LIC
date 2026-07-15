#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <cassert>
#include "../loop_verifier.h"

using namespace cocolic;

// Helper to generate a room with three orthogonal planes
PosCloud::Ptr generateRoomCloud(int points_per_plane, double size)
{
  PosCloud::Ptr cloud(new PosCloud());
  cloud->points.reserve(points_per_plane * 3);

  // XY plane (z = 0)
  for (int i = 0; i < points_per_plane; ++i)
  {
    PosPoint p;
    p.x = (double)rand() / RAND_MAX * size;
    p.y = (double)rand() / RAND_MAX * size;
    p.z = 0.0;
    p.intensity = 0.f;
    p.timestamp = 0;
    cloud->push_back(p);
  }

  // YZ plane (x = 0)
  for (int i = 0; i < points_per_plane; ++i)
  {
    PosPoint p;
    p.x = 0.0;
    p.y = (double)rand() / RAND_MAX * size;
    p.z = (double)rand() / RAND_MAX * size;
    p.intensity = 0.f;
    p.timestamp = 0;
    cloud->push_back(p);
  }

  // XZ plane (y = 0)
  for (int i = 0; i < points_per_plane; ++i)
  {
    PosPoint p;
    p.x = (double)rand() / RAND_MAX * size;
    p.y = 0.0;
    p.z = (double)rand() / RAND_MAX * size;
    p.intensity = 0.f;
    p.timestamp = 0;
    cloud->push_back(p);
  }

  return cloud;
}

// Apply transformation to cloud
PosCloud::Ptr transformCloud(const PosCloud::Ptr &in, const SE3d &T)
{
  PosCloud::Ptr out(new PosCloud());
  out->points.reserve(in->size());
  for (const auto &p : in->points)
  {
    Eigen::Vector3d pt_in(p.x, p.y, p.z);
    Eigen::Vector3d pt_out = T * pt_in;
    PosPoint q;
    q.x = pt_out.x();
    q.y = pt_out.y();
    q.z = pt_out.z();
    q.intensity = p.intensity;
    q.timestamp = p.timestamp;
    out->push_back(q);
  }
  return out;
}

int main()
{
  std::cout << "Starting LoopVerifier smoke test..." << std::endl;

  // Configure verifier parameters
  LoopClosureConfig config;
  config.icp_fitness_max = 0.5;
  config.overlap_ratio_min = 0.5;

  // Initialize trajectory and extrinsics
  auto trajectory = std::make_shared<Trajectory>(0.1, 0.0);
  ExtrinsicParam ep;
  ep.se3 = SE3d();
  trajectory->SetSensorExtrinsics(LiDARSensor, ep);
  // Extend knots up to 10 seconds to support evaluation at time_ns = 5e9
  trajectory->extendKnotsTo(10LL * 1000000000LL, SE3d());

  // Generate source cloud (query scan)
  PosCloud::Ptr query_scan = generateRoomCloud(200, 5.0);

  // Shift query scan by a known offset to make a fake submap (target)
  // Shift: translation [0.2, 0.1, -0.1], rotation around Z by 3 degrees
  Eigen::Vector3d translation(0.2, 0.1, -0.1);
  Eigen::Matrix3d rotation;
  rotation = Eigen::AngleAxisd(3.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ());
  SE3d T_true(Eigen::Quaterniond(rotation).normalized(), translation);

  PosCloud::Ptr submap = transformCloud(query_scan, T_true);

  // Set up snapshots
  std::vector<KeyframeSnapshot> snapshots(2);
  // Match keyframe (index 0)
  snapshots[0].index = 0;
  snapshots[0].time_ns = 0LL;
  snapshots[0].T_LtoG_odom = SE3d();
  snapshots[0].scan_ds = query_scan; // not used directly by matcher for index 0, but set

  // Query keyframe (index 1)
  snapshots[1].index = 1;
  snapshots[1].time_ns = 5ULL * 1000000000ULL; // 5s
  snapshots[1].T_LtoG_odom = SE3d();
  snapshots[1].scan_ds = query_scan;

  LoopCandidate candidate;
  candidate.query_index = 1;
  candidate.match_index = 0;
  candidate.descriptor_score = -1.0;
  candidate.yaw_init = 0.0;

  LoopConstraint constraint;
  VerificationReport report;

  bool success = VerifyAgainstSubmap(candidate, snapshots, submap, config, trajectory, constraint, report);

  if (!success)
  {
    std::cerr << "Verification failed! Rejected by: " << report.rejected_by << std::endl;
    return 1;
  }

  std::cout << "Verification succeeded! ICP fitness: " << report.icp_fitness
            << ", Overlap ratio: " << report.overlap_ratio << std::endl;

  // We expect constraint.T_match_query to recover T_true
  // Since query pose is identity, and target pose is identity,
  // constraint.T_match_query = T_icp.
  // GICP aligns query to target (submap = T_true * query), so T_icp should be close to T_true.
  SE3d T_err = T_true.inverse() * constraint.T_match_query;
  double trans_err = T_err.translation().norm();
  double rot_err = std::abs(T_err.so3().log().norm()) * 180.0 / M_PI;

  std::cout << "Translation error: " << trans_err << " m (threshold: 0.05 m)" << std::endl;
  std::cout << "Rotation error: " << rot_err << " deg (threshold: 1.0 deg)" << std::endl;

  assert(trans_err < 0.05 && "Translation error too large!");
  assert(rot_err < 1.0 && "Rotation error too large!");

  std::cout << "LoopVerifier smoke test passed successfully!" << std::endl;
  return 0;
}
