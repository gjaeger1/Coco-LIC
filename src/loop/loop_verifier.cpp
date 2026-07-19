#include "loop_verifier.h"

#include <pcl/registration/gicp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/centroid.h>
#include <algorithm>

namespace cocolic
{

  bool VerifyAgainstSubmap(const LoopCandidate &candidate,
                           const std::vector<KeyframeSnapshot> &snapshots,
                           const PosCloud::Ptr &submap_in_G,
                           const LoopClosureConfig &config,
                           std::shared_ptr<Trajectory> trajectory,
                           LoopConstraint &constraint,
                           VerificationReport &report)
  {
    // Stage 0 "input": query/match index in range, scan_ds non-null and non-empty.
    int q = candidate.query_index;
    int m = candidate.match_index;
    if (q < 0 || q >= (int)snapshots.size() || m < 0 || m >= (int)snapshots.size())
    {
      report.rejected_by = "input";
      return false;
    }
    if (!snapshots[q].scan_ds || snapshots[q].scan_ds->empty())
    {
      report.rejected_by = "input";
      return false;
    }
    if (!submap_in_G || submap_in_G->empty())
    {
      report.rejected_by = "input";
      return false;
    }

    // Stage 1 "icp":
    //   source = query scan (LiDAR frame) -> world via snapshots[q].T_LtoG_odom
    //   target = submap_in_G
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_world(new pcl::PointCloud<pcl::PointXYZ>());
    source_world->reserve(snapshots[q].scan_ds->size());
    for (const auto &p : snapshots[q].scan_ds->points)
    {
      Eigen::Vector3d pt_in_L(p.x, p.y, p.z);
      Eigen::Vector3d pt_in_G = snapshots[q].T_LtoG_odom * pt_in_L;
      source_world->push_back(pcl::PointXYZ(pt_in_G.x(), pt_in_G.y(), pt_in_G.z()));
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr target_world(new pcl::PointCloud<pcl::PointXYZ>());
    target_world->reserve(submap_in_G->size());
    for (const auto &p : submap_in_G->points)
    {
      target_world->push_back(pcl::PointXYZ(p.x, p.y, p.z));
    }

    // GICP settings
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setInputSource(source_world);
    gicp.setInputTarget(target_world);
    gicp.setMaxCorrespondenceDistance(30.0);
    gicp.setMaximumIterations(100);
    gicp.setTransformationEpsilon(1e-6);

    pcl::PointCloud<pcl::PointXYZ> aligned_source;
    // Initial guess: IDENTITY. Both clouds are already expressed in the world
    // frame through their odometry poses, so under bounded drift the true
    // correction is small and identity is inside the GICP basin (max
    // correspondence 30 m). Do NOT seed with centroid alignment: on
    // self-similar geometry (corridors, building faces) it yanks the query
    // cloud onto the submap centroid and GICP converges to a SHIFTED local
    // minimum with good fitness -- confidently wrong T_meas 5-30 m off
    // (observed on lisbon 0714: backend pulled 2.6 m-consistent revisits 13 m
    // apart). If identity fails the gates and the detector supplied a yaw hint
    // (e.g. ScanContext), retry once rotated about the source centroid
    // (rotation only, no translation).
    gicp.align(aligned_source);
    bool converged = gicp.hasConverged();
    double fitness = converged ? gicp.getFitnessScore() : 1e9;
    if ((!converged || fitness > config.icp_fitness_max) &&
        std::abs(candidate.yaw_init) > 0.1)
    {
      Eigen::Vector4f src_centroid;
      pcl::compute3DCentroid(*source_world, src_centroid);
      Eigen::Matrix3f Rz(Eigen::AngleAxisf((float)candidate.yaw_init,
                                           Eigen::Vector3f::UnitZ()));
      Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
      guess.block<3, 3>(0, 0) = Rz;
      guess.block<3, 1>(0, 3) =
          src_centroid.head<3>() - Rz * src_centroid.head<3>();
      gicp.align(aligned_source, guess);
    }

    if (!gicp.hasConverged())
    {
      report.rejected_by = "icp";
      return false;
    }

    report.icp_fitness = gicp.getFitnessScore();
    if (report.icp_fitness > config.icp_fitness_max)
    {
      report.rejected_by = "icp_fitness";
      return false;
    }

    // Stage 2 "overlap":
    //   Build a pcl::KdTreeFLANN on the target cloud. overlap_ratio = fraction of
    //   ALIGNED source points whose nearest target neighbor is < 0.5 m.
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(target_world);

    int num_overlap = 0;
    int num_source = aligned_source.size();
    if (num_source > 0)
    {
      for (const auto &p : aligned_source.points)
      {
        std::vector<int> nn_indices(1);
        std::vector<float> nn_dists(1);
        if (kdtree.nearestKSearch(p, 1, nn_indices, nn_dists) > 0)
        {
          // PCL KdTree returns squared distance. 0.5m threshold corresponds to 0.25m^2.
          if (nn_dists[0] < 0.25)
          {
            num_overlap++;
          }
        }
      }
      report.overlap_ratio = (double)num_overlap / num_source;
    }
    else
    {
      report.overlap_ratio = 0.0;
    }

    if (report.overlap_ratio < config.overlap_ratio_min)
    {
      report.rejected_by = "overlap";
      return false;
    }

    // Stage 3 "constraint": all gates passed.
    Eigen::Matrix4d T_icp_double = gicp.getFinalTransformation().cast<double>();
    Eigen::Matrix3d R = T_icp_double.block<3, 3>(0, 0);
    Eigen::Vector3d t = T_icp_double.block<3, 1>(0, 3);
    Sophus::SE3d T_icp(Eigen::Quaterniond(R).normalized(), t);

    // Aliasing guard: how far did ICP move the query keyframe? A registration
    // that displaces the query further than any plausible odometry drift is a
    // shifted local minimum on self-similar geometry (good fitness, wrong pose)
    // -- exactly the failure mode that poisons the back-end with confidently
    // wrong constraints. Displacement is evaluated AT the query position, so a
    // rotation about a far-away origin is measured by its actual effect.
    const Eigen::Vector3d p_q = snapshots[q].T_LtoG_odom.translation();
    const double correction = (R * p_q + t - p_q).norm();
    if (correction > config.max_correction_m)
    {
      report.rejected_by = "max_correction";
      return false;
    }

    Sophus::SE3d T_q_corrected = T_icp * snapshots[q].T_LtoG_odom; // world frame

    // Use the match keyframe's captured odometry pose. Re-querying the spline at
    // snapshots[m].time_ns is unsafe: its leading knots may have been
    // marginalized, so the query can evaluate out of range and crash. In shadow
    // mode the captured pose is exactly what a re-query would have returned.
    Sophus::SE3d T_m = snapshots[m].T_LtoG_odom;
    
    constraint.query_index = q;
    constraint.match_index = m;
    constraint.T_match_query = T_m.inverse() * T_q_corrected;

    // Information (heuristic, fusion-ready): confidence grows with overlap and
    // shrinks with the ICP residual. First-class 6x6 diagonal in [trans; rot]
    // order (matching LoopRelativePoseFunctor / InformationProvider), so Step-2
    // can replace it with a covariance-derived / multi-sensor-fused matrix
    // without changing the factor. Isotropic per-block for now.
    double s = std::max(report.icp_fitness, 1e-3);
    double conf = std::max(report.overlap_ratio, 1e-3) / s;
    Eigen::Matrix<double, 6, 1> diag;
    diag << conf, conf, conf, conf, conf, conf;  // [tx,ty,tz, rx,ry,rz]
    constraint.information = diag.asDiagonal();
    constraint.injected = false;

    report.accepted = true;
    return true;
  }

  LoopVerifier::LoopVerifier(const LoopClosureConfig &config,
                             std::shared_ptr<LidarHandler> lidar_handler,
                             std::shared_ptr<Trajectory> trajectory)
      : config_(config), lidar_handler_(lidar_handler), trajectory_(trajectory) {}

  bool LoopVerifier::Verify(const LoopCandidate &candidate,
                            const std::vector<KeyframeSnapshot> &snapshots,
                            LoopConstraint &constraint,
                            VerificationReport &report)
  {
    PosCloud::Ptr submap_in_G(new PosCloud());
    lidar_handler_->FindNearbyKeyFrames(candidate.match_index, config_.submap_half_size, submap_in_G);

    return VerifyAgainstSubmap(candidate, snapshots, submap_in_G, config_, trajectory_, constraint, report);
  }

} // namespace cocolic
