#include "loop_closure_manager.h"
#include "scan_context_detector.h"
#include "pose_graph.h"

#include <boost/filesystem.hpp>
#include <iostream>
#include <iomanip>
#include <stdexcept>

namespace cocolic
{

  LoopClosureManager::LoopClosureManager(const LoopClosureConfig &config,
                                         std::shared_ptr<Trajectory> trajectory,
                                         std::shared_ptr<LidarHandler> lidar_handler,
                                         const std::string &default_log_dir)
      : config_(config), trajectory_(trajectory), lidar_handler_(lidar_handler)
  {
    if (!config_.enable) return;

    log_dir_ = config_.log_dir.empty() ? default_log_dir + "/loop_log" : config_.log_dir;
    boost::filesystem::create_directories(log_dir_);

    if (config_.detector == "scan_context")
    {
      detector_.reset(new ScanContextDetector(config_));
    }
    else
    {
      throw std::runtime_error("unknown detector: " + config_.detector);
    }

    verifier_.reset(new LoopVerifier(config_, lidar_handler_, trajectory_));
    pose_graph_.reset(new PoseGraph(config_));

    log_.open(log_dir_ + "/loop_candidates.jsonl");
    if (!log_.is_open())
    {
      std::cerr << "Failed to open loop candidate log file at " << log_dir_ + "/loop_candidates.jsonl" << std::endl;
    }
  }

  LoopClosureManager::~LoopClosureManager() = default;

  void LoopClosureManager::OnKeyframe(int64_t kf_time_ns)
  {
    if (!config_.enable) return;

    // 1. snapshot
    KeyframeSnapshot kf;
    kf.index = (int)snapshots_.size();
    kf.time_ns = kf_time_ns;
    kf.T_LtoG_odom = trajectory_->GetLidarPoseNURBS(kf_time_ns);
    const LiDARFeature *lf = lidar_handler_->GetKeyframeScanDs(kf_time_ns);
    kf.scan_ds = (lf && lf->full_cloud && !lf->full_cloud->empty())
                     ? lf->full_cloud : (lf ? lf->surface_features : nullptr);
    if (!kf.scan_ds)
    {
      std::cout << "[LoopClosureManager] Warning: Keyframe scan_ds is empty at time " << kf_time_ns << std::endl;
    }
    snapshots_.push_back(kf);
    pose_graph_->AddKeyframe(kf);

    // 2. detect (respect stride), 3. verify, 4. add to graph, 5. always log
    // With stride > 1, skipped keyframes are not registered in the detector database but
    // ARE in the pose graph.
    if (kf.index % config_.detection_stride != 0) return;
    for (const LoopCandidate &c : detector_->AddAndQuery(kf))
    {
      LoopConstraint constraint;
      VerificationReport report;
      if (verifier_->Verify(c, snapshots_, constraint, report))
      {
        pose_graph_->AddLoop(constraint);
      }
      LogCandidate(c, report);
    }
  }

  void LoopClosureManager::LogCandidate(const LoopCandidate &c, const VerificationReport &r)
  {
    if (!log_.is_open()) return;

    log_ << "{\"query\": " << c.query_index
         << ", \"match\": " << c.match_index
         << ", \"t_query_ns\": " << snapshots_[c.query_index].time_ns
         << ", \"t_match_ns\": " << snapshots_[c.match_index].time_ns
         << ", \"detector\": \"" << config_.detector << "\""
         << ", \"descriptor_score\": " << c.descriptor_score
         << ", \"yaw_init\": " << c.yaw_init
         << ", \"icp_fitness\": " << r.icp_fitness
         << ", \"overlap_ratio\": " << r.overlap_ratio
         << ", \"accepted\": " << (r.accepted ? "true" : "false")
         << ", \"rejected_by\": \"" << r.rejected_by << "\"}"
         << std::endl;
  }

  std::vector<KeyframeCorrection> LoopClosureManager::Finalize()
  {
    if (finalized_ || !config_.enable) return {};
    finalized_ = true;

    if (snapshots_.empty()) return {};

    if (NumAcceptedLoops() == 0)
    {
      std::cout << "🔁 LoopClosure: finalized, but no loops found." << std::endl;
      
      std::ofstream odom_file(log_dir_ + "/poses_pgo_tum.txt");
      if (odom_file.is_open())
      {
        for (const auto &kf : snapshots_)
        {
          Eigen::Quaterniond q = kf.T_LtoG_odom.unit_quaternion();
          Eigen::Vector3d t = kf.T_LtoG_odom.translation();
          odom_file << std::fixed << std::setprecision(9)
                    << (double)kf.time_ns * 1e-9 << " "
                    << t.x() << " " << t.y() << " " << t.z() << " "
                    << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
        }
        odom_file.close();
      }
      return {};
    }

    auto corrected = pose_graph_->Optimize();

    std::ofstream pgo_file(log_dir_ + "/poses_pgo_tum.txt");
    if (pgo_file.is_open())
    {
      for (size_t i = 0; i < snapshots_.size(); ++i)
      {
        Eigen::Quaterniond q = corrected[i].unit_quaternion();
        Eigen::Vector3d t = corrected[i].translation();
        pgo_file << std::fixed << std::setprecision(9)
                 << (double)snapshots_[i].time_ns * 1e-9 << " "
                 << t.x() << " " << t.y() << " " << t.z() << " "
                 << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
      }
      pgo_file.close();
    }

    std::cout << "🔁 LoopClosure: " << snapshots_.size() << " keyframes, "
              << NumAcceptedLoops() << " accepted loops, corrected TUM -> "
              << log_dir_ << "/poses_pgo_tum.txt" << std::endl;

    std::vector<KeyframeCorrection> corrections;
    corrections.reserve(snapshots_.size());
    for (size_t i = 0; i < snapshots_.size(); ++i)
    {
      KeyframeCorrection c;
      c.time_ns = snapshots_[i].time_ns;
      c.delta = corrected[i] * snapshots_[i].T_LtoG_odom.inverse();
      corrections.push_back(c);
    }

    if (log_.is_open())
    {
      log_.close();
    }

    return corrections;
  }

  size_t LoopClosureManager::NumAcceptedLoops() const
  {
    return pose_graph_ ? pose_graph_->NumLoops() : 0;
  }

} // namespace cocolic
