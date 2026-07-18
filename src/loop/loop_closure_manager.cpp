#include "loop_closure_manager.h"
#include "scan_context_detector.h"
#include "spatial_detector.h"
#include "visual_bow_detector.h"

#include <boost/filesystem.hpp>
#include <iostream>
#include <iomanip>
#include <map>
#include <stdexcept>
#include <random>
#include <utility>

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

    // Build the detector list. `detectors:` (multi) takes precedence; otherwise
    // fall back to the legacy single `detector:` for backward compatibility.
    std::vector<std::string> names =
        config_.detectors.empty() ? std::vector<std::string>{config_.detector}
                                   : config_.detectors;
    for (const std::string &name : names)
    {
      if (name == "scan_context")
        detectors_.emplace_back(new ScanContextDetector(config_));
      else if (name == "spatial")
        detectors_.emplace_back(new SpatialDetector(config_));
      else if (name == "visual_bow")
        detectors_.emplace_back(new VisualBoWDetector(config_));
      else
        throw std::runtime_error("unknown detector: " + name);
    }

    verifier_.reset(new LoopVerifier(config_, lidar_handler_, trajectory_));

    log_.open(log_dir_ + "/loop_candidates.jsonl");
    if (!log_.is_open())
    {
      std::cerr << "Failed to open loop candidate log file at " << log_dir_ + "/loop_candidates.jsonl" << std::endl;
    }
  }

  LoopClosureManager::~LoopClosureManager() = default;

  void LoopClosureManager::OnKeyframe(int64_t kf_time_ns, const SE3d &T_LtoG_odom,
                                      const cv::Mat &image)
  {
    if (!config_.enable) return;

    // 1. snapshot. The pose is captured by the caller at keyframe-creation time
    // (LidarHandler), NOT re-queried here: the spline's leading knots get
    // marginalized after creation, so a re-query at kf_time_ns can evaluate out
    // of range and crash.
    KeyframeSnapshot kf;
    kf.index = (int)snapshots_.size();
    kf.time_ns = kf_time_ns;
    kf.T_LtoG_odom = T_LtoG_odom;
    const LiDARFeature *lf = lidar_handler_->GetKeyframeScanDs(kf_time_ns);
    kf.scan_ds = (lf && lf->full_cloud && !lf->full_cloud->empty())
                     ? lf->full_cloud : (lf ? lf->surface_features : nullptr);
    if (!kf.scan_ds)
    {
      std::cout << "[LoopClosureManager] Warning: Keyframe scan_ds is empty at time " << kf_time_ns << std::endl;
    }
    // Camera image for VPR (LICO mode). Clone: the caller reuses msg.image.
    if (!image.empty())
    {
      kf.image = image.clone();
      kf.has_image = true;
    }
    snapshots_.push_back(kf);

    // 2. detect (respect stride), 3. verify, 4. collect, 5. always log
    // With stride > 1, skipped keyframes are not registered in the detector
    // database but ARE available as backbone/loop endpoints in the back-end.
    if (kf.index % config_.detection_stride != 0)
    {
      snapshots_.back().image.release();  // not queried: free the image
      return;
    }

    // Run every detector on this keyframe and union their candidates. Distinct
    // detectors may propose the same (query, match) pair; de-duplicate on the
    // pair, preferring a candidate that carries a yaw init (spatial provides one,
    // visual BoW does not) so the verifier gets the best GICP seed.
    std::map<std::pair<int, int>, LoopCandidate> unique_cands;
    for (const auto &det : detectors_)
    {
      for (const LoopCandidate &c : det->AddAndQuery(kf))
      {
        auto key = std::make_pair(c.query_index, c.match_index);
        auto it = unique_cands.find(key);
        if (it == unique_cands.end())
        {
          unique_cands.emplace(key, c);
        }
        else if (it->second.yaw_init == 0.0 && c.yaw_init != 0.0)
        {
          it->second.yaw_init = c.yaw_init;
        }
      }
    }

    // The image has now been consumed by the visual detector; release it so we
    // do not retain a full cv::Mat per keyframe.
    snapshots_.back().image.release();

    for (const auto &kv : unique_cands)
    {
      const LoopCandidate &c = kv.second;
      LoopConstraint constraint;
      VerificationReport report;
      if (verifier_->Verify(c, snapshots_, constraint, report))
      {
        accepted_loops_.push_back(constraint);
      }
      LogCandidate(c, report);
    }
  }

  void LoopClosureManager::LogCandidate(const LoopCandidate &c, const VerificationReport &r, bool injected)
  {
    if (!log_.is_open()) return;

    // Log absolute (sensor-epoch) timestamps, not trajectory-relative ones, so
    // the JSONL is directly comparable to epoch-based ground truth (data_start
    // is the bag/sensor epoch offset in ns; keyframe time_ns is relative to it).
    const int64_t t0 = trajectory_->GetDataStartTime();
    log_ << "{\"query\": " << c.query_index
         << ", \"match\": " << c.match_index
         << ", \"t_query_ns\": " << (snapshots_[c.query_index].time_ns + t0)
         << ", \"t_match_ns\": " << (snapshots_[c.match_index].time_ns + t0)
         << ", \"detector\": \"" << config_.detector << "\""
         << ", \"descriptor_score\": " << c.descriptor_score
         << ", \"yaw_init\": " << c.yaw_init
         << ", \"icp_fitness\": " << r.icp_fitness
         << ", \"overlap_ratio\": " << r.overlap_ratio
         << ", \"accepted\": " << (r.accepted ? "true" : "false")
         << ", \"rejected_by\": \"" << r.rejected_by << "\""
         << ", \"injected\": " << (injected ? "true" : "false") << "}"
         << std::endl;
  }

  size_t LoopClosureManager::Finalize()
  {
    if (finalized_ || !config_.enable) return 0;
    finalized_ = true;

    if (snapshots_.empty()) return 0;

    // Deterministic adversary for robustness research: inject N wrong loop
    // constraints between far-apart keyframes with a random relative pose.
    if (config_.inject_false_loops > 0 &&
        snapshots_.size() > 2 * config_.min_index_gap)
    {
      if (!config_.shadow_mode)
      {
        std::cout << "⚠️ Warning: [LoopClosureManager] Skipped false-loop injection because shadow_mode is false (active feedback mode)." << std::endl;
      }
      else
      {
        std::mt19937 rng(42);  // fixed seed: same bag + config -> same injected loops
        std::uniform_int_distribution<int> dist_q(config_.min_index_gap, snapshots_.size() - 1);
        std::uniform_real_distribution<double> dist_angle(-M_PI, M_PI);
        std::uniform_real_distribution<double> dist_xy(-10.0, 10.0);
        std::uniform_real_distribution<double> dist_z(-2.0, 2.0);

        for (int k = 0; k < config_.inject_false_loops; ++k)
        {
          int q = dist_q(rng);
          std::uniform_int_distribution<int> dist_m(0, q - config_.min_index_gap);
          int m = dist_m(rng);

          LoopConstraint c;
          c.query_index = q;
          c.match_index = m;
          c.T_match_query = SE3d(Eigen::Quaterniond(Eigen::AngleAxisd(
                                     dist_angle(rng), Eigen::Vector3d::UnitZ())),
                                 Eigen::Vector3d(dist_xy(rng),
                                                 dist_xy(rng),
                                                 dist_z(rng)));
          c.information = Eigen::Matrix<double, 6, 6>::Identity() * 10.0;  // confidently wrong
          c.injected = true;
          accepted_loops_.push_back(c);

          LoopCandidate cand;
          cand.query_index = q;
          cand.match_index = m;
          cand.descriptor_score = -1.0;
          cand.yaw_init = 0.0;

          VerificationReport rep;
          rep.accepted = true;
          rep.icp_fitness = -1.0;
          rep.overlap_ratio = -1.0;
          rep.rejected_by = "";

          LogCandidate(cand, rep, true);
        }
      }
    }

    if (log_.is_open())
      log_.close();

    if (accepted_loops_.empty())
      std::cout << "🔁 LoopClosure: finalized, but no loops found." << std::endl;
    else
      std::cout << "🔁 LoopClosure: " << snapshots_.size() << " keyframes, "
                << accepted_loops_.size() << " accepted loops." << std::endl;
    return accepted_loops_.size();
  }

  void LoopClosureManager::WritePgoTum()
  {
    if (!config_.enable || snapshots_.empty()) return;
    // Valid spline eval range at finalize (all control points retained).
    const int64_t t_lo = trajectory_->knts.size() > (size_t)SplineOrder
                             ? trajectory_->knts[SplineOrder - 1] : 0;
    const int64_t t_hi = trajectory_->maxTimeNsNURBS();
    std::ofstream pgo(log_dir_ + "/poses_pgo_tum.txt");
    if (!pgo.is_open()) return;
    for (const auto &kf : snapshots_)
    {
      if (kf.time_ns < t_lo || kf.time_ns >= t_hi) continue;
      SE3d T = trajectory_->GetLidarPoseNURBS(kf.time_ns);  // deformed spline
      Eigen::Quaterniond q = T.unit_quaternion();
      Eigen::Vector3d t = T.translation();
      pgo << std::fixed << std::setprecision(9) << (double)kf.time_ns * 1e-9 << " "
          << t.x() << " " << t.y() << " " << t.z() << " "
          << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
    }
    pgo.close();
  }

  void LoopClosureManager::WriteOdomTum()
  {
    if (!config_.enable || snapshots_.empty()) return;
    std::ofstream odom_file(log_dir_ + "/poses_odom_tum.txt");
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
  }

} // namespace cocolic
