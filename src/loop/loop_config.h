#pragma once

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace cocolic
{

  // Parsed once from the `loop_closure:` block of ct_odometry_*.yaml.
  // Every field has a default equal to "feature off / conservative", so an
  // absent block or absent key changes nothing.
  struct LoopClosureConfig
  {
    bool enable = false;
    bool shadow_mode = true;          // true: never touch the live odometry
    std::string detector = "scan_context";  // legacy single detector (used if `detectors` empty)
    std::vector<std::string> detectors;     // multi-detector: e.g. [visual_bow, spatial]; empty -> {detector}
    int min_index_gap = 50;           // reject matches younger than this many keyframes
    int detection_stride = 1;         // run the detector every k-th keyframe

    // visual place recognition (VisualBoWDetector, DBoW3 + ORB)
    std::string bow_vocab_path = "/home/ubuntu/Software/ORBvoc.txt";
    int bow_num_features = 1000;      // ORB features per keyframe image
    double bow_score_min = 0.015;     // min DBoW3 similarity score to propose a candidate
    int bow_max_candidates = 3;       // top-N DB matches considered per query

    // spatial proposer (odometry-pose proximity; sensor-agnostic)
    double spatial_search_radius_m = 10.0;  // propose past keyframes within this radius

    // verifier
    double icp_fitness_max = 0.3;     // PCL fitness (mean sq. dist), accept below
    double overlap_ratio_min = 0.6;   // source points with a close neighbor in target
    int submap_half_size = 12;        // keyframes on each side for the ICP target
    bool pcm_enable = false;          // reserved (pairwise consistency), not read yet

    // pose graph
    std::string robust_kernel = "cauchy";  // none | huber | cauchy | dcs | gnc
    double odom_rotation_sigma_deg = 0.5;
    double odom_translation_sigma_m = 0.05;
    double rp_information_scale = 100.0;   // roll/pitch inflation (gravity consistency)

    // continuous-time loop back-end (Ceres global solve on spline control points)
    double backbone_dt_s = 0.4;        // relative-pose backbone edge spacing [s]
    double loop_robust_cauchy = 1.0;   // Cauchy loss scale for loop edges
    int loop_max_iterations = 30;      // LM iterations for the global solve
    double knot_prior_weight = 1.0;    // adjacent-knot position-difference prior (kills knot null space)

    std::string log_dir;              // empty -> "<cache_path>/loop_log"
    int inject_false_loops = 0;       // research tool, see eval tooling

    static LoopClosureConfig FromYaml(const YAML::Node &root)
    {
      LoopClosureConfig c;
      if (!root["loop_closure"]) return c;
      const YAML::Node n = root["loop_closure"];
      auto b = [&](const char *k, bool d) { return n[k] ? n[k].as<bool>() : d; };
      auto i = [&](const char *k, int d) { return n[k] ? n[k].as<int>() : d; };
      auto f = [&](const char *k, double d) { return n[k] ? n[k].as<double>() : d; };
      auto s = [&](const char *k, std::string d) { return n[k] ? n[k].as<std::string>() : d; };
      c.enable = b("enable", c.enable);
      c.shadow_mode = b("shadow_mode", c.shadow_mode);
      c.detector = s("detector", c.detector);
      if (n["detectors"] && n["detectors"].IsSequence())
        for (const auto &d : n["detectors"]) c.detectors.push_back(d.as<std::string>());
      c.min_index_gap = i("min_index_gap", c.min_index_gap);
      c.detection_stride = i("detection_stride", c.detection_stride);
      if (n["visual_bow"])
      {
        const YAML::Node vb = n["visual_bow"];
        c.bow_vocab_path = vb["vocab_path"] ? vb["vocab_path"].as<std::string>() : c.bow_vocab_path;
        c.bow_num_features = vb["num_features"] ? vb["num_features"].as<int>() : c.bow_num_features;
        c.bow_score_min = vb["score_min"] ? vb["score_min"].as<double>() : c.bow_score_min;
        c.bow_max_candidates = vb["max_candidates"] ? vb["max_candidates"].as<int>() : c.bow_max_candidates;
      }
      if (n["spatial"])
      {
        const YAML::Node sp = n["spatial"];
        c.spatial_search_radius_m = sp["search_radius_m"] ? sp["search_radius_m"].as<double>() : c.spatial_search_radius_m;
      }
      if (n["verifier"])
      {
        const YAML::Node v = n["verifier"];
        c.icp_fitness_max = v["icp_fitness_max"] ? v["icp_fitness_max"].as<double>() : c.icp_fitness_max;
        c.overlap_ratio_min = v["overlap_ratio_min"] ? v["overlap_ratio_min"].as<double>() : c.overlap_ratio_min;
        c.submap_half_size = v["submap_half_size"] ? v["submap_half_size"].as<int>() : c.submap_half_size;
        c.pcm_enable = v["pcm_enable"] ? v["pcm_enable"].as<bool>() : c.pcm_enable;
      }
      if (n["pgo"])
      {
        const YAML::Node p = n["pgo"];
        c.robust_kernel = p["robust_kernel"] ? p["robust_kernel"].as<std::string>() : c.robust_kernel;
        c.odom_rotation_sigma_deg = p["odom_rotation_sigma_deg"] ? p["odom_rotation_sigma_deg"].as<double>() : c.odom_rotation_sigma_deg;
        c.odom_translation_sigma_m = p["odom_translation_sigma_m"] ? p["odom_translation_sigma_m"].as<double>() : c.odom_translation_sigma_m;
        c.rp_information_scale = p["rp_information_scale"] ? p["rp_information_scale"].as<double>() : c.rp_information_scale;
        c.backbone_dt_s = p["backbone_dt_s"] ? p["backbone_dt_s"].as<double>() : c.backbone_dt_s;
        c.loop_robust_cauchy = p["loop_robust_cauchy"] ? p["loop_robust_cauchy"].as<double>() : c.loop_robust_cauchy;
        c.loop_max_iterations = p["loop_max_iterations"] ? p["loop_max_iterations"].as<int>() : c.loop_max_iterations;
        c.knot_prior_weight = p["knot_prior_weight"] ? p["knot_prior_weight"].as<double>() : c.knot_prior_weight;
      }
      c.log_dir = s("log_dir", c.log_dir);
      c.inject_false_loops = i("inject_false_loops", c.inject_false_loops);
      return c;
    }
  };

} // namespace cocolic
