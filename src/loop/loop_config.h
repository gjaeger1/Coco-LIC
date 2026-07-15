#pragma once

#include <string>
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
    std::string detector = "scan_context";  // scan_context | std
    int min_index_gap = 50;           // reject matches younger than this many keyframes
    int detection_stride = 1;         // run the detector every k-th keyframe

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
      c.min_index_gap = i("min_index_gap", c.min_index_gap);
      c.detection_stride = i("detection_stride", c.detection_stride);
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
      }
      c.log_dir = s("log_dir", c.log_dir);
      c.inject_false_loops = i("inject_false_loops", c.inject_false_loops);
      return c;
    }
  };

} // namespace cocolic
