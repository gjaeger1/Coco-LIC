/*
 * Lossless 3DGS dataset export for Coco-LIC.
 *
 * Writes every frame synchronously from the odometry processing thread, so
 * unlike the ROS topic path (/image_for_gs etc.) nothing can be dropped:
 * Coco-LIC reads the bag at its own pace and simply pauses while writing.
 *
 * Output layout (identical to tools/gs_exporter.py):
 *   images/frame_%05d.png   undistorted BGR image
 *   depth/frame_%05d.npy    float32 sparse metric depth map, 0 = no data
 *   poses_tum.txt           T_wc per frame (OpenCV conv.): t tx ty tz qx qy qz qw
 *   transforms.json         nerfstudio-style OpenGL c2w + pinhole intrinsics
 *   points3D.ply            accumulated colored LiDAR points, world frame
 *
 * Enabled via `if_3dgs_export: true` and `gs_export_path: <dir>` in the
 * ct_odometry_*.yaml config.
 */

#pragma once

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <utility>
#include <cstring>

#include <boost/filesystem.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace cocolic
{

  class GsExporter
  {
  public:
    struct FramePose
    {
      int64_t t_ns;
      Eigen::Quaterniond q;
      Eigen::Vector3d t;
    };

    void Init(const std::string &out_dir, const Eigen::Matrix3d &K)
    {
      out_dir_ = out_dir;
      K_ = K;
      namespace fs = boost::filesystem;
      fs::create_directories(fs::path(out_dir_) / "images");
      fs::create_directories(fs::path(out_dir_) / "depth");

      points_bin_path_ = out_dir_ + "/points3D.bin";
      points_file_.open(points_bin_path_, std::ios::binary | std::ios::trunc);

      enabled_ = true;
      std::cout << "\n🍺 GsExporter writing to " << out_dir_ << "\n";
    }

    bool Enabled() const { return enabled_; }

    /// PointVec: Vector3d xyz in world frame; ColorVec: Vector3i (r, g, b).
    template <typename PointVec, typename ColorVec>
    void AddFrame(const cv::Mat &img_bgr, const cv::Mat &depth32f,
                  const Eigen::Quaterniond &q_wc, const Eigen::Vector3d &t_wc,
                  int64_t t_ns, const PointVec &points, const ColorVec &colors_rgb)
    {
      if (!enabled_ || finalized_) return;

      char name[32];
      std::snprintf(name, sizeof(name), "frame_%05d", frame_idx_);
      if (frame_idx_ == 0)
      {
        img_w_ = img_bgr.cols;
        img_h_ = img_bgr.rows;
      }

      cv::imwrite(out_dir_ + "/images/" + name + ".png", img_bgr,
                  {cv::IMWRITE_PNG_COMPRESSION, 1});
      WriteNpy32f(out_dir_ + "/depth/" + name + ".npy", depth32f);

      // Store per frame: t_ns and the drain-time pose (q_wc, t_wc)
      frames_.push_back({t_ns, q_wc, t_wc});

      // Transform points to the camera frame before writing to binary:
      // p_cam = q_wc.conjugate() * (p_world - t_wc)
      Eigen::Quaterniond q_wc_conj = q_wc.conjugate();
      uint32_t f_idx = static_cast<uint32_t>(frame_idx_);

      for (size_t i = 0; i < points.size(); ++i)
      {
        Eigen::Vector3d p_world(points[i].x(), points[i].y(), points[i].z());
        Eigen::Vector3d p_cam = q_wc_conj * (p_world - t_wc);

        float xyz_cam[3] = {
          static_cast<float>(p_cam.x()),
          static_cast<float>(p_cam.y()),
          static_cast<float>(p_cam.z())
        };
        uint8_t rgb[3] = {
          static_cast<uint8_t>(colors_rgb[i].x()),
          static_cast<uint8_t>(colors_rgb[i].y()),
          static_cast<uint8_t>(colors_rgb[i].z())
        };

        points_file_.write(reinterpret_cast<const char *>(&f_idx), 4);
        points_file_.write(reinterpret_cast<const char *>(xyz_cam), 12);
        points_file_.write(reinterpret_cast<const char *>(rgb), 3);
      }
      num_points_ += points.size();

      frame_idx_++;
    }

    // corrected_T_wc: per exported frame (same order as AddFrame calls), the
    // final camera->world pose to bake into all outputs. Empty vector: use the
    // poses captured at AddFrame time (odometry-only behavior, and the
    // fallback when loop closure is disabled).
    void Finalize(const std::vector<std::pair<Eigen::Quaterniond, Eigen::Vector3d>>
                      &corrected_T_wc = {})
    {
      if (!enabled_ || finalized_) return;
      finalized_ = true;
      points_file_.close();

      // Choose pose source
      std::vector<std::pair<Eigen::Quaterniond, Eigen::Vector3d>> chosen_poses;
      if (!corrected_T_wc.empty())
      {
        if (corrected_T_wc.size() != frames_.size())
        {
          std::cerr << "GsExporter error: corrected_T_wc size (" << corrected_T_wc.size()
                    << ") does not match frames size (" << frames_.size()
                    << "). Falling back to stored poses." << std::endl;
          chosen_poses.reserve(frames_.size());
          for (const auto &f : frames_)
            chosen_poses.emplace_back(f.q, f.t);
        }
        else
        {
          chosen_poses = corrected_T_wc;
        }
      }
      else
      {
        chosen_poses.reserve(frames_.size());
        for (const auto &f : frames_)
          chosen_poses.emplace_back(f.q, f.t);
      }

      // Write poses_tum.txt
      std::ofstream pose_file(out_dir_ + "/poses_tum.txt", std::ios::trunc);
      if (pose_file.is_open())
      {
        pose_file << "# timestamp tx ty tz qx qy qz qw  (T_wc, OpenCV camera convention)\n";
        for (size_t i = 0; i < frames_.size(); ++i)
        {
          double t_s = frames_[i].t_ns * 1e-9;
          const auto &t_wc = chosen_poses[i].second;
          const auto &q_wc = chosen_poses[i].first;
          char line[256];
          std::snprintf(line, sizeof(line),
                        "%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f",
                        t_s, t_wc.x(), t_wc.y(), t_wc.z(),
                        q_wc.x(), q_wc.y(), q_wc.z(), q_wc.w());
          pose_file << line << "\n";
        }
        pose_file.close();
      }

      // Write transforms.json
      std::ofstream tf(out_dir_ + "/transforms.json", std::ios::trunc);
      tf.precision(17);
      tf << "{\n"
         << "  \"camera_model\": \"OPENCV\",\n"
         << "  \"fl_x\": " << K_(0, 0) << ",\n"
         << "  \"fl_y\": " << K_(1, 1) << ",\n"
         << "  \"cx\": " << K_(0, 2) << ",\n"
         << "  \"cy\": " << K_(1, 2) << ",\n"
         << "  \"w\": " << img_w_ << ",\n"
         << "  \"h\": " << img_h_ << ",\n"
         << "  \"k1\": 0.0, \"k2\": 0.0, \"p1\": 0.0, \"p2\": 0.0,\n"
         << "  \"ply_file_path\": \"points3D.ply\",\n"
         << "  \"frames\": [\n";

      for (size_t i = 0; i < frames_.size(); ++i)
      {
        char name[32];
        std::snprintf(name, sizeof(name), "frame_%05d", (int)i);
        double t_s = frames_[i].t_ns * 1e-9;
        const auto &q_wc = chosen_poses[i].first;
        const auto &t_wc = chosen_poses[i].second;

        // OpenCV c2w -> OpenGL c2w (negate col 1, 2)
        Eigen::Matrix4d c2w = Eigen::Matrix4d::Identity();
        c2w.block<3, 3>(0, 0) = q_wc.toRotationMatrix();
        c2w.block<3, 1>(0, 3) = t_wc;
        c2w.block<3, 1>(0, 1) *= -1.0;
        c2w.block<3, 1>(0, 2) *= -1.0;

        tf << "    {\n"
           << "      \"file_path\": \"images/" << name << ".png\",\n"
           << "      \"depth_file_path\": \"depth/" << name << ".npy\",\n"
           << "      \"transform_matrix\": [";
        for (int r = 0; r < 4; ++r)
        {
          tf << (r ? ", [" : "[");
          for (int c = 0; c < 4; ++c)
            tf << (c ? ", " : "") << c2w(r, c);
          tf << "]";
        }
        tf << "],\n"
           << "      \"timestamp\": " << t_s << "\n"
           << "    }" << (i + 1 < frames_.size() ? ",\n" : "\n");
      }
      tf << "  ]\n}\n";
      tf.close();

      // Assemble points3D.ply
      std::ofstream ply(out_dir_ + "/points3D.ply", std::ios::binary | std::ios::trunc);
      ply << "ply\nformat binary_little_endian 1.0\n"
          << "element vertex " << num_points_ << "\n"
          << "property float x\nproperty float y\nproperty float z\n"
          << "property uchar red\nproperty uchar green\nproperty uchar blue\n"
          << "end_header\n";

      if (num_points_ > 0)
      {
        std::ifstream bin(points_bin_path_, std::ios::binary);
        if (bin.is_open())
        {
          // Read in chunks of 50000 points
          const size_t record_size = 19;
          const size_t chunk_points = 50000;
          std::vector<char> buffer(chunk_points * record_size);

          while (bin)
          {
            bin.read(buffer.data(), buffer.size());
            std::streamsize bytes_read = bin.gcount();
            size_t num_records = bytes_read / record_size;
            if (num_records == 0) break;

            std::vector<char> write_buffer(num_records * 15);
            for (size_t r = 0; r < num_records; ++r)
            {
              const char *ptr = buffer.data() + r * record_size;
              uint32_t f_idx = *reinterpret_cast<const uint32_t *>(ptr);
              const float *xyz_cam = reinterpret_cast<const float *>(ptr + 4);
              const uint8_t *rgb = reinterpret_cast<const uint8_t *>(ptr + 16);

              Eigen::Vector3d p_cam(xyz_cam[0], xyz_cam[1], xyz_cam[2]);
              
              // Transform by chosen pose of the corresponding frame index
              Eigen::Quaterniond q_wc = Eigen::Quaterniond::Identity();
              Eigen::Vector3d t_wc = Eigen::Vector3d::Zero();
              if (f_idx < chosen_poses.size())
              {
                q_wc = chosen_poses[f_idx].first;
                t_wc = chosen_poses[f_idx].second;
              }
              Eigen::Vector3d p_world = q_wc * p_cam + t_wc;

              float xyz_world[3] = {
                static_cast<float>(p_world.x()),
                static_cast<float>(p_world.y()),
                static_cast<float>(p_world.z())
              };

              char *wptr = write_buffer.data() + r * 15;
              std::memcpy(wptr, xyz_world, 12);
              std::memcpy(wptr + 12, rgb, 3);
            }
            ply.write(write_buffer.data(), num_records * 15);
          }
          bin.close();
        }
      }
      ply.close();
      boost::filesystem::remove(points_bin_path_);

      std::cout << "\n🍺 GsExporter finalized: " << frame_idx_ << " frames, "
                << num_points_ << " points -> " << out_dir_ << "\n";
    }

    const std::vector<int64_t> FrameTimesNs() const
    {
      std::vector<int64_t> times;
      times.reserve(frames_.size());
      for (const auto &f : frames_)
        times.push_back(f.t_ns);
      return times;
    }

  private:
    /// Minimal .npy (format v1.0) writer for a CV_32FC1 matrix.
    static void WriteNpy32f(const std::string &path, const cv::Mat &m_in)
    {
      cv::Mat m = m_in.isContinuous() ? m_in : m_in.clone();
      std::string dict = "{'descr': '<f4', 'fortran_order': False, 'shape': (" +
                         std::to_string(m.rows) + ", " + std::to_string(m.cols) + "), }";
      size_t unpadded = 10 + dict.size() + 1;  // magic(6)+ver(2)+len(2) + dict + '\n'
      dict += std::string((64 - unpadded % 64) % 64, ' ');
      dict += '\n';
      uint16_t hlen = static_cast<uint16_t>(dict.size());
      std::ofstream f(path, std::ios::binary | std::ios::trunc);
      f.write("\x93NUMPY\x01\x00", 8);
      f.write(reinterpret_cast<const char *>(&hlen), 2);
      f.write(dict.data(), dict.size());
      f.write(reinterpret_cast<const char *>(m.data),
              (size_t)m.rows * m.cols * sizeof(float));
    }

    bool enabled_ = false;
    bool finalized_ = false;
    std::string out_dir_;
    std::string points_bin_path_;
    Eigen::Matrix3d K_ = Eigen::Matrix3d::Identity();
    int img_w_ = 0, img_h_ = 0;
    int frame_idx_ = 0;
    size_t num_points_ = 0;
    std::ofstream points_file_;
    std::vector<FramePose> frames_;
  };

} // namespace cocolic
