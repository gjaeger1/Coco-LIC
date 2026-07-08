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
    void Init(const std::string &out_dir, const Eigen::Matrix3d &K)
    {
      out_dir_ = out_dir;
      K_ = K;
      namespace fs = boost::filesystem;
      fs::create_directories(fs::path(out_dir_) / "images");
      fs::create_directories(fs::path(out_dir_) / "depth");

      pose_file_.open(out_dir_ + "/poses_tum.txt", std::ios::trunc);
      pose_file_ << "# timestamp tx ty tz qx qy qz qw  (T_wc, OpenCV camera convention)\n";

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

      double t_s = t_ns * 1e-9;
      char line[256];
      std::snprintf(line, sizeof(line),
                    "%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f",
                    t_s, t_wc.x(), t_wc.y(), t_wc.z(),
                    q_wc.x(), q_wc.y(), q_wc.z(), q_wc.w());
      pose_file_ << line << std::endl;  // std::endl: flush so file is always usable

      // frame entry for transforms.json: OpenCV c2w -> OpenGL c2w (negate col 1, 2)
      Eigen::Matrix4d c2w = Eigen::Matrix4d::Identity();
      c2w.block<3, 3>(0, 0) = q_wc.toRotationMatrix();
      c2w.block<3, 1>(0, 3) = t_wc;
      c2w.block<3, 1>(0, 1) *= -1.0;
      c2w.block<3, 1>(0, 2) *= -1.0;
      std::ostringstream fjson;
      fjson.precision(17);
      fjson << "    {\n"
            << "      \"file_path\": \"images/" << name << ".png\",\n"
            << "      \"depth_file_path\": \"depth/" << name << ".npy\",\n"
            << "      \"transform_matrix\": [";
      for (int r = 0; r < 4; ++r)
      {
        fjson << (r ? ", [" : "[");
        for (int c = 0; c < 4; ++c)
          fjson << (c ? ", " : "") << c2w(r, c);
        fjson << "]";
      }
      fjson << "],\n"
            << "      \"timestamp\": " << t_s << "\n    }";
      frame_entries_.push_back(fjson.str());

      for (size_t i = 0; i < points.size(); ++i)
      {
        float xyz[3] = {(float)points[i].x(), (float)points[i].y(), (float)points[i].z()};
        uint8_t rgb[3] = {(uint8_t)colors_rgb[i].x(), (uint8_t)colors_rgb[i].y(),
                          (uint8_t)colors_rgb[i].z()};
        points_file_.write(reinterpret_cast<const char *>(xyz), 12);
        points_file_.write(reinterpret_cast<const char *>(rgb), 3);
      }
      num_points_ += points.size();

      frame_idx_++;
    }

    void Finalize()
    {
      if (!enabled_ || finalized_) return;
      finalized_ = true;
      pose_file_.close();
      points_file_.close();

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
      for (size_t i = 0; i < frame_entries_.size(); ++i)
        tf << frame_entries_[i] << (i + 1 < frame_entries_.size() ? ",\n" : "\n");
      tf << "  ]\n}\n";
      tf.close();

      std::ofstream ply(out_dir_ + "/points3D.ply", std::ios::binary | std::ios::trunc);
      ply << "ply\nformat binary_little_endian 1.0\n"
          << "element vertex " << num_points_ << "\n"
          << "property float x\nproperty float y\nproperty float z\n"
          << "property uchar red\nproperty uchar green\nproperty uchar blue\n"
          << "end_header\n";
      if (num_points_ > 0)
      {
        std::ifstream bin(points_bin_path_, std::ios::binary);
        ply << bin.rdbuf();
      }
      ply.close();
      boost::filesystem::remove(points_bin_path_);

      std::cout << "\n🍺 GsExporter finalized: " << frame_idx_ << " frames, "
                << num_points_ << " points -> " << out_dir_ << "\n";
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
    std::ofstream pose_file_;
    std::ofstream points_file_;
    std::vector<std::string> frame_entries_;
  };

} // namespace cocolic
