// Compile and run on host:
// g++ -std=c++17 -I Coco-LIC/src -I /usr/include/eigen3 \
//     Coco-LIC/src/loop/trajectory_deformer.cpp \
//     Coco-LIC/src/loop/test/test_trajectory_deformer.cpp -o /tmp/test_deformer && /tmp/test_deformer

#include <cmath>
namespace Eigen {
  using std::atan2;
}

#include "../trajectory_deformer.h"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace cocolic;

void test_identity()
{
  std::vector<int64_t> knot_times = {1000, 2000, 3000};
  std::vector<SO3d> knot_R = {SO3d::exp(Eigen::Vector3d(0.1, 0.2, 0.3)),
                              SO3d::exp(Eigen::Vector3d(-0.1, 0.5, 0.2)),
                              SO3d::exp(Eigen::Vector3d(0.0, 0.0, 0.1))};
  std::vector<Eigen::Vector3d> knot_p = {Eigen::Vector3d(1.0, 2.0, 3.0),
                                         Eigen::Vector3d(4.0, 5.0, 6.0),
                                         Eigen::Vector3d(7.0, 8.0, 9.0)};

  std::vector<SO3d> orig_R = knot_R;
  std::vector<Eigen::Vector3d> orig_p = knot_p;

  std::vector<KeyframeCorrection> corrections = {
      {1000, SE3d()},
      {2000, SE3d()},
      {3000, SE3d()}};

  DeformKnots(knot_times, knot_R, knot_p, corrections);

  for (size_t i = 0; i < knot_times.size(); ++i)
  {
    assert((knot_p[i] - orig_p[i]).norm() < 1e-12);
    assert((knot_R[i].matrix() - orig_R[i].matrix()).norm() < 1e-12);
  }
  std::cout << "test_identity: PASS" << std::endl;
}

void test_rigid()
{
  std::vector<int64_t> knot_times = {1000, 2000, 3000};
  std::vector<SO3d> knot_R = {SO3d::exp(Eigen::Vector3d(0.1, 0.2, 0.3)),
                              SO3d::exp(Eigen::Vector3d(-0.1, 0.5, 0.2)),
                              SO3d::exp(Eigen::Vector3d(0.0, 0.0, 0.1))};
  std::vector<Eigen::Vector3d> knot_p = {Eigen::Vector3d(1.0, 2.0, 3.0),
                                         Eigen::Vector3d(4.0, 5.0, 6.0),
                                         Eigen::Vector3d(7.0, 8.0, 9.0)};

  std::vector<SO3d> orig_R = knot_R;
  std::vector<Eigen::Vector3d> orig_p = knot_p;

  SE3d rigid_tf(SO3d::exp(Eigen::Vector3d(0.05, -0.02, 0.1)), Eigen::Vector3d(1.5, -2.0, 0.5));

  std::vector<KeyframeCorrection> corrections = {
      {1000, rigid_tf},
      {2000, rigid_tf},
      {3000, rigid_tf}};

  DeformKnots(knot_times, knot_R, knot_p, corrections);

  for (size_t i = 0; i < knot_times.size(); ++i)
  {
    Eigen::Vector3d expected_p = rigid_tf.so3() * orig_p[i] + rigid_tf.translation();
    SO3d expected_R = rigid_tf.so3() * orig_R[i];
    assert((knot_p[i] - expected_p).norm() < 1e-12);
    assert((knot_R[i].matrix() - expected_R.matrix()).norm() < 1e-12);
  }
  std::cout << "test_rigid: PASS" << std::endl;
}

void test_interpolation()
{
  // Two corrections:
  // t = 0s: identity
  // t = 10s (10e9 ns): yaw = 10 degrees (0.174532925 rad), translation = (1, 0, 0)
  int64_t t0 = 0;
  int64_t t1 = 10ULL * 1000000000ULL;
  SE3d delta0;
  double yaw = 10.0 * M_PI / 180.0;
  SE3d delta1(SO3d::exp(Eigen::Vector3d(0.0, 0.0, yaw)), Eigen::Vector3d(1.0, 0.0, 0.0));

  std::vector<KeyframeCorrection> corrections = {{t0, delta0}, {t1, delta1}};

  // Knot at t = 5s (5e9 ns)
  int64_t t_knot = 5ULL * 1000000000ULL;
  std::vector<int64_t> knot_times = {t_knot};
  // Origin knot: rotation term vanishes for translation
  std::vector<SO3d> knot_R = {SO3d()};
  std::vector<Eigen::Vector3d> knot_p = {Eigen::Vector3d::Zero()};

  DeformKnots(knot_times, knot_R, knot_p, corrections);

  // Expect: yaw = 5 degrees (0.08726646 rad), translation = (0.5, 0, 0)
  double expected_yaw = 5.0 * M_PI / 180.0;
  Eigen::Vector3d expected_p(0.5, 0.0, 0.0);
  SO3d expected_R = SO3d::exp(Eigen::Vector3d(0.0, 0.0, expected_yaw));

  assert((knot_p[0] - expected_p).norm() < 1e-9);
  assert((knot_R[0].matrix() - expected_R.matrix()).norm() < 1e-9);

  std::cout << "test_interpolation: PASS" << std::endl;
}

void test_extrapolation()
{
  int64_t t_first = 1000;
  int64_t t_last = 2000;
  SE3d delta_first(SO3d::exp(Eigen::Vector3d(0.1, 0.0, 0.0)), Eigen::Vector3d(1.0, 2.0, 3.0));
  SE3d delta_last(SO3d::exp(Eigen::Vector3d(0.0, 0.2, 0.0)), Eigen::Vector3d(-1.0, -2.0, -3.0));

  std::vector<KeyframeCorrection> corrections = {{t_first, delta_first}, {t_last, delta_last}};

  std::vector<int64_t> knot_times = {500, 2500};
  std::vector<SO3d> knot_R = {SO3d(), SO3d()};
  std::vector<Eigen::Vector3d> knot_p = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};

  DeformKnots(knot_times, knot_R, knot_p, corrections);

  // Knot at 500 should get delta_first
  assert((knot_p[0] - delta_first.translation()).norm() < 1e-12);
  assert((knot_R[0].matrix() - delta_first.so3().matrix()).norm() < 1e-12);

  // Knot at 2500 should get delta_last
  assert((knot_p[1] - delta_last.translation()).norm() < 1e-12);
  assert((knot_R[1].matrix() - delta_last.so3().matrix()).norm() < 1e-12);

  std::cout << "test_extrapolation: PASS" << std::endl;
}

void test_empty()
{
  std::vector<int64_t> knot_times = {1000};
  std::vector<SO3d> knot_R = {SO3d()};
  std::vector<Eigen::Vector3d> knot_p = {Eigen::Vector3d(1.0, 2.0, 3.0)};

  std::vector<SO3d> orig_R = knot_R;
  std::vector<Eigen::Vector3d> orig_p = knot_p;

  std::vector<KeyframeCorrection> corrections = {};

  DeformKnots(knot_times, knot_R, knot_p, corrections);

  assert((knot_p[0] - orig_p[0]).norm() < 1e-12);
  assert((knot_R[0].matrix() - orig_R[0].matrix()).norm() < 1e-12);

  std::cout << "test_empty: PASS" << std::endl;
}

int main()
{
  test_identity();
  test_rigid();
  test_interpolation();
  test_extrapolation();
  test_empty();
  std::cout << "All deformer tests passed!" << std::endl;
  return 0;
}
