/*
 * Loop-closure / backbone relative-pose factor for the continuous-time
 * (split SO(3)xR3 non-uniform B-spline) trajectory.
 *
 * This is the single factor type used by the offline loop-closure back-end
 * (src/loop/loop_backend). It constrains the RELATIVE pose between two spline
 * times t_i and t_j:
 *
 *     residual = sqrt_info * Log( T_meas^-1 * ( T(t_i)^-1 * T(t_j) ) )
 *
 * where T(t) is the IMU pose read off the spline. The same factor serves:
 *   - loop edges:     T_meas = verified relative pose between two revisit times.
 *   - backbone edges: T_meas = the pre-loop online relative pose between two
 *                     consecutive keyframe times (preserves local trajectory
 *                     shape and propagates the loop correction).
 *
 * It is a Ceres *auto-diff* functor (offline finalize solve — performance is not
 * critical, and auto-diff avoids a hand-derived two-time SE(3) Jacobian). It
 * reuses the Jet-templated NURBS spline evaluation in
 * src/spline/ceres_spline_helper_jet.h. The information matrix is passed in as a
 * per-DOF sqrt vector; this is the seam where Step-2 uncertainty re-weighting
 * plugs in (see src/loop/information_provider.h).
 *
 * Assumption: the two times lie in DISJOINT spline segments (>= SplineOrder
 * apart), i.e. they share no control points. Consecutive keyframes (>~1 m / 1 s,
 * t_add = 0.1 s) and loop pairs always satisfy this; the back-end skips any pair
 * that would violate it. Parameter-block layout (16 blocks) matches
 * TrajectoryEstimator::AddControlPointsNURBS(idx_i-3, idx_j-3, vec) then the
 * same with addPosKnot=true:
 *     [ SO3_i(4) x4 ][ SO3_j(4) x4 ][ pos_i(3) x4 ][ pos_j(3) x4 ]
 */
#pragma once

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <utils/sophus_utils.hpp>

#include "spline/ceres_spline_helper_jet.h"
#include "spline/spline_common.h"  // SplineOrder

namespace cocolic
{
  namespace analytic_derivative
  {

    struct LoopRelativePoseFunctor
    {
      static constexpr int N = SplineOrder;  // = 4

      // u_* are the normalized in-segment times in [0,1); the blending matrices
      // are the non-uniform (NURBS) matrices for the segment starting at idx_*-3.
      LoopRelativePoseFunctor(double u_i, double u_j,
                              const Eigen::Matrix4d &cumul_blend_i,
                              const Eigen::Matrix4d &cumul_blend_j,
                              const Eigen::Matrix4d &blend_i,
                              const Eigen::Matrix4d &blend_j,
                              const Sophus::SE3d &T_meas_i_to_j,
                              const Eigen::Matrix<double, 6, 1> &sqrt_info)
          : u_i_(u_i), u_j_(u_j),
            cumul_blend_i_(cumul_blend_i), cumul_blend_j_(cumul_blend_j),
            blend_i_(blend_i), blend_j_(blend_j),
            T_meas_(T_meas_i_to_j), sqrt_info_(sqrt_info) {}

      template <typename T>
      bool operator()(T const *const *knots, T *residuals) const
      {
        using SO3T = Sophus::SO3<T>;
        using Vec3T = Eigen::Matrix<T, 3, 1>;
        using Helper = CeresSplineHelperJet<T, N>;

        // Layout: knots[0..3]=SO3_i, [4..7]=SO3_j, [8..11]=pos_i, [12..15]=pos_j
        SO3T R_i, R_j;
        Helper::template evaluate_lie_NURBS<Sophus::SO3>(
            knots + 0, T(u_i_), 1.0, cumul_blend_i_, &R_i);
        Helper::template evaluate_lie_NURBS<Sophus::SO3>(
            knots + 4, T(u_j_), 1.0, cumul_blend_j_, &R_j);

        Vec3T p_i, p_j;
        Helper::template evaluateNURBS<3, 0>(knots + 8, T(u_i_), 1.0, blend_i_, &p_i);
        Helper::template evaluateNURBS<3, 0>(knots + 12, T(u_j_), 1.0, blend_j_, &p_j);

        Sophus::SE3<T> T_i(R_i, p_i);
        Sophus::SE3<T> T_j(R_j, p_j);
        Sophus::SE3<T> pred = T_i.inverse() * T_j;          // predicted i->j
        Sophus::SE3<T> err = T_meas_.cast<T>().inverse() * pred;

        // Sophus SE3::log() = [translation(3); rotation(3)]; sqrt_info follows
        // the same ordering.
        Eigen::Matrix<T, 6, 1> r = err.log();
        Eigen::Map<Eigen::Matrix<T, 6, 1>> res(residuals);
        for (int k = 0; k < 6; ++k)
          res[k] = T(sqrt_info_[k]) * r[k];
        return true;
      }

      // Build the Ceres cost function (16 disjoint parameter blocks:
      // 8 SO(3) quaternion knots of size 4, then 8 R3 position knots of size 3).
      // DynamicAutoDiffCostFunction so operator() receives `const T* const*`.
      static ceres::CostFunction *Create(
          double u_i, double u_j,
          const Eigen::Matrix4d &cumul_blend_i, const Eigen::Matrix4d &cumul_blend_j,
          const Eigen::Matrix4d &blend_i, const Eigen::Matrix4d &blend_j,
          const Sophus::SE3d &T_meas_i_to_j,
          const Eigen::Matrix<double, 6, 1> &sqrt_info)
      {
        auto *cost = new ceres::DynamicAutoDiffCostFunction<LoopRelativePoseFunctor, 4>(
            new LoopRelativePoseFunctor(u_i, u_j, cumul_blend_i, cumul_blend_j,
                                        blend_i, blend_j, T_meas_i_to_j, sqrt_info));
        for (int i = 0; i < 8; ++i) cost->AddParameterBlock(4);  // SO(3) knots
        for (int i = 0; i < 8; ++i) cost->AddParameterBlock(3);  // R3 knots
        cost->SetNumResiduals(6);
        return cost;
      }

    private:
      double u_i_, u_j_;
      Eigen::Matrix4d cumul_blend_i_, cumul_blend_j_;
      Eigen::Matrix4d blend_i_, blend_j_;
      Sophus::SE3d T_meas_;
      Eigen::Matrix<double, 6, 1> sqrt_info_;
    };

    // Adjacent-knot shape priors, RIGID-MOTION INVARIANT.
    //
    // The loop back-end constrains the spline only at isolated sample times, so
    // the knots of each segment share a null space (many knot values give the
    // same blended pose at the sample time); the optimizer wanders it and the
    // dense trajectory blows up. Two earlier prior designs failed:
    //   * absolute per-knot priors anchor knots to their DRIFTED positions and
    //     fight the loop correction (loops left unclosed);
    //   * WORLD-frame difference priors (p_b - p_a) are translation-invariant
    //     but NOT rotation-invariant -- with largely-rotational drift (35 deg on
    //     lisbon 0714) the closure must rotate the trajectory tail, which
    //     changes every world-frame difference and the priors fight it again.
    // The fix: anchor the LOCAL-frame difference R_a^-1 (p_b - p_a) and the
    // relative rotation R_a^-1 R_b. Both are invariant under any rigid transform
    // of the trajectory (p -> R0 p + t0, R -> R0 R), so the smooth loop
    // correction is free, while intra-segment wiggle (which changes local shape)
    // is punished.
    struct LocalPositionDiffPriorFunctor
    {
      LocalPositionDiffPriorFunctor(const Eigen::Vector3d &d_local0, double sqrt_w)
          : d_local0_(d_local0), sqrt_w_(sqrt_w) {}

      // qa: SO3 knot a (quaternion, 4); pa/pb: position knots a/b (3 each).
      template <class T>
      bool operator()(const T *const qa, const T *const pa, const T *const pb,
                      T *residual) const
      {
        Eigen::Map<Sophus::SO3<T> const> R_a(qa);
        Eigen::Matrix<T, 3, 1> d;
        d << pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2];
        Eigen::Matrix<T, 3, 1> r = R_a.inverse() * d - d_local0_.cast<T>();
        residual[0] = sqrt_w_ * r[0];
        residual[1] = sqrt_w_ * r[1];
        residual[2] = sqrt_w_ * r[2];
        return true;
      }

      static ceres::CostFunction *Create(const Eigen::Vector3d &d_local0,
                                         double sqrt_w)
      {
        return new ceres::AutoDiffCostFunction<LocalPositionDiffPriorFunctor, 3,
                                               4, 3, 3>(
            new LocalPositionDiffPriorFunctor(d_local0, sqrt_w));
      }

      Eigen::Vector3d d_local0_;
      double sqrt_w_;
    };

    // Relative-rotation prior between adjacent SO3 knots (see above):
    // residual = sqrt_w * Log(D0^-1 * R_a^-1 R_b), D0 the online relative rotation.
    struct So3DiffPriorFunctor
    {
      So3DiffPriorFunctor(const Sophus::SO3d &D0, double sqrt_w)
          : D0_inv_(D0.inverse()), sqrt_w_(sqrt_w) {}

      template <class T>
      bool operator()(const T *const qa, const T *const qb, T *residual) const
      {
        Eigen::Map<Sophus::SO3<T> const> R_a(qa);
        Eigen::Map<Sophus::SO3<T> const> R_b(qb);
        Eigen::Matrix<T, 3, 1> r =
            (D0_inv_.cast<T>() * (R_a.inverse() * R_b)).log();
        residual[0] = sqrt_w_ * r[0];
        residual[1] = sqrt_w_ * r[1];
        residual[2] = sqrt_w_ * r[2];
        return true;
      }

      static ceres::CostFunction *Create(const Sophus::SO3d &D0, double sqrt_w)
      {
        return new ceres::AutoDiffCostFunction<So3DiffPriorFunctor, 3, 4, 4>(
            new So3DiffPriorFunctor(D0, sqrt_w));
      }

      Sophus::SO3d D0_inv_;
      double sqrt_w_;
    };

  }  // namespace analytic_derivative
}  // namespace cocolic
