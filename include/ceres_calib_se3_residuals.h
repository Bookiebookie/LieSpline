#pragma once

#include <basalt/calibration/aprilgrid.h>
#include <basalt/calibration/calibration_helper.h>
#include <basalt/utils/common_types.h>

#include <ceres_spline_helper_old.h>
#include <basalt/calibration/calibration.hpp>

template <int _N, bool OLD_TIME_DERIV>
struct CalibGyroCostFunctorSE3 {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CalibGyroCostFunctorSE3(const Eigen::Vector3d& measurement, double u,
                          double inv_dt, double inv_std = 1)
      : measurement(measurement), u(u), inv_dt(inv_dt), inv_std(inv_std) {}

  template <class T>
  bool operator()(T const* const* sKnots, T* sResiduals) const {
    using Vec3 = Eigen::Matrix<T, 3, 1>;
    using Vec6 = Eigen::Matrix<T, 6, 1>;

    Eigen::Map<Vec3> residuals(sResiduals);

    Vec6 rot_vel;

    if constexpr (OLD_TIME_DERIV) {
      CeresSplineHelperOld<_N>::template evaluate_lie_vel_old<T, Sophus::SE3>(
          sKnots, u, inv_dt, nullptr, &rot_vel);
    } else {
      CeresSplineHelper<_N>::template evaluate_lie<T, Sophus::SE3>(
          sKnots, u, inv_dt, nullptr, &rot_vel, nullptr);
    }

    Eigen::Map<Vec3 const> const bias(sKnots[_N]);

    residuals =
        inv_std * (rot_vel.template tail<3>() - measurement.cast<T>() + bias);

    return true;
  }

  Eigen::Vector3d measurement;
  double u, inv_dt, inv_std;
};

template <int _N, bool OLD_TIME_DERIV>
struct CalibAccelerationCostFunctorSE3 {
  static constexpr int N = _N;  // Order of the spline.

  using VecN = Eigen::Matrix<double, _N, 1>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CalibAccelerationCostFunctorSE3(const Eigen::Vector3d& measurement, double u,
                                  double inv_dt, double inv_std)
      : measurement(measurement), u(u), inv_dt(inv_dt), inv_std(inv_std) {}

  template <class T>
  bool operator()(T const* const* sKnots, T* sResiduals) const {
    using Vector3 = Eigen::Matrix<T, 3, 1>;
    using Vector6 = Eigen::Matrix<T, 6, 1>;

    using Matrix4 = Eigen::Matrix<T, 4, 4>;

    Eigen::Map<Vector3> residuals(sResiduals);

    Sophus::SE3<T> T_w_i;
    Vector6 vel, accel;

    if constexpr (OLD_TIME_DERIV) {
      CeresSplineHelperOld<N>::template evaluate_lie_accel_old<T, Sophus::SE3>(
          sKnots, u, inv_dt, &T_w_i, &vel, &accel);
    } else {
      CeresSplineHelper<N>::template evaluate_lie<T, Sophus::SE3>(
          sKnots, u, inv_dt, &T_w_i, &vel, &accel);
    }

    Matrix4 vel_hat = Sophus::SE3<T>::hat(vel);
    Matrix4 accel_hat = Sophus::SE3<T>::hat(accel);

    Matrix4 ddpose = T_w_i.matrix() * (vel_hat * vel_hat + accel_hat);

    Vector3 accel_w = ddpose.col(3).template head<3>();

    // Gravity
    Eigen::Map<Vector3 const> const g(sKnots[N]);
    Eigen::Map<Vector3 const> const bias(sKnots[N + 1]);

    residuals = inv_std * (T_w_i.so3().inverse() * (accel_w + g) -
                           measurement.cast<T>() + bias);

    return true;
  }

  Eigen::Vector3d measurement;
  double u, inv_dt, inv_std;
};

template <int _N>
struct CalibReprojectionCostFunctorSE3 : public CeresSplineHelper<_N> {
  static constexpr int N = _N;        // Order of the spline.
  static constexpr int DEG = _N - 1;  // Degree of the spline.

  using MatN = Eigen::Matrix<double, _N, _N>;
  using VecN = Eigen::Matrix<double, _N, 1>;

  using Vec3 = Eigen::Matrix<double, 3, 1>;
  using Mat3 = Eigen::Matrix<double, 3, 3>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CalibReprojectionCostFunctorSE3(const basalt::CalibCornerData* corners,
                                  const basalt::AprilGrid* aprilgrid,
                                  const basalt::GenericCamera<double>& cam,
                                  double u, double inv_dt)
      : corners(corners),
        aprilgrid(aprilgrid),
        cam(cam),
        u(u),
        inv_dt(inv_dt) {}

  template <class T>
  bool operator()(T const* const* sKnots, T* sResiduals) const {
    using Vector2 = Eigen::Matrix<T, 2, 1>;
    using Vector4 = Eigen::Matrix<T, 4, 1>;

    using Matrix4 = Eigen::Matrix<T, 4, 4>;

    Sophus::SE3<T> T_w_i;
    CeresSplineHelper<N>::template evaluate_lie<T, Sophus::SE3>(sKnots, u,
                                                                inv_dt, &T_w_i);

    Eigen::Map<Sophus::SE3<T> const> const T_i_c(sKnots[N]);

    Sophus::SE3<T> T_w_c = T_w_i * T_i_c;
    Matrix4 T_c_w_matrix = T_w_c.inverse().matrix();

    basalt::GenericCamera<T> cam_t = cam.template cast<T>();

    std::visit(
        [&](const auto& cam_tt) {
          for (size_t i = 0; i < corners->corner_ids.size(); i++) {
            Vector4 p3d =
                T_c_w_matrix *
                aprilgrid->aprilgrid_corner_pos_3d[corners->corner_ids[i]]
                    .cast<T>();

            Vector2 proj;
            bool success = cam_tt.project(p3d, proj);

            if (success) {
              sResiduals[2 * i + 0] = proj[0] - corners->corners[i][0];
              sResiduals[2 * i + 1] = proj[1] - corners->corners[i][1];
            } else {
              sResiduals[2 * i + 0] = T(0);
              sResiduals[2 * i + 1] = T(0);
            }
          }
        },
        cam_t.variant);

    return true;
  }

  const basalt::CalibCornerData* corners;
  const basalt::AprilGrid* aprilgrid;
  const basalt::GenericCamera<double> cam;

  double u, inv_dt;
};
