#pragma once

#include <basalt/calibration/aprilgrid.h>
#include <basalt/calibration/calibration_helper.h>
#include <basalt/utils/common_types.h>
#include <ceres_spline_helper_old.h>
#include <basalt/calibration/calibration.hpp>

template <int _N>
struct CalibAccelerationCostFunctorSplit : public CeresSplineHelper<_N> {
  static constexpr int N = _N;        // Order of the spline.
  static constexpr int DEG = _N - 1;  // Degree of the spline.

  using MatN = Eigen::Matrix<double, _N, _N>;
  using VecN = Eigen::Matrix<double, _N, 1>;

  using Vec3 = Eigen::Matrix<double, 3, 1>;
  using Mat3 = Eigen::Matrix<double, 3, 3>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CalibAccelerationCostFunctorSplit(const Eigen::Vector3d& measurement,
                                    double u, double inv_dt, double inv_std)
      : measurement(measurement), u(u), inv_dt(inv_dt), inv_std(inv_std) {}

  template <class T>
  bool operator()(T const* const* sKnots, T* sResiduals) const {
    using Vector3 = Eigen::Matrix<T, 3, 1>;

    Eigen::Map<Vector3> residuals(sResiduals);

    Sophus::SO3<T> R_w_i;
    CeresSplineHelper<N>::template evaluate_lie<T, Sophus::SO3>(sKnots, u,
                                                                inv_dt, &R_w_i);

    Vector3 accel_w;
    CeresSplineHelper<N>::template evaluate<T, 3, 2>(sKnots + N, u, inv_dt,
                                                     &accel_w);

    // Gravity
    Eigen::Map<Vector3 const> const g(sKnots[2 * N]);
    Eigen::Map<Vector3 const> const bias(sKnots[2 * N + 1]);

    residuals =
        inv_std * (R_w_i.inverse() * (accel_w + g) - measurement + bias);

    return true;
  }

  Eigen::Vector3d measurement;
  double u, inv_dt, inv_std;
};

template <int _N, template <class> class GroupT, bool OLD_TIME_DERIV>
struct CalibGyroCostFunctorSplit : public CeresSplineHelper<_N> {
  static constexpr int N = _N;        // Order of the spline.
  static constexpr int DEG = _N - 1;  // Degree of the spline.

  using MatN = Eigen::Matrix<double, _N, _N>;
  using VecN = Eigen::Matrix<double, _N, 1>;

  using Vec3 = Eigen::Matrix<double, 3, 1>;
  using Mat3 = Eigen::Matrix<double, 3, 3>;

  using Tangentd = typename GroupT<double>::Tangent;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CalibGyroCostFunctorSplit(const Tangentd& measurement, double u,
                            double inv_dt, double inv_std = 1)
      : measurement(measurement), u(u), inv_dt(inv_dt), inv_std(inv_std) {}

  template <class T>
  bool operator()(T const* const* sKnots, T* sResiduals) const {
    using Tangent = typename GroupT<T>::Tangent;

    Eigen::Map<Tangent> residuals(sResiduals);

    Tangent rot_vel;

    if constexpr (OLD_TIME_DERIV) {
      CeresSplineHelperOld<N>::template evaluate_lie_vel_old<T, GroupT>(
          sKnots, u, inv_dt, nullptr, &rot_vel);
    } else {
      CeresSplineHelper<N>::template evaluate_lie<T, GroupT>(sKnots, u, inv_dt,
                                                             nullptr, &rot_vel);
    }

    Eigen::Map<Tangent const> const bias(sKnots[N]);

    residuals = inv_std * (rot_vel - measurement + bias);

    return true;
  }

  Tangentd measurement;
  double u, inv_dt, inv_std;
};

template <int _N>
struct CalibReprojectionCostFunctorSplit : public CeresSplineHelper<_N> {
  static constexpr int N = _N;        // Order of the spline.
  static constexpr int DEG = _N - 1;  // Degree of the spline.

  using MatN = Eigen::Matrix<double, _N, _N>;
  using VecN = Eigen::Matrix<double, _N, 1>;

  using Vec3 = Eigen::Matrix<double, 3, 1>;
  using Mat3 = Eigen::Matrix<double, 3, 3>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CalibReprojectionCostFunctorSplit(const basalt::CalibCornerData* corners,
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
    using Vector3 = Eigen::Matrix<T, 3, 1>;
    using Vector4 = Eigen::Matrix<T, 4, 1>;

    using Matrix4 = Eigen::Matrix<T, 4, 4>;

    Sophus::SO3<T> R_w_i;
    CeresSplineHelper<N>::template evaluate_lie<T, Sophus::SO3>(sKnots, u,
                                                                inv_dt, &R_w_i);

    Vector3 t_w_i;
    CeresSplineHelper<N>::template evaluate<T, 3, 0>(sKnots + N, u, inv_dt,
                                                     &t_w_i);

    Eigen::Map<Sophus::SE3<T> const> const T_i_c(sKnots[2 * N]);

    Sophus::SE3<T> T_w_c = Sophus::SE3<T>(R_w_i, t_w_i) * T_i_c;
    Matrix4 T_c_w_matrix = T_w_c.inverse().matrix();

    basalt::GenericCamera<T> cam_t = cam.template cast<T>();


    // through first camera 内参和外参 to get the 3d coordinate of corner point, then project it to the second camera
    std::visit(
        [&](const auto& cam_tt) {// anonymous function
          for (size_t i = 0; i < corners->corner_ids.size(); i++) {
            //3d coordinates of each corner point
            Vector4 p3d =
                T_c_w_matrix *
                aprilgrid->aprilgrid_corner_pos_3d[corners->corner_ids[i]]
                    .cast<T>();

            Vector2 proj;
            bool success = cam_tt.project(p3d, proj);

            //compute the reprojection error
            if (success) {
              sResiduals[2 * i + 0] = proj[0] - corners->corners[i][0];
              sResiduals[2 * i + 1] = proj[1] - corners->corners[i][1];
            } else {
              sResiduals[2 * i + 0] = T(0);//double(0)
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
