

#include <iostream>

#include "gtest/gtest.h"

#include <basalt/spline/so3_spline.h>
#include <ceres_spline_helper_old.h>

template <int N>
void test_ceres_spline_helper_old_so3() {
  static const int64_t dt_ns = 2e9;

  basalt::So3Spline<N> spline(dt_ns);
  spline.genRandomTrajectory(3 * N);

  for (int64_t t_ns = 0; t_ns < spline.maxTimeNs(); t_ns += 1e8) {
    Sophus::SO3d pos1 = spline.evaluate(t_ns);
    Eigen::Vector3d vel1 = spline.velocityBody(t_ns);
    Eigen::Vector3d accel1 = spline.accelerationBody(t_ns);

    Sophus::SO3d pos2;
    Eigen::Vector3d vel2, accel2;

    {
      double pow_inv_dt = 1e9 / dt_ns;

      int64_t st_ns = (t_ns);

      BASALT_ASSERT_STREAM(st_ns >= 0, "st_ns " << st_ns << " time_ns " << t_ns
                                                << " start_t_ns " << 0);

      int64_t s = st_ns / dt_ns;
      double u = double(st_ns % dt_ns) / double(dt_ns);

      BASALT_ASSERT_STREAM(s >= 0, "s " << s);
      BASALT_ASSERT_STREAM(size_t(s + N) <= spline.getKnots().size(),
                           "s " << s << " N " << N << " knots.size() "
                                << spline.getKnots().size());

      std::vector<const double*> vec;
      for (int i = 0; i < N; i++) {
        vec.emplace_back(spline.getKnots()[s + i].data());
      }

      CeresSplineHelper<N>::template evaluate_lie<double, Sophus::SO3>(
          &vec[0], u, pow_inv_dt, &pos2);
      CeresSplineHelperOld<N>::template evaluate_lie_vel_old<double,
                                                             Sophus::SO3>(
          &vec[0], u, pow_inv_dt, nullptr, &vel2);
      CeresSplineHelperOld<N>::template evaluate_lie_accel_old<double,
                                                               Sophus::SO3>(
          &vec[0], u, pow_inv_dt, nullptr, nullptr, &accel2);
    }

    EXPECT_TRUE(pos1.matrix().isApprox(pos2.matrix()));
    EXPECT_TRUE(vel1.isApprox(vel2));
    EXPECT_TRUE(accel1.isApprox(accel2));
  }
}

template <int N>
void test_ceres_spline_helper_old_se3() {
  static const int64_t dt_ns = 2e9;

  Eigen::aligned_vector<Sophus::SE3d> knots;

  for (int i = 0; i < 3 * N; i++) {
    knots.emplace_back(Sophus::SE3d::exp(Sophus::Vector6d::Random()));
  }

  for (int i = 0; i < 2 * N; i++)
    for (double u = 0; u < 1; u += 0.01) {
      Sophus::Vector6d vel1, accel1, vel2, accel2, vel3;
      Sophus::SE3d pos1, pos2, pos3;

      {
        double pow_inv_dt = 1e9 / dt_ns;

        std::vector<const double*> vec;
        for (int j = 0; j < N; j++) {
          vec.emplace_back(knots[i + j].data());
        }

        CeresSplineHelper<N>::template evaluate_lie<double, Sophus::SE3>(
            &vec[0], u, pow_inv_dt, &pos1, &vel1, &accel1);

        CeresSplineHelperOld<N>::template evaluate_lie_vel_old<double,
                                                               Sophus::SE3>(
            &vec[0], u, pow_inv_dt, &pos2, &vel2);
        CeresSplineHelperOld<N>::template evaluate_lie_accel_old<double,
                                                                 Sophus::SE3>(
            &vec[0], u, pow_inv_dt, &pos3, &vel3, &accel2);
      }

      EXPECT_TRUE(pos1.matrix().isApprox(pos2.matrix()));
      EXPECT_TRUE(pos1.matrix().isApprox(pos3.matrix()));
      EXPECT_TRUE(vel1.isApprox(vel2));
      EXPECT_TRUE(vel1.isApprox(vel3));
      EXPECT_TRUE(accel1.isApprox(accel2));
    }
}

TEST(SplineCeresTestSuite, CeresSplineHelperOldSO3_4) {
  test_ceres_spline_helper_old_so3<4>();
}

TEST(SplineCeresTestSuite, CeresSplineHelperOldSO3_5) {
  test_ceres_spline_helper_old_so3<5>();
}

TEST(SplineCeresTestSuite, CeresSplineHelperOldSO3_6) {
  test_ceres_spline_helper_old_so3<6>();
}

TEST(SplineCeresTestSuite, CeresSplineHelperOldSE3_4) {
  test_ceres_spline_helper_old_se3<4>();
}

TEST(SplineCeresTestSuite, CeresSplineHelperOldSE3_5) {
  test_ceres_spline_helper_old_se3<5>();
}

TEST(SplineCeresTestSuite, CeresSplineHelperOldSE3_6) {
  test_ceres_spline_helper_old_se3<6>();
}
