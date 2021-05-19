
#include <ceres/ceres.h>
#include <iomanip>
#include <iostream>
#include <sophus/se3.hpp>

#include <basalt/spline/se3_spline.h>
#include <basalt/spline/so3_spline.h>

#include <ceres_lie_spline.h>

template <int N, template <class> class GroupT>
void test_optimization(
    const std::string& group_name, bool use_accel,
    std::map<std::string, std::pair<double, double>>& res_map) {
  using Groupd = GroupT<double>;
  using Tangentd = typename GroupT<double>::Tangent;

  const int NUM_KNOTS = 100 + N;

  int64_t dt = 2e9;

  int64_t pose_meas_t_ns = 8e9;
  int64_t deriv_meas_t_ns = 1e8;

  int num_pose_meas = 0;
  int num_deriv_meas = 0;

  CeresLieGroupSpline<N, GroupT> gt_spline(dt);
  CeresLieGroupSpline<N, GroupT> spline_new(dt);
  CeresLieGroupSpline<N, GroupT, true> spline_old(dt);

  gt_spline.initRandom(NUM_KNOTS);
  spline_new.initRandom(NUM_KNOTS);
  spline_old.initRandom(NUM_KNOTS);

  for (int i = 0; i < NUM_KNOTS; i++) {
    Groupd noisy_knot =
        gt_spline.getKnot(i) * Groupd::exp(Tangentd::Random() / 3.1);

    spline_new.getKnot(i) = noisy_knot;
    spline_old.getKnot(i) = noisy_knot;
  }

  for (int64_t t_ns = pose_meas_t_ns / 2; t_ns < gt_spline.maxTimeNs();
       t_ns += pose_meas_t_ns) {
    num_pose_meas++;
    spline_new.addMeasurement(gt_spline.getValue(t_ns), t_ns);
    spline_old.addMeasurement(gt_spline.getValue(t_ns), t_ns);
  }

  for (int64_t t_ns = deriv_meas_t_ns / 2; t_ns < gt_spline.maxTimeNs();
       t_ns += deriv_meas_t_ns) {
    num_deriv_meas++;
    if (use_accel) {
      spline_new.addAccelMeasurement(gt_spline.getAccel(t_ns), t_ns);
      spline_old.addAccelMeasurement(gt_spline.getAccel(t_ns), t_ns);
    } else {
      spline_new.addVelMeasurement(gt_spline.getVel(t_ns), t_ns);
      spline_old.addVelMeasurement(gt_spline.getVel(t_ns), t_ns);
    }
  }

  std::cout << "===============================================" << std::endl;

  std::cout << "Optimizing " << group_name << " splines of order " << N
            << " with " << num_pose_meas << " value measurements and "
            << num_deriv_meas << " "
            << (use_accel ? "accelleration" : "velocity") << " measurements"
            << std::endl;

  auto summary_new = spline_new.optimize();
  auto summary_old = spline_old.optimize();

  res_map[group_name + " order " + std::to_string(N) +
          (use_accel ? " acc" : " vel")] =
      std::make_pair(summary_new.total_time_in_seconds,
                     summary_old.total_time_in_seconds);

  std::cout << "===============================================" << std::endl;
}

int main(int, char**) {
  std::map<std::string, std::pair<double, double>> results;
  
  test_optimization<4, Sophus::SO3>("SO3", false, results);
  test_optimization<4, Sophus::SO3>("SO3", true, results);

  test_optimization<4, Sophus::SE3>("SE3", false, results);
  test_optimization<4, Sophus::SE3>("SE3", true, results);

  test_optimization<5, Sophus::SO3>("SO3", false, results);
  test_optimization<5, Sophus::SO3>("SO3", true, results);

  test_optimization<5, Sophus::SE3>("SE3", false, results);
  test_optimization<5, Sophus::SE3>("SE3", true, results);

  test_optimization<6, Sophus::SO3>("SO3", false, results);
  test_optimization<6, Sophus::SO3>("SO3", true, results);

  test_optimization<6, Sophus::SE3>("SE3", false, results);
  test_optimization<6, Sophus::SE3>("SE3", true, results);

  std::cout << "Overall Summary" << std::endl;

  for (auto kv : results) {
    std::cout << kv.first << ": " << std::fixed << std::setprecision(3)
              << kv.second.first << "s. " << kv.second.second << "s. "
              << kv.second.second / kv.second.first << "x" << std::endl;
  }

  return 0;
}
