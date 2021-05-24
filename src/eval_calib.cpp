
#include <ceres_calib_spline_se3.h>
#include <ceres_calib_spline_split.h>

#include <basalt/io/dataset_io_euroc.h>
#include <basalt/optimization/spline_optimize.h>
#include <basalt/spline/so3_spline.h>
#include <basalt/utils/common_types.h>

#include <basalt/serialization/headers_serialization.h>
#include <basalt/calibration/calibration.hpp>

#include <tbb/global_control.h>

#include <sophus/average.hpp>

basalt::Calibration<double> calib;

std::unordered_map<basalt::TimeCamId, basalt::CalibCornerData> calib_corners,
    calib_corners_rejected;
std::unordered_map<basalt::TimeCamId, basalt::CalibInitPoseData>
    calib_init_poses;

struct CalibResults {
  std::string method_name;

  basalt::Calibration<double> calib;
  Eigen::Vector3d g, accel_bias, gyro_bias;

  double mean_reproj;

  double opt_time_s;
  int num_iter;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

template <class SplineT>
void run_calibration(const basalt::VioDatasetPtr& vio_dataset,
                     std::shared_ptr<basalt::AprilGrid>& aprilgrid,
                     const std::string& method_name,
                     Eigen::aligned_vector<CalibResults>& results) {
  std::cout << "=============================================" << std::endl;
  std::cout << "Running calibration with " << method_name << " method"
            << std::endl;

  constexpr int N = SplineT::N;
  const int64_t dt_ns = 1e7;

  int64_t start_t_ns =
      std::max(vio_dataset->get_image_timestamps().front(),
               vio_dataset->get_gyro_data().front().timestamp_ns);

  int64_t end_t_ns = std::min(vio_dataset->get_image_timestamps().back(),
                              vio_dataset->get_gyro_data().back().timestamp_ns);

  SplineT calib_spline(dt_ns, start_t_ns);
  calib_spline.setAprilgrid(aprilgrid);
  calib_spline.setCalib(calib);

  basalt::TimeCamId tcid_init(vio_dataset->get_image_timestamps().front(), 0);
  Sophus::SE3d T_w_i_init =
      calib_init_poses.at(tcid_init).T_a_c * calib.T_i_c[0].inverse();

  calib_spline.init(T_w_i_init, (end_t_ns - start_t_ns) / dt_ns + N);

  // Initialize gravity
  bool g_initialized = false;
  Eigen::Vector3d g_a_init;

  for (size_t j = 0; j < vio_dataset->get_image_timestamps().size(); ++j) {
    int64_t timestamp_ns = vio_dataset->get_image_timestamps()[j];

    basalt::TimeCamId tcid(timestamp_ns, 0);
    const auto cp_it = calib_init_poses.find(tcid);

    if (cp_it != calib_init_poses.end()) {
      // linear interpolation
      Sophus::SE3d T_a_i = cp_it->second.T_a_c * calib.T_i_c[0].inverse();

      if (!g_initialized) {
        for (size_t i = 0;
             i < vio_dataset->get_accel_data().size() && !g_initialized; i++) {
          const basalt::AccelData& ad = vio_dataset->get_accel_data()[i];
          if (std::abs(ad.timestamp_ns - timestamp_ns) < 3000000) {
            g_a_init = T_a_i.so3() * ad.data;
            g_initialized = true;
            std::cout << "g_a initialized with " << g_a_init.transpose()
                      << std::endl;
          }
        }
      }
    }
  }
  // set gravity
  calib_spline.setG(g_a_init);

  int num_gyro = 0;
  int num_accel = 0;
  int num_corner = 0;
  int num_frames = 0;

  for (const auto& v : vio_dataset->get_gyro_data()) {
    if (v.timestamp_ns >= start_t_ns && v.timestamp_ns < end_t_ns) {
      calib_spline.addGyroMeasurement(v.data, v.timestamp_ns);
      num_gyro++;
    }
  }

  for (const auto& v : vio_dataset->get_accel_data()) {
    if (v.timestamp_ns >= start_t_ns && v.timestamp_ns < end_t_ns) {
      calib_spline.addAccelMeasurement(v.data, v.timestamp_ns);
      num_accel++;
    }
  }

  // done
  for (const auto& kv : calib_corners) {
    if (kv.first.frame_id >= start_t_ns && kv.first.frame_id < end_t_ns) {
      calib_spline.addCornersMeasurement(&kv.second, kv.first.cam_id,
                                         kv.first.frame_id);

      num_corner += kv.second.corner_ids.size();
      num_frames++;
    }
  }

  calib_spline.meanReprojection(calib_corners);
  ceres::Solver::Summary summary = calib_spline.optimize();

  double mean_reproj = calib_spline.meanReprojection(calib_corners);

  std::cout << "num_gyro " << num_gyro << " num_accel " << num_accel
            << " num_corner " << num_corner << " num_frames " << num_frames
            << " duration " << (end_t_ns - start_t_ns) * 1e-9 << std::endl;

  std::cout << "g: " << calib_spline.getG().transpose() << std::endl;
  std::cout << "accel_bias: " << calib_spline.getAccelBias().transpose()
            << std::endl;
  std::cout << "gyro_bias: " << calib_spline.getGyroBias().transpose()
            << std::endl;
  for (size_t i = 0; i < calib_spline.getCalib().T_i_c.size(); i++) {
    std::cout << "T_i_c" << i << ":\n"
              << calib_spline.getCalib().T_i_c[i].matrix() << std::endl;
  }

  std::ofstream f(method_name + ".csv");

  for (int64_t t_ns = start_t_ns; t_ns < end_t_ns; t_ns += 1e6) {
    Eigen::Vector3d gyro, accel;

    gyro = calib_spline.getGyro(t_ns);
    accel = calib_spline.getAccel(t_ns);

    f << t_ns << "," << gyro[0] << "," << gyro[1] << "," << gyro[2] << ","
      << accel[0] << "," << accel[1] << "," << accel[2] << std::endl;
  }
  f.close();

  CalibResults r;
  r.calib = calib_spline.getCalib();
  r.g = calib_spline.getG();
  r.opt_time_s = summary.total_time_in_seconds;
  r.method_name = method_name;
  r.num_iter = summary.num_successful_steps;
  r.accel_bias = calib_spline.getAccelBias();
  r.gyro_bias = calib_spline.getGyroBias();
  r.mean_reproj = mean_reproj;

  results.emplace_back(r);
}

void run_calibration_custom(const basalt::VioDatasetPtr& vio_dataset,
                            std::shared_ptr<basalt::AprilGrid>& aprilgrid,
                            Eigen::aligned_vector<CalibResults>& results) {
  std::cout << "=============================================" << std::endl;
  std::cout << "Running calibration with custom_split method" << std::endl;

  constexpr int N = 5;
  const int64_t dt_ns = 1e7;

  //设定开始时间
  int64_t start_t_ns =
      std::max(vio_dataset->get_image_timestamps().front(),
               vio_dataset->get_gyro_data().front().timestamp_ns);

  //设定结束时间
  int64_t end_t_ns = std::min(vio_dataset->get_image_timestamps().back(),
                              vio_dataset->get_gyro_data().back().timestamp_ns);

  basalt::SplineOptimization<N, double> spline_opt(dt_ns, 1e-6);

  spline_opt.setAprilgridCorners3d(aprilgrid->aprilgrid_corner_pos_3d);
  spline_opt.calib.reset(new basalt::Calibration<double>(calib));
  spline_opt.resetMocapCalib();

  basalt::TimeCamId tcid_init(vio_dataset->get_image_timestamps().front(), 0);
  Sophus::SE3d T_w_i_init =
      calib_init_poses.at(tcid_init).T_a_c * calib.T_i_c[0].inverse();

  spline_opt.initSpline(T_w_i_init, (end_t_ns - start_t_ns) / dt_ns + N);

  // Initialize gravity
  bool g_initialized = false;
  Eigen::Vector3d g_a_init;

  for (size_t j = 0; j < vio_dataset->get_image_timestamps().size(); ++j) {
    int64_t timestamp_ns = vio_dataset->get_image_timestamps()[j];

    basalt::TimeCamId tcid(timestamp_ns, 0);
    const auto cp_it = calib_init_poses.find(tcid);

    if (cp_it != calib_init_poses.end()) {
      Sophus::SE3d T_a_i = cp_it->second.T_a_c * calib.T_i_c[0].inverse();

      if (!g_initialized) {
        for (size_t i = 0;
             i < vio_dataset->get_accel_data().size() && !g_initialized; i++) {
          const basalt::AccelData& ad = vio_dataset->get_accel_data()[i];
          if (std::abs(ad.timestamp_ns - timestamp_ns) < 3000000) {
            g_a_init = T_a_i.so3() * ad.data;
            g_initialized = true;
            std::cout << "g_a initialized with " << g_a_init.transpose()
                      << std::endl;
          }
        }
      }
    }
  }
  spline_opt.setG(g_a_init);

  for (const auto& v : vio_dataset->get_gyro_data()) {
    if (v.timestamp_ns >= start_t_ns && v.timestamp_ns < end_t_ns)
      spline_opt.addGyroMeasurement(v.timestamp_ns - start_t_ns, v.data);
  }

  for (const auto& v : vio_dataset->get_accel_data()) {
    if (v.timestamp_ns >= start_t_ns && v.timestamp_ns < end_t_ns)
      spline_opt.addAccelMeasurement(v.timestamp_ns - start_t_ns, v.data);
  }

  for (const auto& kv : calib_corners) {
    if (kv.first.frame_id >= start_t_ns && kv.first.frame_id < end_t_ns)
      spline_opt.addAprilgridMeasurement(kv.first.frame_id - start_t_ns,
                                         kv.first.cam_id, kv.second.corners,
                                         kv.second.corner_ids);
  }

  spline_opt.init();

  // calib_spline.meanReprojection(calib_corners);

  double error, reprojection_error;
  int num_points;

  bool converged = false;
  int opt_iter = 0;

  auto start = std::chrono::high_resolution_clock::now();

  while (!converged) {
    converged =
        spline_opt.optimize(false, false, true, false, false, false, 100.0,
                            1e-9, error, num_points, reprojection_error, false);
    opt_iter++;
  }

  auto stop = std::chrono::high_resolution_clock::now();

  double opt_time_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start)
          .count();

  std::cout << "time: " << opt_time_ms << "ms." << std::endl;

  std::cout << "num_iter " << opt_iter << std::endl;
  std::cout << "reprojection error: " << reprojection_error / num_points
            << std::endl;

  // calib_spline.meanReprojection(calib_corners);

  std::cout << "g: " << spline_opt.getG().transpose() << std::endl;

  Eigen::Vector3d accel_bias;
  Eigen::Matrix3d accel_scale;
  spline_opt.calib->calib_accel_bias.getBiasAndScale(accel_bias, accel_scale);

  std::cout << "accel_bias: " << accel_bias.transpose() << "\naccel_scale:\n"
            << Eigen::Matrix3d::Identity() + accel_scale << std::endl;

  Eigen::Vector3d gyro_bias;
  Eigen::Matrix3d gyro_scale;
  spline_opt.calib->calib_gyro_bias.getBiasAndScale(gyro_bias, gyro_scale);

  std::cout << "gyro_bias: " << gyro_bias.transpose() << "\ngyro_scale:\n"
            << Eigen::Matrix3d::Identity() + gyro_scale << std::endl;

  for (size_t i = 0; i < spline_opt.calib->T_i_c.size(); i++) {
    std::cout << "T_i_c" << i << ":\n"
              << spline_opt.calib->T_i_c[i].matrix() << std::endl;
  }

  // std::ofstream f("generated_imu.csv");

  //  for (int64_t t_ns = start_t_ns; t_ns < end_t_ns; t_ns += 1e6) {
  //    Eigen::Vector3d gyro, accel;

  //    gyro = calib_spline.getGyro(t_ns);
  //    accel = calib_spline.getAccel(t_ns);

  //    f << t_ns << "," << gyro[0] << "," << gyro[1] << "," << gyro[2] << ","
  //      << accel[0] << "," << accel[1] << "," << accel[2] << std::endl;
  //  }
  //  f.close();

  CalibResults r;
  r.calib = *(spline_opt.calib);
  r.g = spline_opt.getG();
  r.opt_time_s = opt_time_ms / 1000.0;
  r.method_name = "custom_split";
  r.num_iter = opt_iter;
  r.accel_bias = accel_bias;
  r.gyro_bias = gyro_bias;
  r.mean_reproj = reprojection_error / num_points;

  results.emplace_back(r);
}

int main() {
  // test_trans_spline();
  // test_rot_spline();

  // global thread limit is in effect until global_control object is destroyed
  tbb::global_control tbb_global_control(
      tbb::global_control::max_allowed_parallelism, 1);

  std::string data_path = "../data/";
  std::string calibration_path = data_path + "initial_calibration.json";
  std::string dataset_path = data_path + "dataset-calib-imu1_512_16/";
  std::string detected_corners_path =
      data_path + "cache/calib-cam-imu_detected_corners.cereal";
  std::string initial_poses_path =
      data_path + "cache/calib-cam-imu_init_poses.cereal";
  std::string aprilgrid_path = data_path + "aprilgrid_6x6.json";

  std::ifstream is(calibration_path);

  if (is.good()) {
    cereal::JSONInputArchive archive(is);
    archive(calib);
    std::cout << "Loaded calibration from: " << calibration_path << std::endl;
  } else {
    std::cerr << "No calibration found" << std::endl;
    std::abort();
  }

  basalt::VioDatasetPtr vio_dataset;

  basalt::DatasetIoInterfacePtr dataset_io(new basalt::EurocIO(true));

  dataset_io->read(dataset_path);
  vio_dataset = dataset_io->get_data();

  std::cout << "Loaded a dataset with "
            << vio_dataset->get_image_timestamps().size() << " images."
            << std::endl;

  // load detected corners
  {
    std::ifstream is(detected_corners_path, std::ios::binary);

    if (is.good()) {
      cereal::BinaryInputArchive archive(is);

      calib_corners.clear();
      calib_corners_rejected.clear();
      archive(calib_corners);
      archive(calib_corners_rejected);

      std::cout << "Loaded " << calib_corners.size()
                << " detected corners from: " << detected_corners_path
                << std::endl;
    } else {
      std::cout << "No pre-processed detected corners found" << std::endl;
    }
  }

  // load initial poses
  {
    std::ifstream is(initial_poses_path, std::ios::binary);

    if (is.good()) {
      cereal::BinaryInputArchive archive(is);

      calib_init_poses.clear();
      archive(calib_init_poses);

      std::cout << "Loaded " << calib_init_poses.size()
                << " initial poses from: " << initial_poses_path << std::endl;
    } else {
      std::cout << "No pre-processed initial poses found" << std::endl;
    }
  }

  std::shared_ptr<basalt::AprilGrid> aprilgrid(
      new basalt::AprilGrid(aprilgrid_path));

  Eigen::aligned_vector<CalibResults> results;

  run_calibration_custom(vio_dataset, aprilgrid, results);

  run_calibration<CeresCalibrationSplineSplit<5>>(vio_dataset, aprilgrid,
                                                  "ceres_split", results);
  run_calibration<CeresCalibrationSplineSplit<5, true>>(
      vio_dataset, aprilgrid, "ceres_split_old", results);

  run_calibration<CeresCalibrationSplineSe3<5>>(vio_dataset, aprilgrid,
                                                "ceres_se3", results);
  run_calibration<CeresCalibrationSplineSe3<5, true>>(vio_dataset, aprilgrid,
                                                      "ceres_se3_old", results);

  Eigen::Vector3d g_mean(0, 0, 0), accel_bias_mean(0, 0, 0),
      gyro_bias_mean(0, 0, 0), t_i_c0_mean(0, 0, 0), t_i_c1_mean(0, 0, 0);

  Sophus::SO3d R_i_c0_mean, R_i_c1_mean;

  double mean_reproj_mean = 0;

  Eigen::aligned_vector<Sophus::SO3d> R_i_c0, R_i_c1;

  int num_methods = 0;

  std::cout << "=============================================" << std::endl;

  for (const auto& r : results) {
    std::cout << r.method_name << "\t: opt_time " << r.opt_time_s
              << "\tnum_iter " << r.num_iter << std::endl;

    g_mean += r.g;
    accel_bias_mean += r.accel_bias;
    gyro_bias_mean += r.gyro_bias;
    t_i_c0_mean += r.calib.T_i_c[0].translation();
    t_i_c1_mean += r.calib.T_i_c[1].translation();

    R_i_c0.emplace_back(r.calib.T_i_c[0].so3());
    R_i_c1.emplace_back(r.calib.T_i_c[1].so3());

    mean_reproj_mean += r.mean_reproj;

    num_methods++;
  }

  g_mean /= num_methods;
  accel_bias_mean /= num_methods;
  gyro_bias_mean /= num_methods;
  t_i_c0_mean /= num_methods;
  t_i_c1_mean /= num_methods;

  R_i_c0_mean = *Sophus::average(R_i_c0);
  R_i_c1_mean = *Sophus::average(R_i_c1);

  mean_reproj_mean /= num_methods;

  double g_max_dist = 0, accel_bias_max_dist = 0, gyro_bias_max_dist = 0,
         t_i_c0_max_dist = 0, t_i_c1_max_dist = 0, R_i_c0_max_dist = 0,
         R_i_c1_max_dist = 0, mean_reproj_max_dist = 0;

  for (const auto& r : results) {
    g_max_dist = std::max(g_max_dist, (g_mean - r.g).norm());
    accel_bias_max_dist =
        std::max(accel_bias_max_dist, (accel_bias_mean - r.accel_bias).norm());
    gyro_bias_max_dist =
        std::max(gyro_bias_max_dist, (gyro_bias_mean - r.gyro_bias).norm());

    t_i_c0_max_dist = std::max(
        t_i_c0_max_dist, (t_i_c0_mean - r.calib.T_i_c[0].translation()).norm());
    t_i_c1_max_dist = std::max(
        t_i_c1_max_dist, (t_i_c1_mean - r.calib.T_i_c[1].translation()).norm());

    R_i_c0_max_dist = std::max(R_i_c0_max_dist,
                               R_i_c0_mean.unit_quaternion().angularDistance(
                                   r.calib.T_i_c[0].so3().unit_quaternion()));

    R_i_c1_max_dist = std::max(R_i_c1_max_dist,
                               R_i_c1_mean.unit_quaternion().angularDistance(
                                   r.calib.T_i_c[1].so3().unit_quaternion()));

    mean_reproj_max_dist = std::max(mean_reproj_max_dist,
                                    std::abs(mean_reproj_mean - r.mean_reproj));
  }

  std::cout << "=============================================" << std::endl;

  std::cout << "g: " << g_mean.transpose() << " max_diff_norm " << g_max_dist
            << std::endl;
  std::cout << "accel_bias: " << accel_bias_mean.transpose()
            << " max_diff_norm " << accel_bias_max_dist << std::endl;
  std::cout << "gyro_bias: " << gyro_bias_mean.transpose() << " max_diff_norm "
            << gyro_bias_max_dist << std::endl;

  std::cout << "t_i_c0: " << t_i_c0_mean.transpose() << " max_diff_norm "
            << t_i_c0_max_dist << std::endl;
  std::cout << "t_i_c1: " << t_i_c1_mean.transpose() << " max_diff_norm "
            << t_i_c1_max_dist << std::endl;

  std::cout << "R_i_c0: " << R_i_c0_mean.unit_quaternion().coeffs().transpose()
            << " max_diff_norm " << R_i_c0_max_dist << std::endl;

  std::cout << "R_i_c1: " << R_i_c1_mean.unit_quaternion().coeffs().transpose()
            << " max_diff_norm " << R_i_c1_max_dist << std::endl;

  std::cout << "mean_reproj: " << mean_reproj_mean << " max_diff_norm "
            << mean_reproj_max_dist << std::endl;

  return 0;
}
