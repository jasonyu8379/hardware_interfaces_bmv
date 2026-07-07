#include <unistd.h>
#include <QApplication>
#include <iostream>
#include <chrono>
#include <cmath>

#include <RobotUtilities/spatial_utilities.h>
#include <table_top_manip/manip_server.h>
#include "teleop_gui.h"

static const std::string CONFIG_PATH =
    "/home/bmv/hardware_interfaces/workcell/table_top_manip/"
    "config/bmv/JCY_bmv_single_arm_multicam_typodont_data_collection.yaml";

int main(int argc, char* argv[]) {
  QApplication app(argc, argv);

  ManipServer server(CONFIG_PATH);
  while (!server.is_ready()) {
    std::cout << "Waiting for server to be ready...\n";
    usleep(400000);
  }

  // Stiffness/damping tuned for teleoperation (matches MAA_data_collection_v2.py)
  RUT::Matrix6d teleop_stiffness = RUT::Matrix6d::Zero();
  teleop_stiffness.diagonal() << 3000, 3000, 3000, 1, 1, 1;
  RUT::Matrix6d teleop_damping = RUT::Matrix6d::Zero();
  teleop_damping.diagonal() << 10, 10, 10, 0.3, 0.3, 0.3;
  server.set_stiffness_matrix(teleop_stiffness, 0);
  server.set_damping_matrix(teleop_damping, 0);
  server.set_high_level_maintain_position();

  // ── Homing — matches Python homeing() in MAA_data_collection_v2.py ──────
  // Orientation: identity → Rx(-120°) → Rz(-160°)
  // Position: [0.15, -0.35, 0.50] m
  {
    static constexpr double HOME_X    = 0.15;
    static constexpr double HOME_Y    = -0.35;
    static constexpr double HOME_Z    = 0.50;
    static constexpr double DEG2RAD   = M_PI / 180.0;
    static constexpr double TOL       = 0.01;  // 1 cm per axis
    static constexpr int    TIMEOUT_S = 30;

    Eigen::Quaterniond q_rx(Eigen::AngleAxisd(-120.0 * DEG2RAD, Eigen::Vector3d::UnitX()));
    Eigen::Quaterniond q_rz(Eigen::AngleAxisd(-160.0 * DEG2RAD, Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond q_home = (q_rz * q_rx).normalized();

    RUT::Vector7d home_pose;
    home_pose << HOME_X, HOME_Y, HOME_Z,
                 q_home.w(), q_home.x(), q_home.y(), q_home.z();

    std::cout << "Homing robot to ["
              << HOME_X << ", " << HOME_Y << ", " << HOME_Z << "]...\n";
    server.set_target_pose(home_pose, 2000.0, 0);

    auto t_start = std::chrono::steady_clock::now();
    while (true) {
      double elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - t_start).count();
      if (elapsed > TIMEOUT_S) {
        std::cerr << "Homing timed out after " << TIMEOUT_S << " s.\n";
        break;
      }
      Eigen::MatrixXd pose_mat = server.get_pose(1);
      if (pose_mat.cols() > 0) {
        double ex = std::abs(HOME_X - pose_mat(0, 0));
        double ey = std::abs(HOME_Y - pose_mat(1, 0));
        double ez = std::abs(HOME_Z - pose_mat(2, 0));
        if (ex < TOL && ey < TOL && ez < TOL) {
          std::cout << "Homing complete. Took " << elapsed << " s.\n";
          break;
        }
      }
      usleep(250000);  // 0.25 s polling
    }
  }

  TeleopGui gui(server);
  gui.show();

  app.exec();

  server.stop_saving_data();  // JY: clean stop before joining threads
  server.join_threads();
  std::cout << "Exiting.\n";
  return 0;
}
