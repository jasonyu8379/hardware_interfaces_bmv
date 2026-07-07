#include <iostream>
#include <mutex>
#include <thread>

#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>
// #include <robotiq_ft_modbus/robotiq_ft_modbus.h>
#include <wsg_gripper/wsg_gripper.h>
#include <wsg_gripper/wsg_gripper_driver.h>

int main() {
  std::cout << "WSG Gripper Test" << std::endl;

  RUT::Timer timer;
  RUT::TimePoint time0 = timer.tic();

  const std::string CONFIG_PATH =
      "/home/yifan/git/hardware_interfaces/workcell/table_top_manip/config/"
      "right_arm_coinft.yaml";
  YAML::Node config_node;
  try {
    config_node = YAML::LoadFile(CONFIG_PATH);
  } catch (const std::exception& e) {
    std::cerr << "Failed to load the config file: " << e.what() << std::endl;
    return false;
  }

  WSGGripper wsg_gripper;
  WSGGripper::WSGGripperConfig config;
  try {
    config.deserialize(config_node["wsg_gripper0"]);
  } catch (const std::exception& e) {
    std::cerr << "Failed to load the WSG Gripper config file: " << e.what()
              << std::endl;
    return false;
  }
  wsg_gripper.init(time0, config);
  std::cout << "WSG Gripper initialized." << std::endl;

  // CoinFT sensor;
  // CoinFT::CoinFTConfig coinft_config;
  // try {
  //   coinft_config.deserialize(config_node["coinft0"]);
  // } catch (const std::exception& e) {
  //   std::cerr << "Failed to load the CoinFT config file: " << e.what()
  //             << std::endl;
  //   return false;
  // }
  // std::cout << "Creating CoinFT object..." << std::endl;
  // try {
  //   sensor.init(time0, coinft_config);
  //   std::cout << "CoinFT object created." << std::endl;

  //   while (!sensor.is_data_ready()) {
  //     std::cout << "CoinFT Waiting for data..." << std::endl;
  //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
  //   }
  // } catch (const std::exception& e) {
  //   std::cerr << "An error occurred with CoinFT: " << e.what() << std::endl;
  //   return -1;
  // }

  // RUT::VectorXd wrench = RUT::VectorXd::Zero(12);
  RUT::VectorXd target_pos = RUT::VectorXd::Zero(1);
  RUT::VectorXd target_force = RUT::VectorXd::Zero(1);

  target_pos[0] = 25.0;
  double force_target = 40.0;

  std::cout << "[main] Starting control loop." << std::endl;
  while (timer.toc_ms() < 50000) {
    // // read force feedback
    // sensor.getWrenchTool(wrench, 2);
    // double force_fb = (-wrench[0] + wrench[6]) / 2;
    // target_force[0] = force_target - force_fb;
    target_force[0] = force_target;

    std::cout << "Time: " << timer.toc_ms()
              << " ms, target_force: " << target_force[0] << std::endl;
    // send command to gripper
    wsg_gripper.setJointsPosForce(target_pos, target_force);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }

  std::cout << "Done" << std::endl;

  return 0;
}