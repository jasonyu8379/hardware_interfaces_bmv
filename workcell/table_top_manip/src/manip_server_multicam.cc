#include "table_top_manip/manip_server.h"
#include "helpers.hpp"

ManipServer::ManipServer(const std::string& config_path) {
  initialize(config_path);
}

ManipServer::~ManipServer() {}

bool ManipServer::initialize(const std::string& config_path) {
  std::cout << "[ManipServer] Initializing.\n";

  RUT::TimePoint time0 = _timer.tic();

  // read config files
  std::cout << "[ManipServer] Reading config files.\n";
  YAML::Node config;
  try {
    config = YAML::LoadFile(config_path);
    _config.deserialize(config);
  } catch (const std::exception& e) {
    std::cerr << "Failed to load the config file: " << e.what() << std::endl;
    return false;
  }

  if (_config.bimanual) {
    _id_list = {0, 1};
  } else {
    _id_list = {0};
  }

  // MAA: added this
  if (_config.cam_count == 1) {
    _cam_id_list = {0};
    std::cout << "[ManipServer] One Camera in configuration file.\n";
  } else if (_config.cam_count == 2) {
    _cam_id_list = {0, 1};
    std::cout << "[ManipServer] Two Cameras found in configuration file.\n";
  } else if (_config.cam_count == 3) {
    _cam_id_list = {0, 1, 2};
    std::cout << "[ManipServer] Three Cameras found in configuration file.\n";
  } else {
    std::cerr << "Invalid cam_count. Exiting." << std::endl;
    return false;
  }

  std::cout << "_id_list: " << _id_list.size() << std::endl;

  // parameters to be obtained from config
  std::vector<int> wrench_publish_rate;

  std::cout << "[ManipServer] bimanual: " << _config.bimanual << std::endl;
  std::cout << "[ManipServer] Initialize each hardware interface.\n";

  // initialize hardwares
  if (!_config.mock_hardware) {
    for (int id : _id_list) {
      // Robot
      URRTDE::URRTDEConfig robot_config;
      try {
        robot_config.deserialize(config["ur_rtde" + std::to_string(id)]);
      } catch (const std::exception& e) {
        std::cerr << "Failed to load the robot config file: " << e.what()
                  << std::endl;
        return false;
      }
      robot_ptrs.emplace_back(new URRTDE);
      URRTDE* urrtde_ptr = static_cast<URRTDE*>(robot_ptrs[id].get());
      if (!urrtde_ptr->init(time0, robot_config)) {
        std::cerr << "Failed to initialize UR RTDE for id " << id
                  << ". Exiting." << std::endl;
        return false;
      } else {
        std::cout << "[ManipServer] Robot " << id << " initialized.\n";
      }

      // EoAT
      if (_config.run_eoat_thread) {
        WSGGripper::WSGGripperConfig eoat_config;
        try {
          eoat_config.deserialize(config["wsg_gripper" + std::to_string(id)]);
        } catch (const std::exception& e) {
          std::cerr << "Failed to load the eoat config file: " << e.what()
                    << std::endl;
          return false;
        }
        eoat_ptrs.emplace_back(new WSGGripper);
        WSGGripper* wsggripper_ptr =
            static_cast<WSGGripper*>(eoat_ptrs[id].get());
        if (!wsggripper_ptr->init(time0, eoat_config)) {
          std::cerr << "Failed to initialize WSGGripper for id " << id
                    << ". Exiting." << std::endl;
          return false;
        } else {
          std::cout << "[ManipServer] EoAT " << id << " initialized.\n";
        }
      }

      if (_config.run_wrench_thread) {
        // force sensor
        if (_config.force_sensing_mode == ForceSensingMode::FORCE_MODE_ATI) {
          ATINetft::ATINetftConfig ati_config;
          try {
            ati_config.deserialize(config["ati_netft" + std::to_string(id)]);
          } catch (const std::exception& e) {
            std::cerr << "Failed to load the ATI Netft config file: "
                      << e.what() << std::endl;
            return false;
          }
          force_sensor_ptrs.emplace_back(new ATINetft);
          ATINetft* ati_ptr =
              static_cast<ATINetft*>(force_sensor_ptrs[id].get());
          if (!ati_ptr->init(time0, ati_config)) {
            std::cerr << "Failed to initialize ATI Netft for id " << id
                      << ". Exiting." << std::endl;
            return false;
          }
          wrench_publish_rate.push_back(ati_config.publish_rate);
        } else if (_config.force_sensing_mode ==
                   ForceSensingMode::FORCE_MODE_ROBOTIQ) {
          RobotiqFTModbus::RobotiqFTModbusConfig robotiq_config;
          try {
            robotiq_config.deserialize(
                config["robotiq_ft_modbus" + std::to_string(id)]);
          } catch (const std::exception& e) {
            std::cerr << "Failed to load the Robotiq FT Modbus config file: "
                      << e.what() << std::endl;
            return false;
          }
          force_sensor_ptrs.emplace_back(new RobotiqFTModbus);
          RobotiqFTModbus* robotiq_ptr =
              static_cast<RobotiqFTModbus*>(force_sensor_ptrs[id].get());
          if (!robotiq_ptr->init(time0, robotiq_config)) {
            std::cerr << "Failed to initialize Robotiq FT Modbus for id " << id
                      << ". Exiting." << std::endl;
            return false;
          }
          wrench_publish_rate.push_back(robotiq_config.publish_rate);
        // CoinFT disabled: onnxruntime not installed
        // } else if (_config.force_sensing_mode ==
        //            ForceSensingMode::FORCE_MODE_COINFT) {
        //   CoinFT::CoinFTConfig coinft_config;
        //   try {
        //     coinft_config.deserialize(config["coinft" + std::to_string(id)]);
        //   } catch (const std::exception& e) {
        //     std::cerr << "Failed to load the CoinFT config file: " << e.what()
        //               << std::endl;
        //     return false;
        //   }
        //   force_sensor_ptrs.emplace_back(new CoinFT);
        //   CoinFT* coinft_ptr =
        //       static_cast<CoinFT*>(force_sensor_ptrs[id].get());
        //   if (!coinft_ptr->init(time0, coinft_config)) {
        //     std::cerr << "Failed to initialize CoinFT for id " << id
        //               << ". Exiting." << std::endl;
        //     return false;
        //   }
        //   wrench_publish_rate.push_back(1000);
        }
////////////////////////////////////////////////////MAA//////////////////////////////////////////////////
        else if (_config.force_sensing_mode ==
                 ForceSensingMode::FORCE_MODE_LEPTRINO) {
          Leptrinoft::LEPTRINOFTConfig leptrino_config;
          try {
            leptrino_config.deserialize(
                config["leptrino" + std::to_string(id)]);
          } catch (const std::exception& e) {
            std::cerr << "Failed to load the Leptrino config file: " << e.what()
                      << std::endl;
            return false;
          }
          force_sensor_ptrs.emplace_back(new Leptrinoft);
          Leptrinoft* leptrino_ptr =
              static_cast<Leptrinoft*>(force_sensor_ptrs[id].get());
          if (!leptrino_ptr->init(time0, leptrino_config)) {
            std::cerr << "Failed to initialize Leptrino for id " << id
                      << ". Exiting." << std::endl;
            return false;
          }
          wrench_publish_rate.push_back(leptrino_config.publish_rate);
        }
////////////////////////////////////////////////////MAA//////////////////////////////////////////////////
        else {
          std::cerr << "Invalid force sensing mode. Exiting." << std::endl;
          return false;
        }
        std::cout << "[ManipServer] Force sensor " << id << " initialized.\n";
        _num_ft_sensors.push_back(force_sensor_ptrs[id]->getNumSensors());
      }
    }

    // MAA: Camera initialization moved outside robot loop, iterates over _cam_id_list
    if (_config.run_rgb_thread) {
      for (int cam_id : _cam_id_list) {
        if (_config.camera_selection == CameraSelection::GOPRO) {
          GoPro::GoProConfig gopro_config;
          try {
            gopro_config.deserialize(config["gopro" + std::to_string(cam_id)]);
          } catch (const std::exception& e) {
            std::cerr << "Failed to load the GoPro config file: " << e.what()
                      << std::endl;
            return false;
          }
          camera_ptrs.emplace_back(new GoPro);
          GoPro* gopro_ptr = static_cast<GoPro*>(camera_ptrs[cam_id].get());
          if (!gopro_ptr->init(time0, gopro_config)) {
            std::cerr << "Failed to initialize GoPro for cam_id " << cam_id
                      << ". Exiting." << std::endl;
            return false;
          }
        } else if (_config.camera_selection == CameraSelection::REALSENSE) {
          Realsense::RealsenseConfig realsense_config;
          try {
            realsense_config.deserialize(
                config["realsense" + std::to_string(cam_id)]);
          } catch (const std::exception& e) {
            std::cerr << "Failed to load the Realsense config file: "
                      << e.what() << std::endl;
            return false;
          }
          camera_ptrs.emplace_back(new Realsense);
          Realsense* realsense_ptr =
              static_cast<Realsense*>(camera_ptrs[cam_id].get());
          if (!realsense_ptr->init(time0, realsense_config)) {
            std::cerr << "Failed to initialize Realsense for cam_id " << cam_id
                      << ". Exiting." << std::endl;
            return false;
          }
        // OAK disabled: library not built
        // } else if (_config.camera_selection == CameraSelection::OAK) {
        //   OAK::OAKConfig oak_config;
        //   try {
        //     oak_config.deserialize(config["oak" + std::to_string(cam_id)]);
        //   } catch (const std::exception& e) {
        //     std::cerr << "Failed to load the OAK config file: " << e.what()
        //               << std::endl;
        //     return false;
        //   }
        //   camera_ptrs.emplace_back(new OAK);
        //   OAK* oak_ptr = static_cast<OAK*>(camera_ptrs[cam_id].get());
        //   if (!oak_ptr->init(time0, oak_config)) {
        //     std::cerr << "Failed to initialize OAK camera for cam_id " << cam_id
        //               << ". Exiting." << std::endl;
        //     return false;
        //   }
        // }
        } else {
          std::cerr << "Invalid camera selection. Exiting." << std::endl;
          return false;
        }
        std::cout << "[ManipServer] Camera " << cam_id << " initialized.\n";
      }
    }

  } else {
    // mock hardware
    for (int id : _id_list) {
      wrench_publish_rate.push_back(500);
      if (_config.force_sensing_mode == ForceSensingMode::FORCE_MODE_ATI) {
        _num_ft_sensors.push_back(1);
      } else if (_config.force_sensing_mode ==
                 ForceSensingMode::FORCE_MODE_ROBOTIQ) {
        _num_ft_sensors.push_back(1);
      } else if (_config.force_sensing_mode ==
                 ForceSensingMode::FORCE_MODE_COINFT) {
        _num_ft_sensors.push_back(2);
      } else if (_config.force_sensing_mode ==
                 ForceSensingMode::FORCE_MODE_LEPTRINO) {
        _num_ft_sensors.push_back(1);
      } else {
        std::cerr << "Invalid force sensing mode. Exiting." << std::endl;
        return false;
      }
    }
  }

  // initialize wrench filter
  std::cout << "[ManipServer] Initializing wrench filters.\n";
  for (int id : _id_list) {
    int num_ft_sensors = 1;
    if (!_config.mock_hardware) {
      num_ft_sensors = force_sensor_ptrs[id]->getNumSensors();
    }
    _wrench_filters.emplace_back(_config.wrench_filter_parameters[0],
                                 _config.wrench_filter_parameters[1],
                                 _config.wrench_filter_parameters[2],
                                 6 * num_ft_sensors);
  }

  // initialize Admittance controller and perturbation generator
  std::cout << "[ManipServer] Initializing Admittance controllers.\n";
  for (int id : _id_list) {
    AdmittanceController::AdmittanceControllerConfig admittance_config;
    try {
      deserialize(config["admittance_controller" + std::to_string(id)],
                  admittance_config);
    } catch (const std::exception& e) {
      std::cerr << "Failed to load the admittance controller config file: "
                << e.what() << std::endl;
      return false;
    }

    _controllers.emplace_back();
    _controller_mtxs.emplace_back();
    RUT::Vector7d pose = RUT::Vector7d::Zero();
    if (!_config.mock_hardware) {
      robot_ptrs[id]->getCartesian(pose);
    }
    if (!_controllers[id].init(time0, admittance_config, pose)) {
      std::cerr << "Failed to initialize admittance controller for id " << id
                << ". Exiting." << std::endl;
      return false;
    }
    RUT::Matrix6d Tr = RUT::Matrix6d::Identity();
    int n_af = 0;
    _controllers[id].setForceControlledAxis(Tr, n_af);

    PerturbationGenerator::PerturbationGeneratorConfig perturbation_config;
    try {
      perturbation_config.deserialize(
          config["perturbation_generator" + std::to_string(id)]);
    } catch (const std::exception& e) {
      std::cerr << "Failed to load the perturbation generator config file: "
                << e.what() << std::endl;
      return false;
    }
    _perturbation_generators.emplace_back();
    _perturbation_generators[id].init(perturbation_config);

    _stiffnesses_high.push_back(admittance_config.compliance6d.stiffness);
    _stiffnesses_low.push_back(RUT::Matrix6d::Zero());
    _dampings_high.push_back(admittance_config.compliance6d.damping);
    _dampings_low.push_back(_config.low_damping);
  }

  std::cout << "[ManipServer] Initializing data buffers.\n";

  // MAA: camera buffers now iterate over _cam_id_list
  if (_config.run_rgb_thread) {
    for (int cam_id : _cam_id_list) {
      _states_rgb_thread_ready.push_back(false);
      _states_rgb_thread_saving.push_back(false);
      _states_rgb_seq_id.push_back(0);

      _camera_rgb_buffers.push_back(RUT::DataBuffer<Eigen::MatrixXd>());
      _camera_rgb_timestamp_ms_buffers.push_back(RUT::DataBuffer<double>());
      _camera_rgb_buffers[cam_id].initialize(
          _config.rgb_buffer_size, 3 * _config.output_rgb_hw[0],
          _config.output_rgb_hw[1], "camera_rgb" + std::to_string(cam_id));
      _camera_rgb_timestamp_ms_buffers[cam_id].initialize(
          _config.rgb_buffer_size, 1, 1,
          "camera_rgb" + std::to_string(cam_id) + "_timestamp_ms");
      _camera_rgb_buffer_mtxs.emplace_back();
    }
  }

  // wrench sensors
  if (_config.run_wrench_thread) {
    for (int id : _id_list) {
      _states_wrench_thread_ready.push_back(false);
      _states_wrench_thread_saving.push_back(false);
      _states_wrench_seq_id.push_back(0);

      _wrench_buffers.push_back(RUT::DataBuffer<Eigen::VectorXd>());
      _wrench_filtered_buffers.push_back(RUT::DataBuffer<Eigen::VectorXd>());
      _wrench_timestamp_ms_buffers.push_back(RUT::DataBuffer<double>());
      _wrench_filtered_timestamp_ms_buffers.push_back(RUT::DataBuffer<double>());

      _wrench_buffers[id].initialize(_config.wrench_buffer_size,
                                     6 * _num_ft_sensors[id], 1,
                                     "wrench" + std::to_string(id));
      _wrench_filtered_buffers[id].initialize(
          _config.wrench_buffer_size, 6, 1,
          "wrench_filtered" + std::to_string(id));
      _wrench_timestamp_ms_buffers[id].initialize(
          _config.wrench_buffer_size, 1, 1,
          "wrench" + std::to_string(id) + "_timestamp_ms");
      _wrench_filtered_timestamp_ms_buffers[id].initialize(
          _config.wrench_buffer_size, 1, 1,
          "wrench_filtered" + std::to_string(id) + "_timestamp_ms");
      _wrench_buffer_mtxs.emplace_back();
      _wrench_filtered_buffer_mtxs.emplace_back();
    }
  }

  // eoat
  if (_config.run_eoat_thread) {
    for (int id : _id_list) {
      _states_eoat_thread_ready.push_back(false);
      _states_eoat_thread_saving.push_back(false);
      _states_eoat_seq_id.push_back(0);

      _eoat_buffers.push_back(RUT::DataBuffer<Eigen::VectorXd>());
      _eoat_timestamp_ms_buffers.push_back(RUT::DataBuffer<double>());

      _eoat_buffers[id].initialize(_config.eoat_buffer_size, 1, 1,
                                   "eoat" + std::to_string(id));
      _eoat_timestamp_ms_buffers[id].initialize(
          _config.eoat_buffer_size, 1, 1,
          "eoat" + std::to_string(id) + "_timestamp_ms");
      _eoat_buffer_mtxs.emplace_back();

      _eoat_waypoints_buffers.push_back(RUT::DataBuffer<Eigen::VectorXd>());
      _eoat_waypoints_timestamp_ms_buffers.push_back(RUT::DataBuffer<double>());
      _eoat_waypoints_buffers[id].initialize(
          -1, 2, 1, "eoat_waypoints" + std::to_string(id));
      _eoat_waypoints_timestamp_ms_buffers[id].initialize(
          -1, 1, 1, "eoat_waypoints" + std::to_string(id) + "_timestamp_ms");
      _eoat_waypoints_buffer_mtxs.emplace_back();
    }
  }

  // robot
  if (_config.run_robot_thread) {
    for (int id : _id_list) {
      _states_robot_thread_ready.push_back(false);
      _states_robot_thread_saving.push_back(false);
      _states_robot_seq_id.push_back(0);

      _pose_buffers.push_back(RUT::DataBuffer<Eigen::VectorXd>());
      _vel_buffers.push_back(RUT::DataBuffer<Eigen::VectorXd>());
      _robot_wrench_buffers.push_back(RUT::DataBuffer<Eigen::VectorXd>());
      _pose_timestamp_ms_buffers.push_back(RUT::DataBuffer<double>());
      _vel_timestamp_ms_buffers.push_back(RUT::DataBuffer<double>());
      _robot_wrench_timestamp_ms_buffers.push_back(RUT::DataBuffer<double>());

      _pose_buffers[id].initialize(_config.robot_buffer_size, 7, 1,
                                   "pose" + std::to_string(id));
      _vel_buffers[id].initialize(_config.robot_buffer_size, 6, 1,
                                  "vel" + std::to_string(id));
      _robot_wrench_buffers[id].initialize(_config.robot_buffer_size, 6, 1,
                                           "robot_wrench" + std::to_string(id));
      _pose_timestamp_ms_buffers[id].initialize(
          _config.robot_buffer_size, 1, 1,
          "pose" + std::to_string(id) + "_timestamp_ms");
      _vel_timestamp_ms_buffers[id].initialize(
          _config.robot_buffer_size, 1, 1,
          "vel" + std::to_string(id) + "_timestamp_ms");
      _robot_wrench_timestamp_ms_buffers[id].initialize(
          _config.robot_buffer_size, 1, 1,
          "robot_wrench" + std::to_string(id) + "_timestamp_ms");

      _pose_buffer_mtxs.emplace_back();
      _vel_buffer_mtxs.emplace_back();
      _robot_wrench_buffer_mtxs.emplace_back();

      _waypoints_buffers.push_back(RUT::DataBuffer<Eigen::VectorXd>());
      _stiffness_buffers.push_back(RUT::DataBuffer<Eigen::MatrixXd>());
      _waypoints_timestamp_ms_buffers.push_back(RUT::DataBuffer<double>());
      _stiffness_timestamp_ms_buffers.push_back(RUT::DataBuffer<double>());
      _waypoints_buffers[id].initialize(-1, 7, 1,
                                        "waypoints" + std::to_string(id));
      _stiffness_buffers[id].initialize(-1, 6, 6,
                                        "stiffness" + std::to_string(id));
      _waypoints_timestamp_ms_buffers[id].initialize(
          -1, 1, 1, "waypoints" + std::to_string(id) + "_timestamp_ms");
      _stiffness_timestamp_ms_buffers[id].initialize(
          -1, 1, 1, "stiffness" + std::to_string(id) + "_timestamp_ms");
      _waypoints_buffer_mtxs.emplace_back();
      _stiffness_buffer_mtxs.emplace_back();
    }
  }

  // initialize additional shared variables
  std::cout << "[ManipServer] Initializing shared variables.\n";
  for (int id : _id_list) {
    _ctrl_robot_data_streams.push_back(std::ofstream());
    _ctrl_eoat_data_streams.push_back(std::ofstream());
    _ctrl_wrench_data_streams.push_back(std::ofstream());
    _poses_fb.push_back(Eigen::VectorXd());
    _poses_fb_mtxs.emplace_back();
    _perturbation.push_back(Eigen::VectorXd::Zero(6));
    _perturbation_mtxs.emplace_back();
    _wrench_fb.push_back(Eigen::VectorXd::Zero(12));
    _wrench_fb_mtxs.emplace_back();
    _pose_timestamps_ms.push_back(Eigen::VectorXd());
    _vel_timestamps_ms.push_back(Eigen::VectorXd());
    _eoat_timestamps_ms.push_back(Eigen::VectorXd());
    _wrench_timestamps_ms.push_back(Eigen::VectorXd());
    _wrench_filtered_timestamps_ms.push_back(Eigen::VectorXd());
    _robot_wrench_timestamps_ms.push_back(Eigen::VectorXd());
  }
  // MAA: camera shared variables iterate over _cam_id_list
  for (int cam_id : _cam_id_list) {
    _ctrl_rgb_folders.push_back("");
    _color_mats.push_back(cv::Mat());
    _color_mat_mtxs.emplace_back();
    _camera_rgb_timestamps_ms.push_back(Eigen::VectorXd());
  }
  _key_delayed_timer.tic();

  // kickoff the threads
  _ctrl_flag_running = true;
  std::cout << "[ManipServer] Starting the threads.\n";

  // MAA: rgb threads spawn one per camera
  if (_config.run_rgb_thread) {
    for (int cam_id : _cam_id_list) {
      _rgb_threads.emplace_back(&ManipServer::rgb_loop, this, std::ref(time0),
                                cam_id);
      std::cout << "[ManipServer] Started rgb thread for camera id " << cam_id
                << ".\n";
    }
  }
  for (int id : _id_list) {
    if (_config.run_wrench_thread) {
      _wrench_threads.emplace_back(&ManipServer::wrench_loop, this,
                                   std::ref(time0), wrench_publish_rate[id],
                                   id);
      std::cout << "[ManipServer] Started wrench thread for id " << id << ".\n";
    }
    if (_config.run_robot_thread) {
      _robot_threads.emplace_back(&ManipServer::robot_loop, this,
                                  std::ref(time0), id);
      std::cout << "[ManipServer] Started robot thread for id " << id << ".\n";
    }
    if (_config.run_eoat_thread) {
      _eoat_threads.emplace_back(&ManipServer::eoat_loop, this,
                                 std::ref(time0), id);
      std::cout << "[ManipServer] Started eoat thread for id " << id << ".\n";
    }
  }
  if (_config.plot_rgb) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    _rgb_plot_thread = std::thread(&ManipServer::rgb_plot_loop, this);
  }
  if (_config.run_key_thread) {
    _key_thread = std::thread(&ManipServer::key_loop, this, std::ref(time0));
  }
  if (_config.run_teleop_thread) {  // JY: init Touch device and start teleop loop
    _touch_config.deserialize(config["touch"]);
    _touch_ptr = std::make_shared<Touch>();
    if (!_touch_ptr->init(time0, _touch_config)) {
      std::cerr << "[ManipServer] Failed to initialize Touch device. Exiting."
                << std::endl;
      return false;
    }
    _teleop_thread = std::thread(&ManipServer::teleop_loop, this, std::ref(time0));
    std::cout << "[ManipServer] Started teleop thread.\n";
  }

  // wait for threads to be ready
  std::cout << "[ManipServer] Waiting for threads to be ready.\n";
  RUT::Timer timeout_timer;
  timeout_timer.tic();
  while (true) {
    bool all_ready = true;
    {
      std::lock_guard<std::mutex> lock(_ctrl_mtx);
      // MAA: check rgb readiness per camera
      if (_config.run_rgb_thread) {
        for (int cam_id : _cam_id_list) {
          all_ready = all_ready && _states_rgb_thread_ready[cam_id];
        }
      }
      for (int id : _id_list) {
        if (_config.run_wrench_thread) {
          all_ready = all_ready && _states_wrench_thread_ready[id];
        }
        if (_config.run_robot_thread) {
          all_ready = all_ready && _states_robot_thread_ready[id];
        }
        if (_config.run_eoat_thread) {
          all_ready = all_ready && _states_eoat_thread_ready[id];
        }
      }
      if (_config.plot_rgb) {
        all_ready = all_ready && _state_plot_thread_ready;
      }
      if (_config.run_key_thread) {
        all_ready = all_ready && _state_key_thread_ready;
      }
      if (_config.run_teleop_thread) {  // JY
        all_ready = all_ready && _state_teleop_thread_ready;
      }
    }
    if (all_ready) {
      break;
    }
    if (timeout_timer.toc_ms() > 20000) {
      std::cerr << "\033[1;31m"
                << "[ManipServer] Timeout waiting for threads to be ready."
                << "\033[0m" << std::endl;
      exit(1);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  std::cout << "[ManipServer] All threads are ready." << std::endl;
  std::cout << "[ManipServer] Done initialization." << std::endl;
  return true;
}

void ManipServer::join_threads() {
  std::cout << "[ManipServer]: Waiting for threads to join." << std::endl;
  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _ctrl_flag_running = false;
  }

  if (_config.run_rgb_thread) {
    std::cout << "[ManipServer]: Waiting for rgb threads to join." << std::endl;
    for (auto& rgb_thread : _rgb_threads) {
      rgb_thread.join();
    }
  }
  if (_config.run_wrench_thread) {
    std::cout << "[ManipServer]: Waiting for wrench threads to join."
              << std::endl;
    for (auto& wrench_thread : _wrench_threads) {
      wrench_thread.join();
    }
  }
  if (_config.run_robot_thread) {
    std::cout << "[ManipServer]: Waiting for robot threads to join."
              << std::endl;
    for (auto& robot_thread : _robot_threads) {
      robot_thread.join();
    }
  }
  if (_config.run_eoat_thread) {
    std::cout << "[ManipServer]: Waiting for eoat threads to join."
              << std::endl;
    for (auto& eoat_thread : _eoat_threads) {
      eoat_thread.join();
    }
  }
  if (_config.plot_rgb) {
    std::cout << "[ManipServer]: Waiting for plotting thread to join."
              << std::endl;
    _rgb_plot_thread.join();
  }
  if (_config.run_key_thread) {
    std::cout << "[ManipServer]: Waiting for key thread to join." << std::endl;
    _key_thread.join();
  }
  if (_config.run_teleop_thread) {  // JY
    std::cout << "[ManipServer]: Waiting for teleop thread to join." << std::endl;
    _teleop_thread.join();
  }

  std::cout << "[ManipServer]: Threads have joined. Exiting." << std::endl;
}

bool ManipServer::is_ready() {
  // MAA: camera readiness check per camera
  if (_config.run_rgb_thread) {
    for (int cam_id : _cam_id_list) {
      std::lock_guard<std::mutex> lock(_camera_rgb_buffer_mtxs[cam_id]);
      if (!_camera_rgb_buffers[cam_id].is_full()) {
        std::cout << cam_id << ": Camera RGB buffer not full: size: "
                  << _camera_rgb_buffers[cam_id].size() << std::endl;
        return false;
      }
    }
  }

  for (int id : _id_list) {
    if (_config.run_robot_thread) {
      std::lock_guard<std::mutex> lock(_pose_buffer_mtxs[id]);
      if (!_pose_buffers[id].is_full()) {
        std::cout << id << ": Pose buffer not full: size: "
                  << _pose_buffers[id].size() << std::endl;
        return false;
      }
    }

    if (_config.run_eoat_thread) {
      std::lock_guard<std::mutex> lock(_eoat_buffer_mtxs[id]);
      if (!_eoat_buffers[id].is_full()) {
        std::cout << id << ": EoAT buffer not full: size: "
                  << _eoat_buffers[id].size() << std::endl;
        return false;
      }
    }

    if (_config.run_wrench_thread) {
      std::lock_guard<std::mutex> lock(_wrench_buffer_mtxs[id]);
      if (!_wrench_buffers[id].is_full()) {
        std::cout << id << ": wrench buffer not full: size: "
                  << _wrench_buffers[id].size() << std::endl;
        return false;
      }
    }
  }
  return true;
}

bool ManipServer::is_running() {
  std::lock_guard<std::mutex> lock(_ctrl_mtx);
  return _ctrl_flag_running;
}

bool ManipServer::is_teleop_active() {  // JY
  std::lock_guard<std::mutex> lock(_ctrl_mtx);
  return _teleop_active;
}

const Eigen::MatrixXd ManipServer::get_camera_rgb(int k, int id) {
  try {
    std::lock_guard<std::mutex> lock(_camera_rgb_buffer_mtxs[id]);
    _camera_rgb_timestamps_ms[id] =
        _camera_rgb_timestamp_ms_buffers[id].get_last_k(k);
    return _camera_rgb_buffers[id].get_last_k(k);
  } catch (const std::exception& e) {
    std::cerr << "[ManipServer] Failed to get camera rgb: " << e.what()
              << std::endl;
    return Eigen::MatrixXd{};
  }
}

const Eigen::MatrixXd ManipServer::get_wrench(int k, int id) {
  try {
    std::lock_guard<std::mutex> lock(_wrench_buffer_mtxs[id]);
    _wrench_timestamps_ms[id] = _wrench_timestamp_ms_buffers[id].get_last_k(k);
    return _wrench_buffers[id].get_last_k(k);
  } catch (const std::exception& e) {
    std::cerr << "[ManipServer] Failed to get wrench: " << e.what()
              << std::endl;
    return Eigen::MatrixXd{};
  }
}

const Eigen::MatrixXd ManipServer::get_wrench_filtered(int k, int id) {
  try {
    std::lock_guard<std::mutex> lock(_wrench_filtered_buffer_mtxs[id]);
    _wrench_filtered_timestamps_ms[id] =
        _wrench_filtered_timestamp_ms_buffers[id].get_last_k(k);
    return _wrench_filtered_buffers[id].get_last_k(k);
  } catch (const std::exception& e) {
    std::cerr << "[ManipServer] Failed to get filtered wrench: " << e.what()
              << std::endl;
    return Eigen::MatrixXd{};
  }
}

const Eigen::MatrixXd ManipServer::get_robot_wrench(int k, int id) {
  try {
    std::lock_guard<std::mutex> lock(_robot_wrench_buffer_mtxs[id]);
    _robot_wrench_timestamps_ms[id] =
        _robot_wrench_timestamp_ms_buffers[id].get_last_k(k);
    return _robot_wrench_buffers[id].get_last_k(k);
  } catch (const std::exception& e) {
    std::cerr << "[ManipServer] Failed to get robot wrench: " << e.what()
              << std::endl;
    return Eigen::MatrixXd{};
  }
}

const Eigen::MatrixXd ManipServer::get_pose(int k, int id) {
  try {
    std::lock_guard<std::mutex> lock(_pose_buffer_mtxs[id]);
    _pose_timestamps_ms[id] = _pose_timestamp_ms_buffers[id].get_last_k(k);
    return _pose_buffers[id].get_last_k(k);
  } catch (const std::exception& e) {
    std::cerr << "[ManipServer] Failed to get pose: " << e.what() << std::endl;
    return Eigen::MatrixXd{};
  }
}

const Eigen::MatrixXd ManipServer::get_vel(int k, int id) {
  try {
    std::lock_guard<std::mutex> lock(_vel_buffer_mtxs[id]);
    _vel_timestamps_ms[id] = _vel_timestamp_ms_buffers[id].get_last_k(k);
    return _vel_buffers[id].get_last_k(k);
  } catch (const std::exception& e) {
    std::cerr << "[ManipServer] Failed to get vel: " << e.what() << std::endl;
    return Eigen::MatrixXd{};
  }
}

const Eigen::MatrixXd ManipServer::get_eoat(int k, int id) {
  try {
    std::lock_guard<std::mutex> lock(_eoat_buffer_mtxs[id]);
    _eoat_timestamps_ms[id] = _eoat_timestamp_ms_buffers[id].get_last_k(k);
    return _eoat_buffers[id].get_last_k(k);
  } catch (const std::exception& e) {
    std::cerr << "[ManipServer] Failed to get eoat: " << e.what() << std::endl;
    return Eigen::MatrixXd{};
  }
}

const int ManipServer::get_test() {
  _test_timestamp_ms = _timer.toc_ms();
  return 0;
}

const Eigen::VectorXd ManipServer::get_camera_rgb_timestamps_ms(int id) {
  return _camera_rgb_timestamps_ms[id];
}
const Eigen::VectorXd ManipServer::get_wrench_timestamps_ms(int id) {
  return _wrench_timestamps_ms[id];
}
const Eigen::VectorXd ManipServer::get_wrench_filtered_timestamps_ms(int id) {
  return _wrench_filtered_timestamps_ms[id];
}
const Eigen::VectorXd ManipServer::get_robot_wrench_timestamps_ms(int id) {
  return _robot_wrench_timestamps_ms[id];
}
const Eigen::VectorXd ManipServer::get_pose_timestamps_ms(int id) {
  return _pose_timestamps_ms[id];
}
const Eigen::VectorXd ManipServer::get_vel_timestamps_ms(int id) {
  return _vel_timestamps_ms[id];
}
const Eigen::VectorXd ManipServer::get_eoat_timestamps_ms(int id) {
  return _eoat_timestamps_ms[id];
}

const double ManipServer::get_test_timestamp_ms() {
  return _test_timestamp_ms;
}

double ManipServer::get_timestamp_now_ms() {
  return _timer.toc_ms();
}

void ManipServer::set_high_level_maintain_position() {
  if (!_config.run_robot_thread) {
    return;
  }
  clear_cmd_buffer();

  RUT::Vector7d pose_fb;
  for (int id : _id_list) {
    if (!_config.mock_hardware) {
      robot_ptrs[id]->getCartesian(pose_fb);
    }
    set_target_pose(pose_fb, 200, id);
    set_target_pose(pose_fb, 1000, id);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  for (int id : _id_list) {
    std::lock_guard<std::mutex> lock(_controller_mtxs[id]);
    _controllers[id].setStiffnessMatrix(_stiffnesses_high[id]);
    _controllers[id].setDampingMatrix(_dampings_high[id]);
  }
}

void ManipServer::set_high_level_free_jogging() {
  if (!_config.run_robot_thread) {
    return;
  }
  for (int id : _id_list) {
    std::lock_guard<std::mutex> lock(_controller_mtxs[id]);
    _controllers[id].setStiffnessMatrix(_stiffnesses_low[id]);
    _controllers[id].setDampingMatrix(_dampings_low[id]);
  }
}

void ManipServer::calibrate_robot_wrench(int NSamples) {
  if (!_config.run_robot_thread) {
    return;
  }
  for (int id : _id_list) {
    URRTDE* urrtde_ptr;
    urrtde_ptr = static_cast<URRTDE*>(robot_ptrs[id].get());
    urrtde_ptr->calibrateFTSensor(NSamples);
  }
}

void ManipServer::set_target_pose(const Eigen::Ref<RUT::Vector7d> pose,
                                  double dt_in_future_ms, int robot_id) {
  std::lock_guard<std::mutex> lock(_waypoints_buffer_mtxs[robot_id]);
  _waypoints_buffers[robot_id].put(pose);
  _waypoints_timestamp_ms_buffers[robot_id].put(
      _timer.toc_ms() + dt_in_future_ms);
}

void ManipServer::set_force_controlled_axis(const RUT::Matrix6d& Tr, int n_af,
                                            int robot_id) {
  std::lock_guard<std::mutex> lock(_controller_mtxs[robot_id]);
  _controllers[robot_id].setForceControlledAxis(Tr, n_af);
}

void ManipServer::set_stiffness_matrix(const RUT::Matrix6d& stiffness,
                                       int robot_id) {
  std::lock_guard<std::mutex> lock(_controller_mtxs[robot_id]);
  _controllers[robot_id].setStiffnessMatrix(stiffness);
}

void ManipServer::set_damping_matrix(const RUT::Matrix6d& damping,
  int robot_id) {
std::lock_guard<std::mutex> lock(_controller_mtxs[robot_id]);
_controllers[robot_id].setDampingMatrix(damping);
} // newly added based on old code JY

void ManipServer::clear_cmd_buffer() {
  if (_config.run_robot_thread) {
    for (int id : _id_list) {
      std::lock_guard<std::mutex> lock(_waypoints_buffer_mtxs[id]);
      _waypoints_buffers[id].clear();
      _waypoints_timestamp_ms_buffers[id].clear();
    }
    for (int id : _id_list) {
      std::lock_guard<std::mutex> lock(_stiffness_buffer_mtxs[id]);
      _stiffness_buffers[id].clear();
      _stiffness_timestamp_ms_buffers[id].clear();
    }
  }
  if (_config.run_eoat_thread) {
    for (int id : _id_list) {
      std::lock_guard<std::mutex> lock(_eoat_waypoints_buffer_mtxs[id]);
      _eoat_waypoints_buffers[id].clear();
      _eoat_waypoints_timestamp_ms_buffers[id].clear();
    }
  }
}

void ManipServer::start_listening_key_events() {
  if (_config.run_key_thread) {
    std::lock_guard<std::mutex> lock(_ctrl_key_mtx);
    _ctrl_listen_key_event = true;
  }
}

void ManipServer::stop_listening_key_events() {
  if (_config.run_key_thread) {
    std::lock_guard<std::mutex> lock(_ctrl_key_mtx);
    _ctrl_listen_key_event = false;
  }
}

void ManipServer::schedule_waypoints(const Eigen::MatrixXd& waypoints,
                                     const Eigen::VectorXd& timepoints_ms,
                                     int robot_id) {
  if (_config.take_over_mode) {
    std::lock_guard<std::mutex> lock(_key_mtx);
    if (_key_is_pressed_delayed > 0)
      return;
  }

  double curr_time = _timer.toc_ms();
  if (waypoints.rows() != 7) {
    std::cerr << "[ManipServer][schedule_waypoints] Waypoints should have 7 "
                 "rows. Exiting."
              << std::endl;
    return;
  }
  if (timepoints_ms.size() != waypoints.cols()) {
    std::cerr << "[ManipServer][schedule_waypoints] Waypoints and "
                 "timepoints_ms should have the same "
                 "number of columns. Exiting."
              << std::endl;
    return;
  }

  int input_id_start = 0;
  for (int i = 0; i < timepoints_ms.size(); i++) {
    if (timepoints_ms(i) > curr_time) {
      input_id_start = i;
      break;
    }
  }
  if (input_id_start >= timepoints_ms.size()) {
    std::cerr << "[debug][schedule_waypoints] all input points are in the past."
              << std::endl;
    return;
  }

  {
    std::lock_guard<std::mutex> lock(_waypoints_buffer_mtxs[robot_id]);
    int existing_id_end = 0;
    for (int i = 0; i < _waypoints_timestamp_ms_buffers[robot_id].size(); i++) {
      if (_waypoints_timestamp_ms_buffers[robot_id][i] >
          timepoints_ms(input_id_start)) {
        existing_id_end = i;
        break;
      }
    }
    _waypoints_buffers[robot_id].remove_last_k(
        _waypoints_buffers[robot_id].size() - existing_id_end);
    _waypoints_timestamp_ms_buffers[robot_id].remove_last_k(
        _waypoints_timestamp_ms_buffers[robot_id].size() - existing_id_end);
    assert(_waypoints_buffers[robot_id].size() ==
           _waypoints_timestamp_ms_buffers[robot_id].size());

    int input_id_end = timepoints_ms.size();
    for (int i = input_id_start; i < input_id_end; i++) {
      _waypoints_buffers[robot_id].put(waypoints.col(i));
      _waypoints_timestamp_ms_buffers[robot_id].put(timepoints_ms(i));
    }
  }
}

void ManipServer::schedule_eoat_waypoints(
    const Eigen::MatrixXd& eoat_waypoints,
    const Eigen::VectorXd& timepoints_ms, int robot_id) {
  if (_config.take_over_mode) {
    std::lock_guard<std::mutex> lock(_key_mtx);
    if (_key_is_pressed_delayed > 0)
      return;
  }

  double curr_time = _timer.toc_ms();
  if (eoat_waypoints.rows() != 2) {
    std::cerr
        << "[ManipServer][schedule_eoat_waypoints] Waypoints should have 2 "
           "rows. Exiting."
        << std::endl;
    return;
  }
  if (timepoints_ms.size() != eoat_waypoints.cols()) {
    std::cerr << "[ManipServer][schedule_eoat_waypoints] Waypoints and "
                 "timepoints_ms should have the same number of columns. Exiting."
              << std::endl;
    return;
  }

  int input_id_start = 0;
  for (int i = 0; i < timepoints_ms.size(); i++) {
    if (timepoints_ms(i) > curr_time) {
      input_id_start = i;
      break;
    }
  }
  if (input_id_start >= timepoints_ms.size()) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(_eoat_waypoints_buffer_mtxs[robot_id]);
    int existing_id_end = 0;
    for (int i = 0;
         i < _eoat_waypoints_timestamp_ms_buffers[robot_id].size(); i++) {
      if (_eoat_waypoints_timestamp_ms_buffers[robot_id][i] >
          timepoints_ms(input_id_start)) {
        existing_id_end = i;
        break;
      }
    }
    _eoat_waypoints_buffers[robot_id].remove_last_k(
        _eoat_waypoints_buffers[robot_id].size() - existing_id_end);
    _eoat_waypoints_timestamp_ms_buffers[robot_id].remove_last_k(
        _eoat_waypoints_timestamp_ms_buffers[robot_id].size() - existing_id_end);
    assert(_eoat_waypoints_buffers[robot_id].size() ==
           _eoat_waypoints_timestamp_ms_buffers[robot_id].size());

    int input_id_end = timepoints_ms.size();
    for (int i = input_id_start; i < input_id_end; i++) {
      _eoat_waypoints_buffers[robot_id].put(eoat_waypoints.col(i));
      _eoat_waypoints_timestamp_ms_buffers[robot_id].put(timepoints_ms(i));
    }
  }
}

void ManipServer::schedule_stiffness(const Eigen::MatrixXd& stiffnesses,
                                     const Eigen::VectorXd& timepoints_ms,
                                     int robot_id) {
  if (_config.take_over_mode) {
    std::lock_guard<std::mutex> lock(_key_mtx);
    if (_key_is_pressed_delayed > 0)
      return;
  }

  double curr_time = _timer.toc_ms();
  if (stiffnesses.rows() != 6) {
    std::cerr << "[ManipServer][schedule_stiffness] stiffnesses should have 6 "
                 "rows. Exiting."
              << std::endl;
    return;
  }
  if (stiffnesses.cols() / timepoints_ms.size() != 6) {
    std::cerr << "[ManipServer][schedule_stiffness] stiffnesses should have "
                 "6x number of columns as timepoints_ms. Exiting."
              << std::endl;
    return;
  }

  int input_id_start = 0;
  for (int i = 0; i < timepoints_ms.size(); i++) {
    if (timepoints_ms(i) > curr_time) {
      input_id_start = i;
      break;
    }
  }
  if (input_id_start >= timepoints_ms.size()) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(_stiffness_buffer_mtxs[robot_id]);
    int existing_id_end = 0;
    for (int i = 0; i < _stiffness_timestamp_ms_buffers[robot_id].size(); i++) {
      if (_stiffness_timestamp_ms_buffers[robot_id][i] >
          timepoints_ms(input_id_start)) {
        existing_id_end = i;
        break;
      }
    }
    _stiffness_buffers[robot_id].remove_last_k(
        _stiffness_buffers[robot_id].size() - existing_id_end);
    _stiffness_timestamp_ms_buffers[robot_id].remove_last_k(
        _stiffness_timestamp_ms_buffers[robot_id].size() - existing_id_end);
    assert(_stiffness_buffers[robot_id].size() ==
           _stiffness_timestamp_ms_buffers[robot_id].size());

    int input_id_end = timepoints_ms.size();
    for (int i = input_id_start; i < input_id_end; i++) {
      _stiffness_buffers[robot_id].put(stiffnesses.middleCols<6>(i * 6));
      _stiffness_timestamp_ms_buffers[robot_id].put(timepoints_ms(i));
    }
  }
}

void ManipServer::start_saving_data_for_a_new_episode(
    const std::string& data_folder) {
  std::string data_folder_to_use;
  if (_config.data_folder.empty()) {
    assert(!data_folder.empty() &&
           "[ManipServer] When the default data_folder is empty, you must "
           "specify a data_folder as input.");
    data_folder_to_use = data_folder;
  } else {
    assert(data_folder.empty() &&
           "[ManipServer] When the default data_folder is not empty, you must "
           "NOT specify a data_folder as input.");
    data_folder_to_use = _config.data_folder;
  }

  std::vector<std::string> robot_json_file_names;
  std::vector<std::string> eoat_json_file_names;
  std::vector<std::string> wrench_json_file_names;
  std::string key_json_file_name;
  std::string teleop_json_file_name;  // JY
  // MAA: use multicam version of folder creation
  _episode_folder = create_folder_for_new_episode_multicam(
      data_folder_to_use, _id_list, _cam_id_list, _ctrl_rgb_folders,
      robot_json_file_names, eoat_json_file_names, wrench_json_file_names,
      key_json_file_name, teleop_json_file_name);  // JY
  std::cout << "[main] New episode. rgb_folder_name: " << _ctrl_rgb_folders[0]
            << std::endl;

  for (int id : _id_list) {
    if (_config.run_robot_thread)  _ctrl_robot_data_streams[id].open(robot_json_file_names[id]);  // JY
    if (_config.run_eoat_thread)   _ctrl_eoat_data_streams[id].open(eoat_json_file_names[id]);    // JY
    if (_config.run_wrench_thread) _ctrl_wrench_data_streams[id].open(wrench_json_file_names[id]); // JY
  }
  if (_config.run_key_thread)    _ctrl_key_data_stream.open(key_json_file_name);            // JY
  if (_config.run_teleop_thread) _ctrl_teleop_data_stream.open(teleop_json_file_name);      // JY

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _ctrl_flag_saving = true;
  }
}

void ManipServer::stop_saving_data() {
  std::lock_guard<std::mutex> lock(_ctrl_mtx);
  _ctrl_flag_saving = false;
}

bool ManipServer::is_saving_data() {
  bool is_saving = false;
  // MAA: rgb saving check per camera
  if (_config.run_rgb_thread) {
    for (int cam_id : _cam_id_list) {
      is_saving = is_saving || _states_rgb_thread_saving[cam_id];
    }
  }
  for (int id : _id_list) {
    if (_config.run_robot_thread) {
      is_saving = is_saving || _states_robot_thread_saving[id];
    }
    if (_config.run_eoat_thread) {
      is_saving = is_saving || _states_eoat_thread_saving[id];
    }
    if (_config.run_wrench_thread) {
      is_saving = is_saving || _states_wrench_thread_saving[id];
    }
  }
  if (_config.run_key_thread) {
    is_saving = is_saving || _state_key_thread_saving;
  }
  if (_config.run_teleop_thread) {  // JY
    is_saving = is_saving || _state_teleop_thread_saving;
  }
  return is_saving;
}

std::string ManipServer::get_episode_folder() const {
  return _episode_folder;
}

void ManipServer::request_teleop_start() {  // JY
  std::lock_guard<std::mutex> lock(_ctrl_mtx);
  _teleop_start_requested = true;
}

bool ManipServer::is_amplify_mode() {  // JY
  std::lock_guard<std::mutex> lock(_ctrl_mtx);
  return _amplify_mode;
}

bool ManipServer::is_idle_mode() {  // JY
  std::lock_guard<std::mutex> lock(_ctrl_mtx);
  return _idle_mode;
}