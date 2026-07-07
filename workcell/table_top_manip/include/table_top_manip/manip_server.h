#pragma once

#include <unistd.h>
#include <csignal>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <unsupported/Eigen/CXX11/Tensor>

#include <RobotUtilities/butterworth.h>
#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>

#include <Leptrino_ft.h>  //MAA
#include <Leptrino_driver.h>
#include <touch/touch.h>  // JY

#include <force_control/admittance_controller.h>
#include <force_control/config_deserialize.h>
#include <hardware_interfaces/js_interfaces.h>
#include <hardware_interfaces/robot_interfaces.h>
#include <hardware_interfaces/types.h>
// hardware used in this app
#include <ati_netft/ati_netft.h>
// #include <coinft/coin_ft.h>  // disabled: onnxruntime not installed
#include <gopro/gopro.h>
// #include <oak/oak.h>  // disabled: OAK library not built
#include <realsense/realsense.h>
#include <robotiq_ft_modbus/robotiq_ft_modbus.h>
#include <table_top_manip/perturbation_generator.h>
#include <ur_rtde/ur_rtde.h>
#include <wsg_gripper/wsg_gripper.h>

#include <RobotUtilities/data_buffer.h>

struct ManipServerConfig {
  std::string data_folder{""};
  bool run_robot_thread{false};
  bool run_eoat_thread{false};
  bool run_wrench_thread{false};
  bool run_rgb_thread{false};
  bool run_key_thread{false};
  bool run_teleop_thread{false};  // JY: also controls Touch device init
  // JY: haptic force feedback parameters (ported from MAA_data_collection_v2.py)
  bool   haptic_filtering_enabled{false}; // JY: gates deadband/ramp/IIR/slew; kept for reference, disabled by default
  double haptic_deadband{0.30};       // N, per-axis deadband to prevent buzzing
  double haptic_contact_th{0.60};     // N, contact threshold — also freezes spring equilibrium above this
  double haptic_ramp_up_time{0.15};   // s, soft-start time constant
  double haptic_ramp_down_time{0.70}; // s, soft-stop time constant
  double haptic_iir_cutoff_hz{18.0};  // Hz, 1st-order IIR lowpass cutoff
  double haptic_z_multiplier{1.5};    // dimensionless, extra Z-axis scale in remap
  double haptic_f_max{3.0};           // N, per-axis hard saturation on spring-damper output
  // JY: gravity vector used to cancel orientation-induced wrench drift after tare
  Eigen::Vector3d haptic_gravity{0.0, 0.0, 0.0};
  // JY: force-dependent spring-damper gains (replace fixed haptic_stiffness/damping in TouchConfig)
  double haptic_k_per_N{0.02};        // JY: N/mm per N of contact force (stiffness gain)
  double haptic_b_per_N{0.0002};      // JY: N·s/mm per N of contact force (damping gain)
  double haptic_k_slew_rate{0.5};     // JY: N/mm/s max rise rate for stiffness (limits abrupt jump on contact)
  bool plot_rgb{false};
  int rgb_buffer_size{5};
  int robot_buffer_size{100};
  int eoat_buffer_size{100};
  int wrench_buffer_size{100};
  bool mock_hardware{false};
  bool bimanual{false};
  int cam_count{1}; //MAA: added this
  bool use_perturbation_generator{false};
  CameraSelection camera_selection{CameraSelection::NONE};
  ForceSensingMode force_sensing_mode{ForceSensingMode::NONE};
  ComplianceControlForceSource compliance_control_force_source{
      ComplianceControlForceSource::NONE};
  RUT::Matrix6d low_damping{};
  std::vector<int> output_rgb_hw{};
  std::vector<double>
      wrench_filter_parameters{};  // cutoff frequency, sampling time, order
  bool check_robot_loop_overrun{false};
  std::string key_event_device{"/dev/input/event0"};
  std::vector<int> keys_to_monitor{};
  bool take_over_mode{
      false};  // let human to take over via kinethetic teaching when key is pressed
  RUT::Vector7d pose7_offset_TC{
      0, 0, 0, 1, 0, 0, 0};  // Toolframe offset for compliance control

  bool deserialize(const YAML::Node& node) {
    try {
      data_folder = node["data_folder"].as<std::string>();
      run_robot_thread = node["run_robot_thread"].as<bool>();
      run_eoat_thread = node["run_eoat_thread"].as<bool>();
      run_wrench_thread = node["run_wrench_thread"].as<bool>();
      run_rgb_thread = node["run_rgb_thread"].as<bool>();
      run_key_thread = node["run_key_thread"].as<bool>();
      plot_rgb = node["plot_rgb"].as<bool>();
      rgb_buffer_size = node["rgb_buffer_size"].as<int>();
      robot_buffer_size = node["robot_buffer_size"].as<int>();
      eoat_buffer_size = node["eoat_buffer_size"].as<int>();
      wrench_buffer_size = node["wrench_buffer_size"].as<int>();
      mock_hardware = node["mock_hardware"].as<bool>();
      bimanual = node["bimanual"].as<bool>();
      cam_count = node["cam_count"].as<int>(); //MAA: added this
      use_perturbation_generator =
          node["use_perturbation_generator"].as<bool>();
      camera_selection = string_to_enum<CameraSelection>(
          node["camera_selection"].as<std::string>());
      force_sensing_mode = string_to_enum<ForceSensingMode>(
          node["force_sensing_mode"].as<std::string>());
      compliance_control_force_source =
          string_to_enum<ComplianceControlForceSource>(
              node["compliance_control_force_source"].as<std::string>());

      low_damping = RUT::deserialize_vector<RUT::Vector6d>(node["low_damping"])
                        .asDiagonal();
      output_rgb_hw = node["output_rgb_hw"].as<std::vector<int>>();
      wrench_filter_parameters =
          node["wrench_filter_parameters"].as<std::vector<double>>();
      check_robot_loop_overrun = node["check_robot_loop_overrun"].as<bool>();
      // optional parameters
      if (node["key_event_device"]) {
        key_event_device = node["key_event_device"].as<std::string>();
        keys_to_monitor = node["keys_to_monitor"].as<std::vector<int>>();
      }
      if (node["take_over_mode"]) {
        take_over_mode = node["take_over_mode"].as<bool>();
      }
      if (node["run_teleop_thread"]) {  // JY
        run_teleop_thread = node["run_teleop_thread"].as<bool>();
      }
      // JY: teleop_vel and haptic parameters now live under touch: in the YAML
      if (node["touch"]) {
        const YAML::Node& t = node["touch"];
        // JY: haptic force feedback parameters
        if (t["haptic_filtering_enabled"]) haptic_filtering_enabled = t["haptic_filtering_enabled"].as<bool>();
        if (t["haptic_deadband"])       haptic_deadband       = t["haptic_deadband"].as<double>();
        if (t["haptic_contact_th"])     haptic_contact_th     = t["haptic_contact_th"].as<double>();
        if (t["haptic_ramp_up_time"])   haptic_ramp_up_time   = t["haptic_ramp_up_time"].as<double>();
        if (t["haptic_ramp_down_time"]) haptic_ramp_down_time = t["haptic_ramp_down_time"].as<double>();
        if (t["haptic_iir_cutoff_hz"])  haptic_iir_cutoff_hz  = t["haptic_iir_cutoff_hz"].as<double>();
        if (t["haptic_z_multiplier"])   haptic_z_multiplier   = t["haptic_z_multiplier"].as<double>();
        if (t["haptic_f_max"])          haptic_f_max          = t["haptic_f_max"].as<double>();
        if (t["haptic_gravity"])  // JY: gravity vector for haptic drift correction
          haptic_gravity = RUT::deserialize_vector<Eigen::Vector3d>(t["haptic_gravity"]);
        if (t["haptic_k_per_N"])       haptic_k_per_N       = t["haptic_k_per_N"].as<double>();       // JY: stiffness gain
        if (t["haptic_b_per_N"])       haptic_b_per_N       = t["haptic_b_per_N"].as<double>();       // JY: damping gain
        if (t["haptic_k_slew_rate"])   haptic_k_slew_rate   = t["haptic_k_slew_rate"].as<double>();   // JY: stiffness rise rate limit
      }
      if (node["pose7_offset_TC"]) {
        pose7_offset_TC =
            RUT::deserialize_vector<RUT::Vector7d>(node["pose7_offset_TC"]);
      }
    } catch (const std::exception& e) {
      std::cerr << "Failed to load the config file: " << e.what() << std::endl;
      return false;
    }
    return true;
  }
};

/// @brief ManipServer class
/// This class is the main interface to the hardware. It initializes the hardware
/// interfaces, starts the threads to collect data, and provides the most recent
/// data points to the user.
/// Usage:
///   ManipServer server;
///   server.initialize("config.yaml");
///   // wait until the server is ready
///   while (!server.is_ready()) {
///     usleep(1000);
///   }
///   while (server.is_running()) {
///     Eigen::MatrixXd camera_rgb = server.get_camera_rgb(5); // get the most recent 5 data points
///     Eigen::MatrixXd pose = server.get_pose(5);
///     Eigen::MatrixXd wrench = server.get_wrench(5);
///     // get the timestamps of the most recently fetched data points
///     Eigen::VectorXd camera_rgb_timestamps = server.get_camera_rgb_timestamps_ms();
///     Eigen::VectorXd pose_timestamps = server.get_pose_timestamps_ms();
///     Eigen::VectorXd wrench_timestamps = server.get_wrench_timestamps_ms();
///     // do something with the data
///   }
///   server.join_threads();
class ManipServer {
 public:
  ManipServer(){};
  ManipServer(const std::string&);
  ~ManipServer();

  bool initialize(const std::string& config_file);
  void join_threads();
  bool is_ready();  // check if all buffers are full
  bool is_running();
  bool is_bimanual() { return _config.bimanual; }
  bool has_eoat() { return _config.run_eoat_thread; }

  // getters: get the most recent k data points in the buffer
  const Eigen::MatrixXd get_camera_rgb(int k, int camera_id = 0);
  const Eigen::MatrixXd get_wrench(int k, int sensor_id = 0);
  const Eigen::MatrixXd get_wrench_filtered(int k, int sensor_id = 0);
  const Eigen::MatrixXd get_robot_wrench(int k, int robot_id = 0);
  const Eigen::MatrixXd get_pose(int k, int robot_id = 0);
  const Eigen::MatrixXd get_vel(int k, int robot_id = 0);
  const Eigen::MatrixXd get_eoat(int k, int robot_id = 0);
  const int get_test();

  // the following functions return the timestamps of
  //  the most recent getter call of the corresponding feedback
  //  So size is already know
  const Eigen::VectorXd get_camera_rgb_timestamps_ms(int id = 0);
  const Eigen::VectorXd get_wrench_timestamps_ms(int id = 0);
  const Eigen::VectorXd get_wrench_filtered_timestamps_ms(int id = 0);
  const Eigen::VectorXd get_robot_wrench_timestamps_ms(int id = 0);
  const Eigen::VectorXd get_pose_timestamps_ms(int id = 0);
  const Eigen::VectorXd get_vel_timestamps_ms(int id = 0);
  const Eigen::VectorXd get_eoat_timestamps_ms(int id = 0);
  const double get_test_timestamp_ms();

  double get_timestamp_now_ms();  // access the current hardware time

  void set_high_level_maintain_position();
  void set_high_level_free_jogging();
  void calibrate_robot_wrench(int NSamples = 100);

  void set_target_pose(const Eigen::Ref<RUT::Vector7d> pose,
                       double dt_in_future_ms = 1000, int robot_id = 0);
  void set_force_controlled_axis(const RUT::Matrix6d& Tr, int n_af,
                                 int robot_id = 0);
  void set_stiffness_matrix(const RUT::Matrix6d& stiffness, int robot_id = 0);
  void set_damping_matrix(const RUT::Matrix6d& damping, int robot_id = 0);

  void schedule_waypoints(const Eigen::MatrixXd& waypoints,
                          const Eigen::VectorXd& timepoints_ms,
                          int robot_id = 0);
  void schedule_eoat_waypoints(const Eigen::MatrixXd& waypoints,
                               const Eigen::VectorXd& timepoints_ms,
                               int robot_id = 0);
  void schedule_stiffness(const Eigen::MatrixXd& stiffness,
                          const Eigen::VectorXd& timepoints_ms,
                          int robot_id = 0);

  void clear_cmd_buffer();

  void start_listening_key_events();
  void stop_listening_key_events();

  // data logging

  /***
   * @brief Start saving data for a new episode.
   * The default data folder path is specified in the config file.
   * In order to avoid confusion, the data folder in the config file must be empty if @p data_folder is specified.
   * @param data_folder The data folder path. If not specified, the default data folder path is used.
   */
  void start_saving_data_for_a_new_episode(const std::string& data_folder = "");
  void stop_saving_data();
  bool is_saving_data();
  std::string get_episode_folder() const;

  bool is_teleop_active();      // JY
  bool is_amplify_mode();       // JY
  bool is_idle_mode();          // JY
  void request_teleop_start();  // JY: called by main() on Enter key to start teleop

 private:
  // config
  ManipServerConfig _config;

  // additional configs as local variables
  std::vector<RUT::Matrix6d> _stiffnesses_high{};
  std::vector<RUT::Matrix6d> _stiffnesses_low{};
  std::vector<RUT::Matrix6d> _dampings_high{};
  std::vector<RUT::Matrix6d> _dampings_low{};
  std::vector<int> _num_ft_sensors{};  // number of force sensors

  // list of id
  std::vector<int> _id_list;
  std::vector<int> _cam_id_list; //MAA: added this

  // data buffers
  std::vector<RUT::DataBuffer<Eigen::MatrixXd>> _camera_rgb_buffers;
  std::vector<RUT::DataBuffer<Eigen::VectorXd>> _pose_buffers;
  std::vector<RUT::DataBuffer<Eigen::VectorXd>> _vel_buffers;
  std::vector<RUT::DataBuffer<Eigen::VectorXd>> _eoat_buffers;
  std::vector<RUT::DataBuffer<Eigen::VectorXd>>
      _wrench_buffers;  // wrench from external sensors
  std::vector<RUT::DataBuffer<Eigen::VectorXd>> _wrench_filtered_buffers;
  std::vector<RUT::DataBuffer<Eigen::VectorXd>>
      _robot_wrench_buffers;  // wrench from the robot itself
  // data buffer mutexes
  std::deque<std::mutex> _camera_rgb_buffer_mtxs;
  std::deque<std::mutex> _pose_buffer_mtxs;
  std::deque<std::mutex> _vel_buffer_mtxs;
  std::deque<std::mutex> _eoat_buffer_mtxs;
  std::deque<std::mutex> _wrench_buffer_mtxs;
  std::deque<std::mutex> _wrench_filtered_buffer_mtxs;
  std::deque<std::mutex> _robot_wrench_buffer_mtxs;
  // data buffer timestamps
  std::vector<RUT::DataBuffer<double>> _camera_rgb_timestamp_ms_buffers;
  std::vector<RUT::DataBuffer<double>> _pose_timestamp_ms_buffers;
  std::vector<RUT::DataBuffer<double>> _vel_timestamp_ms_buffers;
  std::vector<RUT::DataBuffer<double>> _eoat_timestamp_ms_buffers;
  std::vector<RUT::DataBuffer<double>> _wrench_timestamp_ms_buffers;
  std::vector<RUT::DataBuffer<double>> _wrench_filtered_timestamp_ms_buffers;
  std::vector<RUT::DataBuffer<double>> _robot_wrench_timestamp_ms_buffers;
  // data buffer timestamps just being fetched
  std::vector<Eigen::VectorXd> _camera_rgb_timestamps_ms;
  std::vector<Eigen::VectorXd> _pose_timestamps_ms;
  std::vector<Eigen::VectorXd> _vel_timestamps_ms;
  std::vector<Eigen::VectorXd> _eoat_timestamps_ms;
  std::vector<Eigen::VectorXd> _wrench_timestamps_ms;
  std::vector<Eigen::VectorXd> _wrench_filtered_timestamps_ms;
  std::vector<Eigen::VectorXd> _robot_wrench_timestamps_ms;

  // action buffers
  std::vector<RUT::DataBuffer<Eigen::VectorXd>> _waypoints_buffers;
  std::vector<RUT::DataBuffer<Eigen::VectorXd>> _eoat_waypoints_buffers;
  std::vector<RUT::DataBuffer<Eigen::MatrixXd>> _stiffness_buffers;
  // action buffer mutexes
  std::deque<std::mutex> _waypoints_buffer_mtxs;
  std::deque<std::mutex> _eoat_waypoints_buffer_mtxs;
  std::deque<std::mutex> _stiffness_buffer_mtxs;
  // action buffer timestamps
  std::vector<RUT::DataBuffer<double>> _waypoints_timestamp_ms_buffers;
  std::vector<RUT::DataBuffer<double>> _eoat_waypoints_timestamp_ms_buffers;
  std::vector<RUT::DataBuffer<double>> _stiffness_timestamp_ms_buffers;
  double _test_timestamp_ms;

  // timing
  RUT::Timer _timer;

  // episode start and end flags
  bool _start_episode = false;
  bool _end_episode = false;
  std::mutex _flag_mtx;
  std::condition_variable _flag_cv;

  //  hardware interfaces
  std::vector<std::shared_ptr<CameraInterfaces>> camera_ptrs;
  std::vector<std::shared_ptr<FTInterfaces>> force_sensor_ptrs;
  std::vector<std::shared_ptr<RobotInterfaces>> robot_ptrs;
  std::vector<std::shared_ptr<JSInterfaces>> eoat_ptrs;
  std::shared_ptr<Touch> _touch_ptr;  // JY
  Touch::TouchConfig _touch_config;   // JY: stored so teleop_loop can read it

  // controllers
  std::vector<AdmittanceController> _controllers;
  std::vector<PerturbationGenerator> _perturbation_generators;
  std::deque<std::mutex> _controller_mtxs;

  // wrench filters
  std::vector<RUT::Butterworth> _wrench_filters;

  // threads
  std::vector<std::thread> _robot_threads;
  std::vector<std::thread> _eoat_threads;
  std::vector<std::thread> _wrench_threads;
  std::vector<std::thread> _rgb_threads;
  std::thread _rgb_plot_thread;
  std::thread _key_thread;
  std::thread _teleop_thread;  // JY

  // control variables to control the threads
  std::string _episode_folder;
  std::vector<std::string> _ctrl_rgb_folders;
  std::vector<std::ofstream> _ctrl_robot_data_streams;
  std::vector<std::ofstream> _ctrl_eoat_data_streams;
  std::vector<std::ofstream> _ctrl_wrench_data_streams;
  std::ofstream _ctrl_key_data_stream;
  std::ofstream _ctrl_teleop_data_stream;  // JY
  bool _ctrl_flag_running = false;  // flag to terminate the program
  bool _ctrl_flag_saving = false;   // flag for ongoing data collection
  std::mutex _ctrl_mtx;
  bool _ctrl_listen_key_event = false;  // flag to enable key event detection
  std::mutex _ctrl_key_mtx;             // give it a separate mutex

  // state variable indicating the status of the threads
  std::vector<bool> _states_robot_thread_ready{};
  std::vector<bool> _states_eoat_thread_ready{};
  std::vector<bool> _states_rgb_thread_ready{};
  std::vector<bool> _states_wrench_thread_ready{};
  bool _state_plot_thread_ready{false};
  bool _state_key_thread_ready{false};
  bool _state_teleop_thread_ready{false};   // JY
  bool _teleop_active{false};               // JY
  bool _teleop_start_requested{false};      // JY: set by main() on Enter key, consumed by teleop_loop
  bool _amplify_mode{false};               // JY: mirrors teleop_loop local, read by GUI
  bool _idle_mode{false};                  // JY: mirrors teleop_loop local, read by GUI

  std::vector<bool> _states_robot_thread_saving{};
  std::vector<bool> _states_eoat_thread_saving{};
  std::vector<bool> _states_rgb_thread_saving{};
  std::vector<bool> _states_wrench_thread_saving{};
  bool _state_key_thread_saving{false};
  bool _state_teleop_thread_saving{false};  // JY

  std::vector<int> _states_robot_seq_id{};
  std::vector<int> _states_eoat_seq_id{};
  std::vector<int> _states_rgb_seq_id{};
  std::vector<int> _states_wrench_seq_id{};
  int _state_key_seq_id{0};
  int _state_teleop_seq_id{0};  // JY

  // shared variables between camera thread and plot thread
  std::vector<cv::Mat> _color_mats;
  std::deque<std::mutex> _color_mat_mtxs;

  // shared variables between robot thread and wrench thread
  std::vector<Eigen::VectorXd> _poses_fb;
  std::vector<Eigen::VectorXd> _perturbation;
  std::deque<std::mutex> _poses_fb_mtxs;
  std::deque<std::mutex> _perturbation_mtxs;

  // shared variables between eoat thread and wrench thread
  std::vector<Eigen::VectorXd> _wrench_fb;
  std::deque<std::mutex> _wrench_fb_mtxs;

  // shared variables between key thread and other threads
  std::mutex _key_mtx;
  int _key_is_pressed{0};          // 0: not pressed, 1: first key is pressed
  int _key_is_pressed_delayed{0};  // 0: not pressed, 1: first key is pressed
  RUT::Timer _key_delayed_timer;
  double _last_key_released_time_ms{0};  // time when the key was released

  // loop functions
  void robot_loop(const RUT::TimePoint& time0, int robot_id);
  void eoat_loop(const RUT::TimePoint& time0, int robot_id);
  void rgb_loop(const RUT::TimePoint& time0, int camera_id);
  void wrench_loop(const RUT::TimePoint& time0, int publish_rate,
                   int sensor_id);
  void rgb_plot_loop();  // opencv plotting does not support multi-threading
  void key_loop(const RUT::TimePoint& time0);
  void teleop_loop(const RUT::TimePoint& time0);  // JY
};