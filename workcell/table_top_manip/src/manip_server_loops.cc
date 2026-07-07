#include "table_top_manip/manip_server.h"

#include <RobotUtilities/interpolation_controller.h>
#include <opencv2/core/eigen.hpp>
#include <optional>

#include <fcntl.h>        // for key loop
#include <linux/input.h>  // for key loop
#include <stdlib.h>       // for key loop
#include <unistd.h>       // for key loop

#include "helpers.hpp"

#include <algorithm>  // JY: std::clamp for teleop_loop velocity clamping
#include <cmath>
#include <opencv2/imgproc.hpp>
#include <vector>

// Key code for 'a' (use 'a' lowercase)
#define KEY_CODE_A 30

void ManipServer::robot_loop(const RUT::TimePoint& time0, int id) {
  std::string header =
      "[ManipServer][Robot thread] " + std::to_string(id) + ": ";
  std::cout << header + "starting thread.\n";

  RUT::Timer timer;
  timer.tic(time0);  // so this timer is synced with the main timer

  RUT::Vector7d pose_fb;
  RUT::Vector7d pose_target_waypoint;
  RUT::Vector6d vel_fb;
  RUT::Vector7d force_control_ref_pose;
  RUT::Vector7d pose_rdte_cmd;
  // The following two initial values are used in mock hardware mode
  pose_fb << id, 0, 0, 1, 0, 0, 0;
  pose_rdte_cmd = pose_fb;
  vel_fb << 0, 0, 0, 0, 0, 0;

  std::cout << "[DEBUG] initializing wrench_fb" << std::endl;
  RUT::Vector6d wrench_fb_ur;  // wrench feedback from ur
  RUT::VectorXd wrench_fb_sensor = RUT::VectorXd::Zero(
      6 * _num_ft_sensors[id]);  // wrench feedback from sensor
  RUT::Vector6d wrench_fb;       // wrench feedback selected
  RUT::Vector6d wrench_WTr;      // wrench command
  RUT::Matrix6d stiffness;

  // TODO: use base pointer robot_ptr instead of URRTDE
  //       Need to create interfaces for all used functions here in RobotInterfaces
  URRTDE* urrtde_ptr;

  if (!_config.mock_hardware) {
    urrtde_ptr = static_cast<URRTDE*>(robot_ptrs[id].get());
    urrtde_ptr->getCartesian(pose_fb);
  }

  force_control_ref_pose = pose_fb;
  wrench_WTr.setZero();

  // for compliance control tool offset
  RUT::Matrix4d SE3_WTfb, SE3_CT, SE3_TC, SE3_WCfb, SE3_WTref, SE3_WCref,
      SE3_WCcmd, SE3_WTcmd;
  RUT::Matrix6d Adj_TC;
  SE3_TC = RUT::pose2SE3(_config.pose7_offset_TC);
  SE3_CT = RUT::SE3Inv(SE3_TC);
  bool apply_tc_offset = true;
  RUT::Vector7d temp_vec = _config.pose7_offset_TC;
  temp_vec[3] = 0;
  if (temp_vec.norm() < 1e-6) {
    apply_tc_offset = false;
  }

  bool ctrl_flag_saving = false;  // local copy

  bool perturbation_is_applied = false;
  RUT::VectorXd perturbation = RUT::VectorXd::Zero(6);

  RUT::TaskSpaceInterpolationController intp_controller;
  intp_controller.initialize(pose_fb, timer.toc_ms());
  std::cout << header << "intp_controller initialized with pose_fb: "
            << pose_fb.transpose() << std::endl;

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _states_robot_thread_ready[id] = true;
  }

  RUT::Profiler loop_profiler;
  std::cout << header << "Loop started." << std::endl;

  RUT::Timer mock_loop_timer;
  mock_loop_timer.set_loop_rate_hz(500);
  mock_loop_timer.start_timed_loop();
  while (true) {
    // Update robot status
    loop_profiler.start();
    mock_loop_timer.tic();
    RUT::TimePoint t_start;
    double time_now_ms;
    if (!_config.mock_hardware) {
      // real hardware
      // t_start = urrtde_ptr->rtde_init_period();
      urrtde_ptr->getCartesian(pose_fb);
      urrtde_ptr->getCartesianVelocity(vel_fb);
      urrtde_ptr->getWrenchToolCalibrated(wrench_fb_ur);

      {
        std::lock_guard<std::mutex> lock(_wrench_fb_mtxs[id]);
        wrench_fb_sensor = _wrench_fb[id];
      }

      if (_config.compliance_control_force_source ==
          ComplianceControlForceSource::UR) {
        wrench_fb = wrench_fb_ur;
      } else if (_config.compliance_control_force_source ==
                 ComplianceControlForceSource::COINFT) {
        wrench_fb = wrench_fb_sensor.head<6>() + wrench_fb_sensor.tail<6>();
      } else if (_config.compliance_control_force_source ==
                 ComplianceControlForceSource::ATI) {
        wrench_fb = wrench_fb_sensor.head<6>();
      } else {
        std::cerr << header << "Invalid compliance control force source: "
                  << to_string(_config.compliance_control_force_source)
                  << std::endl;
        break;
      }

      time_now_ms = timer.toc_ms();
      loop_profiler.stop("compute");
      loop_profiler.start();
      {
        std::lock_guard<std::mutex> lock(_poses_fb_mtxs[id]);
        _poses_fb[id] = pose_fb;
      }
      loop_profiler.stop("lock");
      loop_profiler.start();

    } else {
      // mock hardware
      time_now_ms = timer.toc_ms();
      pose_fb = pose_rdte_cmd;
      vel_fb.setZero();
      wrench_fb.setZero();
    }
    // buffer robot pose
    loop_profiler.stop("compute");
    loop_profiler.start();
    {
      std::lock_guard<std::mutex> lock(_pose_buffer_mtxs[id]);
      _pose_buffers[id].put(pose_fb);
      _pose_timestamp_ms_buffers[id].put(time_now_ms);
    }
    {
      std::lock_guard<std::mutex> lock(_vel_buffer_mtxs[id]);
      _vel_buffers[id].put(vel_fb);
      _vel_timestamp_ms_buffers[id].put(time_now_ms);
    }
    {
      std::lock_guard<std::mutex> lock(_robot_wrench_buffer_mtxs[id]);
      _robot_wrench_buffers[id].put(wrench_fb);
      _robot_wrench_timestamp_ms_buffers[id].put(time_now_ms);
    }
    loop_profiler.stop("lock");
    loop_profiler.start();

    // update control target from interpolation controller
    if (!intp_controller.get_control(time_now_ms, force_control_ref_pose, std::nullopt)) {
      bool new_wp_found = false;
      {
        // need to get new waypoint from buffer
        std::lock_guard<std::mutex> lock(_waypoints_buffer_mtxs[id]);
        while (!_waypoints_buffers[id].is_empty()) {
          // keep querying buffer until we get a target that is in the future
          pose_target_waypoint = _waypoints_buffers[id].pop();
          double target_time_ms = _waypoints_timestamp_ms_buffers[id].pop();
          if (target_time_ms > time_now_ms) {
            intp_controller.set_new_target(pose_target_waypoint,
                                           target_time_ms);
            new_wp_found = true;
            // std::cout << "[debug] time_now_ms: " << time_now_ms
            //           << ", time now: " << timer.toc_ms()
            //           << ", target_time_ms:" << target_time_ms
            //           << ", pose_target_waypoint: "
            //           << pose_target_waypoint.transpose() << std::endl;

            break;
          }
        }
      }
      if (!new_wp_found) {
        // std::cout << "[debug] no new wp found at time_now_ms: " << time_now_ms
        //           << ", time now: " << timer.toc_ms()
        //           << ", pose_target_waypoint: "
        //           << pose_target_waypoint.transpose() << std::endl;
        intp_controller.keep_the_last_target(time_now_ms);
      }
      intp_controller.get_control(time_now_ms, force_control_ref_pose, std::nullopt);
    }

    loop_profiler.stop("intp_controller");
    loop_profiler.start();

    // update stiffness matrix from buffer

    // condition:
    //   time_now_ms < time[0], do nothing
    //   time_now_ms >= time[0], look for next
    bool new_stiffness_found = false;
    {
      std::lock_guard<std::mutex> lock(_stiffness_buffer_mtxs[id]);
      if (!_stiffness_buffers[id].is_empty()) {
        double next_available_time_ms = _stiffness_timestamp_ms_buffers[id][0];
        if (time_now_ms > next_available_time_ms) {
          new_stiffness_found = true;
          while ((!_stiffness_timestamp_ms_buffers[id].is_empty()) &&
                 (_stiffness_timestamp_ms_buffers[id][0] < time_now_ms)) {
            stiffness = _stiffness_buffers[id].pop();
            next_available_time_ms = _stiffness_timestamp_ms_buffers[id].pop();
          }
        }
      }
    }
    loop_profiler.stop("stiffness");
    loop_profiler.start();

    // apply perturbation
    perturbation.setZero();
    if (_config.use_perturbation_generator) {
      perturbation_is_applied =
          _perturbation_generators[id].generate_perturbation(perturbation);
      wrench_fb += perturbation;
    }
    // record perturbation so wrench thread knows it
    {
      std::lock_guard<std::mutex> lock(_perturbation_mtxs[id]);
      _perturbation[id] = perturbation;
    }

    loop_profiler.stop("perturbation");
    loop_profiler.start();

    wrench_WTr.setZero();
    // std::cout << "[debug] time: " << time_now_ms
    //           << ", wrench_fb: " << wrench_fb.transpose()
    //           << ", wrench_WTr: " << wrench_WTr.transpose() << std::endl;

    // Update the compliance controller
    if (apply_tc_offset) {
      SE3_WTfb = RUT::pose2SE3(pose_fb);
      SE3_WCfb = SE3_WTfb * SE3_TC;
      RUT::SE32Pose(SE3_WCfb, pose_fb);
      Adj_TC = RUT::SE32Adj(SE3_TC);
      wrench_fb = Adj_TC.transpose() * wrench_fb;
      SE3_WTref = RUT::pose2SE3(force_control_ref_pose);
      SE3_WCref = SE3_WTref * SE3_TC;
      RUT::SE32Pose(SE3_WCref, force_control_ref_pose);
    }

    {
      std::lock_guard<std::mutex> lock(_controller_mtxs[id]);
      loop_profiler.stop("controller_lock");
      loop_profiler.start();
      _controllers[id].setRobotStatus(pose_fb, wrench_fb);
      // Update robot reference
      _controllers[id].setRobotReference(force_control_ref_pose, wrench_WTr);

      // Update stiffness matrix
      if (new_stiffness_found) {
        _controllers[id].setStiffnessMatrix(stiffness);
      }
      loop_profiler.stop("controller_set");
      loop_profiler.start();
      // Compute the control output
      _controllers[id].step(pose_rdte_cmd);
      loop_profiler.stop("controller_step");
      loop_profiler.start();
    }

    if (apply_tc_offset) {
      SE3_WCcmd = RUT::pose2SE3(pose_rdte_cmd);
      SE3_WTcmd = SE3_WCcmd * SE3_CT;
      RUT::SE32Pose(SE3_WTcmd, pose_rdte_cmd);
    }

    // Send control command to the robot
    if ((!_config.mock_hardware) &&
        (!urrtde_ptr->streamCartesian(pose_rdte_cmd))) {
      std::cout << header << "streamCartesian failed. Ending thread."
                << std::endl;
      std::cout << header << "last pose_fb: " << pose_fb.transpose()
                << std::endl;
      std::cout << header << "last wrench_fb: " << wrench_fb.transpose()
                << std::endl;
      std::cout << header << "last force_control_ref_pose: "
                << force_control_ref_pose.transpose() << std::endl;
      std::cout << header << "last pose_rdte_cmd: " << pose_rdte_cmd.transpose()
                << std::endl;
      break;
    }

    // std::cout << "t = " << timer.toc_ms()
    //           << ", pose_rdte_cmd: " << pose_rdte_cmd.transpose() << std::endl;

    // logging
    if (std::lock_guard<std::mutex> lock(_ctrl_mtx); _ctrl_flag_saving) {
      if (!ctrl_flag_saving) {
        std::cout << "[robot thread] Start saving low dim data." << std::endl;
        json_file_start(_ctrl_robot_data_streams[id]);
        ctrl_flag_saving = true;
      }

      _states_robot_thread_saving[id] = true;
      save_robot_data_json(_ctrl_robot_data_streams[id],
                           _states_robot_seq_id[id], timer.toc_ms(), pose_fb,
                           wrench_fb_ur, perturbation_is_applied);
      json_frame_ending(_ctrl_robot_data_streams[id]);
      _states_robot_seq_id[id]++;
    } else {
      if (ctrl_flag_saving) {
        std::cout << "[robot thread] Stop saving low dim data." << std::endl;
        // save one last frame, so we can do the correct different frame ending
        save_robot_data_json(_ctrl_robot_data_streams[id],
                             _states_robot_seq_id[id], timer.toc_ms(), pose_fb,
                             wrench_fb_ur, perturbation_is_applied);
        json_file_ending(_ctrl_robot_data_streams[id]);
        _ctrl_robot_data_streams[id].close();
        ctrl_flag_saving = false;
        _states_robot_thread_saving[id] = false;
        std::cout << "[robot thread] Low dim data saved." << std::endl;
      }
    }

    loop_profiler.stop("logging");
    loop_profiler.start();

    // loop control
    if (std::lock_guard<std::mutex> lock(_ctrl_mtx); !_ctrl_flag_running) {
      std::cout << "[robot thread] _ctrl_flag_running is false. Shuting "
                   "down this thread."
                << std::endl;
      break;
    }

    loop_profiler.stop("lock");

    // loop timing and overrun check
    double overrun_ms = mock_loop_timer.check_for_overrun_ms(false);
    // TODO: this overrun check does not work. Needs to debug
    if (_config.check_robot_loop_overrun && (overrun_ms > 0)) {
      std::cout << "\033[33m";  // set color to bold yellow
      std::cout << header << "Overrun: " << overrun_ms << "ms" << std::endl;
      std::cout << "\033[0m";  // reset color to default
      loop_profiler.show();
    }
    loop_profiler.clear();
    mock_loop_timer.sleep_till_next();
  }  // end of while loop

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _ctrl_flag_running = false;
  }
  std::cout << "[robot thread] Joined." << std::endl;
}

void ManipServer::eoat_loop(const RUT::TimePoint& time0, int id) {
  std::string header =
      "[ManipServer][EoAT thread] " + std::to_string(id) + ": ";
  std::cout << header + "starting thread.\n";

  RUT::Timer timer;
  timer.tic(time0);  // so this timer is synced with the main timer

  RUT::VectorXd pos_fb = RUT::VectorXd::Zero(1);
  RUT::VectorXd eoat_target_waypoint = RUT::VectorXd::Zero(2);
  RUT::VectorXd eoat_cmd = RUT::VectorXd::Zero(2);

  if (!_config.mock_hardware) {
    eoat_ptrs[id]->getJoints(pos_fb);
  }
  eoat_cmd << pos_fb[0], 0;

  bool ctrl_flag_saving = false;  // local copy

  RUT::JointSpaceInterpolationController intp_controller;
  intp_controller.initialize(eoat_cmd, timer.toc_ms());
  std::cout << header
            << "intp_controller initialized with pos_fb: " << pos_fb.transpose()
            << std::endl;

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _states_eoat_thread_ready[id] = true;
  }

  std::cout << header << "Loop started." << std::endl;

  RUT::Timer loop_timer;
  loop_timer.set_loop_rate_hz(100);
  loop_timer.start_timed_loop();
  while (true) {
    // Update EoAT status (for query and logging)
    double time_now_ms;
    if (!_config.mock_hardware) {
      // real hardware
      eoat_ptrs[id]->getJoints(pos_fb);
      time_now_ms = timer.toc_ms();
    } else {
      // mock hardware
      time_now_ms = timer.toc_ms();
      pos_fb[0] = eoat_cmd[0];
    }
    // save state to eoat fb buffer
    {
      std::lock_guard<std::mutex> lock(_eoat_buffer_mtxs[id]);
      _eoat_buffers[id].put(pos_fb);
      _eoat_timestamp_ms_buffers[id].put(time_now_ms);
    }

    // update control target from interpolation controller
    if (!intp_controller.get_control(time_now_ms, eoat_cmd)) {
      bool new_wp_found = false;
      {
        // need to get new waypoint from buffer
        std::lock_guard<std::mutex> lock(_eoat_waypoints_buffer_mtxs[id]);
        while (!_eoat_waypoints_buffers[id].is_empty()) {
          // keep querying buffer until we get a target that is in the future
          eoat_target_waypoint = _eoat_waypoints_buffers[id].pop();
          double target_time_ms =
              _eoat_waypoints_timestamp_ms_buffers[id].pop();
          if (target_time_ms > time_now_ms) {
            intp_controller.set_new_target(eoat_target_waypoint,
                                           target_time_ms);
            new_wp_found = true;
            break;
          }
        }
      }
      if (!new_wp_found) {
        // std::cout << "[debug] time_now_ms: " << time_now_ms
        //           << ", time now: " << timer.toc_ms()
        //           << ", target_time_ms:" << target_time_ms
        //           << ", eoat_target_waypoint: "
        //           << eoat_target_waypoint.transpose() << std::endl;
        intp_controller.keep_the_last_target(time_now_ms);
      }
      intp_controller.get_control(time_now_ms, eoat_cmd);
    }

    // keyboard interuption of gripper control
    {
      std::lock_guard<std::mutex> lock(_key_mtx);
      if (_key_is_pressed == 2) {
        eoat_cmd[0] = 0;  // close the gripper
      }
      if (_key_is_pressed == 1) {
        eoat_cmd[0] = 110;  // open the gripper
      }
    }

    // Send command to EoAT
    double force_fb = 0;
    {
      std::lock_guard<std::mutex> lock(_wrench_fb_mtxs[id]);
      // TODO: currently, assuming the grasping force is captured by Z axis of the first wrench sensor.
      // Need to find a better way to specify it.
      //force_fb = _wrench_fb[id][2];
      // _wrench_fb is a 12 dimensinoal vector with [left_wrench, right_wrench] in TCP.

      force_fb = (-_wrench_fb[id][0] + _wrench_fb[id][6]) / 2;
    }
    // std::cout << header << "force_fb: " << force_fb
    //           << ", force cmd: " << eoat_cmd[1] << std::endl;
    eoat_cmd[1] -= force_fb;
    if ((!_config.mock_hardware) && (!eoat_ptrs[id]->setJointsPosForce(
                                        eoat_cmd.head(1), eoat_cmd.tail(1)))) {
      std::cout << header << "setJointsPosForce failed. Ending thread."
                << std::endl;
      std::cout << header << "last pos_fb: " << pos_fb.transpose() << std::endl;
      std::cout << header << "last eoat_cmd: " << eoat_cmd.transpose()
                << std::endl;
      break;
    }

    // std::cout << "deubg: t = " << timer.toc_ms()
    //           << ", eoat_cmd: " << eoat_cmd.transpose() << std::endl;

    // logging
    if (std::lock_guard<std::mutex> lock(_ctrl_mtx); _ctrl_flag_saving) {
      if (!ctrl_flag_saving) {
        std::cout << header << "Start saving eoat data." << std::endl;
        json_file_start(_ctrl_eoat_data_streams[id]);
        ctrl_flag_saving = true;
      }

      _states_eoat_thread_saving[id] = true;
      save_eoat_data_json(_ctrl_eoat_data_streams[id], _states_eoat_seq_id[id],
                          timer.toc_ms(), pos_fb);
      json_frame_ending(_ctrl_eoat_data_streams[id]);
      _states_eoat_seq_id[id]++;
    } else {
      if (ctrl_flag_saving) {
        std::cout << header << "Stop saving eoat data." << std::endl;
        // save one last frame, so we can do the correct different frame ending
        save_eoat_data_json(_ctrl_eoat_data_streams[id],
                            _states_eoat_seq_id[id], timer.toc_ms(), pos_fb);
        json_file_ending(_ctrl_eoat_data_streams[id]);
        _ctrl_eoat_data_streams[id].close();
        ctrl_flag_saving = false;
        _states_eoat_thread_saving[id] = false;
      }
    }

    if (std::lock_guard<std::mutex> lock(_ctrl_mtx); !_ctrl_flag_running) {
      std::cout << header
                << "_ctrl_flag_running is false. Shuting "
                   "down this thread."
                << std::endl;
      break;
    }

    loop_timer.sleep_till_next();
  }  // end of while loop

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _ctrl_flag_running = false;
  }
  std::cout << "[EoAT thread] Joined." << std::endl;
}

void ManipServer::wrench_loop(const RUT::TimePoint& time0, int publish_rate,
                              int id) {
  std::string header =
      "[ManipServer][Wrench thread] " + std::to_string(id) + ": ";
  std::cout << header << "thread starting." << std::endl;
  std::cout << header << "Rate at " << publish_rate << "Hz." << std::endl;
  RUT::Timer timer;
  timer.tic(time0);  // so this timer is synced with the main timer

  int num_ft_sensors = _num_ft_sensors[id];
  std::cout << header << "Number of FT sensors: " << num_ft_sensors
            << std::endl;
  if (!_config.mock_hardware) {
    // wait for force sensor to be ready
    std::cout << header
              << "Waiting for force sensor to start "
                 "streaming.\n";
    while (!force_sensor_ptrs[id]->is_data_ready()) {
      usleep(100000);
    }
  }

  RUT::VectorXd wrench_fb = RUT::VectorXd::Zero(6 * num_ft_sensors);
  RUT::VectorXd wrench_fb_filtered = RUT::VectorXd::Zero(6 * num_ft_sensors);

  // wait for pose_fb to be ready
  std::cout << header
            << "Waiting for robot thread to "
               "populate pose_fb. \n";
  while (true) {
    {
      std::lock_guard<std::mutex> lock(_pose_buffer_mtxs[id]);
      if (_pose_buffers[id].size() > 0) {
        break;
      }
    }
    usleep(300 * 1000);  // 300ms
  }

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _states_wrench_thread_ready[id] = true;
  }

  std::cout << header << "Loop started." << std::endl;
  bool ctrl_flag_saving = false;  // local copy

  RUT::Vector7d pose_fb;
  RUT::VectorXd perturbation;

  RUT::Timer loop_timer;
  loop_timer.set_loop_rate_hz(publish_rate);
  loop_timer.start_timed_loop();
  while (true) {
    // Update robot status
    RUT::TimePoint t_start;
    double time_now_ms;

    // read perturbations
    if (num_ft_sensors == 1) {
      std::lock_guard<std::mutex> lock(_perturbation_mtxs[id]);
      perturbation = _perturbation[id];
    }

    if (!_config.mock_hardware) {
      // get the most recent tool pose (for static calibration)
      {
        std::lock_guard<std::mutex> lock(_poses_fb_mtxs[id]);
        pose_fb = _poses_fb[id];
      }
      int safety_flag = force_sensor_ptrs[id]->getWrenchNetTool(
          pose_fb, wrench_fb, num_ft_sensors);
      if (safety_flag < 0) {
        std::cout << header
                  << "Wrench is above safety threshold. Ending thread."
                  << std::endl;
        break;
      }
      if (num_ft_sensors == 1) {
        wrench_fb += perturbation;  // apply perturbation
      }
      time_now_ms = timer.toc_ms();
      {
        std::lock_guard<std::mutex> lock(_wrench_buffer_mtxs[id]);
        _wrench_buffers[id].put(wrench_fb);
        _wrench_timestamp_ms_buffers[id].put(time_now_ms);
      }
      time_now_ms = timer.toc_ms();
      {
        std::lock_guard<std::mutex> lock(_wrench_filtered_buffer_mtxs[id]);
        wrench_fb_filtered = _wrench_filters[id].step(wrench_fb);
        _wrench_filtered_buffers[id].put(wrench_fb_filtered);
        _wrench_filtered_timestamp_ms_buffers[id].put(time_now_ms);
      }
      {
        std::lock_guard<std::mutex> lock(_wrench_fb_mtxs[id]);
        _wrench_fb[id] = wrench_fb;
      }
    } else {
      // mock hardware
      wrench_fb.setZero(num_ft_sensors * 6);
      if (num_ft_sensors == 1) {
        wrench_fb += perturbation;  // apply perturbation
      }
      time_now_ms = timer.toc_ms();
      {
        std::lock_guard<std::mutex> lock(_wrench_buffer_mtxs[id]);
        _wrench_buffers[id].put(wrench_fb);
        _wrench_timestamp_ms_buffers[id].put(time_now_ms);
      }
      time_now_ms = timer.toc_ms();
      wrench_fb_filtered.setZero();
      {
        std::lock_guard<std::mutex> lock(_wrench_filtered_buffer_mtxs[id]);
        wrench_fb_filtered = _wrench_filters[id].step(wrench_fb);
        _wrench_filtered_buffers[id].put(wrench_fb_filtered);
        _wrench_filtered_timestamp_ms_buffers[id].put(time_now_ms);
      }
    }

    // logging
    if (std::lock_guard<std::mutex> lock(_ctrl_mtx); _ctrl_flag_saving) {
      if (!ctrl_flag_saving) {
        std::cout << "[wrench thread] Start saving wrench data." << std::endl;
        json_file_start(_ctrl_wrench_data_streams[id]);
        ctrl_flag_saving = true;
      }

      _states_wrench_thread_saving[id] = true;
      save_wrench_data_json(_ctrl_wrench_data_streams[id],
                            _states_wrench_seq_id[id], timer.toc_ms(),
                            wrench_fb, wrench_fb_filtered);
      json_frame_ending(_ctrl_wrench_data_streams[id]);
      _states_wrench_seq_id[id]++;
    } else {
      if (ctrl_flag_saving) {
        std::cout << "[wrench thread] Stop saving wrench data." << std::endl;
        // save one last frame, so we can do the correct different frame ending
        save_wrench_data_json(_ctrl_wrench_data_streams[id],
                              _states_wrench_seq_id[id], timer.toc_ms(),
                              wrench_fb, wrench_fb_filtered);
        json_file_ending(_ctrl_wrench_data_streams[id]);
        _ctrl_wrench_data_streams[id].close();
        ctrl_flag_saving = false;
        _states_wrench_thread_saving[id] = false;
      }
    }

    if (std::lock_guard<std::mutex> lock(_ctrl_mtx); !_ctrl_flag_running) {
      std::cout << "[wrench thread] _ctrl_flag_running is false. Shuting "
                   "down this thread."
                << std::endl;
      break;
    }
    loop_timer.sleep_till_next();
  }  // end of while loop

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _ctrl_flag_running = false;
  }
  std::cout << "[wrench thread] Joined." << std::endl;
}

void ManipServer::rgb_loop(const RUT::TimePoint& time0, int id) {
  std::string header = "[ManipServer][rgb thread] " + std::to_string(id) + ": ";
  std::cout << header << "starting thread" << std::endl;

  RUT::Timer timer;
  timer.tic(time0);
  double time_start = timer.toc_ms();
  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _states_rgb_thread_ready[id] = true;
  }
  std::cout << header << "Loop started." << std::endl;

  cv::Mat raw, tmp, resized_color_mat;
  cv::Mat bgr[3];
  Eigen::MatrixXd bm, gm, rm;
  Eigen::MatrixXd rgb_row_combined;

  const int ow = _config.output_rgb_hw[1];  // 224
  const int oh = _config.output_rgb_hw[0];  // 224
  // const std::string debug_dir = "/home/zhanyi/hardware_interfaces/debug_images";

  int frame_counter = 0;
  while (true) {
    double time_now_ms = 0;
    {
      std::lock_guard<std::mutex> lock(_color_mat_mtxs[id]);
      if (!_config.mock_hardware) {
        raw = camera_ptrs[id]->next_rgb_frame_blocking();  // **already RGB**
      } else {
        raw = cv::Mat::zeros(oh, ow, CV_8UC3);
        usleep(20 * 1000);  // 20ms, 50hz
      }
      _color_mats[id] = raw;
      time_now_ms = timer.toc_ms();
    }

    // === resize → center‐crop ===
    int iw = raw.cols, ih = raw.rows;
    int rw, rh;
    int interp = cv::INTER_AREA;
    if (float(iw) / ih >= float(ow) / oh) {
      rh = oh;
      rw = int(std::ceil(float(rh) / ih * iw));
      if (oh > ih)
        interp = cv::INTER_LINEAR;
    } else {
      rw = ow;
      rh = int(std::ceil(float(rw) / iw * ih));
      if (ow > iw)
        interp = cv::INTER_LINEAR;
    }
    cv::resize(raw, tmp, cv::Size(rw, rh), 0.0, 0.0, interp);
    int x0 = (rw - ow) / 2, y0 = (rh - oh) / 2;
    resized_color_mat = tmp(cv::Rect(x0, y0, ow, oh)).clone();

    // // === DEBUG save first 20 frames ===
    // if (frame_counter < 20) {
    //   std::ostringstream oss;
    //   oss << debug_dir
    //       << "/rgb_" << id << "_"
    //       << std::setw(5) << std::setfill('0')
    //       << frame_counter << ".png";
    //   cv::imwrite(oss.str(), resized_color_mat);
    // }
    // ++frame_counter;

    // === pipeline into Eigen buffer (unchanged) ===
    cv::split(resized_color_mat, bgr);
    cv::cv2eigen(bgr[0], bm);
    cv::cv2eigen(bgr[1], gm);
    cv::cv2eigen(bgr[2], rm);
    rgb_row_combined.resize(oh * 3, ow);
    rgb_row_combined << rm, gm, bm;
    {
      std::lock_guard<std::mutex> lock(_camera_rgb_buffer_mtxs[id]);
      _camera_rgb_buffers[id].put(rgb_row_combined);
      _camera_rgb_timestamp_ms_buffers[id].put(time_now_ms);
    }

    if (_ctrl_flag_saving) {
      _states_rgb_thread_saving[id] = true;
      {
        std::lock_guard<std::mutex> lock(_color_mat_mtxs[id]);
        save_rgb_data(_ctrl_rgb_folders[id], _states_rgb_seq_id[id],
                      timer.toc_ms(), _color_mats[id]);
      }
      _states_rgb_seq_id[id]++;
    } else {
      _states_rgb_thread_saving[id] = false;
    }

    // std::cout << "t = " << timer.toc_ms() << ", get new rgb frame."
    //           << std::endl;
    if (std::lock_guard<std::mutex> lock(_ctrl_mtx); !_ctrl_flag_running) {
      std::cout << "[rgb thread] _ctrl_flag_running is false. Shuting "
                   "down this thread."
                << std::endl;

      break;
    }
  }
  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _ctrl_flag_running = false;
  }
  std::cout << "[rgb thread] Joined." << std::endl;
}

void ManipServer::rgb_plot_loop() {
  std::string header = "[ManipServer][plot thread]: ";
  std::cout << header << "starting thread." << std::endl;
  cv::namedWindow("RGB", cv::WINDOW_AUTOSIZE);
  std::vector<cv::Mat> color_mat_copy;
  cv::Mat canvas;

  for (int id : _id_list) {
    color_mat_copy.push_back(cv::Mat());
  }

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _state_plot_thread_ready = true;
  }

  std::cout << header << "Loop started." << std::endl;

  while (true) {
    for (int id : _id_list) {
      std::lock_guard<std::mutex> lock(_color_mat_mtxs[id]);
      color_mat_copy[id] = _color_mats[id].clone();
    }

    cv::vconcat(color_mat_copy, canvas);

    cv::imshow("RGB", canvas);

    if (std::lock_guard<std::mutex> lock(_ctrl_mtx); !_ctrl_flag_running) {
      std::cout << header
                << "[rgb plot thread] _ctrl_flag_running is false. Shuting "
                   "down this thread"
                << std::endl;
      break;
    }

    if (cv::waitKey(30) >= 0)
      break;
  }
  std::cout << "[plot thread] Joined." << std::endl;
}

void ManipServer::key_loop(const RUT::TimePoint& time0) {
  std::string header = "[ManipServer][key thread]: ";
  std::cout << header << "starting thread." << std::endl;
  RUT::Timer timer;
  timer.tic(time0);

  struct input_event ev;
  int fd;

  const char* device = _config.key_event_device.c_str();
  std::cout << header << "Using device: " << device << std::endl;
  fd = open(device, O_RDONLY | O_NONBLOCK);
  if (fd == -1) {
    std::cerr << header << "Cannot open input device: " << device << std::endl;
    std::cerr << "Check your key_event_device parameter It should be something "
                 "like /dev/input/eventxx."
              << std::endl;
    std::cerr << "You can find the correct device by running 'ls -l "
                 "/dev/input/by-path/ | grep kbd'."
              << std::endl;
    std::cerr << "If you have permission issue, run 'sudo chmod 777 "
                 "/dev/input/eventxx'."
              << std::endl;
    throw std::runtime_error("Cannot open input device. Exiting key loop.");
  }

  std::cout << header << "Monitoring 'a' key ...\n" << std::endl;

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _state_key_thread_ready = true;
  }

  // Set frame rate to 500Hz
  timer.set_loop_rate_hz(500);

  timer.start_timed_loop();
  std::cout << header << "Loop started." << std::endl;
  bool ctrl_flag_saving = false;  // local copy
  while (true) {
    int key_event =
        -1;  // -1: no event. 0: key is not pressed. 1: key is pressed
    if (std::lock_guard<std::mutex> lock(_ctrl_key_mtx);
        _ctrl_listen_key_event) {
      // Process input events
      while (read(fd, &ev, sizeof(ev)) > 0) {
        if (ev.type != EV_KEY)
          continue;  // Skip non-key events
        for (int kid = 0; kid < _config.keys_to_monitor.size(); kid++) {
          if (ev.code == _config.keys_to_monitor[kid]) {
            if (ev.value == 1) {
              // Key down event
              key_event = 1;
              // save current state
              {
                std::lock_guard<std::mutex> lock(_key_mtx);
                _key_is_pressed = kid + 1;
              }
              _key_is_pressed_delayed = key_event;
              std::cout << "\nKey DOWN detected for key # " << kid + 1
                        << std::endl;
            } else if (ev.value == 0) {
              // Key up event
              key_event = 0;
              // save current state
              {
                std::lock_guard<std::mutex> lock(_key_mtx);
                _key_is_pressed = key_event;
              }
              _last_key_released_time_ms = _key_delayed_timer.toc_ms();
              std::cout << "\nKey UP detected for key # " << kid + 1
                        << std::endl;
            }
            break;  // break when we detect a key to avoid getting values overwritten
          }
        }  // end for kid
      }
    }

    if (_config.take_over_mode) {
      if ((_key_is_pressed_delayed > 0) && (_key_is_pressed == 0) &&
          (_key_delayed_timer.toc_ms() - _last_key_released_time_ms > 1000)) {
        _key_is_pressed_delayed = 0;
      }
      if (key_event == 1) {
        std::cout << "\n===== taking over =====" << std::endl;
        clear_cmd_buffer();
        set_high_level_free_jogging();
      } else if (key_event == 0) {
        std::cout << "\n===== releasing control =====" << std::endl;
        clear_cmd_buffer();  // still need to clear the command buffer
        set_high_level_maintain_position();
      }
    }

    // logging
    if (std::lock_guard<std::mutex> lock(_ctrl_mtx); _ctrl_flag_saving) {
      if (!ctrl_flag_saving) {
        std::cout << "[key thread] Start saving key pressing data."
                  << std::endl;
        json_file_start(_ctrl_key_data_stream);
        ctrl_flag_saving = true;
      }

      _state_key_thread_saving = true;
      if (key_event >= 0) {
        // new key event!
        save_key_data_json(_ctrl_key_data_stream, _state_key_seq_id,
                           timer.toc_ms(), key_event);
        json_frame_ending(_ctrl_key_data_stream);
        _state_key_seq_id++;
      }
    } else {
      if (ctrl_flag_saving) {
        std::cout << "[key thread] Stop saving key data." << std::endl;
        // save one last frame, so we can do the correct different frame ending
        save_key_data_json(_ctrl_key_data_stream, _state_key_seq_id,
                           timer.toc_ms(), key_event);
        json_file_ending(_ctrl_key_data_stream);
        _ctrl_key_data_stream.close();
        ctrl_flag_saving = false;
        _state_key_thread_saving = false;
      }
    }
    // Check if the key event thread should stop
    if (std::lock_guard<std::mutex> lock(_ctrl_mtx); !_ctrl_flag_running) {
      std::cout << header
                << "_ctrl_flag_running is false. Shuting "
                   "down this thread"
                << std::endl;
      break;
    }
    timer.sleep_till_next();
  }

  // Close the input device
  close(fd);

  std::cout << header << "Joined." << std::endl;
}

// JY: Touch haptic teleoperation loop — runs at 200 Hz.
// Waits for Button 1 rising edge to activate, then streams target poses to
// the robot. Button 2 toggles amplify mode (velocity integration instead of
// absolute delta). Button 1 + Button 2 together deactivates teleoperation.
void ManipServer::teleop_loop(const RUT::TimePoint& time0) {
  const std::string header = "[ManipServer][teleop thread]: ";
  std::cout << header << "starting thread." << std::endl;

  if (!_touch_ptr || !_touch_ptr->is_data_ready()) {
    std::cerr << header << "Touch not available. Exiting.\n";
    {
      std::lock_guard<std::mutex> lock(_ctrl_mtx);
      _state_teleop_thread_ready = true;
    }
    return;
  }

  // Build HD -> robot rotation from config: Ry * Rx * Rz ordering,
  // matching the original MAA_data_collection_v2.py convention.
  const double DEG2RAD = M_PI / 180.0;
  Eigen::Matrix3d hd2rob_R =
      Eigen::AngleAxisd(_touch_config.hd2rob_euler_deg[0] * DEG2RAD,
                        Eigen::Vector3d::UnitY()).toRotationMatrix() *
      Eigen::AngleAxisd(_touch_config.hd2rob_euler_deg[1] * DEG2RAD,
                        Eigen::Vector3d::UnitX()).toRotationMatrix() *
      Eigen::AngleAxisd(_touch_config.hd2rob_euler_deg[2] * DEG2RAD,
                        Eigen::Vector3d::UnitZ()).toRotationMatrix();

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _state_teleop_thread_ready = true;
  }
  std::cout << header << "Loop started." << std::endl;

  bool was_btn1 = false;
  bool was_btn2 = false;
  bool amplify_mode = false;
  bool teleop_active_local = false;   // JY: tracked locally, updated on start/stop
  bool saving_was_active = false;     // JY: detect _ctrl_flag_saving transitions driven by R key

  // Reference state captured when teleop activates (or re-references)
  double ref_x{0}, ref_y{0}, ref_z{0};
  double ref_gimbals[3] = {0.0, 0.0, 0.0};  // JY: replaces q_ref; gimbal angles captured at activation
  RUT::Vector7d robot_initial_pose;
  robot_initial_pose << 0, 0, 0, 1, 0, 0, 0;
  Eigen::Quaterniond q_robot_initial = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond q_prev = Eigen::Quaterniond::Identity();


  int print_counter = 0;  // JY: diagnostic

  // JY: haptic force feedback state (persists across the while loop iterations)
  Eigen::Vector3d f_filt = Eigen::Vector3d::Zero();      // IIR filter state
  Eigen::Vector3d f_cmd_prev = Eigen::Vector3d::Zero();  // for slew-rate limiting (filtering path)
  double hap_ramp = 0.0;                                 // contact ramp [0,1]
  Eigen::Vector3d hap_wrench_offset = Eigen::Vector3d::Zero();  // tare captured at activation
  Eigen::Vector3d GinF_tare = Eigen::Vector3d::Zero();   // JY: gravity-in-sensor-frame at tare pose
  double hap_eq_pos[3] = {0.0, 0.0, 0.0};               // JY: Touch position spring equilibrium (mm)
  double k_prev[3]    = {0.0, 0.0, 0.0};                // JY: per-axis stiffness from previous step (for slew-rate limiting)
  const int    control_freq = 900;                       // JY: haptic loop rate (Hz)
  const double dt_hap      = 1.0 / control_freq;        // JY: derived period (s); round() lives on the usleep side

  while (true) {
    {
      std::lock_guard<std::mutex> lock(_ctrl_mtx);
      if (!_ctrl_flag_running) break;
    }

    Touch::TouchState state;
    _touch_ptr->getState(state);
    bool btn1 = (state.buttons & HD_DEVICE_BUTTON_1) != 0;
    bool btn2 = (state.buttons & HD_DEVICE_BUTTON_2) != 0;

    // JY: check _ctrl_flag_saving transitions driven by R key in main.cc
    {
      bool saving_now;
      {
        std::lock_guard<std::mutex> lock(_ctrl_mtx);
        saving_now = _ctrl_flag_saving;
      }
      if (saving_now && !saving_was_active) {
        _state_teleop_seq_id = 0;
        json_file_start(_ctrl_teleop_data_stream);
        save_teleop_data_json(_ctrl_teleop_data_stream, _state_teleop_seq_id++,
                              get_timestamp_now_ms(), (int)btn1, (int)btn2, (int)amplify_mode);
        json_frame_ending(_ctrl_teleop_data_stream);
        _state_teleop_thread_saving = true;
      } else if (!saving_now && saving_was_active) {
        save_teleop_data_json(_ctrl_teleop_data_stream, _state_teleop_seq_id++,
                              get_timestamp_now_ms(), (int)btn1, (int)btn2, (int)amplify_mode);
        json_file_ending(_ctrl_teleop_data_stream);
        _ctrl_teleop_data_stream.close();
        _state_teleop_thread_saving = false;
      }
      saving_was_active = saving_now;
    }

    // JY: check Enter key start request from main.cc — only activates, never deactivates
    if (!teleop_active_local) {
      bool start_req = false;
      {
        std::lock_guard<std::mutex> lock(_ctrl_mtx);
        if (_teleop_start_requested) {
          _teleop_start_requested = false;
          start_req = true;
        }
      }
      if (start_req) {
        ref_x = state.position[0] / 1000.0;
        ref_y = state.position[1] / 1000.0;
        ref_z = state.position[2] / 1000.0;
        ref_gimbals[0] = state.gimbal_angles[0];
        ref_gimbals[1] = state.gimbal_angles[1];
        ref_gimbals[2] = state.gimbal_angles[2];
        // JY: tare from mean of last 100 wrench readings
        if (_config.run_wrench_thread) {
          Eigen::MatrixXd w = get_wrench(100, 0);
          for (int i = 0; i < 3; i++)
            hap_wrench_offset[i] = w.row(i).mean();
          if (_config.haptic_gravity.norm() > 0.0) {
            RUT::Vector7d tare_pose;
            {
              std::lock_guard<std::mutex> lock(_poses_fb_mtxs[0]);
              tare_pose = _poses_fb[0];
            }
            Eigen::Matrix3d R_tare =
                RUT::quat2SO3(tare_pose[3], tare_pose[4], tare_pose[5], tare_pose[6]);
            GinF_tare = R_tare.transpose() * _config.haptic_gravity;
          }
        }
        f_filt.setZero();
        f_cmd_prev.setZero();
        hap_ramp = 0.0;
        for (int i = 0; i < 3; i++) hap_eq_pos[i] = state.position[i];
        for (int i = 0; i < 3; i++) k_prev[i] = 0.0;
        if (!_config.mock_hardware)
          robot_ptrs[0]->getCartesian(robot_initial_pose);
        else
          robot_initial_pose << 0, 0, 0, 1, 0, 0, 0;
        q_robot_initial = Eigen::Quaterniond(
            robot_initial_pose[3], robot_initial_pose[4],
            robot_initial_pose[5], robot_initial_pose[6]);
        q_prev = q_robot_initial;
        amplify_mode = false;
        {
          std::lock_guard<std::mutex> lock(_ctrl_mtx);
          _teleop_active = true;
        }
        teleop_active_local = true;
        was_btn1 = btn1;  // JY: seed so held buttons don't fire as rising edges immediately
        was_btn2 = btn2;
        std::cout << header << "Teleop activated.\n";
      }
      if (!teleop_active_local) {
        was_btn1 = btn1;
        was_btn2 = btn2;
        usleep(5000);  // 200 Hz
        continue;
      }
    }

    // ── Active: Button 1 + Button 2 together -> deactivate ───────────────
    if (btn1 && btn2) {
      {
        std::lock_guard<std::mutex> lock(_ctrl_mtx);
        _teleop_active = false;
      }
      teleop_active_local = false;
      amplify_mode = false;
      {
        std::lock_guard<std::mutex> lock(_ctrl_mtx);
        _amplify_mode = false;  // JY
        _idle_mode    = false;  // JY
      }
      set_high_level_maintain_position();
      _touch_ptr->disableForce();
      std::cout << header << "Teleop deactivated.\n";

      // Wait for both buttons to be fully released before returning to idle
      // so the next Enter press does not re-trigger immediately from stale state.
      bool should_exit = false;
      while (true) {
        {
          std::lock_guard<std::mutex> lock(_ctrl_mtx);
          if (!_ctrl_flag_running) { should_exit = true; break; }
        }
        _touch_ptr->getState(state);
        if (!((state.buttons & HD_DEVICE_BUTTON_1) != 0) &&
            !((state.buttons & HD_DEVICE_BUTTON_2) != 0))
          break;
        usleep(5000);
      }
      if (should_exit) break;

      was_btn1 = false;
      was_btn2 = false;
      usleep(5000);
      continue;
    }

    // ── Button 1 rising edge -> enter amplify mode ────────────────────────
    // JY: moved from Button 2; same re-reference logic
    if (btn1 && !was_btn1) {
      if (!_config.mock_hardware)
        robot_ptrs[0]->getCartesian(robot_initial_pose);
      ref_x = state.position[0] / 1000.0;
      ref_y = state.position[1] / 1000.0;
      ref_z = state.position[2] / 1000.0;
      ref_gimbals[0] = state.gimbal_angles[0];
      ref_gimbals[1] = state.gimbal_angles[1];
      ref_gimbals[2] = state.gimbal_angles[2];
      q_robot_initial = Eigen::Quaterniond(
          robot_initial_pose[3], robot_initial_pose[4],
          robot_initial_pose[5], robot_initial_pose[6]);
      q_prev = q_robot_initial;
      amplify_mode = true;
      { std::lock_guard<std::mutex> lock(_ctrl_mtx); _amplify_mode = true; }  // JY
      if (_state_teleop_thread_saving) {  // JY: log if recording active
        save_teleop_data_json(_ctrl_teleop_data_stream, _state_teleop_seq_id++,
                              get_timestamp_now_ms(), 1, (int)btn2, 1);
        json_frame_ending(_ctrl_teleop_data_stream);
      }
      std::cout << header << "Amplify mode ON.\n";
    }
    // ── Button 1 falling edge -> exit amplify, re-reference ────────────────
    // JY: same re-reference on exit so normal mode resumes with zero displacement
    else if (!btn1 && was_btn1 && amplify_mode) {
      if (!_config.mock_hardware)
        robot_ptrs[0]->getCartesian(robot_initial_pose);
      ref_x = state.position[0] / 1000.0;
      ref_y = state.position[1] / 1000.0;
      ref_z = state.position[2] / 1000.0;
      ref_gimbals[0] = state.gimbal_angles[0];
      ref_gimbals[1] = state.gimbal_angles[1];
      ref_gimbals[2] = state.gimbal_angles[2];
      q_robot_initial = Eigen::Quaterniond(
          robot_initial_pose[3], robot_initial_pose[4],
          robot_initial_pose[5], robot_initial_pose[6]);
      q_prev = q_robot_initial;
      amplify_mode = false;
      { std::lock_guard<std::mutex> lock(_ctrl_mtx); _amplify_mode = false; }  // JY
      if (_state_teleop_thread_saving) {  // JY: log if recording active
        save_teleop_data_json(_ctrl_teleop_data_stream, _state_teleop_seq_id++,
                              get_timestamp_now_ms(), 0, (int)btn2, 0);
        json_frame_ending(_ctrl_teleop_data_stream);
      }
      std::cout << header << "Amplify mode OFF. Re-referenced.\n";
    }

    // ── Button 2 held -> idle mode (robot ignores Touch position) ──────────
    // JY: on falling edge re-reference so resuming does not cause a position jump
    if (btn2 && !was_btn2) {
      { std::lock_guard<std::mutex> lock(_ctrl_mtx); _idle_mode = true; }  // JY
      if (_config.run_wrench_thread) {  // JY: zero haptic force immediately on idle entry — Touch holds last commanded force otherwise
        double zero_f[3] = {0.0, 0.0, 0.0};
        _touch_ptr->setForce(zero_f);
      }
      std::cout << header << "Idle mode ON.\n";
    } else if (!btn2 && was_btn2) {
      { std::lock_guard<std::mutex> lock(_ctrl_mtx); _idle_mode = false; }  // JY
      if (!_config.mock_hardware)
        robot_ptrs[0]->getCartesian(robot_initial_pose);
      ref_x = state.position[0] / 1000.0;
      ref_y = state.position[1] / 1000.0;
      ref_z = state.position[2] / 1000.0;
      ref_gimbals[0] = state.gimbal_angles[0];
      ref_gimbals[1] = state.gimbal_angles[1];
      ref_gimbals[2] = state.gimbal_angles[2];
      q_robot_initial = Eigen::Quaterniond(
          robot_initial_pose[3], robot_initial_pose[4],
          robot_initial_pose[5], robot_initial_pose[6]);
      q_prev = q_robot_initial;
      for (int i = 0; i < 3; i++) hap_eq_pos[i] = state.position[i];  // JY: reset spring eq to current Touch pos on idle exit
      for (int i = 0; i < 3; i++) k_prev[i] = 0.0;  // JY: reset stiffness slew so force ramps up from zero on idle exit
      f_filt.setZero();  // JY: clear IIR filter state accumulated before idle
      hap_ramp = 0.0;    // JY: reset contact ramp so force builds up gradually on idle exit
      std::cout << header << "Idle mode OFF. Re-referenced.\n";
    }
    if (btn2) {  // JY: skip position and haptic while idle
      was_btn1 = btn1;
      was_btn2 = btn2;
      usleep(static_cast<useconds_t>(std::round(dt_hap * 1.0e6)));
      continue;
    }

    // JY: Orientation via gimbal angles — matches MAA_data_collection_v2.py exactly.
    // Previous approach (quaternion delta from HD_CURRENT_TRANSFORM) had no axis
    // remapping and used a different data source, causing wrong rotation directions.
    const double orientation_scale = _touch_config.orientation_scale;  // JY: from touch.orientation_scale in YAML
    double j_rel[3] = {
        orientation_scale * (state.gimbal_angles[0] - ref_gimbals[0]),
        orientation_scale * (state.gimbal_angles[1] - ref_gimbals[1]),
        orientation_scale * (state.gimbal_angles[2] - ref_gimbals[2])
    };
    // hd_j_rel[[0,1]] = -hd_j_rel[[1,0]]: swap axes 0 & 1 and negate both
    double tmp0 = j_rel[0], tmp1 = j_rel[1];
    j_rel[0] = -tmp1;
    j_rel[1] = -tmp0;
    // Intrinsic XYZ Euler -> rotation (matches scipy R.from_euler('xyz', ...))
    Eigen::Matrix3d hd_R_rel =
        (Eigen::AngleAxisd(j_rel[0], Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(j_rel[1], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(j_rel[2], Eigen::Vector3d::UnitZ())).toRotationMatrix();
    Eigen::Quaterniond q_new =
        (q_robot_initial * Eigen::Quaterniond(hd_R_rel)).normalized();
    if (q_prev.dot(q_new) < 0) q_new.coeffs() = -q_new.coeffs();
    q_prev = q_new;

    // ── Position ──────────────────────────────────────────────────────────
    RUT::Vector7d target_pose = robot_initial_pose;

    // JY: both modes use the same references; amplify just applies amp_scale_multiplier on top
    {
      double scale = _touch_config.pos_scale *
                     (amplify_mode ? _touch_config.amp_scale_multiplier : 1.0);  // JY
      double px = scale * (state.position[0] / 1000.0 - ref_x);
      double py = scale * (state.position[1] / 1000.0 - ref_y);
      double pz = scale * (state.position[2] / 1000.0 - ref_z);
      Eigen::Vector3d pos_corrected = hd2rob_R * Eigen::Vector3d(px, py, pz);
      target_pose[0] = -pos_corrected[0] + robot_initial_pose[0];
      target_pose[1] = -pos_corrected[1] + robot_initial_pose[1];
      target_pose[2] =  pos_corrected[2] + robot_initial_pose[2];
    }

    target_pose[3] = q_new.w();
    target_pose[4] = q_new.x();
    target_pose[5] = q_new.y();
    target_pose[6] = q_new.z();

    set_target_pose(target_pose, 5.0, 0);  // 5 ms lookahead

    // JY: haptic force feedback — force-dependent spring-damper on Touch device
    if (_config.run_wrench_thread) {
      // JY: median of last 3 wrench readings; guard against empty buffer
      Eigen::Vector3d haptic_raw = Eigen::Vector3d::Zero();
      {
        Eigen::MatrixXd w = get_wrench(3, 0);
        if (w.rows() >= 6 && w.cols() >= 2) {  // JY: need >=2 cols for nth_element median
          for (int i = 0; i < 3; i++) {
            Eigen::VectorXd row = w.row(i);
            std::nth_element(row.data(), row.data() + 1, row.data() + row.size());
            haptic_raw[i] = row[1];
          }
        }
      }
      Eigen::Vector3d delta_f = haptic_raw - hap_wrench_offset;  // JY: tare-subtracted contact force

      // JY: cancel gravity drift when robot orientation changes after tare. Possibly remove this because it might be unnecessary
      if (_config.haptic_gravity.norm() > 0.0) {
        RUT::Vector7d cur_pose;
        {
          std::lock_guard<std::mutex> lock(_poses_fb_mtxs[0]);
          cur_pose = _poses_fb[0];
        }
        Eigen::Matrix3d R_now =
            RUT::quat2SO3(cur_pose[3], cur_pose[4], cur_pose[5], cur_pose[6]);
        Eigen::Vector3d GinF_now = R_now.transpose() * _config.haptic_gravity;
        delta_f += (GinF_now - GinF_tare);
      }

      // JY: optional filtering — kept for reference, disabled by default (haptic_filtering_enabled: false)
      if (_config.haptic_filtering_enabled) {
        // Deadband: zero any axis below threshold
        for (int i = 0; i < 3; i++)
          if (std::abs(delta_f[i]) < _config.haptic_deadband) delta_f[i] = 0.0;

        // Contact ramp: soft-start (ramp_up_time) / soft-stop (ramp_down_time)
        bool in_contact = (delta_f.norm() > _config.haptic_contact_th);
        double target_ramp = in_contact ? 1.0 : 0.0;
        double tau = std::max(1e-3, (target_ramp > hap_ramp)
                                    ? _config.haptic_ramp_up_time
                                    : _config.haptic_ramp_down_time);
        hap_ramp += std::min(1.0, dt_hap / tau) * (target_ramp - hap_ramp);
        delta_f *= hap_ramp;

        // 1st-order IIR lowpass: y = y_prev + a*(x - y_prev), a = dt/(rc+dt)
        const double rc = 1.0 / (2.0 * M_PI * _config.haptic_iir_cutoff_hz);
        const double a  = dt_hap / (rc + dt_hap);
        f_filt += a * (delta_f - f_filt);
      } else {
        f_filt = delta_f;
      }

      // JY: axis remap — robot [X,Y,Z] → Touch device axes [0,1,2]
      // Matches the original directionality: Fx → axis 0, -Fz*z_mult → axis 1, Fy → axis 2
      double f_touch[3] = {
        f_filt[0],
        -f_filt[2] * _config.haptic_z_multiplier,
        f_filt[1]
      };

      // JY: freeze spring equilibrium globally when any contact force crosses threshold;
      //     below threshold, equilibrium tracks hand so spring produces zero displacement.
      double contact_norm = f_filt.norm();
      if (contact_norm < _config.haptic_contact_th) {
        for (int i = 0; i < 3; i++) hap_eq_pos[i] = state.position[i];
      }

      // JY: per-axis directional spring-damper — stiffness and damping on each Touch axis
      //     scale with the contact force in that axis after remap, so only the axes where
      //     the robot actually feels force become stiff/damped.
      double f_spring[3];
      for (int i = 0; i < 3; i++) {
        double k_target = std::min(std::abs(f_touch[i]) * _config.haptic_k_per_N,
                                   _touch_ptr->getMaxStiffness());  // JY: N/mm, unclamped target
        // JY: slew-rate limit on stiffness rise only — prevents abrupt force jump when contact is made;
        //     stiffness is allowed to drop instantly so the device feels free immediately on release.
        double k_i = std::min(k_target, k_prev[i] + _config.haptic_k_slew_rate * dt_hap);  // JY: N/mm
        k_prev[i] = k_i;  // JY: save for next step
        double b_i = std::min(std::abs(f_touch[i]) * _config.haptic_b_per_N,
                              _touch_ptr->getMaxDamping());    // JY: N·s/mm
        f_spring[i] = -k_i * (state.position[i] - hap_eq_pos[i]) - b_i * state.velocity[i]; //MAA: equation for stiffness
        f_spring[i] = std::clamp(f_spring[i], -_config.haptic_f_max, _config.haptic_f_max);
      }
      // std::cout << header << "f_touch = [" << f_touch[0] << ", " << f_touch[1] << ", " << f_touch[2]
      //           << "] N, f_spring = [" << f_spring[0] << ", " << f_spring[1] << ", " << f_spring[2]
      //           << "] N\n";
      _touch_ptr->setForce(f_spring);
    }

    was_btn1 = btn1;
    was_btn2 = btn2;
    usleep(static_cast<useconds_t>(std::round(dt_hap * 1.0e6)));  // JY: sleep derived from dt_hap so loop rate stays consistent with dt used in slew/ramp calculations
  }

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _teleop_active = false;
  }
  // JY: if teleop was still active when loop exited (e.g. q pressed mid-session), stop robot cleanly
  if (teleop_active_local) set_high_level_maintain_position();
  // JY: if loop exits while recording was still active, close stream cleanly
  if (_state_teleop_thread_saving) {
    save_teleop_data_json(_ctrl_teleop_data_stream, _state_teleop_seq_id++,
                          get_timestamp_now_ms(), 0, 0, 0);
    json_file_ending(_ctrl_teleop_data_stream);
    _ctrl_teleop_data_stream.close();
    _state_teleop_thread_saving = false;
  }
  stop_saving_data();  // JY: signal other threads to stop saving if still active
  if (_touch_ptr) _touch_ptr->disableForce();
  std::cout << header << "Joined." << std::endl;
}
