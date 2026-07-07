#pragma once

#include <HD/hd.h>
#include <HDU/hduError.h>
#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>
#include <yaml-cpp/yaml.h>
#include <cstring>
#include <string>

class Touch {
 public:
  struct TouchConfig {
    // Device identification
    std::string device_name{"Default Device"};

    // Teleoperation scaling
    // pos_scale: multiplier applied to Touch delta_pos (mm) before unit conversion
    //   robot_delta_m = touch_delta_mm * pos_scale * 1e-3
    double pos_scale{1.0};
    double orientation_scale{1.0};       // JY: multiplier applied to gimbal angle delta before sending to robot
    double amp_scale_multiplier{2.0};    // JY: additional pos_scale multiplier when amplify mode (button 2) is active

    // force_scale: multiplier applied to robot wrench (N) for haptic feedback
    //   touch_force_N = robot_wrench_N * force_scale
    double force_scale{0.05};

    // max_haptic_force: safety clamp on haptic output (N)
    double max_haptic_force{3.0};

    // hd2rob_euler_deg: extrinsic XYZ Euler angles (degrees) rotating the
    //   Touch HD API frame into the robot world frame.
    //   Adjust based on physical mounting of the Touch relative to the robot.
    RUT::Vector3d hd2rob_euler_deg{90.0, 90.0, 90.0};

    // print_flag: enable verbose state logging
    bool print_flag{false};

    bool deserialize(const YAML::Node& node);
  };

  struct TouchState {
    double position[3]{0, 0, 0};       // Cartesian position in mm
    double orientation[3]{0, 0, 0};    // roll, pitch, yaw in degrees
    double velocity[3]{0, 0, 0};       // mm/s
    double quat[4]{1, 0, 0, 0};        // orientation as quaternion [w, x, y, z]
    double gimbal_angles[3]{0, 0, 0};  // HD_CURRENT_GIMBAL_ANGLES, radians
    int    buttons{0};
  };

  Touch() = default;
  ~Touch();

  bool   init(RUT::TimePoint time0, const TouchConfig& config);
  bool   is_data_ready() const { return _flag_started; }
  void   getState(TouchState& out);
  void   setForce(const double force[3]);
  void   disableForce();
  double getMaxStiffness() const { return _max_stiffness; }
  double getMaxDamping()   const { return _max_damping; }
  void   close();

 private:
  static void          transform_to_euler(const double m[16],
                                          double& roll, double& pitch,
                                          double& yaw);
  static HDCallbackCode HDCALLBACK schedulerCallback(void* pUserData);
  static HDCallbackCode HDCALLBACK copyCallback(void* pUserData);

  HHD               _hHD{HD_INVALID_HANDLE};
  HDSchedulerHandle _hUpdate{HD_INVALID_HANDLE};

  // written by scheduler, read via copyCallback (HD-thread-safe)
  TouchState _state;

  // written by main thread, read by scheduler (loose sync, acceptable)
  double _force_cmd[3]{0, 0, 0};
  bool   _force_enabled{false};

  double _max_stiffness{0};   // JY: kept for teleop_loop to clamp force-dependent spring
  double _max_damping{0};     // JY: kept for teleop_loop to clamp force-dependent damping
  double _max_haptic_force{3.0};
  bool   _flag_started{false};
};
