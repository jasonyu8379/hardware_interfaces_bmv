#pragma once

#include <HD/hd.h>
#include <HDU/hduError.h>
#include <RobotUtilities/timer_linux.h>
#include <yaml-cpp/yaml.h>
#include <cstring>
#include <string>

class Touch {
 public:
  struct TouchConfig {
    std::string device_name{"Default Device"};
    bool deserialize(const YAML::Node& node);
  };

  struct TouchState {
    double position[3]{0, 0, 0};    // Cartesian position in mm
    double orientation[3]{0, 0, 0}; // roll, pitch, yaw in degrees
    double velocity[3]{0, 0, 0};    // mm/s
    double quat[4]{1, 0, 0, 0};     // orientation as quaternion [w, x, y, z]
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

  double _max_stiffness{0};
  double _max_damping{0};
  bool   _flag_started{false};
};
