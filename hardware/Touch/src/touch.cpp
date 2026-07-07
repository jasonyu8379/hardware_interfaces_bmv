#include "touch/touch.h"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <unistd.h>

namespace {
struct CopyArgs {
    const Touch::TouchState* src;
    Touch::TouchState*       dst;
};
}  // namespace

void Touch::transform_to_euler(const double m[16],
                                double& roll, double& pitch, double& yaw) {
    double r00 = m[0], r10 = m[1], r20 = m[2];
    double r11 = m[5], r12 = m[9];
    double r21 = m[6], r22 = m[10];

    pitch = std::asin(-std::clamp(r20, -1.0, 1.0));
    if (std::fabs(r20) < 0.9999) {
        roll = std::atan2(r21, r22);
        yaw  = std::atan2(r10, r00);
    } else {
        roll = std::atan2(-r12, r11);
        yaw  = 0.0;
    }
    const double RAD2DEG = 180.0 / M_PI;
    roll  *= RAD2DEG;
    pitch *= RAD2DEG;
    yaw   *= RAD2DEG;
}

HDCallbackCode HDCALLBACK Touch::schedulerCallback(void* pUserData) {
    Touch* self = static_cast<Touch*>(pUserData);

    hdBeginFrame(hdGetCurrentDevice());

    double transform[16];
    hdGetDoublev(HD_CURRENT_TRANSFORM, transform);
    self->_state.position[0] = transform[12];
    self->_state.position[1] = transform[13];
    self->_state.position[2] = transform[14];

    transform_to_euler(transform,
                       self->_state.orientation[0],
                       self->_state.orientation[1],
                       self->_state.orientation[2]);

    hdGetDoublev(HD_CURRENT_VELOCITY, self->_state.velocity);
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, self->_state.gimbal_angles);
    hdGetIntegerv(HD_CURRENT_BUTTONS, &self->_state.buttons);

    // Extract quaternion directly from the rotation submatrix of the transform.
    // Column-major layout: m[0..2] = col0, m[4..6] = col1, m[8..10] = col2.
    Eigen::Matrix3d R;
    R << transform[0], transform[4], transform[8],
         transform[1], transform[5], transform[9],
         transform[2], transform[6], transform[10];
    Eigen::Quaterniond q(R);
    q.normalize();
    self->_state.quat[0] = q.w();
    self->_state.quat[1] = q.x();
    self->_state.quat[2] = q.y();
    self->_state.quat[3] = q.z();

    double force[3];
    for (int i = 0; i < 3; i++) {
        force[i] = self->_force_enabled ? self->_force_cmd[i] : 0.0;  // JY: removed fixed stiffness/damping; force-dependent spring computed in teleop_loop
        force[i] = std::clamp(force[i], -self->_max_haptic_force, self->_max_haptic_force);
    }
    hdSetDoublev(HD_CURRENT_FORCE, force);

    hdEndFrame(hdGetCurrentDevice());
    return HD_CALLBACK_CONTINUE;
}

HDCallbackCode HDCALLBACK Touch::copyCallback(void* pUserData) {
    auto* args = static_cast<CopyArgs*>(pUserData);
    *args->dst = *args->src;
    return HD_CALLBACK_DONE;
}

Touch::~Touch() {
    close();
}

bool Touch::init(RUT::TimePoint /*time0*/, const TouchConfig& config) {
    HDErrorInfo error;

    const char* dev = (config.device_name == "Default Device")
                      ? HD_DEFAULT_DEVICE
                      : config.device_name.c_str();
    _hHD = hdInitDevice(dev);
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "[Touch] Failed to initialize device");
        return false;
    }

    hdGetDoublev(HD_NOMINAL_MAX_STIFFNESS, &_max_stiffness);  // JY: kept so teleop_loop can clamp force-dependent spring
    hdGetDoublev(HD_NOMINAL_MAX_DAMPING,   &_max_damping);    // JY: same
    _max_haptic_force = config.max_haptic_force;

    _hUpdate = hdScheduleAsynchronous(
        schedulerCallback, this, HD_MAX_SCHEDULER_PRIORITY);

    hdEnable(HD_FORCE_OUTPUT);
    hdStartScheduler();

    if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "[Touch] Failed to start scheduler");
        hdDisableDevice(_hHD);
        return false;
    }

    _flag_started = true;
    if (config.print_flag)
        printf("[Touch] Initialized '%s'. Max stiffness: %.4f  Max damping: %.6f\n",
               config.device_name.c_str(), _max_stiffness, _max_damping);
    else
        printf("[Touch] Initialized. Max stiffness: %.4f  Max damping: %.6f\n",
               _max_stiffness, _max_damping);
    return true;
}

void Touch::getState(TouchState& out) {
    CopyArgs args{&_state, &out};
    hdScheduleSynchronous(copyCallback, &args, HD_MIN_SCHEDULER_PRIORITY);
}

void Touch::setForce(const double force[3]) {
    memcpy(_force_cmd, force, sizeof(_force_cmd));
    _force_enabled = true;
}

void Touch::disableForce() {
    _force_enabled = false;
    memset(_force_cmd, 0, sizeof(_force_cmd));
}

void Touch::close() {
    if (!_flag_started)
        return;
    _force_enabled = false;
    memset(_force_cmd, 0, sizeof(_force_cmd));
    usleep(100000);  // let scheduler apply zero force
    hdStopScheduler();
    hdUnschedule(_hUpdate);
    hdDisableDevice(_hHD);
    _flag_started = false;
}

bool Touch::TouchConfig::deserialize(const YAML::Node& node) {
    try {
        if (node["device_name"])
            device_name = node["device_name"].as<std::string>();
        if (node["pos_scale"])
            pos_scale = node["pos_scale"].as<double>();
        if (node["orientation_scale"])
            orientation_scale = node["orientation_scale"].as<double>();  // JY
        if (node["amp_scale_multiplier"])
            amp_scale_multiplier = node["amp_scale_multiplier"].as<double>();  // JY
        if (node["force_scale"])
            force_scale = node["force_scale"].as<double>();
        if (node["max_haptic_force"])
            max_haptic_force = node["max_haptic_force"].as<double>();
        if (node["hd2rob_euler_deg"]) {
            const auto& v = node["hd2rob_euler_deg"];
            for (int i = 0; i < 3; ++i)
                hd2rob_euler_deg[i] = v[i].as<double>();
        }
        if (node["print_flag"])
            print_flag = node["print_flag"].as<bool>();
    } catch (const std::exception& e) {
        std::cerr << "[Touch] Config deserialization failed: "
                  << e.what() << "\n";
        return false;
    }
    return true;
}
