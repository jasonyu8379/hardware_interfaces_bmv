#include "realsense/realsense.h"
#include "realsense/cv-helpers.hpp"

#include <iostream>
#include <unistd.h>
#include <librealsense2/rs.hpp>

struct Realsense::Implementation {
  Realsense::RealsenseConfig config{};
  RUT::TimePoint time0;

  rs2::pipeline pipe;
  std::shared_ptr<rs2::align> align_to_ptr;
  rs2::frameset frames;
  rs2::frame color_frame;
  int last_frame_number{-1};  // JY: per-instance (was static — shared across all cameras, causing multi-camera deadlock)

  Implementation();
  ~Implementation();

  bool initialize(RUT::TimePoint time0,
                  const Realsense::RealsenseConfig& config);
  bool start_pipeline();  // JY: extracted so recovery path can restart without reinitializing
  cv::Mat next_rgb_frame_blocking();
};

Realsense::Implementation::Implementation() {}

Realsense::Implementation::~Implementation() {
  std::cout << "[Realsense] finishing.." << std::endl;
}

bool Realsense::Implementation::start_pipeline() {
  // stream: https://intelrealsense.github.io/librealsense/doxygen/rs__sensor_8h.html#a01b4027af33139de861408872dd11b93
  // format: https://intelrealsense.github.io/librealsense/doxygen/rs__sensor_8h.html#ae04b7887ce35d16dbd9d2d295d23aac7
  // also see this issue for acceptable formats: https://github.com/IntelRealSense/librealsense/issues/6341
  rs2::config rs_cfg;
  if (!config.serial_number.empty())  // JY: pin to specific device so multiple cameras don't collide
    rs_cfg.enable_device(config.serial_number);
  if (config.enable_color && !config.enable_depth) {
    rs_cfg.enable_stream(rs2_stream::RS2_STREAM_COLOR, config.width,
                         config.height, rs2_format::RS2_FORMAT_ANY,
                         config.framerate);
  } else if (!config.enable_color && config.enable_depth) {
    rs_cfg.enable_stream(rs2_stream::RS2_STREAM_DEPTH, config.width,
                         config.height, rs2_format::RS2_FORMAT_Z16,
                         config.framerate);
  } else if (config.enable_color && config.enable_depth) {
    rs_cfg.enable_stream(rs2_stream::RS2_STREAM_ANY, config.width,
                         config.height, rs2_format::RS2_FORMAT_ANY,
                         config.framerate);
  } else {
    std::cerr << "[Realsense] Error: no stream enabled." << std::endl;
    return false;
  }
  pipe.start(rs_cfg);
  return true;
}

bool Realsense::Implementation::initialize(
    RUT::TimePoint time0, const Realsense::RealsenseConfig& realsense_config) {
  std::cout << "[Realsense] Initializing realsense pipeline.." << std::endl;
  time0 = time0;
  config = realsense_config;

  align_to_ptr = std::make_shared<rs2::align>(rs2_stream::RS2_STREAM_COLOR);

  if (!start_pipeline())
    return false;

  std::cout << "[Realsense] Pipeline started.\n";
  return true;
}

cv::Mat Realsense::Implementation::next_rgb_frame_blocking() {
  try {
    while (true) {
      frames = pipe.wait_for_frames();
      if (config.align_depth_to_color) {
        frames = align_to_ptr->process(frames);
      }
      color_frame = frames.get_color_frame();
      // JY: skip if frame number unchanged (using per-instance member, not static)
      if (static_cast<int>(color_frame.get_frame_number()) == last_frame_number)
        continue;
      last_frame_number = static_cast<int>(color_frame.get_frame_number());
      break;
    }
    return frame_to_mat(color_frame);
  } catch (const rs2::error& e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "("
              << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    // JY: pipeline timeout (stale UVC state after crash, or transient USB issue) —
    //     stop and restart to re-issue VIDIOC_STREAMON and clear the stale state.
    try {
      pipe.stop();
      usleep(500000);  // 0.5s — give the USB device time to settle before restarting
      if (start_pipeline()) {
        last_frame_number = -1;
        std::cerr << "[Realsense] Pipeline restarted after timeout.\n";
      }
    } catch (const std::exception& restart_err) {
      std::cerr << "[Realsense] Pipeline restart failed: " << restart_err.what() << "\n";
    }
    return cv::Mat();  // rgb thread skips empty frames and calls us again
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return cv::Mat();
  }
}

Realsense::Realsense() : m_impl{std::make_unique<Implementation>()} {}

Realsense::~Realsense() {}

bool Realsense::init(RUT::TimePoint time0,
                     const RealsenseConfig& realsense_config) {
  return m_impl->initialize(time0, realsense_config);
}

cv::Mat Realsense::next_rgb_frame_blocking() {
  return m_impl->next_rgb_frame_blocking();
}
