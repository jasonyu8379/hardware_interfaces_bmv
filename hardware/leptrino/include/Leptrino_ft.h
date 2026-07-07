#pragma once

#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>
#include "hardware_interfaces/ft_interfaces.h"
#include "Leptrino_driver.h"
#include <yaml-cpp/yaml.h>
#include <pthread.h>
#include <atomic>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

class Leptrinoft : public FTInterfaces {
public:
    struct LEPTRINOFTConfig {
        double counts_per_force   {1000000.0};
        double counts_per_torque  {1000000.0};
        std::string sensor_name   {"leptrino"};
        std::string fullpath      {};
        bool   print_flag         {false};
        double publish_rate       {100.0};
        double noise_level        {0.0};
        int    stall_threshold    {50};
        RUT::Vector3d Foffset     {};
        RUT::Vector3d Toffset     {};
        RUT::Vector3d Gravity     {};
        RUT::Vector3d Pcom        {};
        RUT::Vector6d WrenchSafety{};
        RUT::Vector7d PoseSensorTool{};

        bool deserialize(const YAML::Node& node) {
            try {
                counts_per_force  = node["counts_per_force"].as<double>();
                counts_per_torque = node["counts_per_torque"].as<double>();
                sensor_name       = node["sensor_name"].as<std::string>();
                fullpath          = node["fullpath"].as<std::string>();
                print_flag        = node["print_flag"].as<bool>();
                publish_rate      = node["publish_rate"].as<double>();
                noise_level       = node["noise_level"].as<double>();
                stall_threshold   = node["stall_threshold"].as<int>();
                Foffset           = RUT::deserialize_vector<RUT::Vector3d>(node["Foffset"]);
                Toffset           = RUT::deserialize_vector<RUT::Vector3d>(node["Toffset"]);
                Gravity           = RUT::deserialize_vector<RUT::Vector3d>(node["Gravity"]);
                Pcom              = RUT::deserialize_vector<RUT::Vector3d>(node["Pcom"]);
                WrenchSafety      = RUT::deserialize_vector<RUT::Vector6d>(node["WrenchSafety"]);
                PoseSensorTool    = RUT::deserialize_vector<RUT::Vector7d>(node["PoseSensorTool"]);
            } catch (const std::exception& e) {
                std::cerr << "[Leptrino] Config deserialization failed: " << e.what() << "\n";
                return false;
            }
            return true;
        }
    };

    Leptrinoft();
    ~Leptrinoft();

    bool init(const RUT::TimePoint& time0, const LEPTRINOFTConfig& config);

    int getWrenchSensor(RUT::VectorXd& wrench, int num_of_sensors = 1)                             override;
    int getWrenchTool(RUT::VectorXd& wrench, int num_of_sensors = 1)                               override;
    int getWrenchNetTool(const RUT::Vector7d& pose, RUT::VectorXd& wrench, int num_of_sensors = 1) override;
    int getNumSensors()                                                                             override;

    RUT::TimePoint   _time0;
    std::ofstream    _file;
    LEPTRINOFTConfig _config;

    std::shared_ptr<Leptrino_driver> _netft;
    std::atomic<bool> _stop_flag{false};

private:
    pthread_t _thread;
    bool      _thread_started{false};
};