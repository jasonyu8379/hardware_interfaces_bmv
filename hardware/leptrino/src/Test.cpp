#include "Leptrino_ft.h"
#include <iostream>
#include <iomanip>
#include <unistd.h>

int main() {
    RUT::Timer     timer;
    RUT::TimePoint time0 = timer.tic();

    Leptrinoft::LEPTRINOFTConfig config;
    config.sensor_name      = "leptrino";
    config.fullpath         = "";
    config.print_flag       = true;
    config.publish_rate     = 1000.0;
    config.noise_level      = 0.0;
    config.stall_threshold  = 100;
    config.Foffset          << -0.767,  0.155, -0.053;
    config.Toffset          << -0.021, -0.112, -0.010;
    config.Gravity          <<  0.484,  0.037, -0.948;
    config.Pcom             << -0.003,  0.005,  0.159;
    config.WrenchSafety     << 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0;
    config.PoseSensorTool   <<  0.0, 0.0, -0.222, 1.0, 0.0, 0.0, 0.0;

    Leptrinoft leptrino;
    if (!leptrino.init(time0, config)) {
        std::cerr << "[Error] Failed to initialize Leptrino hardware." << std::endl;
        return -1;
    }

    const RUT::Vector7d pose = (RUT::Vector7d() << 0.4, 0.4, 0.2, 1.0, 0.0, 0.0, 0.0).finished();
    RUT::VectorXd wrench_net_T;
    RUT::VectorXd wrench;

    std::cout << std::fixed << std::setprecision(6);
    while (true) {
        int status = leptrino.getWrenchNetTool(pose, wrench_net_T);
        // int status = leptrino.getWrenchSensor(wrench_net_T);
        if (status == -1) {
            std::cerr << "[Error] Wrench exceeded safety threshold. Exiting." << std::endl;
            break;
        }
        if (status == 2)
            std::cerr << "[Warning] Dead stream detected." << std::endl;

        std::cout << wrench_net_T.transpose() << "\n";
        usleep(1000);
    }

    return 0;
}