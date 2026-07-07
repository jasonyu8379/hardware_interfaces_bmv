#include "Leptrino_ft.h"

using namespace RUT;

// ─── Helpers ──────────────────────────────────────────────────────────────────

static void set_realtime_priority() {
    pthread_t this_thread = pthread_self();
    sched_param sch{};
    int policy;
    pthread_getschedparam(this_thread, &policy, &sch);
    sch.sched_priority = 99;
    if (pthread_setschedparam(this_thread, SCHED_FIFO, &sch))
        std::cerr << "[Leptrino] Failed to set thread priority: " << std::strerror(errno) << "\n";
    else
        std::cout << "[Leptrino] Thread priority set to " << sch.sched_priority << "\n";
}

// ─── Monitor Thread ───────────────────────────────────────────────────────────

static void* LeptrinoMonitor(void* pParam) {
    auto* lep = static_cast<Leptrinoft*>(pParam);

    RUT::Timer loop_timer;
    loop_timer.set_loop_rate_hz(lep->_config.publish_rate);
    loop_timer.start_timed_loop();
    loop_timer.tic();

    set_realtime_priority();

    while (!lep->_stop_flag.load()) {
        pthread_mutex_lock(&lep->_netft->dataMutex);

        if (lep->_netft->forceTorqueData.dataAvailable) {
            const ForceTorqueData& ftd = lep->_netft->getForceTorqueData();
            lep->_force  = { ftd.force[0], ftd.force[1], ftd.force[2] };
            lep->_torque = { ftd.force[3], ftd.force[4], ftd.force[5] };
            lep->_netft->forceTorqueData.dataAvailable = false;
            lep->_flag_started = true;
        }

        pthread_mutex_unlock(&lep->_netft->dataMutex);

        if (lep->_config.print_flag) {
            lep->_file << RUT::Clock::now().time_since_epoch().count() << "\t"
                       << lep->_force.transpose()  << "\t"
                       << lep->_torque.transpose() << "\n";
        }

        loop_timer.sleep_till_next();
    }

    return nullptr;
}

// ─── Constructor / Destructor ─────────────────────────────────────────────────

Leptrinoft::Leptrinoft() {
    _force        = Vector3d::Zero();
    _force_old    = Vector3d::Zero();
    _torque       = Vector3d::Zero();
    _torque_old   = Vector3d::Zero();
    _stall_counts = 0;
    _wrench_sensor_temp.resize(6);
    _wrench_tool_temp.resize(6);
}

Leptrinoft::~Leptrinoft() {
    _stop_flag.store(true);

    if (_netft)
        _netft->App_Close();

    if (_thread_started)
        pthread_join(_thread, nullptr);

    if (_config.print_flag)
        _file.close();
}

// ─── Init ─────────────────────────────────────────────────────────────────────

bool Leptrinoft::init(const RUT::TimePoint& time0, const LEPTRINOFTConfig& config) {
    std::cout << "[Leptrino] Initializing..." << std::endl;
    _time0        = time0;
    _config       = config;
    _flag_started = false;

    _netft = std::make_shared<Leptrino_driver>();
    _adj_sensor_tool = SE32Adj(pose2SE3(config.PoseSensorTool));

    if (_config.print_flag) {
        _file.open(config.fullpath);
        if (_file.is_open())
            std::cout << "[Leptrino] Log file opened successfully." << std::endl;
        else
            std::cerr << "[Leptrino] Failed to open log file." << std::endl;
    }

    _netft->App_Init();

    std::cout << "[Leptrino] Creating monitor thread..." << std::endl;
    if (pthread_create(&_thread, nullptr, LeptrinoMonitor, this) != 0) {
        std::cerr << "[Leptrino] Failed to create monitor thread." << std::endl;
        return false;
    }
    _thread_started = true;

    std::cout << "[Leptrino] Initialized successfully." << std::endl;
    return true;
}

// ─── Interface ────────────────────────────────────────────────────────────────

int Leptrinoft::getNumSensors() { return 1; }

int Leptrinoft::getWrenchSensor(RUT::VectorXd& wrench, int num_of_sensors) {
    wrench.resize(6);
    {
        pthread_mutex_lock(&_netft->dataMutex);
        wrench.head(3) = _force;
        wrench.tail(3) = _torque;
        pthread_mutex_unlock(&_netft->dataMutex);
    }

    double data_change = (wrench.head(3) - _force_old).norm()
                       + 10.0 * (wrench.tail(3) - _torque_old).norm();


    _force_old  = _force;
    _torque_old = _torque;

    if (data_change > _config.noise_level) {
        _stall_counts = 0;
    } else if (++_stall_counts >= _config.stall_threshold) {
        return 2;
    }

    for (int i = 0; i < 6; ++i) {
        if (std::abs(wrench[i]) > _config.WrenchSafety[i]) {
            std::cerr << "\033[1;31m[Leptrino] Wrench exceeds safety threshold:\033[0m\n"
                      << "  feedback:     " << wrench.transpose()               << "\n"
                      << "  safety limit: " << _config.WrenchSafety.transpose() << "\n";
            return -1;
        }
    }

    return 0;
}

int Leptrinoft::getWrenchTool(RUT::VectorXd& wrench, int num_of_sensors) {
    int flag = getWrenchSensor(_wrench_sensor_temp);
    wrench = _adj_sensor_tool.transpose() * _wrench_sensor_temp;
    return flag;
}

int Leptrinoft::getWrenchNetTool(const RUT::Vector7d& pose, RUT::VectorXd& wrench, int num_of_sensors) {
    int flag = getWrenchTool(_wrench_tool_temp);

    _R_WT = RUT::quat2SO3(pose[3], pose[4], pose[5], pose[6]);
    _GinF = _R_WT.transpose() * _config.Gravity;
    _GinT = _config.Pcom.cross(_GinF);

    wrench.resize(6);
    wrench.head(3) = _wrench_tool_temp.head(3) + _config.Foffset - _GinF;
    wrench.tail(3) = _wrench_tool_temp.tail(3) + _config.Toffset - _GinT;

    return flag;
}