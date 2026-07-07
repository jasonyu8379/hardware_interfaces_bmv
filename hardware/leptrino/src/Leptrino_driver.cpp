#include "SerialCommunication.h"
#include "Leptrino_driver.h"
#include "Common.h"
#include <iostream>
#include <iomanip>
#include <cstring>
#include <unistd.h>

static constexpr float RATED_FORCES[3]  = {300.0f, 300.0f, 300.0f};  // Fx, Fy, Fz (N)
static constexpr float RATED_TORQUES[3] = {4.0f,   4.0f,   2.0f};    // Mx, My, Mz (Nm)
static constexpr float SCALE_FACTOR     = 10000.0f;

// ─── Constructors / Destructors ───────────────────────────────────────────────

Leptrino_driver::Leptrino_driver() : comPortOpen(false) {
    pthread_mutex_init(&dataMutex, nullptr);
}

Leptrino_driver::~Leptrino_driver() {
    App_Close();
    pthread_mutex_destroy(&dataMutex);
}

// ─── Getters ──────────────────────────────────────────────────────────────────

const ForceTorqueData& Leptrino_driver::getForceTorqueData() const { return forceTorqueData; }
ForceTorqueData&       Leptrino_driver::getForceTorqueData()       { return forceTorqueData; }

// ─── Lifecycle ────────────────────────────────────────────────────────────────

void Leptrino_driver::App_Init() {
    comPortOpen = (comm.openDevice("/dev/ttyACM0") == OK);
    if (!comPortOpen) {
        std::cerr << "Failed to open serial port." << std::endl;
        return;
    }

    comm.setup(460800, PAR_NON, BIT_LEN_8, 0, 0, CHR_ETX);
    std::cout << "Serial port opened and configured successfully." << std::endl;

    if (pthread_create(&recvThread, nullptr, receiveThreadFunction, this) != 0) {
        std::cerr << "Failed to create receive thread." << std::endl;
        comPortOpen = false;
        return;
    }

    SerialStart();
}

void Leptrino_driver::App_Close() {
    if (!comPortOpen) return;

    std::cout << "Closing application." << std::endl;
    SerialStop();
    comm.closeDevice();

    pthread_cancel(recvThread);
    pthread_join(recvThread, nullptr);

    comPortOpen = false;
}

// ─── Communication ────────────────────────────────────────────────────────────

ULONG Leptrino_driver::SendData(UCHAR* pucInput, USHORT usSize) {
    UCHAR*  pucWrite  = CommSendBuff;
    USHORT  usRealSize = 2;
    UCHAR   ucBCC     = 0;

    *pucWrite++ = CHR_DLE;
    *pucWrite++ = CHR_STX;

    for (USHORT i = 0; i < usSize; ++i) {
        if (pucInput[i] == CHR_DLE) {
            *pucWrite++ = CHR_DLE;
            ++usRealSize;
        }
        *pucWrite++ = pucInput[i];
        ucBCC ^= pucInput[i];
        ++usRealSize;
    }

    *pucWrite++ = CHR_DLE;
    *pucWrite++ = CHR_ETX;
    *pucWrite   = ucBCC ^ CHR_ETX;

    return comm.sendData(CommSendBuff, usRealSize + 3);
}

void Leptrino_driver::buildAndSend(UCHAR cmd) {
    SendBuff[0] = 4;
    SendBuff[1] = 0xFF;
    SendBuff[2] = cmd;
    SendBuff[3] = 0;
    SendData(SendBuff, 4);
}

void Leptrino_driver::SerialStart() {
    std::cout << "Starting data transmission..." << std::endl;
    buildAndSend(CMD_DATA_START);
}

void Leptrino_driver::SerialStop() {
    std::cout << "Stopping data transmission..." << std::endl;
    buildAndSend(CMD_DATA_STOP);
}

void Leptrino_driver::GetProductInfo() {
    std::cout << "Requesting product info..." << std::endl;
    buildAndSend(CMD_GET_INF);

    for (int attempts = 0; attempts < 10; ++attempts) {
        usleep(100000);
        pthread_mutex_lock(&dataMutex);
        comm.receive();
        bool hasData = comm.checkReceivedData() > 0;
        if (hasData) comm.getReceivedData(CommRcvBuff);
        pthread_mutex_unlock(&dataMutex);

        if (hasData) {
            ST_R_GET_INF* info = reinterpret_cast<ST_R_GET_INF*>(CommRcvBuff);
            info->scFVer[F_VER_SIZE   - 1] = 0;
            info->scSerial[SERIAL_SIZE - 1] = 0;
            info->scPName[P_NAME_SIZE  - 1] = 0;

            std::cout << "Version: "       << info->scFVer   << "\n"
                      << "Serial Number: " << info->scSerial << "\n"
                      << "Type: "          << info->scPName  << "\n";
            return;
        }
    }
    std::cerr << "Product info not received." << std::endl;
}

// ─── Receive Thread ───────────────────────────────────────────────────────────

void* Leptrino_driver::receiveThreadFunction(void* arg) {
    auto* app = static_cast<Leptrino_driver*>(arg);

    while (true) {
        pthread_mutex_lock(&app->dataMutex);
        app->comm.receive();

        if (app->comm.checkReceivedData() > 0) {
            app->comm.getReceivedData(app->CommRcvBuff);
            const auto* fd = reinterpret_cast<ST_R_DATA_GET_F*>(app->CommRcvBuff);

            for (int i = 0; i < 3; ++i) {
                app->forceTorqueData.force[i]     = static_cast<float>(fd->ssForce[i])     * (RATED_FORCES[i]  / SCALE_FACTOR);
                app->forceTorqueData.force[i + 3] = static_cast<float>(fd->ssForce[i + 3]) * (RATED_TORQUES[i] / SCALE_FACTOR);
            }

            // std::cout << std::fixed << std::setprecision(3)
            //      << "Force: [" << app->forceTorqueData.force[0] << ", "
            //      << app->forceTorqueData.force[1] << ", "
            //      << app->forceTorqueData.force[2] << "] N, "
            //      << "Torque: [" << app->forceTorqueData.force[3] << ", "
            //      << app->forceTorqueData.force[4] << ", "
            //      << app->forceTorqueData.force[5] << "] Nm" << std::endl;

            app->forceTorqueData.dataAvailable = true;
        }

        pthread_mutex_unlock(&app->dataMutex);
        usleep(1000);
    }

    return nullptr;
}

// ─── Entry Point ──────────────────────────────────────────────────────────────

void Leptrino_driver::run() {
    App_Init();
    if (!comPortOpen) return;

    GetProductInfo();

    // Run for ~10 seconds at 1ms polling
    for (int i = 0; i < 10000; ++i)
        usleep(1000);

    App_Close();
}