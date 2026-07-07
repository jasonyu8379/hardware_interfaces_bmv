#ifndef MAINAPPLICATION_H
#define MAINAPPLICATION_H

#include "SerialCommunication.h"
#include "Common.h"
#include <pthread.h>

struct ForceTorqueData {
    float force[FN_Num] = {};
    bool dataAvailable          = false;
    bool dataAvailableForLeptrino = false;
    int  leptrinoForce[6]       = {};
};

class Leptrino_driver {
public:
    Leptrino_driver();
    ~Leptrino_driver();

    void App_Init();
    void App_Close();
    void run();

    const ForceTorqueData& getForceTorqueData() const;
    ForceTorqueData&       getForceTorqueData();

    pthread_mutex_t  dataMutex;
    bool             comPortOpen;
    ForceTorqueData  forceTorqueData;

private:
    SerialComm comm;
    pthread_t  recvThread;

    UCHAR CommRcvBuff[256];
    UCHAR CommSendBuff[1024];
    UCHAR SendBuff[512];

    static void* receiveThreadFunction(void* arg);

    ULONG SendData(UCHAR* pucInput, USHORT usSize);
    void  buildAndSend(UCHAR cmd);
    void  GetProductInfo();
    void  SerialStart();
    void  SerialStop();
};

#endif // MAINAPPLICATION_H