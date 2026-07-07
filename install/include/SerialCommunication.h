#ifndef _SERIALCOMM_H
#define _SERIALCOMM_H

#include "Common.h"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

class SerialComm {
public:
    SerialComm();
    ~SerialComm();

    int  openDevice(const char* dev);
    void closeDevice();
    void setup(long baud, int parity, int bitlen, int rts, int dtr, char code);
    int  sendData(UCHAR* buff, int len);
    int  checkReceivedData();
    int  getReceivedData(UCHAR* buff);
    void receive();

private:
    int fd;
    int p_rd, p_wr;
    int rcv_n;
    UCHAR delim;

    UCHAR rcv_buff[10][255];
    UCHAR stmp[255];
    struct termios tio;

    // Receive state machine (replaces static locals)
    enum class RcvState { Idle, GotDLE, Data, Escape };
    RcvState rcvState {RcvState::Idle};
    UCHAR    rcvBCC   {0};
    int      rcvIndex {0};
};

#endif // _SERIALCOMM_H