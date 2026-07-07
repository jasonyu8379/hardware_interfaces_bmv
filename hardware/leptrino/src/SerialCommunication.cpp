#include "SerialCommunication.h"
#include <cstring>

// ─── Constructor / Destructor ─────────────────────────────────────────────────

SerialComm::SerialComm() : fd(-1), p_rd(0), p_wr(0), rcv_n(0), delim(0) {}

SerialComm::~SerialComm() {
    closeDevice();
}

// ─── Device Management ────────────────────────────────────────────────────────

int SerialComm::openDevice(const char* dev) {
    if (fd >= 0) closeDevice();
    fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    return (fd < 0) ? NG : OK;
}

void SerialComm::closeDevice() {
    if (fd >= 0) {
        close(fd);
        fd = -1;
    }
}

// ─── Configuration ────────────────────────────────────────────────────────────

void SerialComm::setup(long baud, int parity, int bitlen, int rts, int dtr, char code) {
    speed_t brate;
    switch (baud) {
        case 9600:   brate = B9600;   break;
        case 115200: brate = B115200; break;
        case 460800: brate = B460800; break;
        default:
            std::cerr << "[SerialComm] Unsupported baud rate " << baud << ", defaulting to 9600.\n";
            brate = B9600;
            break;
    }

    memset(&tio, 0, sizeof(tio));
    tio.c_cflag  = CREAD | CLOCAL;
    tio.c_cflag |= (bitlen == BIT_LEN_7) ? CS7 : CS8;
    if      (parity == PAR_ODD)  tio.c_cflag |= PARENB | PARODD;
    else if (parity == PAR_EVEN) tio.c_cflag |= PARENB;

    tio.c_iflag       = 0;
    tio.c_lflag       = 0;
    tio.c_oflag       = 0;
    tio.c_cc[VTIME]   = 0;
    tio.c_cc[VMIN]    = 0;

    cfsetspeed(&tio, brate);
    tcflush(fd, TCIOFLUSH);
    tcsetattr(fd, TCSANOW, &tio);

    delim = code;
}

// ─── Send / Receive ───────────────────────────────────────────────────────────

int SerialComm::sendData(UCHAR* buff, int len) {
    if (fd < 0) return NG;
    return (write(fd, buff, len) > 0) ? OK : NG;
}

int SerialComm::checkReceivedData() {
    return (p_wr - p_rd + 10) % 10;
}

int SerialComm::getReceivedData(UCHAR* buff) {
    if (p_rd == p_wr) return 0;
    int len = rcv_buff[p_rd][0];
    memcpy(buff, rcv_buff[p_rd] + 1, len);  // JY: +1 skips count byte at [0]; without this ssForce was 1 byte misaligned → ~400 N readings
    p_rd = (p_rd + 1) % 10;
    return len;
}

void SerialComm::receive() {
    // Per-instance state (thread-safe if one thread calls receive())
    UCHAR ch;
    int   bytesRead;

    while ((bytesRead = read(fd, &ch, 1)) > 0) {
        switch (rcvState) {
            case RcvState::Idle:
                if (ch == CHR_DLE) {
                    rcvBCC   = 0;
                    rcvIndex = 0;
                    rcvState = RcvState::GotDLE;
                }
                break;

            case RcvState::GotDLE:
                rcvState = (ch == CHR_STX) ? RcvState::Data : RcvState::Idle;
                break;

            case RcvState::Data:
                if (ch == CHR_DLE) {
                    rcvState = RcvState::Escape;
                } else {
                    stmp[rcvIndex++] = ch;
                    rcvBCC ^= ch;
                }
                break;

            case RcvState::Escape:
                if (ch == CHR_ETX) {
                    // End of frame: store BCC and commit to ring buffer
                    stmp[rcvIndex++] = rcvBCC;
                    rcv_buff[p_wr][0] = static_cast<UCHAR>(rcvIndex);
                    memcpy(rcv_buff[p_wr] + 1, stmp, rcvIndex);
                    p_wr     = (p_wr + 1) % 10;
                    rcvState = RcvState::SkipBCC;  // JY: consume wire BCC byte before Idle; prevents BCC==0x10 from triggering spurious GotDLE
                } else if (ch == CHR_DLE) {
                    // Stuffed DLE byte
                    stmp[rcvIndex++] = ch;
                    rcvBCC  ^= ch;
                    rcvState = RcvState::Data;
                } else {
                    rcvState = RcvState::Idle;
                }
                break;

            case RcvState::SkipBCC:  // JY: discard the wire BCC byte that follows DLE+ETX
                rcvState = RcvState::Idle;
                break;
        }
    }
}