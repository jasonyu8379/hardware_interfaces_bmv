#include <HD/hd.h>
#include <HDU/hduError.h>

#include <cstdio>
#include <cstring>
#include <unistd.h>

struct TouchState {
    double joint_angles[3];
    double gimbal_angles[3];
    int    buttons;
};

static TouchState g_state;

static HDCallbackCode HDCALLBACK readStateCallback(void* /*pUserData*/)
{
    hdBeginFrame(hdGetCurrentDevice());
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES,  g_state.joint_angles);
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, g_state.gimbal_angles);
    hdGetIntegerv(HD_CURRENT_BUTTONS,      &g_state.buttons);
    hdEndFrame(hdGetCurrentDevice());
    return HD_CALLBACK_CONTINUE;
}

static HDCallbackCode HDCALLBACK copyStateCallback(void* pUserData)
{
    TouchState* dst = static_cast<TouchState*>(pUserData);
    memcpy(dst, &g_state, sizeof(TouchState));
    return HD_CALLBACK_DONE;
}

int main()
{
    HDErrorInfo error;

    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Failed to initialize Touch device");
        return -1;
    }
    printf("Touch device initialized.\n");

    HDSchedulerHandle hUpdate = hdScheduleAsynchronous(
        readStateCallback, nullptr, HD_MAX_SCHEDULER_PRIORITY);

    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Failed to start scheduler");
        hdDisableDevice(hHD);
        return -1;
    }

    bool btn1_prev = false, btn2_prev = false;

    printf("Reading angles. Press Button 1 or Button 2 on the stylus.\n");
    printf("Hold both buttons simultaneously to quit.\n\n");

    while (true) {
        TouchState state;
        hdScheduleSynchronous(copyStateCallback, &state, HD_MIN_SCHEDULER_PRIORITY);

        bool btn1 = (state.buttons & HD_DEVICE_BUTTON_1) != 0;
        bool btn2 = (state.buttons & HD_DEVICE_BUTTON_2) != 0;

        printf("\rJoint: [%7.3f %7.3f %7.3f]  Gimbal: [%7.3f %7.3f %7.3f]  Btn1:%d Btn2:%d   ",
               state.joint_angles[0],  state.joint_angles[1],  state.joint_angles[2],
               state.gimbal_angles[0], state.gimbal_angles[1], state.gimbal_angles[2],
               (int)btn1, (int)btn2);
        fflush(stdout);

        if (btn1 && !btn1_prev)
            printf("\n[Event] Button 1 pressed\n");
        if (!btn1 && btn1_prev)
            printf("\n[Event] Button 1 released\n");
        if (btn2 && !btn2_prev)
            printf("\n[Event] Button 2 pressed\n");
        if (!btn2 && btn2_prev)
            printf("\n[Event] Button 2 released\n");

        if (btn1 && btn2)
            break;

        btn1_prev = btn1;
        btn2_prev = btn2;

        usleep(10000);  // 100 Hz
    }

    printf("\nQuitting.\n");

    hdStopScheduler();
    hdUnschedule(hUpdate);
    hdDisableDevice(hHD);

    return 0;
}
