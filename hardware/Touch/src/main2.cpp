#include <HD/hd.h>
#include <HDU/hduError.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <termios.h>
#include <unistd.h>

#define KEY_UP    1001
#define KEY_DOWN  1002
#define KEY_LEFT  1003
#define KEY_RIGHT 1004

struct TouchState {
    double position[3];     // Cartesian position in mm (x, y, z)
    double orientation[3];  // Roll, pitch, yaw in degrees
    double velocity[3];     // Cartesian velocity in mm/s
    int    buttons;
};

static TouchState g_state;

// Shared between main thread and scheduler callback
static double g_stiffness     = 0.0;
static double g_damping       = 0.0;
static bool   g_params_active = false;
static const double EQUILIBRIUM[3] = {0.0, 0.0, 0.0};

static void transform_to_euler(const double m[16], double& roll, double& pitch, double& yaw)
{
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

static HDCallbackCode HDCALLBACK readStateCallback(void* /*pUserData*/)
{
    hdBeginFrame(hdGetCurrentDevice());

    double transform[16];
    hdGetDoublev(HD_CURRENT_TRANSFORM, transform);

    g_state.position[0] = transform[12];
    g_state.position[1] = transform[13];
    g_state.position[2] = transform[14];

    transform_to_euler(transform,
                       g_state.orientation[0],
                       g_state.orientation[1],
                       g_state.orientation[2]);

    hdGetDoublev(HD_CURRENT_VELOCITY, g_state.velocity);
    hdGetIntegerv(HD_CURRENT_BUTTONS, &g_state.buttons);

    // Apply spring-damper force toward equilibrium when params are active
    double force[3] = {0.0, 0.0, 0.0};
    if (g_params_active) {
        for (int i = 0; i < 3; i++) {
            force[i] = -g_stiffness * (g_state.position[i] - EQUILIBRIUM[i])
                       -g_damping   *  g_state.velocity[i];
        }
    }
    hdSetDoublev(HD_CURRENT_FORCE, force);

    hdEndFrame(hdGetCurrentDevice());
    return HD_CALLBACK_CONTINUE;
}

static HDCallbackCode HDCALLBACK copyStateCallback(void* pUserData)
{
    TouchState* dst = static_cast<TouchState*>(pUserData);
    memcpy(dst, &g_state, sizeof(TouchState));
    return HD_CALLBACK_DONE;
}

static struct termios orig_termios;

void enableRawMode()
{
    tcgetattr(STDIN_FILENO, &orig_termios);
    struct termios raw = orig_termios;
    raw.c_lflag &= ~(ECHO | ICANON);
    raw.c_cc[VMIN]  = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
}

void disableRawMode()
{
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
}

int readKey()
{
    char c;
    if (read(STDIN_FILENO, &c, 1) != 1) return 0;
    if (c == '\033') {
        char seq[2];
        if (read(STDIN_FILENO, &seq[0], 1) != 1) return '\033';
        if (read(STDIN_FILENO, &seq[1], 1) != 1) return '\033';
        if (seq[0] == '[') {
            switch (seq[1]) {
                case 'A': return KEY_UP;
                case 'B': return KEY_DOWN;
                case 'C': return KEY_RIGHT;
                case 'D': return KEY_LEFT;
            }
        }
    }
    return c;
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

    // Query device safe limits for clamping (matching Python's get_max_stiffness/damping)
    double max_stiffness, max_damping;
    hdGetDoublev(HD_NOMINAL_MAX_STIFFNESS, &max_stiffness);
    hdGetDoublev(HD_NOMINAL_MAX_DAMPING,   &max_damping);
    printf("Max stiffness: %.4f N/mm  |  Max damping: %.6f N·s/mm\n",
           max_stiffness, max_damping);

    HDSchedulerHandle hUpdate = hdScheduleAsynchronous(
        readStateCallback, nullptr, HD_MAX_SCHEDULER_PRIORITY);

    hdEnable(HD_FORCE_OUTPUT);
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Failed to start scheduler");
        hdDisableDevice(hHD);
        return -1;
    }

    enableRawMode();

    bool btn1_prev = false, btn2_prev = false;

    const double STIFFNESS_STEP = 0.01;
    const double DAMPING_STEP   = 0.0001;

    printf("Hold Button 1 + use arrow keys to adjust stiffness/damping.\n");
    printf("↑/↓ stiffness    ←/→ damping\n");
    printf("Hold both buttons simultaneously to quit.\n\n");

    while (true) {
        TouchState state;
        hdScheduleSynchronous(copyStateCallback, &state, HD_MIN_SCHEDULER_PRIORITY);

        bool btn1 = (state.buttons & HD_DEVICE_BUTTON_1) != 0;
        bool btn2 = (state.buttons & HD_DEVICE_BUTTON_2) != 0;

        // Button 1 pressed: activate with saved values
        if (btn1 && !btn1_prev) {
            g_params_active = true;
            printf("\n[Button 1] Pressed — force active.  Stiffness: %.4f  Damping: %.6f\n",
                   g_stiffness, g_damping);
            printf("  Up/Down = stiffness,  Left/Right = damping\n");
        }

        // Button 1 held: read arrow keys and adjust
        if (btn1) {
            int  key     = readKey();
            bool changed = false;
            if (key == KEY_UP)    { g_stiffness = std::min(g_stiffness + STIFFNESS_STEP, max_stiffness); changed = true; }
            if (key == KEY_DOWN)  { g_stiffness = std::max(g_stiffness - STIFFNESS_STEP, 0.0);           changed = true; }
            if (key == KEY_RIGHT) { g_damping   = std::min(g_damping   + DAMPING_STEP,   max_damping);   changed = true; }
            if (key == KEY_LEFT)  { g_damping   = std::max(g_damping   - DAMPING_STEP,   0.0);           changed = true; }

            if (changed) {
                printf("\r  Stiffness: %.4f  Damping: %.6f   ", g_stiffness, g_damping);
                fflush(stdout);
            }
        }

        // Button 1 released: deactivate force, keep saved values
        if (!btn1 && btn1_prev) {
            g_params_active = false;
            printf("\n[Button 1] Released — force deactivated."
                   "  Stiffness: %.4f  Damping: %.6f  (saved)\n", g_stiffness, g_damping);
        }

        // Normal readout when button 1 not held
        if (!btn1) {
            printf("\rPos(mm) X:%7.2f Y:%7.2f Z:%7.2f  |  "
                   "Ori(deg) R:%6.1f P:%6.1f Y:%6.1f  |  "
                   "k:%.4f  b:%.6f  ",
                   state.position[0],    state.position[1],    state.position[2],
                   state.orientation[0], state.orientation[1], state.orientation[2],
                   g_stiffness, g_damping);
            fflush(stdout);
        }

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

    // Zero force and wait for callback to apply it before stopping
    g_params_active = false;
    g_stiffness     = 0.0;
    g_damping       = 0.0;
    usleep(100000);

    disableRawMode();
    hdStopScheduler();
    hdUnschedule(hUpdate);
    hdDisableDevice(hHD);

    return 0;
}
