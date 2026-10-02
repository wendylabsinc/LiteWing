/*
 * app.c: the onboard app. On a start command from the PC it runs the same
 * pre-flight checks as python-demo-app/drone.py, takes control from the link,
 * takes off in position mode with the high-level commander, flies the route
 * below, and lands. Protocol in app.h.
 *
 * Taking control: the high-level commander only flies once the setpoints from
 * the link are stale (commander.c), so the PC stops streaming them while the
 * app runs and sends pings instead, which also keep the telemetry going.
 * commanderNotifySetpointsStop() then hands over at once. Any setpoint from the
 * link afterwards (the cockpit's Esc) stops the high-level commander and so
 * cuts the motors; the app notices and ends the run.
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "stm32_legacy.h"
#include "app.h"
#include "app_channel.h"
#include "commander.h"
#include "crtp_commander_high_level.h"
#include "estimator.h"
#include "param.h"
#include "pm_esplane.h"
#include "range.h"
#define DEBUG_MODULE "APP"
#include "debug_cf.h"

#define HEIGHT_M            0.3f    // takeoff height
#define HOLD_MS             1000    // pause after the takeoff and each move
#define SPEED_M_S           0.2f    // flight speed, takeoff and landing too
#define YAW_RATE_DEG_S      45.0f   // turn speed
#define MIN_MANEUVER_S      1.0f    // shortest takeoff, move or landing
#define MIN_BATTERY_V       3.5f
#define TOF_CHECK_MS        1000    // a ToF reading must come within this
#define SETPOINTS_QUIET_MS  500     // no setpoint from the link for this long
#define ESTIMATOR_SETTLE_MS 500     // after the estimator reset
#define LANDING_MARGIN_MS   1000    // past the landing duration, stop anyway
#define POLL_MS             10      // app-channel poll period while running

/* The route after the takeoff, flown point by point. The frame is the one of
 * the estimator reset just before the takeoff: x/y 0 is the takeoff spot, +X
 * the USB side, +Y the left, z the height, and yaw 0 the takeoff heading. */
typedef struct {
    float x, y, z;    // m
    float yawDeg;     // + turns left (counter-clockwise seen from above)
} waypoint_t;

static const waypoint_t route[] = {
    {0.0f, -0.3f, 0.3f,  0.0f},   // 30 cm to the right
    {0.0f, -0.3f, 0.5f,  0.0f},   // up to 50 cm
    {0.0f,  0.0f, 0.5f,  0.0f},   // back over the takeoff spot
    {0.0f,  0.0f, 0.5f, 90.0f},   // quarter turn left
    {0.0f,  0.0f, 0.3f, 90.0f},   // down to 30 cm
    {0.0f,  0.0f, 0.3f,  0.0f},   // back to the takeoff heading
};

static uint8_t runId;
static bool running;
static uint8_t error = APP_ERROR_NONE;
static bool abortRequested;
static bool flying;         // in the air under the high-level commander
static bool overridden;     // ... and a setpoint from the link stopped it
static bool tofSeen;

static void sendStatus(uint8_t id, bool isRunning, uint8_t err)
{
    uint8_t status[3] = {id, isRunning ? 1 : 0, err};
    appchannelSendPacket(status, sizeof(status));
}

/* Handles one packet from the PC. Returns true for a START to act on. */
static bool handlePacket(const uint8_t *data, size_t length)
{
    if (length < 1) {
        return false;
    }
    switch (data[0]) {
        case APP_CMD_START:
            if (length < 2) {
                return false;
            }
            if (running) {
                sendStatus(data[1], false, APP_ERROR_BUSY);
                return false;
            }
            runId = data[1];
            return true;
        case APP_CMD_ABORT:
            if (running) {
                abortRequested = true;
            }
            return false;
        case APP_CMD_PING:
            sendStatus(runId, running, error);
            return false;
        default:
            return false;
    }
}

/* Waits ms while serving the app channel, calling each() (if any) at every
 * poll. Returns false as soon as the run must end: an abort, or the
 * high-level commander stopped in flight. */
static bool serviceFor(uint32_t ms, void (*each)(void))
{
    uint8_t packet[APPCHANNEL_MTU];
    const TickType_t end = xTaskGetTickCount() + M2T(ms);

    while (true) {
        if (abortRequested) {
            return false;
        }
        if (flying && crtpCommanderHighLevelIsStopped()) {
            overridden = true;
            return false;
        }
        if (each) {
            each();
        }
        int32_t left = (int32_t)(end - xTaskGetTickCount());
        if (left <= 0) {
            return true;
        }
        int wait = T2M(left) < POLL_MS ? T2M(left) : POLL_MS;
        size_t length = appchannelReceivePacket(packet, sizeof(packet), wait);
        if (length > 0) {
            handlePacket(packet, length);
        }
    }
}

static void sampleTof(void)
{
    if (rangeGet(rangeDown) > 0.0f) {
        tofSeen = true;
    }
}

static uint8_t endedBy(void)
{
    if (abortRequested) {
        return APP_ERROR_ABORTED;
    }
    if (overridden) {
        return APP_ERROR_OVERRIDDEN;
    }
    return APP_ERROR_NONE;
}

/* The same checks as preflight_ok() in drone.py, plus nobody else flying. */
static uint8_t check(void)
{
    float battery = pmGetBatteryVoltage();
    if (battery < MIN_BATTERY_V) {
        DEBUG_PRINTW("Battery low (%.2f V), not flying\n", (double)battery);
        return APP_ERROR_LOW_BATTERY;
    }
    // The firmware only runs the Kalman estimator when the PMW3901 passed its
    // self-test; position control needs it.
    if (getStateEstimator() != kalmanEstimator) {
        DEBUG_PRINTW("Kalman estimator off (no optical flow), not flying\n");
        return APP_ERROR_NO_FLOW;
    }
    tofSeen = false;
    if (!serviceFor(TOF_CHECK_MS, sampleTof)) {
        return endedBy();
    }
    if (!tofSeen) {
        DEBUG_PRINTW("No ToF reading, not flying\n");
        return APP_ERROR_NO_TOF;
    }
    // Also lets the last setpoints the PC sent before the start get through.
    if (T2M(commanderGetInactivityTime()) < SETPOINTS_QUIET_MS) {
        DEBUG_PRINTW("Setpoints still coming from the link, not flying\n");
        return APP_ERROR_SETPOINTS_ACTIVE;
    }
    return APP_ERROR_NONE;
}

static uint32_t toMs(float seconds)
{
    return (uint32_t)(seconds * 1000);
}

/* Duration of a move: at SPEED_M_S and YAW_RATE_DEG_S, whichever takes
 * longer, and no less than MIN_MANEUVER_S. */
static float moveDuration(const waypoint_t *from, const waypoint_t *to)
{
    const float dx = to->x - from->x;
    const float dy = to->y - from->y;
    const float dz = to->z - from->z;
    const float distance = sqrtf(dx * dx + dy * dy + dz * dz);
    const float turn = fabsf(to->yawDeg - from->yawDeg);
    return fmaxf(MIN_MANEUVER_S,
                 fmaxf(distance / SPEED_M_S, turn / YAW_RATE_DEG_S));
}

static uint8_t fly(void)
{
    const paramVarId_t resetId = paramGetVarId("kalman", "resetEstimation");
    const paramVarId_t highLevelId = paramGetVarId("commander", "enHighLevel");

    // Restart the position estimate here (x/y -> 0), as drone.py does before
    // each takeoff.
    paramSetInt(resetId, 1);
    if (!serviceFor(ESTIMATOR_SETTLE_MS, NULL)) {
        return endedBy();
    }

    const int highLevelWas = paramGetInt(highLevelId);
    paramSetInt(highLevelId, 1);
    commanderNotifySetpointsStop(0);

    uint8_t result = APP_ERROR_NONE;
    const waypoint_t ground = {0.0f, 0.0f, 0.0f, 0.0f};
    waypoint_t at = {0.0f, 0.0f, HEIGHT_M, 0.0f};    // where the takeoff ends
    float duration = moveDuration(&ground, &at);
    DEBUG_PRINTI("Takeoff to %.2f m\n", (double)HEIGHT_M);
    if (crtpCommanderHighLevelTakeoff(HEIGHT_M, duration) != 0) {
        result = APP_ERROR_PLANNER;
    } else {
        flying = true;
        bool ok = serviceFor(toMs(duration), NULL) && serviceFor(HOLD_MS, NULL);
        for (size_t i = 0; ok && i < sizeof(route) / sizeof(route[0]); i++) {
            const waypoint_t *next = &route[i];
            duration = moveDuration(&at, next);
            DEBUG_PRINTI("Step %d: x %.2f y %.2f z %.2f m, yaw %.0f deg\n",
                         (int)i + 1, (double)next->x, (double)next->y,
                         (double)next->z, (double)next->yawDeg);
            if (crtpCommanderHighLevelGoTo(next->x, next->y, next->z,
                                           radians(next->yawDeg), duration,
                                           false) != 0) {
                result = APP_ERROR_PLANNER;
                break;  // still lands, from where it is
            }
            at = *next;
            ok = serviceFor(toMs(duration), NULL) && serviceFor(HOLD_MS, NULL);
        }
        if (ok) {
            // The high-level commander stops by itself once landed.
            flying = false;
            duration = fmaxf(MIN_MANEUVER_S, at.z / SPEED_M_S);
            DEBUG_PRINTI("Landing\n");
            if (crtpCommanderHighLevelLand(0.0f, duration) != 0) {
                result = APP_ERROR_PLANNER;
            } else {
                const TickType_t end = xTaskGetTickCount()
                        + M2T(toMs(duration) + LANDING_MARGIN_MS);
                while (!crtpCommanderHighLevelIsStopped()
                       && (int32_t)(end - xTaskGetTickCount()) > 0
                       && serviceFor(POLL_MS, NULL)) {
                }
            }
        }
        flying = false;
        if (result == APP_ERROR_NONE) {
            result = endedBy();
        }
    }

    // Motors off at once if still flying, as the cockpit's Esc does.
    crtpCommanderHighLevelStop();
    paramSetInt(highLevelId, highLevelWas);
    return result;
}

static uint8_t run(void)
{
    uint8_t result = check();
    if (result != APP_ERROR_NONE) {
        return result;
    }
    return fly();
}

void appMain(void)
{
    uint8_t packet[APPCHANNEL_MTU];

    DEBUG_PRINTI("Ready, waiting for a start command\n");
    while (true) {
        size_t length = appchannelReceivePacket(packet, sizeof(packet),
                                                APPCHANNEL_WAIT_FOREVER);
        if (!handlePacket(packet, length)) {
            continue;
        }

        DEBUG_PRINTI("Run %d started\n", runId);
        running = true;
        error = APP_ERROR_NONE;
        abortRequested = false;
        overridden = false;
        sendStatus(runId, running, error);

        error = run();

        running = false;
        sendStatus(runId, running, error);
        DEBUG_PRINTI("Run %d ended, error %d\n", runId, error);
    }
}
