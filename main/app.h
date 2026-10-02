/*
 * app.h: the onboard app, started from the PC over the CRTP app channel.
 *
 * This file shadows the Crazyflie app.h (modules/interface) for the sources
 * of this directory; both declare the same appMain().
 *
 * Protocol, one app-channel packet each way (little-endian, see
 * python-demo-app/drone.py for the PC side):
 *
 *   PC -> drone: command byte, then its arguments
 *     APP_CMD_START  run id (uint8): run the app once
 *     APP_CMD_ABORT  end the run at once, motors off
 *     APP_CMD_PING   keep-alive while the app runs; answered with a status
 *
 *   drone -> PC: status, sent when a run starts and ends and on each ping
 *     run id (uint8), running (uint8, 0 or 1), error (uint8, app_error_t)
 *
 * The status says nothing about what the app is doing: only whether it runs,
 * and once it ended, why it failed if it did. A run that ended with
 * APP_ERROR_NONE completed normally.
 */
#pragma once

typedef enum {
    APP_CMD_START = 1,
    APP_CMD_ABORT = 2,
    APP_CMD_PING  = 3,
} app_command_t;

typedef enum {
    APP_ERROR_NONE             = 0,
    APP_ERROR_BUSY             = 1, // START while a run is going on
    APP_ERROR_LOW_BATTERY      = 2,
    APP_ERROR_NO_TOF           = 3, // VL53L1X not found at boot
    APP_ERROR_NO_FLOW          = 4, // Kalman estimator off: optical flow failed
    APP_ERROR_SETPOINTS_ACTIVE = 5, // someone else streams setpoints
    APP_ERROR_PLANNER          = 6, // the high-level commander refused
    APP_ERROR_ABORTED          = 7, // APP_CMD_ABORT
    APP_ERROR_OVERRIDDEN       = 8, // a setpoint from the link took over
} app_error_t;

/* Called once by the app task (app_handler.c) after the system started. */
void appMain(void);
