# LiteWing Firmware Notes

Notes from a code-reading/debugging session on this repo.

## Drone geometry

The front (nose) of the LiteWing is the **USB-C connector side**. The firmware uses the
Crazyflie axes: **+X toward the front, +Y to the left, +Z up**. The position estimate
(`stateEstimate.x/y`) uses the same axes as at the last estimate reset
(`kalman.resetEstimation`, done just before takeoff): +x ahead of the USB connector, +y to
its left.

Nothing on the board marks it: the front comes from how the firmware maps the IMU axes
([`sensors_mpu6050_hm5883L_ms5611.c:356-391`](../components/core/crazyflie/hal/src/sensors_mpu6050_hm5883L_ms5611.c#L356-L391)),
with the MPU6050 mounted rotated 180° on the board. To check, motors off: lifting the USB
edge gives a positive `stabilizer.pitch`, lifting the left edge a positive
`stabilizer.roll`.

Positive pitch is nose up (Crazyflie "legacy" convention), so:

| attitude | tilt | drone moves toward |
| --- | --- | --- |
| pitch + | USB side up | −X (back) |
| pitch − | USB side down | +X (front) |
| roll + | right side down | −Y (right) |
| roll − | left side down | +Y (left) |

The x/y estimate follows these axes only since the optical flow axis fix, see
[Optical flow axis signs](#optical-flow-axis-signs-fixed-2026-10-01).

## PMW3901 optical flow sensor

The PMW3901 on the positioning module is a small downward-facing camera. Each frame it
reports how far the floor texture moved, in pixels (`deltaX`, `deltaY`). It is the drone's
only source of horizontal motion: the firmware turns those pixel shifts into horizontal
velocity and integrates them into the x/y position. There is no absolute x/y reference, so
x/y drift slowly and only mean "relative to the last estimator reset".

### Detection at boot

- [`sensors_mpu6050_hm5883L_ms5611.c:86`](../components/core/crazyflie/hal/src/sensors_mpu6050_hm5883L_ms5611.c#L86)
  unconditionally `#define`s `SENSORS_ENABLE_FLOW_PMW3901` (unlike the mag/pressure/VL53L0X
  sensors above it, which are commented out).
- That guard wraps the init/test block at
  [lines 523-533](../components/core/crazyflie/hal/src/sensors_mpu6050_hm5883L_ms5611.c#L523-L533):
  calls `flowdeck2Init()`, runs `flowdeck2Test()`, and on success sets `isPmw3901Present = true`
  (read-only param `imu_tests.pmw3901`) and switches the commander into `POSHOLD_MODE`.
- The driver (`pmw3901.c`, `flowdeck_v1v2.c`) builds as a component required by `crazyflie`
  ([CMakeLists.txt:72](../components/core/crazyflie/CMakeLists.txt#L72)).
- `ENABLE_POSITION_HOLD_MODE=y` and `ENABLE_COMMAND_MODE_SET=y` are set in
  [`sdkconfig:609-610`](../sdkconfig#L609-L610).

If the sensor does not answer the SPI probe you get a "PMW3901 SPI connection [FAIL]" debug
log, and everything below stays off.

`POSHOLD_MODE` has two effects
([`crtp_commander_rpyt.c:100-105`](../components/core/crazyflie/modules/src/crtp_commander_rpyt.c#L100-L105)):

- **It selects the Kalman estimator.** It registers `kalmanEstimator` as required, and the
  stabilizer starts with the required estimator
  ([`stabilizer.c:196`](../components/core/crazyflie/modules/src/stabilizer.c#L196)), so
  `stabilizer.estimator` reads 2. Without the flow sensor the complementary estimator runs,
  and there is no x/y estimate at all.
- **It changes how legacy RPYT setpoints (`send_setpoint`) are read**
  ([lines 182-201](../components/core/crazyflie/modules/src/crtp_commander_rpyt.c#L182-L201)):
  thrust becomes a vertical velocity, and roll/pitch become body velocities
  (`vx = -pitch/30`, `vy = -roll/30`, in m/s). The generic setpoints (hover, zDistance,
  position) are not affected.

### Optical flow axis signs (fixed 2026-10-01)

The driver came from Bitcraze's Flow deck with its mounting mapping
`accpx = -deltaY; accpy = -deltaX`. On the LiteWing positioning module the sensor sits rotated
180° from that, so the estimate pointed the opposite way from the attitude frame: +x toward
the tail, +y to the right.

Symptoms with the old signs:

- In zDistance flight, positive pitch moved the estimate toward +x (it must go −x, see the
  table in [Drone front](#drone-front)), and positive roll
  toward +y.
- Position setpoints diverged on both axes until the drone flipped: the position controller
  pushed the right way physically, while the reversed estimate reported a growing error.
- Hover (velocity) setpoints use the same estimate, so they were affected too.

LiteWing's own dead-reckoning script, which works from the raw `motion.deltaX/Y` logs, uses
the same mapping as the fix: forward from `+deltaY`, left from `+deltaX`
([`dead-reckoning-optical-position-hold.py:1100-1101`](../Python-Scripts/Flight_Positioning_Module/dead-reckoning-optical-position-hold.py#L1100-L1101)).

Fix: no minus signs,
[`flowdeck_v1v2.c:97-104`](../components/drivers/spi_devices/pmw3901/flowdeck_v1v2.c#L97-L104).
Check after flashing (motors off, about 30 cm above a textured floor): sliding the drone
toward the USB edge must increase `stateEstimate.x`, sliding it left must increase
`stateEstimate.y`.

## Migration to IDF 5.5.4

### `quatdecompress` `-Werror=stringop-overflow`

Upstream Crazyflie code hit a new-GCC strictness issue (not a real bug): `quaternion_t.q0`
is a lone `float`, so casting `&q0` and handing it to `quatdecompress(uint32_t, float q[4])`
(which writes 16 bytes) makes GCC flag a 4-byte object receiving a 16-byte write — even
though `q0..q3` are contiguous in memory.

Fix: added a `float q[4]` array member to the union in `quaternion_t`
([`stabilizer_types.h`](../components/core/crazyflie/modules/interface/stabilizer_types.h)),
and updated both call sites to pass `.q` directly instead of `(float*)&....q0`:
- [`crtp_commander_generic.c:337`](../components/core/crazyflie/modules/src/crtp_commander_generic.c#L337)
- [`crtp_localization_service.c:194`](../components/core/crazyflie/modules/src/crtp_localization_service.c#L194)

### `gpio_pad_select_gpio` implicit declaration

`gpio_pad_select_gpio` no longer exists in ESP-IDF v5.5.4 (this project's target,
`CONFIG_IDF_TARGET="esp32s3"`); the current API is `esp_rom_gpio_pad_select_gpio`
(declared in `esp_rom_gpio.h`, already pulled in transitively by `driver/gpio.h`).

Fix: renamed the call in
[`piezo.c:76`](../components/drivers/general/buzzer/piezo.c#L76). This was the only call
site in the repo.

## Is a CPU core dedicated to control loops?

No. There is no core pinning anywhere in this firmware.

- Every task — including `stabilizerTask`, `sensorsTask`, `kalmanTask`, CRTP RX/TX, etc. —
  is created via `STATIC_MEM_TASK_CREATE`
  ([`static_mem.h:214`](../components/core/crazyflie/modules/interface/static_mem.h#L214)),
  which expands to plain `xTaskCreateStatic(...)` — no core-affinity parameter.
- A repo-wide search for `xTaskCreatePinnedToCore`/`PinnedToCore` found zero real hits.
- Target is `esp32s3` with `CONFIG_FREERTOS_NUMBER_OF_CORES=2`, and neither
  `CONFIG_FREERTOS_UNICORE` nor `CONFIG_FREERTOS_SMP` is set — i.e. the legacy dual-core
  FreeRTOS scheduler with two real cores is active.

All app tasks run with `tskNO_AFFINITY`, so the scheduler is free to place any task on
either core — there is no "core 1 = control loop, core 0 = everything else" split like the
Arduino-on-ESP32 convention. Task priority (`STABILIZER_TASK_PRI` etc.) governs preemption,
not core placement. If flight-loop jitter becomes a problem, pinning `stabilizerTask`
(and ideally the sensor task) to one core while leaving WiFi/CRTP/logging on the other
would be the typical fix — not currently done.

## How to control the drone remotely

The original LiteWing firmware runs its own Wi-Fi access point and uses ESP-NOW; this fork
runs as a Wi-Fi client (station) on an existing network instead.

- The drone joins an existing WiFi network as a station. WiFi is owned by the `wendy_core`
  component: `wendy_core_init()` in [`main.c`](../main/main.c) brings up NVS, the IP stack
  and the STA connection before the flight stack starts.
- IP comes from the router's DHCP; the drone advertises the mDNS service `_wendy-lite._tcp`.
- Listens for CRTP packets over plain UDP on port **2390**
  ([`wifi_esp32.c`](../components/drivers/general/wifi/wifi_esp32.c)) — the same CRTP
  protocol Bitcraze's Crazyflie uses, carried over WiFi/UDP instead of a radio dongle.
  Framing (trailing 1-byte checksum) is unchanged.

Connect: put your computer on the same network, then speak CRTP to `udp://<drone-ip>:2390`.

## APIs for giving position setpoints from *within* the firmware

### Prerequisite: the onboard "app" hook is currently dead

`appMain()` ([`app.h:42`](../components/core/crazyflie/modules/interface/app.h#L42)) is
only invoked if `appInit()` runs, and
[`system.c:146`](../components/core/crazyflie/modules/src/system.c#L146) only calls
`appInit()` inside `#ifdef APP_ENABLED`. Nothing in this repo defines `APP_ENABLED` — no
Kconfig option, no CMake define. So today `appMain()` has no implementation and never runs.
To add onboard control code, either:
- define `APP_ENABLED` (e.g. in the top-level `CMakeLists.txt`) and implement
  `void appMain(void)` in a new source file, or
- wire your own task some other way (e.g. add a `STATIC_MEM_TASK_CREATE` call near
  [`system.c:147`](../components/core/crazyflie/modules/src/system.c#L147)).

### Recommended: the high-level commander

[`crtp_commander_high_level.h`](../components/core/crazyflie/modules/interface/crtp_commander_high_level.h)
is explicitly documented as "Public API - can be used from an app" (lines 74-160). Call
directly from firmware code:

```c
crtpCommanderHighLevelTakeoff(1.0f, 2.0f);                        // to 1m over 2s
crtpCommanderHighLevelGoTo(1.5f, 0.0f, 1.0f, 0.0f, 3.0f, false);  // x,y,z (m), yaw (rad), duration (s), relative?
crtpCommanderHighLevelLand(0.0f, 2.0f);
```

This generates a smooth time-parameterized trajectory internally (`planner.c`); call once
per maneuver, not once per tick. Catch: it only takes effect once the low-level setpoint
queue in `commander.c` is stale, and only if the `enableHighLevel` flag
(param `enHighLevel`, [`commander.c:48,157`](../components/core/crazyflie/modules/src/commander.c#L48))
is `true`. It defaults to `false` — with no ground-station client to flip it, change its
default in `commander.c` or add your own setter.

### Lower-level alternative: stream raw setpoints yourself

Same entry point CRTP/ExtRX use:

```c
setpoint_t sp = {0};
sp.mode.x = sp.mode.y = sp.mode.z = modeAbs;
sp.position.x = x; sp.position.y = y; sp.position.z = z;
sp.mode.yaw = modeAbs;
sp.attitude.yaw = yawDeg;
commanderSetSetpoint(&sp, priority);
```

Template: `positionDecoder()` at
[`crtp_commander_generic.c:353-369`](../components/core/crazyflie/modules/src/crtp_commander_generic.c#L353-L369).

Two things to get right:
- **Priority** ([`commander.h:38-40`](../components/core/crazyflie/modules/interface/commander.h#L38-L40)):
  `COMMANDER_PRIORITY_CRTP=1`, `COMMANDER_PRIORITY_EXTRX=2`; a setpoint only wins if
  `priority >= currentPriority`
  ([`commander.c:80`](../components/core/crazyflie/modules/src/commander.c#L80)). Use
  something higher than 2 to guarantee override of any concurrent remote link.
- **Watchdog**: `commanderGetSetpoint()` degrades a stale setpoint to level-only after
  500ms and to full disarm after 2000ms
  ([`commander.c:104-127`](../components/core/crazyflie/modules/src/commander.c#L104-L127)).
  Must be re-sent faster than 500ms from a periodic task — unlike the high-level commander,
  which manages its own timing.

Either way, position setpoints are only meaningful with a live position estimate: x/y needs
the PMW3901 flow deck, z needs the ToF/baro path, both feeding `estimator_kalman.c`.

### Is the high-level commander API thread/task safe?

Mostly yes, with two known gaps.

**Protected correctly:** all planner-state mutation — `takeoff`/`land`/`go_to`/`stop`/
`start_trajectory` variants — wraps its `plan_*()` call in the same static mutex `lockTraj`
([`crtp_commander_high_level.c:105-106`](../components/core/crazyflie/modules/src/crtp_commander_high_level.c#L105-L106),
e.g. taken/given at
[lines 415-419](../components/core/crazyflie/modules/src/crtp_commander_high_level.c#L415-L419)).
`crtpCommanderHighLevelGetSetpoint()` (called every tick from the stabilizer task) also
takes the lock around `plan_current_goal()`
([lines 295-302](../components/core/crazyflie/modules/src/crtp_commander_high_level.c#L295-L302)).
Calling `GoTo`/`Takeoff`/`Land`/`Stop` from your own onboard task is safe — the module
already expects concurrent callers (stock firmware already has a dedicated CRTP task for
this plus the stabilizer task consuming setpoints).

**Gap 1 — unguarded "last known setpoint" cache.** `pos`/`vel`/`yaw` are read under the lock
by `takeoff`/`land`/`go_to`, but `crtpCommanderHighLevelGetSetpoint()` reads/overwrites the
same globals *outside* the lock, right after releasing it
([lines 305-309, 334-336](../components/core/crazyflie/modules/src/crtp_commander_high_level.c#L305-L336)).
Genuine race between the stabilizer task and whatever task calls Takeoff/Land/GoTo — worst
case a stale position seeds one tick of a new trajectory. Inherited from upstream Crazyflie
firmware, not specific to this fork.

**Gap 2 — trajectory upload/definition has no lock at all.**
`crtpCommanderHighLevelWriteTrajectory()`/`ReadTrajectory()`
([lines 795-816](../components/core/crazyflie/modules/src/crtp_commander_high_level.c#L795-L816))
and `DefineTrajectory()`/`IsTrajectoryDefined()`
([lines 622-629, 754-760](../components/core/crazyflie/modules/src/crtp_commander_high_level.c#L622-L760))
take no mutex, even though `start_trajectory()` reads the same memory/descriptor under
`lockTraj`. Don't write/redefine a trajectory slot that's currently selected and running
from another task; uploading before starting (or after stopping) is safe.

No deadlock risk: all locked sections only call pure computation into `planner.c`/
`pptraj.c`, nothing that blocks or re-enters `lockTraj`.

### How do you know a GoTo has finished?

Use `crtpCommanderHighLevelIsTrajectoryFinished()` — a pure elapsed-time check:
`(t - t_begin) >= duration`
([`pptraj.h:178-181`](../components/core/crazyflie/modules/interface/pptraj.h#L178-L181)),
where `t_begin`/`duration` are recorded when `go_to()`/`takeoff()`/`land()` built the
polynomial. Poll it — there's no callback:

```c
crtpCommanderHighLevelGoTo(x, y, z, yaw, duration_s, false);
while (!crtpCommanderHighLevelIsTrajectoryFinished()) {
    vTaskDelay(pdMS_TO_TICKS(100));
}
// travel done
```

**Do not use `crtpCommanderHighLevelIsStopped()` for this.** It only reflects
`planner.state == TRAJECTORY_STATE_IDLE`
([`planner.c:88-91`](../components/core/crazyflie/modules/src/planner.c#L88-L91)).
`plan_go_to()`/`plan_takeoff()` set state to `TRAJECTORY_STATE_FLYING`
([`planner.c:144,178`](../components/core/crazyflie/modules/src/planner.c#L144)) and it
**never auto-clears** — once `duration_s` elapses the drone just holds at the trajectory's
final point forever, but `IsStopped()` keeps returning `false` until you explicitly call
`Stop()`. The one exception is `plan_land()`, which sets `TRAJECTORY_STATE_LANDING`, and
`plan_current_goal()` auto-flips that to IDLE once finished
([`planner.c:96-99`](../components/core/crazyflie/modules/src/planner.c#L96-L99)) — so
`IsStopped()` does work as an "am I landed" signal, just not as a general "did my last
command finish" signal.

**Caveat: this is open-loop timing, not "arrived."** `IsTrajectoryFinished()` only means the
clock ran out on the requested duration — it says nothing about whether the drone is
actually near the target position. A bad position estimate, a physical obstruction, or an
unrealistic `duration_s` for the distance/dynamics will still make this return `true` on
schedule. There's no closed-loop "within X cm of target" check in this API; that would need
to be built by comparing the live state estimate against the commanded target yourself.
