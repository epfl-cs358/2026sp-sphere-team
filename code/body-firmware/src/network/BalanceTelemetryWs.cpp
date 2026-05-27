/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 */

#include "BalanceTelemetryWs.h"

#include <ESPAsyncWebServer.h>

#include <array>
#include <atomic>
#include <cstdio>
#include <cstring>

#ifdef ARDUINO
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#else
#include <deque>
#endif

// --- canonical CSV header --------------------------------------------------
// MUST match tools/tune_pid.py:CSV_COLUMNS exactly. Any drift here is a wire
// protocol break for the capture daemon and every downstream parser.
namespace {

constexpr const char* kHeaderLine =
    "seq,t_us,dt_measured,dt_used,"
    "cmd_vx_raw,cmd_vy_raw,cmd_omega_raw,"
    "cmd_vx,cmd_vy,cmd_omega,cmd_age_ms,"
    "quat_w,quat_x,quat_y,quat_z,"
    "accel_x,accel_y,accel_z,"
    "gyro_x_raw,gyro_y_raw,gyro_z_raw,"
    "gx,gy,gz,tilt_mag_sin,"
    "pitch_actual,roll_actual,"
    "gyro_pitch_rate,gyro_roll_rate,gyro_yaw_rate,"
    "pitch_target,roll_target,"
    "heading_integrated,heading_setpoint,heading_err,heading_P,"
    "omega_target_raw,omega_target,"
    "pitch_err,pitch_P,pitch_I,pitch_D,pitch_out_raw,pitch_out,"
    "roll_err,roll_P,roll_I,roll_D,roll_out_raw,roll_out,"
    "yaw_rate_err,yaw_rate_P,yaw_rate_I,yaw_rate_D,yaw_rate_out,"
    "body_vx_cmd,body_vy_cmd,body_omega_cmd,"
    "wheel_target_rpm_0,wheel_target_rpm_1,wheel_target_rpm_2,"
    "wheel_meas_rpm_0,wheel_meas_rpm_1,wheel_meas_rpm_2,"
    "wheel_P_0,wheel_P_1,wheel_P_2,"
    "wheel_I_0,wheel_I_1,wheel_I_2,"
    "wheel_D_0,wheel_D_1,wheel_D_2,"
    "wheel_out_0,wheel_out_1,wheel_out_2,"
    "pitch_Kp,pitch_Ki,pitch_Kd,"
    "roll_Kp,roll_Ki,roll_Kd,"
    "yaw_rate_Kp,yaw_rate_Ki,heading_Kp,"
    "pitch_deadband,roll_deadband,"
    "max_output_velocity,"
    "envelope_enter_sin,envelope_exit_sin,"
    "gyro_pitch_sign,gyro_roll_sign,gyro_yaw_sign,"
    "tilt_per_velocity,max_tilt_setpoint,"
    "armed_state,in_fault,cmd_stale,"
    "event_flags";

constexpr std::size_t kQueueLen  = 8;
constexpr std::size_t kCsvBufLen = 1200;
constexpr std::size_t kDtWindow  = 100;

AsyncWebSocket g_ws("/telemetry");

// Double-buffered "latest" — writer alternates slots, then publishes the
// new slot via an atomic pointer swap. Readers load-acquire the pointer
// and copy the slot it points to. Either the old or new struct is seen
// in full; no torn read.
BalanceTelemetry                _slotA{};
BalanceTelemetry                _slotB{};
std::atomic<BalanceTelemetry*>  _latest{nullptr};

float                           _dtRing[kDtWindow] = {};
std::atomic<std::uint32_t>      _dtIdx{0};

std::atomic<std::uint32_t>                 g_writeIdx{0};
std::array<std::atomic<std::uint32_t>, 16> g_eventCounts{};
std::atomic<std::uint32_t>                 g_dropCount{0};
std::atomic<bool>                          g_initDone{false};

#ifdef ARDUINO
QueueHandle_t g_queue = nullptr;
TaskHandle_t  g_pumpTask = nullptr;
#else
std::deque<BalanceTelemetry> g_nativeQueue;
#endif

int writeCsv(char* buf, std::size_t buflen, const BalanceTelemetry& t) {
    return std::snprintf(
        buf, buflen,
        "%u,%u,%g,%g,"
        "%g,%g,%g,"
        "%g,%g,%g,%u,"
        "%g,%g,%g,%g,"
        "%g,%g,%g,"
        "%g,%g,%g,"
        "%g,%g,%g,%g,"
        "%g,%g,"
        "%g,%g,%g,"
        "%g,%g,"
        "%g,%g,%g,%g,"
        "%g,%g,"
        "%g,%g,%g,%g,%g,%g,"
        "%g,%g,%g,%g,%g,%g,"
        "%g,%g,%g,%g,%g,"
        "%g,%g,%g,"
        "%g,%g,%g,"
        "%g,%g,%g,"
        "%g,%g,%g,"
        "%g,%g,%g,"
        "%g,%g,%g,"
        "%g,%g,%g,"
        "%g,%g,%g,"
        "%g,%g,%g,"
        "%g,%g,%g,"
        "%g,%g,"
        "%g,"
        "%g,%g,"
        "%g,%g,%g,"
        "%g,%g,"
        "%u,%u,%u,"
        "%u",
        static_cast<unsigned>(t.seq),
        static_cast<unsigned>(t.t_us),
        static_cast<double>(t.dt_measured),
        static_cast<double>(t.dt_used),

        static_cast<double>(t.cmd_vx_raw),
        static_cast<double>(t.cmd_vy_raw),
        static_cast<double>(t.cmd_omega_raw),

        static_cast<double>(t.cmd_vx),
        static_cast<double>(t.cmd_vy),
        static_cast<double>(t.cmd_omega),
        static_cast<unsigned>(t.cmd_age_ms),

        static_cast<double>(t.quat_w),
        static_cast<double>(t.quat_x),
        static_cast<double>(t.quat_y),
        static_cast<double>(t.quat_z),

        static_cast<double>(t.accel_x),
        static_cast<double>(t.accel_y),
        static_cast<double>(t.accel_z),

        static_cast<double>(t.gyro_x_raw),
        static_cast<double>(t.gyro_y_raw),
        static_cast<double>(t.gyro_z_raw),

        static_cast<double>(t.gx),
        static_cast<double>(t.gy),
        static_cast<double>(t.gz),
        static_cast<double>(t.tilt_mag_sin),

        static_cast<double>(t.pitch_actual),
        static_cast<double>(t.roll_actual),

        static_cast<double>(t.gyro_pitch_rate),
        static_cast<double>(t.gyro_roll_rate),
        static_cast<double>(t.gyro_yaw_rate),

        static_cast<double>(t.pitch_target),
        static_cast<double>(t.roll_target),

        static_cast<double>(t.heading_integrated),
        static_cast<double>(t.heading_setpoint),
        static_cast<double>(t.heading_err),
        static_cast<double>(t.heading_P),

        static_cast<double>(t.omega_target_raw),
        static_cast<double>(t.omega_target),

        static_cast<double>(t.pitch_err),
        static_cast<double>(t.pitch_P),
        static_cast<double>(t.pitch_I),
        static_cast<double>(t.pitch_D),
        static_cast<double>(t.pitch_out_raw),
        static_cast<double>(t.pitch_out),

        static_cast<double>(t.roll_err),
        static_cast<double>(t.roll_P),
        static_cast<double>(t.roll_I),
        static_cast<double>(t.roll_D),
        static_cast<double>(t.roll_out_raw),
        static_cast<double>(t.roll_out),

        static_cast<double>(t.yaw_rate_err),
        static_cast<double>(t.yaw_rate_P),
        static_cast<double>(t.yaw_rate_I),
        static_cast<double>(t.yaw_rate_D),
        static_cast<double>(t.yaw_rate_out),

        static_cast<double>(t.body_vx_cmd),
        static_cast<double>(t.body_vy_cmd),
        static_cast<double>(t.body_omega_cmd),

        static_cast<double>(t.wheel_target_rpm[0]),
        static_cast<double>(t.wheel_target_rpm[1]),
        static_cast<double>(t.wheel_target_rpm[2]),

        static_cast<double>(t.wheel_meas_rpm[0]),
        static_cast<double>(t.wheel_meas_rpm[1]),
        static_cast<double>(t.wheel_meas_rpm[2]),

        static_cast<double>(t.wheel_P[0]),
        static_cast<double>(t.wheel_P[1]),
        static_cast<double>(t.wheel_P[2]),

        static_cast<double>(t.wheel_I[0]),
        static_cast<double>(t.wheel_I[1]),
        static_cast<double>(t.wheel_I[2]),

        static_cast<double>(t.wheel_D[0]),
        static_cast<double>(t.wheel_D[1]),
        static_cast<double>(t.wheel_D[2]),

        static_cast<double>(t.wheel_out[0]),
        static_cast<double>(t.wheel_out[1]),
        static_cast<double>(t.wheel_out[2]),

        static_cast<double>(t.pitch_Kp),
        static_cast<double>(t.pitch_Ki),
        static_cast<double>(t.pitch_Kd),

        static_cast<double>(t.roll_Kp),
        static_cast<double>(t.roll_Ki),
        static_cast<double>(t.roll_Kd),

        static_cast<double>(t.yaw_rate_Kp),
        static_cast<double>(t.yaw_rate_Ki),
        static_cast<double>(t.heading_Kp),

        static_cast<double>(t.pitch_deadband),
        static_cast<double>(t.roll_deadband),

        static_cast<double>(t.max_output_velocity),

        static_cast<double>(t.envelope_enter_sin),
        static_cast<double>(t.envelope_exit_sin),

        static_cast<double>(t.gyro_pitch_sign),
        static_cast<double>(t.gyro_roll_sign),
        static_cast<double>(t.gyro_yaw_sign),

        static_cast<double>(t.tilt_per_velocity),
        static_cast<double>(t.max_tilt_setpoint),

        static_cast<unsigned>(t.armed_state),
        static_cast<unsigned>(t.in_fault),
        static_cast<unsigned>(t.cmd_stale),

        static_cast<unsigned>(t.event_flags));
}

void pumpSnapshot(const BalanceTelemetry& snap) {
    char line[kCsvBufLen];
    int n = writeCsv(line, sizeof(line), snap);
    if (n > 0 && static_cast<std::size_t>(n) < sizeof(line)) {
        g_ws.textAll(String(line));
    }

    // Alternate slot, write, publish pointer. First publish picks _slotA.
    BalanceTelemetry* cur  = _latest.load(std::memory_order_relaxed);
    BalanceTelemetry* next = (cur == &_slotA) ? &_slotB : &_slotA;
    *next = snap;
    _latest.store(next, std::memory_order_release);

    // dt window: skip non-positive samples (pre-first-tick sentinel).
    if (snap.dt_measured > 0.0f) {
        const std::uint32_t i = _dtIdx.fetch_add(1, std::memory_order_relaxed);
        _dtRing[i % kDtWindow] = snap.dt_measured;
    }

    g_writeIdx.fetch_add(1, std::memory_order_release);

    const std::uint32_t flags = snap.event_flags;
    for (std::size_t b = 0; b < g_eventCounts.size(); ++b) {
        if (flags & (1u << b)) {
            g_eventCounts[b].fetch_add(1, std::memory_order_relaxed);
        }
    }
}

#ifdef ARDUINO
void telemetryPumpTask(void* /*arg*/) {
    BalanceTelemetry snap;
    for (;;) {
        if (xQueueReceive(g_queue, &snap, portMAX_DELAY) == pdTRUE) {
            pumpSnapshot(snap);
        }
    }
}

void onWsEvent(AsyncWebSocket* /*server*/, AsyncWebSocketClient* client,
               AwsEventType type, void* /*arg*/, uint8_t* /*data*/,
               size_t /*len*/) {
    if (type == WS_EVT_CONNECT && client) {
        client->text(String(kHeaderLine));
    }
}
#endif

}  // namespace

namespace BalanceTelemetryWs {

void init(AsyncWebServer& server) {
    bool expected = false;
    if (!g_initDone.compare_exchange_strong(expected, true,
                                            std::memory_order_acq_rel)) {
        return;
    }
#ifdef ARDUINO
    if (!g_queue) {
        g_queue = xQueueCreate(kQueueLen, sizeof(BalanceTelemetry));
    }
    g_ws.onEvent(onWsEvent);
    server.addHandler(&g_ws);
    xTaskCreatePinnedToCore(telemetryPumpTask, "telem_pump",
                            /*stack*/ 4096, /*arg*/ nullptr,
                            /*prio*/ 1, &g_pumpTask, /*core*/ 0);
#else
    server.addHandler(&g_ws);
#endif
}

void publish(const BalanceTelemetry& snap) {
#ifdef ARDUINO
    if (!g_queue) {
        g_dropCount.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    if (xQueueSendToBack(g_queue, &snap, 0) != pdTRUE) {
        g_dropCount.fetch_add(1, std::memory_order_relaxed);
    }
#else
    if (g_nativeQueue.size() >= kQueueLen) {
        g_dropCount.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    g_nativeQueue.push_back(snap);
#endif
}

bool latest(BalanceTelemetry* out) {
    if (!out) return false;
    BalanceTelemetry* p = _latest.load(std::memory_order_acquire);
    if (!p) return false;
    *out = *p;
    return true;
}

void dtStats(float* minOut, float* meanOut, float* maxOut) {
    if (minOut)  *minOut  = 0.0f;
    if (meanOut) *meanOut = 0.0f;
    if (maxOut)  *maxOut  = 0.0f;
    const std::uint32_t n = _dtIdx.load(std::memory_order_acquire);
    if (n == 0) return;
    const std::size_t valid = (n < kDtWindow) ? static_cast<std::size_t>(n)
                                              : kDtWindow;
    float mn = 1e9f, mx = -1e9f, sum = 0.0f;
    std::size_t counted = 0;
    for (std::size_t i = 0; i < valid; ++i) {
        const float v = _dtRing[i];
        if (v <= 0.0f) continue;
        if (v < mn) mn = v;
        if (v > mx) mx = v;
        sum += v;
        ++counted;
    }
    if (counted == 0) return;
    if (minOut)  *minOut  = mn;
    if (maxOut)  *maxOut  = mx;
    if (meanOut) *meanOut = sum / static_cast<float>(counted);
}

std::uint32_t eventCount(std::size_t bit) {
    if (bit >= g_eventCounts.size()) return 0;
    return g_eventCounts[bit].load(std::memory_order_relaxed);
}

std::uint32_t dropCount() {
    return g_dropCount.load(std::memory_order_relaxed);
}

std::uint32_t writeIdx() {
    return g_writeIdx.load(std::memory_order_acquire);
}

const char* headerLine() {
    return kHeaderLine;
}

#ifdef BB8_TEST_HOOKS
void pumpOnce() {
#ifdef ARDUINO
    BalanceTelemetry snap;
    if (g_queue && xQueueReceive(g_queue, &snap, 0) == pdTRUE) {
        pumpSnapshot(snap);
    }
#else
    if (g_nativeQueue.empty()) return;
    BalanceTelemetry snap = g_nativeQueue.front();
    g_nativeQueue.pop_front();
    pumpSnapshot(snap);
#endif
}

void resetForTesting() {
    _latest.store(nullptr, std::memory_order_release);
    _slotA = BalanceTelemetry{};
    _slotB = BalanceTelemetry{};
    _dtIdx.store(0, std::memory_order_release);
    for (std::size_t i = 0; i < kDtWindow; ++i) _dtRing[i] = 0.0f;
    g_writeIdx.store(0, std::memory_order_release);
    g_dropCount.store(0, std::memory_order_relaxed);
    for (auto& c : g_eventCounts) c.store(0, std::memory_order_relaxed);
#ifndef ARDUINO
    g_nativeQueue.clear();
#endif
}
#endif

}  // namespace BalanceTelemetryWs
