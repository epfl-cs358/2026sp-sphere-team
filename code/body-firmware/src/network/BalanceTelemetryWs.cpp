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
    "gyro_pitch_rate,gyro_roll_rate,"
    "pitch_target,roll_target,"
    "pitch_err,pitch_P,pitch_I,pitch_D,pitch_out_raw,pitch_out,"
    "roll_err,roll_P,roll_I,roll_D,roll_out_raw,roll_out,"
    "body_vx_cmd,body_vy_cmd,body_omega_cmd,"
    "wheel_target_rpm_0,wheel_target_rpm_1,wheel_target_rpm_2,"
    "wheel_meas_rpm_0,wheel_meas_rpm_1,wheel_meas_rpm_2,"
    "wheel_P_0,wheel_P_1,wheel_P_2,"
    "wheel_I_0,wheel_I_1,wheel_I_2,"
    "wheel_D_0,wheel_D_1,wheel_D_2,"
    "wheel_out_0,wheel_out_1,wheel_out_2,"
    "pitch_Kp,pitch_Ki,pitch_Kd,"
    "roll_Kp,roll_Ki,roll_Kd,"
    "pitch_deadband,roll_deadband,"
    "max_output_velocity,"
    "envelope_enter_sin,envelope_exit_sin,"
    "gyro_pitch_sign,gyro_roll_sign,"
    "tilt_per_velocity,max_tilt_setpoint,"
    "armed_state,in_fault,cmd_stale,"
    "event_flags";

// --- module state (anonymous namespace, single-instance) -------------------

constexpr std::size_t kRingSize  = BalanceTelemetryWs::kRingSize;
constexpr std::size_t kQueueLen  = 8;
constexpr std::size_t kCsvBufLen = 1200;  // ~14 chars/field * 83 fields, padded

AsyncWebSocket g_ws("/telemetry");

std::array<BalanceTelemetry, kRingSize> g_ring{};
std::atomic<std::uint32_t>               g_writeIdx{0};
std::array<std::atomic<std::uint32_t>, 16> g_eventCounts{};
std::atomic<std::uint32_t>               g_dropCount{0};
std::atomic<bool>                        g_initDone{false};

#ifdef ARDUINO
QueueHandle_t g_queue = nullptr;
TaskHandle_t  g_pumpTask = nullptr;
#else
// Native: a simple in-process deque models the producer queue. The pump
// task is not spawned; tests drive pumpOnce() explicitly.
std::deque<BalanceTelemetry> g_nativeQueue;
#endif

// --- CSV formatting --------------------------------------------------------

int writeCsv(char* buf, std::size_t buflen, const BalanceTelemetry& t) {
    // TODO(wave4): wire signs from BalanceConfig once telemetry struct has them.
    // BalanceTelemetry does not (yet) carry gyro_pitch_sign / gyro_roll_sign;
    // Wave 1 Agent B left them in BalanceConfig. Emit literal 0,0 placeholders
    // so the column count matches CSV_COLUMNS — the field positions are fixed,
    // tooling reads by index. Wave 4 reviewer will reconcile once the
    // controller copies signs into the live-gain block of BalanceTelemetry.
    return std::snprintf(
        buf, buflen,
        "%u,%u,%g,%g,"                              // seq, t_us, dt_measured, dt_used
        "%g,%g,%g,"                                 // cmd_vx_raw, cmd_vy_raw, cmd_omega_raw
        "%g,%g,%g,%u,"                              // cmd_vx, cmd_vy, cmd_omega, cmd_age_ms
        "%g,%g,%g,%g,"                              // quat_w/x/y/z
        "%g,%g,%g,"                                 // accel x/y/z
        "%g,%g,%g,"                                 // gyro raw x/y/z
        "%g,%g,%g,%g,"                              // gx, gy, gz, tilt_mag_sin
        "%g,%g,"                                    // pitch_actual, roll_actual
        "%g,%g,"                                    // gyro_pitch_rate, gyro_roll_rate
        "%g,%g,"                                    // pitch_target, roll_target
        "%g,%g,%g,%g,%g,%g,"                        // pitch_err/P/I/D/out_raw/out
        "%g,%g,%g,%g,%g,%g,"                        // roll_err/P/I/D/out_raw/out
        "%g,%g,%g,"                                 // body_vx_cmd, body_vy_cmd, body_omega_cmd
        "%g,%g,%g,"                                 // wheel_target_rpm_0..2
        "%g,%g,%g,"                                 // wheel_meas_rpm_0..2
        "%g,%g,%g,"                                 // wheel_P_0..2
        "%g,%g,%g,"                                 // wheel_I_0..2
        "%g,%g,%g,"                                 // wheel_D_0..2
        "%g,%g,%g,"                                 // wheel_out_0..2
        "%g,%g,%g,"                                 // pitch_Kp, Ki, Kd
        "%g,%g,%g,"                                 // roll_Kp, Ki, Kd
        "%g,%g,"                                    // pitch_deadband, roll_deadband
        "%g,"                                       // max_output_velocity
        "%g,%g,"                                    // envelope_enter_sin, envelope_exit_sin
        "0,0,"                                      // gyro_pitch_sign, gyro_roll_sign (placeholder)
        "%g,%g,"                                    // tilt_per_velocity, max_tilt_setpoint
        "%u,%u,%u,"                                 // armed_state, in_fault, cmd_stale
        "%u",                                       // event_flags
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

        static_cast<double>(t.pitch_target),
        static_cast<double>(t.roll_target),

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

        static_cast<double>(t.pitch_deadband),
        static_cast<double>(t.roll_deadband),

        static_cast<double>(t.max_output_velocity),

        static_cast<double>(t.envelope_enter_sin),
        static_cast<double>(t.envelope_exit_sin),

        static_cast<double>(t.tilt_per_velocity),
        static_cast<double>(t.max_tilt_setpoint),

        static_cast<unsigned>(t.armed_state),
        static_cast<unsigned>(t.in_fault),
        static_cast<unsigned>(t.cmd_stale),

        static_cast<unsigned>(t.event_flags));
}

// --- pump path -------------------------------------------------------------
// Format, broadcast, ring-write, and accumulate event counters for one
// snapshot. Single-writer w.r.t. the ring (only the pump task / pumpOnce
// reaches here), so the release store on g_writeIdx synchronises readers.

void pumpSnapshot(const BalanceTelemetry& snap) {
    char line[kCsvBufLen];
    int n = writeCsv(line, sizeof(line), snap);
    if (n > 0 && static_cast<std::size_t>(n) < sizeof(line)) {
        g_ws.textAll(String(line));
    }

    const std::uint32_t idx = g_writeIdx.load(std::memory_order_relaxed);
    g_ring[idx % kRingSize] = snap;
    g_writeIdx.store(idx + 1, std::memory_order_release);

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
        return;  // already initialised — idempotent
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
    server.addHandler(&g_ws);  // no-op stub
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

std::size_t snapshotRecent(BalanceTelemetry* out, std::size_t maxN) {
    if (!out || maxN == 0) return 0;
    const std::uint32_t total = g_writeIdx.load(std::memory_order_acquire);
    std::size_t avail = total < kRingSize ? static_cast<std::size_t>(total)
                                          : kRingSize;
    std::size_t n = avail < maxN ? avail : maxN;
    if (n == 0) return 0;
    // Source range: the n most-recent slots in chronological order. The
    // oldest of those is at index (total - n).
    const std::uint32_t start = total - static_cast<std::uint32_t>(n);
    for (std::size_t i = 0; i < n; ++i) {
        out[i] = g_ring[(start + i) % kRingSize];
    }
    return n;
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
    g_writeIdx.store(0, std::memory_order_release);
    g_dropCount.store(0, std::memory_order_relaxed);
    for (auto& c : g_eventCounts) c.store(0, std::memory_order_relaxed);
    for (auto& r : g_ring) r = BalanceTelemetry{};
#ifndef ARDUINO
    g_nativeQueue.clear();
#endif
}
#endif

}  // namespace BalanceTelemetryWs
