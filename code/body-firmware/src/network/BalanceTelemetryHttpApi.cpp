/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 */

#include "BalanceTelemetryHttpApi.h"

#include <ESPAsyncWebServer.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "BalanceTelemetry.h"
#include "BalanceTelemetryWs.h"

#ifndef ARDUINO
// Native tests: no millis(). The /telemetry/stats uptime field still needs
// a value; return 0 so tests don't depend on wall-clock.
static unsigned long millis() { return 0UL; }
#endif

namespace {

// --- CORS ------------------------------------------------------------------
// Mirror BalanceHttpApi.cpp exactly. GET-only here (no POSTs). The native
// test stub omits AsyncWebServerResponse / beginResponse / send, so these
// are gated behind ARDUINO — the JSON builders below are independent.

#ifdef ARDUINO
const char* const kCorsOrigin  = "*";
const char* const kCorsMethods = "GET, OPTIONS";
const char* const kCorsHeaders = "Content-Type";

void addCors(AsyncWebServerResponse* r) {
    r->addHeader("Access-Control-Allow-Origin",  kCorsOrigin);
    r->addHeader("Access-Control-Allow-Methods", kCorsMethods);
    r->addHeader("Access-Control-Allow-Headers", kCorsHeaders);
}

void sendJson(AsyncWebServerRequest* req, int code, const String& body) {
    auto* r = req->beginResponse(code, "application/json", body);
    addCors(r);
    req->send(r);
}

void sendText(AsyncWebServerRequest* req, int code, const String& body) {
    auto* r = req->beginResponse(code, "text/plain", body);
    addCors(r);
    req->send(r);
}
#endif  // ARDUINO

// --- Canonical CSV header --------------------------------------------------
// MUST match BalanceTelemetryWs.cpp's kHttpHeaderLine and tools/tune_pid.py's
// CSV_COLUMNS. Duplicated here intentionally (per Wave 3 split: don't reach
// into Agent H's files); Wave 4 reviewer may collapse to a shared constant.
constexpr const char* kHttpHeaderLine =
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

// --- Schema column metadata ------------------------------------------------
// One row per CSV column (83 entries). desc kept under 60 chars per the spec.

struct Column {
    const char* name;
    const char* type;
    const char* unit;
    const char* desc;
};

constexpr Column kSchema[] = {
    {"seq",                 "uint32", "monotonic", "publish sequence number"},
    {"t_us",                "uint32", "us",        "micros() at tick start"},
    {"dt_measured",         "float",  "s",         "real delta between ticks"},
    {"dt_used",             "float",  "s",         "dt fed to PID (nominal)"},
    {"cmd_vx_raw",          "float",  "m/s",       "freshest cmd vx pre-staleness"},
    {"cmd_vy_raw",          "float",  "m/s",       "freshest cmd vy pre-staleness"},
    {"cmd_omega_raw",       "float",  "rad/s",     "freshest cmd omega pre-stale"},
    {"cmd_vx",              "float",  "m/s",       "cmd vx after staleness ramp"},
    {"cmd_vy",              "float",  "m/s",       "cmd vy after staleness ramp"},
    {"cmd_omega",           "float",  "rad/s",     "cmd omega after stale ramp"},
    {"cmd_age_ms",          "uint32", "ms",        "ms since last fresh command"},
    {"quat_w",              "float",  "",          "IMU quaternion w"},
    {"quat_x",              "float",  "",          "IMU quaternion x"},
    {"quat_y",              "float",  "",          "IMU quaternion y"},
    {"quat_z",              "float",  "",          "IMU quaternion z"},
    {"accel_x",             "float",  "m/s^2",     "linear accel X (gravity-removed)"},
    {"accel_y",             "float",  "m/s^2",     "linear accel Y (gravity-removed)"},
    {"accel_z",             "float",  "m/s^2",     "linear accel Z (gravity-removed)"},
    {"gyro_x_raw",          "float",  "rad/s",     "gyro X pre-sign-flip"},
    {"gyro_y_raw",          "float",  "rad/s",     "gyro Y pre-sign-flip"},
    {"gyro_z_raw",          "float",  "rad/s",     "gyro Z pre-sign-flip"},
    {"gx",                  "float",  "",          "body-frame gravity unit X"},
    {"gy",                  "float",  "",          "body-frame gravity unit Y"},
    {"gz",                  "float",  "",          "body-frame gravity unit Z"},
    {"tilt_mag_sin",        "float",  "",          "sqrt(gx^2 + gy^2)"},
    {"pitch_actual",        "float",  "rad",       "current pitch"},
    {"roll_actual",         "float",  "rad",       "current roll"},
    {"gyro_pitch_rate",     "float",  "rad/s",     "pitch rate (signed, fed to D)"},
    {"gyro_roll_rate",      "float",  "rad/s",     "roll rate (signed, fed to D)"},
    {"gyro_yaw_rate",       "float",  "rad/s",     "yaw rate (signed, fed to inner yaw PID)"},
    {"pitch_target",        "float",  "rad",       "pitch setpoint from cmd"},
    {"roll_target",         "float",  "rad",       "roll setpoint from cmd"},
    {"heading_integrated",  "float",  "rad",       "integrated yaw heading"},
    {"heading_setpoint",    "float",  "rad",       "held heading (NaN if unlatched)"},
    {"heading_err",         "float",  "rad",       "heading_setpoint - heading_integrated"},
    {"heading_P",           "float",  "",          "heading P-term contribution"},
    {"omega_target_raw",    "float",  "rad/s",     "outer heading P output pre-LP"},
    {"omega_target",        "float",  "rad/s",     "yaw-rate setpoint into inner PID"},
    {"pitch_err",           "float",  "rad",       "pitch_target - pitch_actual"},
    {"pitch_P",             "float",  "",          "pitch P term"},
    {"pitch_I",             "float",  "",          "pitch I term (accumulator)"},
    {"pitch_D",             "float",  "",          "pitch D term"},
    {"pitch_out_raw",       "float",  "",          "pitch PID sum pre-clamp"},
    {"pitch_out",           "float",  "m/s",       "pitch PID post-clamp"},
    {"roll_err",            "float",  "rad",       "roll_target - roll_actual"},
    {"roll_P",              "float",  "",          "roll P term"},
    {"roll_I",              "float",  "",          "roll I term (accumulator)"},
    {"roll_D",              "float",  "",          "roll D term"},
    {"roll_out_raw",        "float",  "",          "roll PID sum pre-clamp"},
    {"roll_out",            "float",  "m/s",       "roll PID post-clamp"},
    {"yaw_rate_err",        "float",  "rad/s",     "omega_target - gyro_yaw_rate"},
    {"yaw_rate_P",          "float",  "",          "yaw-rate P term"},
    {"yaw_rate_I",          "float",  "",          "yaw-rate I term (accumulator)"},
    {"yaw_rate_D",          "float",  "",          "yaw-rate D term"},
    {"yaw_rate_out",        "float",  "rad/s",     "inner yaw PID post-clamp"},
    {"body_vx_cmd",         "float",  "m/s",       "body-frame vx to drivetrain"},
    {"body_vy_cmd",         "float",  "m/s",       "body-frame vy to drivetrain"},
    {"body_omega_cmd",      "float",  "rad/s",     "body-frame omega to drivetrain"},
    {"wheel_target_rpm_0",  "float",  "rpm",       "wheel 0 target rpm"},
    {"wheel_target_rpm_1",  "float",  "rpm",       "wheel 1 target rpm"},
    {"wheel_target_rpm_2",  "float",  "rpm",       "wheel 2 target rpm"},
    {"wheel_meas_rpm_0",    "float",  "rpm",       "wheel 0 measured rpm"},
    {"wheel_meas_rpm_1",    "float",  "rpm",       "wheel 1 measured rpm"},
    {"wheel_meas_rpm_2",    "float",  "rpm",       "wheel 2 measured rpm"},
    {"wheel_P_0",           "float",  "",          "wheel 0 PID P term"},
    {"wheel_P_1",           "float",  "",          "wheel 1 PID P term"},
    {"wheel_P_2",           "float",  "",          "wheel 2 PID P term"},
    {"wheel_I_0",           "float",  "",          "wheel 0 PID I term"},
    {"wheel_I_1",           "float",  "",          "wheel 1 PID I term"},
    {"wheel_I_2",           "float",  "",          "wheel 2 PID I term"},
    {"wheel_D_0",           "float",  "",          "wheel 0 PID D term"},
    {"wheel_D_1",           "float",  "",          "wheel 1 PID D term"},
    {"wheel_D_2",           "float",  "",          "wheel 2 PID D term"},
    {"wheel_out_0",         "float",  "pwm",       "wheel 0 PID output"},
    {"wheel_out_1",         "float",  "pwm",       "wheel 1 PID output"},
    {"wheel_out_2",         "float",  "pwm",       "wheel 2 PID output"},
    {"pitch_Kp",            "float",  "",          "live pitch P gain"},
    {"pitch_Ki",            "float",  "",          "live pitch I gain"},
    {"pitch_Kd",            "float",  "",          "live pitch D gain"},
    {"roll_Kp",             "float",  "",          "live roll P gain"},
    {"roll_Ki",             "float",  "",          "live roll I gain"},
    {"roll_Kd",             "float",  "",          "live roll D gain"},
    {"yaw_rate_Kp",         "float",  "",          "live yaw-rate P gain"},
    {"yaw_rate_Ki",         "float",  "",          "live yaw-rate I gain"},
    {"heading_Kp",          "float",  "",          "live heading P gain"},
    {"pitch_deadband",      "float",  "rad",       "pitch error deadband"},
    {"roll_deadband",       "float",  "rad",       "roll error deadband"},
    {"max_output_velocity", "float",  "m/s",       "PID output clamp magnitude"},
    {"envelope_enter_sin",  "float",  "",          "fault enter threshold"},
    {"envelope_exit_sin",   "float",  "",          "fault exit threshold"},
    {"gyro_pitch_sign",     "float",  "",          "+1/-1 gyro pitch sign"},
    {"gyro_roll_sign",      "float",  "",          "+1/-1 gyro roll sign"},
    {"gyro_yaw_sign",       "float",  "",          "+1/-1 gyro yaw sign"},
    {"tilt_per_velocity",   "float",  "rad/(m/s)", "tilt request per velocity"},
    {"max_tilt_setpoint",   "float",  "rad",       "max abs tilt target"},
    {"armed_state",         "uint8",  "",          "0=Disarmed 1=Armed 2=Killed"},
    {"in_fault",            "uint8",  "",          "Schmitt envelope tripped"},
    {"cmd_stale",           "uint8",  "",          "staleness ramp active"},
    {"event_flags",         "uint32", "bitfield",  "see event_bits decode"},
};
constexpr std::size_t kSchemaCount = sizeof(kSchema) / sizeof(kSchema[0]);
static_assert(kSchemaCount == 99, "schema must list all 99 CSV columns");

// Event-bit decode table — name + bit value. Order matches kEvent_* bit
// positions; emission order in the JSON matches BalanceTelemetryWs counters.
struct EventBit {
    const char* name;
    std::uint32_t bit;
};

constexpr EventBit kEventBits[] = {
    {"ARMED_EDGE",           kEvent_ARMED_EDGE},
    {"DISARMED_EDGE",        kEvent_DISARMED_EDGE},
    {"KILLED_EDGE",          kEvent_KILLED_EDGE},
    {"KILL_CLEARED",         kEvent_KILL_CLEARED},
    {"FAULT_ENTER",          kEvent_FAULT_ENTER},
    {"FAULT_EXIT",           kEvent_FAULT_EXIT},
    {"PITCH_DEADBAND_RESET", kEvent_PITCH_DEADBAND_RESET},
    {"ROLL_DEADBAND_RESET",  kEvent_ROLL_DEADBAND_RESET},
    {"PITCH_I_SATURATED",    kEvent_PITCH_I_SATURATED},
    {"ROLL_I_SATURATED",     kEvent_ROLL_I_SATURATED},
    {"PITCH_OUT_SATURATED",  kEvent_PITCH_OUT_SATURATED},
    {"ROLL_OUT_SATURATED",   kEvent_ROLL_OUT_SATURATED},
    {"GAIN_CHANGED",         kEvent_GAIN_CHANGED},
    {"CONFIG_SAVED",         kEvent_CONFIG_SAVED},
    {"CONFIG_RESET",         kEvent_CONFIG_RESET},
    {"STEP_INJECTED",        kEvent_STEP_INJECTED},
    {"YAW_RATE_I_SATURATED",   kEvent_YAW_RATE_I_SATURATED},
    {"YAW_RATE_OUT_SATURATED", kEvent_YAW_RATE_OUT_SATURATED},
    {"HEADING_LATCHED",        kEvent_HEADING_LATCHED},
    {"YAW_SPIN_RECOVERY",      kEvent_YAW_SPIN_RECOVERY},
    {"IMU_INVALID",            kEvent_IMU_INVALID},
    {"PREARM_REJECTED",        kEvent_PREARM_REJECTED},
};
constexpr std::size_t kEventBitsCount =
    sizeof(kEventBits) / sizeof(kEventBits[0]);
static_assert(kEventBitsCount == 22, "expected 22 event bits");

// --- JSON formatting -------------------------------------------------------
// Hand-rolled per BalanceHttpApi convention (no ArduinoJson). %g keeps the
// floats compact; bools/flags emit as 0/1 to match the CSV style.

// Format one BalanceTelemetry as an inline JSON object (no trailing comma).
// Returned via String& append so we can chain into arrays cheaply.
void appendSnapshotJson(String& out, const BalanceTelemetry& t) {
    char buf[2048];  // ~19 chars/field * 100, padded
    // Split at heading_setpoint so we can emit JSON null when it's NaN
    // (RFC 7159 forbids literal 'nan'). Every other float in the snapshot is
    // finite by construction — only heading_setpoint carries a NaN sentinel
    // (for "heading hold not latched"), so keep this minimal-touch.
    int n = std::snprintf(
        buf, sizeof(buf),
        "{"
        "\"seq\":%u,\"t_us\":%u,\"dt_measured\":%g,\"dt_used\":%g,"
        "\"cmd_vx_raw\":%g,\"cmd_vy_raw\":%g,\"cmd_omega_raw\":%g,"
        "\"cmd_vx\":%g,\"cmd_vy\":%g,\"cmd_omega\":%g,\"cmd_age_ms\":%u,"
        "\"quat_w\":%g,\"quat_x\":%g,\"quat_y\":%g,\"quat_z\":%g,"
        "\"accel_x\":%g,\"accel_y\":%g,\"accel_z\":%g,"
        "\"gyro_x_raw\":%g,\"gyro_y_raw\":%g,\"gyro_z_raw\":%g,"
        "\"gx\":%g,\"gy\":%g,\"gz\":%g,\"tilt_mag_sin\":%g,"
        "\"pitch_actual\":%g,\"roll_actual\":%g,"
        "\"gyro_pitch_rate\":%g,\"gyro_roll_rate\":%g,\"gyro_yaw_rate\":%g,"
        "\"pitch_target\":%g,\"roll_target\":%g,"
        "\"heading_integrated\":%g,",
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

        static_cast<double>(t.heading_integrated));
    if (n < 0 || n >= static_cast<int>(sizeof(buf))) {
        // Truncated — best-effort: emit what we have and bail.
        out += buf;
        return;
    }

    // heading_setpoint: NaN -> JSON null (RFC 7159 compliance), else %g value.
    int n2 = std::isnan(t.heading_setpoint)
        ? std::snprintf(buf + n, sizeof(buf) - n, "\"heading_setpoint\":null,")
        : std::snprintf(buf + n, sizeof(buf) - n, "\"heading_setpoint\":%g,",
                        static_cast<double>(t.heading_setpoint));
    if (n2 < 0 || n2 >= static_cast<int>(sizeof(buf) - n)) {
        out += buf;
        return;
    }
    n += n2;

    int n3 = std::snprintf(
        buf + n, sizeof(buf) - n,
        "\"heading_err\":%g,\"heading_P\":%g,"
        "\"omega_target_raw\":%g,\"omega_target\":%g,"
        "\"pitch_err\":%g,\"pitch_P\":%g,\"pitch_I\":%g,\"pitch_D\":%g,"
        "\"pitch_out_raw\":%g,\"pitch_out\":%g,"
        "\"roll_err\":%g,\"roll_P\":%g,\"roll_I\":%g,\"roll_D\":%g,"
        "\"roll_out_raw\":%g,\"roll_out\":%g,"
        "\"yaw_rate_err\":%g,\"yaw_rate_P\":%g,\"yaw_rate_I\":%g,\"yaw_rate_D\":%g,"
        "\"yaw_rate_out\":%g,"
        "\"body_vx_cmd\":%g,\"body_vy_cmd\":%g,\"body_omega_cmd\":%g,"
        "\"wheel_target_rpm_0\":%g,\"wheel_target_rpm_1\":%g,\"wheel_target_rpm_2\":%g,"
        "\"wheel_meas_rpm_0\":%g,\"wheel_meas_rpm_1\":%g,\"wheel_meas_rpm_2\":%g,"
        "\"wheel_P_0\":%g,\"wheel_P_1\":%g,\"wheel_P_2\":%g,"
        "\"wheel_I_0\":%g,\"wheel_I_1\":%g,\"wheel_I_2\":%g,"
        "\"wheel_D_0\":%g,\"wheel_D_1\":%g,\"wheel_D_2\":%g,"
        "\"wheel_out_0\":%g,\"wheel_out_1\":%g,\"wheel_out_2\":%g,"
        "\"pitch_Kp\":%g,\"pitch_Ki\":%g,\"pitch_Kd\":%g,"
        "\"roll_Kp\":%g,\"roll_Ki\":%g,\"roll_Kd\":%g,"
        "\"yaw_rate_Kp\":%g,\"yaw_rate_Ki\":%g,\"heading_Kp\":%g,"
        "\"pitch_deadband\":%g,\"roll_deadband\":%g,"
        "\"max_output_velocity\":%g,"
        "\"envelope_enter_sin\":%g,\"envelope_exit_sin\":%g,"
        "\"gyro_pitch_sign\":%g,\"gyro_roll_sign\":%g,\"gyro_yaw_sign\":%g,"
        "\"tilt_per_velocity\":%g,\"max_tilt_setpoint\":%g,"
        "\"armed_state\":%u,\"in_fault\":%u,\"cmd_stale\":%u,"
        "\"event_flags\":%u"
        "}",
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
    if (n3 < 0 || n3 >= static_cast<int>(sizeof(buf) - n)) {
        // Truncated — best-effort: emit what we have and bail.
        out += buf;
        return;
    }
    out += buf;
}

String latestJson() {
    BalanceTelemetry snap{};
    // Zero-init means a "no data yet" response still has every key — clients
    // can build a column index from /telemetry/latest at boot.
    (void)BalanceTelemetryWs::latest(&snap);
    String out;
    out.reserve(1700);
    appendSnapshotJson(out, snap);
    return out;
}

String statsJson() {
    float mn_s = 0.0f, mean_s = 0.0f, mx_s = 0.0f;
    BalanceTelemetryWs::dtStats(&mn_s, &mean_s, &mx_s);
    const double mn_ms   = static_cast<double>(mn_s)   * 1000.0;
    const double mean_ms = static_cast<double>(mean_s) * 1000.0;
    const double mx_ms   = static_cast<double>(mx_s)   * 1000.0;

    const std::uint32_t seq         = BalanceTelemetryWs::writeIdx();
    const std::uint32_t in_fault    = BalanceTelemetryWs::eventCount(4);
    const std::uint32_t ws_drops    = BalanceTelemetryWs::dropCount();
    const double        uptime_s    = static_cast<double>(millis()) / 1000.0;

    String out;
    out.reserve(1024);

    char head[512];
    std::snprintf(
        head, sizeof(head),
        "{"
        "\"seq\":%u,"
        "\"uptime_s\":%g,"
        "\"dt_ms\":{\"min\":%g,\"mean\":%g,\"max\":%g,\"jitter\":%g},"
        "\"in_fault_count\":%u,"
        "\"events_since_boot\":{",
        static_cast<unsigned>(seq),
        uptime_s,
        mn_ms, mean_ms, mx_ms, (mx_ms - mn_ms),
        static_cast<unsigned>(in_fault));
    out += head;

    for (std::size_t i = 0; i < kEventBitsCount; ++i) {
        if (i > 0) out += ",";
        // bit index = position in kEventBits (matches the underlying counter
        // index because we listed them in bit order).
        const std::uint32_t cnt = BalanceTelemetryWs::eventCount(i);
        char ev[96];
        std::snprintf(ev, sizeof(ev), "\"%s\":%u",
                      kEventBits[i].name, static_cast<unsigned>(cnt));
        out += ev;
    }

    char tail[96];
    std::snprintf(
        tail, sizeof(tail),
        "},\"ws_drops\":%u}",
        static_cast<unsigned>(ws_drops));
    out += tail;
    return out;
}

String schemaJson() {
    String out;
    out.reserve(8192);
    out += "{\"columns\":[";
    for (std::size_t i = 0; i < kSchemaCount; ++i) {
        if (i > 0) out += ",";
        char row[256];
        std::snprintf(row, sizeof(row),
                      "{\"name\":\"%s\",\"type\":\"%s\",\"unit\":\"%s\",\"desc\":\"%s\"}",
                      kSchema[i].name, kSchema[i].type,
                      kSchema[i].unit, kSchema[i].desc);
        out += row;
    }
    out += "],\"event_bits\":{";
    for (std::size_t i = 0; i < kEventBitsCount; ++i) {
        if (i > 0) out += ",";
        char row[64];
        std::snprintf(row, sizeof(row), "\"%s\":%u",
                      kEventBits[i].name,
                      static_cast<unsigned>(kEventBits[i].bit));
        out += row;
    }
    out += "}}";
    return out;
}

String headerText() {
    return String(kHttpHeaderLine);
}

}  // namespace

namespace BalanceTelemetryHttpApi {

void registerRoutes(AsyncWebServer& server) {
#ifdef ARDUINO
    // The native test stub for AsyncWebServer deliberately omits .on() and
    // the HTTP_* method constants — under native, JSON/text builders are
    // exercised directly via BB8_TEST_HOOKS, so the route wiring is a no-op.
    server.on("/telemetry/latest", HTTP_GET, [](AsyncWebServerRequest* req) {
        sendJson(req, 200, latestJson());
    });

    server.on("/telemetry/stats", HTTP_GET, [](AsyncWebServerRequest* req) {
        sendJson(req, 200, statsJson());
    });

    server.on("/telemetry/schema", HTTP_GET, [](AsyncWebServerRequest* req) {
        sendJson(req, 200, schemaJson());
    });

    server.on("/telemetry/header", HTTP_GET, [](AsyncWebServerRequest* req) {
        sendText(req, 200, headerText());
    });

    // CORS preflight — match each /telemetry/* with OPTIONS.
    server.on("/telemetry/latest", HTTP_OPTIONS, [](AsyncWebServerRequest* req) {
        sendJson(req, 204, String());
    });
    server.on("/telemetry/stats", HTTP_OPTIONS, [](AsyncWebServerRequest* req) {
        sendJson(req, 204, String());
    });
    server.on("/telemetry/schema", HTTP_OPTIONS, [](AsyncWebServerRequest* req) {
        sendJson(req, 204, String());
    });
    server.on("/telemetry/header", HTTP_OPTIONS, [](AsyncWebServerRequest* req) {
        sendJson(req, 204, String());
    });
#else
    (void)server;
#endif
}

#ifdef BB8_TEST_HOOKS
String buildLatestJson()                 { return latestJson(); }
String buildStatsJson()                  { return statsJson(); }
String buildSchemaJson()                 { return schemaJson(); }
String buildHeader()                     { return headerText(); }
#endif

}  // namespace BalanceTelemetryHttpApi
