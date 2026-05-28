/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 *
 * WebSocketCommandProducer — receives teleop commands from a WebSocket client
 * and pushes them into a CommandLatch<BodyVelocity>.
 *
 * Wire protocol (text frames, UTF-8):
 *   "vx,vy,omega"          — three comma-separated floats, optional trailing
 *                             whitespace/newline
 *     vx, vy  (m/s)        — body-frame linear velocity (x forward, y left)
 *     omega   (rad/s)      — yaw rate, CCW positive
 *
 * Expected send rate: ~50–100 Hz from the operator. The control loop holds
 * the last command for 200 ms (linear ramp to zero); silence beyond that is
 * a hard stop.
 *
 * Forward compatibility: extra trailing fields after `omega` are silently
 * ignored, so adding a 4th field client-side will not break this parser.
 *
 * Trust boundary: plaintext ws:// with no authentication. Intended for
 * trusted-LAN operation only. Add a token handshake before exposing on any
 * shared network or public demo.
 */

#pragma once

#include "ArmingState.h"
#include "CommandFrameParser.h"
#include "CommandProducer.h"
#include "Lifecycle.h"
#include "sync/CommandLatch.h"
#include "BodyVelocity.h"

#ifdef ARDUINO
#include <Arduino.h>
#endif

#include <atomic>
#include <cstdint>
#include <functional>

class WebSocketsServer;  // forward decl from arduinoWebSockets

class WebSocketCommandProducer : public CommandProducer, public Lifecycle {
public:
    WebSocketCommandProducer(CommandLatch<BodyVelocity>& latch, uint16_t port = 80);
    ~WebSocketCommandProducer() override;

    WebSocketCommandProducer(const WebSocketCommandProducer&) = delete;
    WebSocketCommandProducer& operator=(const WebSocketCommandProducer&) = delete;

    void start() override;
    void stop() override;
    bool connected() const override;

    // Route a parsed frame: velocity frames write the latch, control frames
    // invoke the matching ArmingState transition. QueryArmState is a
    // read-only verb that fires the supplied callback with the current arming
    // state so the caller (typically a WS handler) can echo back a status
    // line. The callback is optional; pass nullptr to no-op the query.
    // Defined inline so the dispatch logic can be unit-tested without linking
    // the transport layer.
    using QueryArmStateCallback = std::function<void(ArmingState::State)>;
    static inline void dispatchFrame(CommandLatch<BodyVelocity>& latch,
                                     std::atomic<uint32_t>& frameCount,
                                     const FrameParseResult& r,
                                     const QueryArmStateCallback& onQueryArmState = nullptr) {
        if (r.kind == FrameKind::Velocity) {
            latch.write(r.velocity);
            frameCount.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        const char* verbName = "?";
        switch (r.control) {
            case ControlVerb::Arm:       ArmingState::arm();       verbName = "arm";       break;
            case ControlVerb::Disarm:    ArmingState::disarm();    verbName = "disarm";    break;
            case ControlVerb::Kill:      ArmingState::kill();      verbName = "kill";      break;
            case ControlVerb::ClearKill: ArmingState::clearKill(); verbName = "clearkill"; break;
            case ControlVerb::QueryArmState:
                verbName = "armstate?";
                if (onQueryArmState) onQueryArmState(ArmingState::get());
                break;
        }
#ifdef ARDUINO
        Serial.printf("[ws] control: %s\n", verbName);
#else
        (void)verbName;
#endif
    }

private:
    static void taskTrampoline(void* arg);
    void taskBody();
    void onWsEvent(uint8_t clientNum, uint8_t type, const uint8_t* payload, size_t length);
    void logParseFail(const char* field);

    static constexpr uint8_t kNoClient = 0xFF;  // sentinel: no active client

    CommandLatch<BodyVelocity>& _latch;
    uint16_t _port;
    WebSocketsServer* _server;          // heap-allocated to avoid pulling library header into ours
    void* _taskHandle;                  // TaskHandle_t under the hood
    void* _exitSemaphore;               // SemaphoreHandle_t under the hood
    std::atomic<bool> _running;
    std::atomic<bool> _connected;
    std::atomic<uint8_t> _activeClient; // single-client policy: only one driver at a time
    std::atomic<uint32_t> _frameCount;
    std::atomic<uint32_t> _parseFailCount;
    uint32_t _lastParseFailLogMs = 0;
};
