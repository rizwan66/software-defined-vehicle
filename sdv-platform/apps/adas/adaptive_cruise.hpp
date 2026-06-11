// =============================================================================
// apps/adas/adaptive_cruise.hpp
//
// Adaptive Cruise Control (ACC) function.
//
// Subscribes to:
//   vehicle.speed          (SpeedSample, km/h)
//   radar.lead_vehicle     (LeadVehicle: range_m, rel_velocity_mps)
//
// Publishes:
//   acc.throttle_request   (AccRequest: throttle [0,1], brake [0,1], active)
//
// Control strategy:
//   * If no lead vehicle: track set-point speed (speed-only control)
//   * If lead vehicle detected: maintain minimum headway_s seconds gap
//     using a proportional-derivative controller on the gap error.
//
// ACC state machine:
//   STANDBY  → (driver sets speed)  → ACTIVE
//   ACTIVE   → (driver brakes)       → STANDBY
//   ACTIVE   → (gap < kMinGapM)      → FOLLOWING
//   FOLLOWING → (lead gone)          → ACTIVE
//   any      → (fault)               → SAFE_STOP
//
// Safety (ISO 15622 / SOTIF relevant):
//   * Speed valid check — no actuation on degraded speed signal
//   * Minimum speed enforcement (no ACC below kMinActiveSpeedKmh)
//   * Deceleration limited to kMaxDecelerationGs × 9.81 m/s²
//   * Gap below kEmergencyGapM → hand off to AEB (this function does NOT brake hard)
// =============================================================================
#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>

#include "middleware/communication/message_bus.hpp"
#include "services/vehicle-signals/vehicle_speed_service.hpp"

namespace sdv::apps::adas {

struct LeadVehicle {
    float range_m{0.0f};           // distance to lead vehicle
    float rel_velocity_mps{0.0f};  // positive = lead moving away
    bool  detected{false};
    uint64_t timestamp_ns{0};
};

struct AccRequest {
    float throttle{0.0f};    // [0, 1]
    float brake{0.0f};       // [0, 1]
    bool  active{false};
    float set_speed_kmh{0.0f};
    float actual_gap_s{0.0f};
    uint64_t timestamp_ns{0};
};

enum class AccState : uint8_t { Standby, Active, Following, SafeStop };

constexpr std::string_view toString(AccState s) {
    switch (s) {
        case AccState::Standby:   return "STANDBY";
        case AccState::Active:    return "ACTIVE";
        case AccState::Following: return "FOLLOWING";
        case AccState::SafeStop:  return "SAFE_STOP";
    }
    return "?";
}

class AdaptiveCruiseControl {
public:
    static constexpr float kDefaultSetSpeedKmh  = 120.0f;
    static constexpr float kMinActiveSpeedKmh   = 30.0f;
    static constexpr float kTargetHeadwayS      = 2.0f;    // 2-second rule
    static constexpr float kMinGapM             = 5.0f;
    static constexpr float kEmergencyGapM       = 2.0f;    // hand off to AEB
    static constexpr float kMaxDecelerationGs   = 0.3f;    // 0.3g max decel
    static constexpr float kSpeedKp             = 0.05f;   // throttle/km/h error
    static constexpr float kGapKp               = 0.02f;   // throttle/m gap error
    static constexpr float kGapKd               = 0.1f;    // derivative gain

    explicit AdaptiveCruiseControl(middleware::MessageBus& bus,
                                   float set_speed_kmh = kDefaultSetSpeedKmh)
        : bus_(bus), set_speed_kmh_(set_speed_kmh) {
        speed_sub_ = bus_.subscribe(
            "vehicle.speed", [this](const middleware::Sample& s) {
                const auto sp = std::any_cast<services::SpeedSample>(s.payload);
                ego_speed_kmh_.store(sp.kmh);
                speed_valid_.store(sp.quality == services::SignalQuality::Valid);
                evaluate(s.timestamp_ns);
            });
        radar_sub_ = bus_.subscribe(
            "radar.lead_vehicle", [this](const middleware::Sample& s) {
                lead_ = std::any_cast<LeadVehicle>(s.payload);
            });
    }

    ~AdaptiveCruiseControl() {
        bus_.unsubscribe("vehicle.speed",      speed_sub_);
        bus_.unsubscribe("radar.lead_vehicle", radar_sub_);
    }

    void setSpeed(float kmh) { set_speed_kmh_.store(std::clamp(kmh, 0.0f, 250.0f)); }

    [[nodiscard]] AccState state()     const noexcept { return state_.load(); }
    [[nodiscard]] bool isActive()      const noexcept { return state_.load() == AccState::Active ||
                                                               state_.load() == AccState::Following; }
    [[nodiscard]] bool isSafeState()   const noexcept { return state_.load() == AccState::Standby ||
                                                               state_.load() == AccState::SafeStop; }
    [[nodiscard]] AccRequest lastRequest() const noexcept { return last_req_; }

private:
    void evaluate(uint64_t ts_ns) {
        const float ego  = ego_speed_kmh_.load();
        const bool  valid = speed_valid_.load();
        const float target = set_speed_kmh_.load();
        const LeadVehicle lead = lead_;

        // ── Safety guard ──────────────────────────────────────────────
        if (!valid || ego < kMinActiveSpeedKmh) {
            transition(AccState::Standby);
            publish({}, ts_ns);
            return;
        }

        // ── Emergency gap → hand off to AEB ──────────────────────────
        if (lead.detected && lead.range_m < kEmergencyGapM) {
            transition(AccState::SafeStop);
            publish({0.0f, 1.0f, false, target, 0.0f, ts_ns}, ts_ns);
            return;
        }

        float throttle = 0.0f;
        float brake    = 0.0f;
        float gap_s    = 0.0f;

        if (lead.detected && lead.range_m > 0.0f) {
            // ── Following mode ────────────────────────────────────────
            transition(AccState::Following);
            const float ego_mps = ego / 3.6f;
            gap_s = (ego_mps > 0.1f) ? (lead.range_m / ego_mps) : 999.0f;
            const float gap_error = gap_s - kTargetHeadwayS;
            const float gap_cmd   = kGapKp * gap_error + kGapKd * lead.rel_velocity_mps;
            throttle = std::clamp(gap_cmd, 0.0f, 1.0f);
            brake    = std::clamp(-gap_cmd, 0.0f, kMaxDecelerationGs);
        } else {
            // ── Speed-only mode ───────────────────────────────────────
            transition(AccState::Active);
            const float speed_error = target - ego;
            const float cmd = kSpeedKp * speed_error;
            throttle = std::clamp(cmd,  0.0f, 1.0f);
            brake    = std::clamp(-cmd, 0.0f, kMaxDecelerationGs);
        }

        last_req_ = {throttle, brake, true, target, gap_s, ts_ns};
        publish(last_req_, ts_ns);
    }

    void transition(AccState s) { state_.store(s); }

    void publish(const AccRequest& req, uint64_t ts_ns) {
        bus_.publish("acc.throttle_request", req, ts_ns);
    }

    middleware::MessageBus& bus_;
    middleware::MessageBus::SubscriptionId speed_sub_{0};
    middleware::MessageBus::SubscriptionId radar_sub_{0};

    std::atomic<float>     ego_speed_kmh_{0.0f};
    std::atomic<bool>      speed_valid_{false};
    std::atomic<float>     set_speed_kmh_;
    std::atomic<AccState>  state_{AccState::Standby};

    LeadVehicle lead_{};
    AccRequest  last_req_{};
};

}  // namespace sdv::apps::adas
