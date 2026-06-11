// =============================================================================
// apps/adas/lane_keeping.hpp
//
// Lane Keeping Assist (LKA) function.
//
// Subscribes to:
//   lane.lateral_error   (float, meters — positive = drifting right)
//   lane.heading_error   (float, radians — positive = yawing right)
//   vehicle.speed        (SpeedSample)
//
// Publishes:
//   lka.steering_torque  (SteeringTorque, Nm)
//
// Control law: PD controller on lateral error and heading.
//   torque = Kp * lateral_error + Kd * heading_error
//   torque is clamped to ±kMaxTorqueNm
//
// Safety contracts (SOTIF-relevant):
//   * LKA deactivates below kMinSpeedKmh (not meaningful at low speed)
//   * LKA deactivates when vehicle.speed quality is not Valid
//   * LKA suppresses output when lane data is unavailable (confidence = 0)
//   * Steering torque is rate-limited to prevent sudden jerks (Δt ≤ kMaxRatioPerCycle)
//   * Driver override detected via steering wheel torque > kDriverTorqueThresholdNm
//
// Production mapping: ISO 11270, AUTOSAR AP ADAS function model
// =============================================================================
#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>

#include "middleware/communication/message_bus.hpp"
#include "services/vehicle-signals/vehicle_speed_service.hpp"

namespace sdv::apps::adas {

struct LaneData {
    float lateral_error_m{0.0f};   // meters from lane center (+right, -left)
    float heading_error_rad{0.0f}; // radians relative to lane tangent
    float confidence{1.0f};        // 0 = no lane, 1 = high confidence
    uint64_t timestamp_ns{0};
};

struct SteeringTorque {
    float torque_nm{0.0f};   // Nm, positive = steer right
    bool  lka_active{false};
    uint64_t timestamp_ns{0};
};

class LaneKeepingAssist {
public:
    // PD gains — tuned for a typical passenger vehicle at highway speeds.
    static constexpr float kKp = 8.0f;         // Nm/m
    static constexpr float kKd = 15.0f;        // Nm/rad
    static constexpr float kMaxTorqueNm = 5.0f;
    static constexpr float kMaxRateNmPerCycle = 0.5f;
    static constexpr float kMinSpeedKmh = 60.0f;
    static constexpr float kMinLaneConfidence = 0.5f;
    static constexpr float kDriverOverrideTorqueNm = 3.0f;

    explicit LaneKeepingAssist(middleware::MessageBus& bus) : bus_(bus) {
        speed_sub_ = bus_.subscribe(
            "vehicle.speed", [this](const middleware::Sample& s) {
                const auto sp = std::any_cast<services::SpeedSample>(s.payload);
                speed_kmh_.store(sp.kmh);
                speed_valid_.store(sp.quality == services::SignalQuality::Valid);
            });
        lane_sub_ = bus_.subscribe(
            "lane.data", [this](const middleware::Sample& s) {
                latest_lane_ = std::any_cast<LaneData>(s.payload);
                evaluate(s.timestamp_ns);
            });
        driver_torque_sub_ = bus_.subscribe(
            "driver.steering_torque", [this](const middleware::Sample& s) {
                driver_torque_nm_.store(std::any_cast<float>(s.payload));
            });
    }

    ~LaneKeepingAssist() {
        bus_.unsubscribe("vehicle.speed",         speed_sub_);
        bus_.unsubscribe("lane.data",             lane_sub_);
        bus_.unsubscribe("driver.steering_torque",driver_sub_);
    }

    // ara::com concept: isActive() reports operational state to ADAS supervisor.
    [[nodiscard]] bool isActive()    const noexcept { return active_.load(); }
    [[nodiscard]] bool isSafeState() const noexcept { return !active_.load(); }

    [[nodiscard]] float lastTorqueNm() const noexcept {
        return last_torque_nm_.load();
    }

private:
    void evaluate(uint64_t ts_ns) {
        const float spd   = speed_kmh_.load();
        const bool  valid = speed_valid_.load();
        const LaneData lane = latest_lane_;
        const float driver_torque = driver_torque_nm_.load();

        // ── Safety gate ──────────────────────────────────────────────
        const bool activate =
            valid &&
            spd >= kMinSpeedKmh &&
            lane.confidence >= kMinLaneConfidence &&
            std::abs(driver_torque) < kDriverOverrideTorqueNm;

        active_.store(activate);

        float torque_cmd = 0.0f;
        if (activate) {
            // PD control
            torque_cmd = kKp * lane.lateral_error_m +
                         kKd * lane.heading_error_rad;
            torque_cmd = std::clamp(torque_cmd, -kMaxTorqueNm, kMaxTorqueNm);

            // Rate limiting — smooth out sudden jumps.
            const float prev = last_torque_nm_.load();
            const float delta = torque_cmd - prev;
            if (std::abs(delta) > kMaxRateNmPerCycle) {
                torque_cmd = prev + std::copysign(kMaxRateNmPerCycle, delta);
            }
        }

        last_torque_nm_.store(torque_cmd);
        bus_.publish("lka.steering_torque",
                     SteeringTorque{torque_cmd, activate, ts_ns},
                     ts_ns);
    }

    middleware::MessageBus& bus_;
    middleware::MessageBus::SubscriptionId speed_sub_{0};
    middleware::MessageBus::SubscriptionId lane_sub_{0};
    middleware::MessageBus::SubscriptionId driver_torque_sub_{0};
    middleware::MessageBus::SubscriptionId driver_sub_{0};

    std::atomic<float> speed_kmh_{0.0f};
    std::atomic<bool>  speed_valid_{false};
    std::atomic<float> driver_torque_nm_{0.0f};
    std::atomic<bool>  active_{false};
    std::atomic<float> last_torque_nm_{0.0f};

    // LaneData is not atomic — must be read/written only from the lane_sub_ callback.
    // In a real system this would be a lock-protected double-buffer.
    LaneData latest_lane_{};
};

}  // namespace sdv::apps::adas
