// =============================================================================
// services/dynamics/vehicle_model.hpp
//
// Kinematic Bicycle Model — Vehicle Dynamics Simulation
//
// The kinematic bicycle model is the standard foundation for ADAS simulation,
// MPC (Model Predictive Control) path planning, and software-in-the-loop (SiL)
// testing. It treats the vehicle as a bicycle: one front wheel, one rear wheel,
// both on the vehicle centreline.
//
// State vector: [x, y, ψ, v]
//   x    — longitudinal position in world frame (m)
//   y    — lateral position in world frame (m)
//   ψ    — heading angle (rad), 0 = East, positive = CCW
//   v    — longitudinal speed (m/s)
//
// Inputs:
//   δ    — front wheel steering angle (rad)
//   a    — longitudinal acceleration (m/s²), positive = forward
//
// Differential equations (continuous time):
//   ẋ   = v · cos(ψ)
//   ẏ   = v · sin(ψ)
//   ψ̇  = v / L · tan(δ)       (L = wheelbase, ~2.7 m for sedan)
//   v̇  = a
//
// Integrated via explicit Euler (dt ≤ 10 ms is sufficiently accurate).
//
// Derived signals (published on message bus):
//   vehicle.speed         — v in km/h
//   vehicle.yaw_rate      — ψ̇ in rad/s
//   vehicle.lateral_accel — centripetal a_y = v²/R = v · ψ̇  (m/s²)
//   obstacle.distance     — distance from vehicle to a fixed obstacle
//
// Usage:
//   VehicleModel model;
//   model.step(dt=0.01f, accel=2.0f, steering=0.05f);
//   auto s = model.state();  // read position, speed, heading
// =============================================================================
#pragma once

#include <cmath>
#include <cstdint>
#include <numbers>  // std::numbers::pi (C++20)

#include "middleware/communication/message_bus.hpp"
#include "middleware/units/physical_units.hpp"

namespace sdv::services::dynamics {

struct VehicleState {
    float x_m{0.0f};             // world X position (m)
    float y_m{0.0f};             // world Y position (m)
    float psi_rad{0.0f};         // heading (rad)
    float speed_mps{0.0f};       // longitudinal speed (m/s)
    float yaw_rate_rps{0.0f};    // yaw rate (rad/s)
    float lat_accel_mps2{0.0f};  // lateral acceleration (m/s²)
    uint64_t timestamp_ns{0};
};

class VehicleModel {
public:
    static constexpr float kWheelbase_m   = 2.7f;   // typical sedan
    static constexpr float kMaxSpeedMps   = 111.0f; // ~400 km/h
    static constexpr float kMaxSteer_rad  = 0.5f;   // ~28° — mechanical stop
    static constexpr float kMaxAccel_mps2 = 10.0f;  // ~1 g

    struct Config {
        float wheelbase_m{kWheelbase_m};
        // World origin position of a stationary obstacle (for AEB sim)
        float obstacle_x_m{200.0f};
        float obstacle_y_m{0.0f};
    };

    explicit VehicleModel(middleware::MessageBus& bus)
        : bus_(bus), cfg_({}) {}
    VehicleModel(middleware::MessageBus& bus, Config cfg)
        : bus_(bus), cfg_(std::move(cfg)) {}

    // Advance the model by dt seconds with the given driver inputs.
    // Returns the new state.
    VehicleState step(float dt_s,
                      float accel_mps2     = 0.0f,
                      float steering_rad   = 0.0f) {
        // Clamp inputs to physical limits
        accel_mps2   = std::clamp(accel_mps2,   -kMaxAccel_mps2, kMaxAccel_mps2);
        steering_rad = std::clamp(steering_rad, -kMaxSteer_rad,  kMaxSteer_rad);

        // ── Euler integration ────────────────────────────────────────
        state_.x_m    += state_.speed_mps * std::cos(state_.psi_rad) * dt_s;
        state_.y_m    += state_.speed_mps * std::sin(state_.psi_rad) * dt_s;

        if (std::abs(state_.speed_mps) > 0.001f) {
            state_.yaw_rate_rps = state_.speed_mps / cfg_.wheelbase_m
                                  * std::tan(steering_rad);
        } else {
            state_.yaw_rate_rps = 0.0f;
        }
        state_.psi_rad    += state_.yaw_rate_rps * dt_s;
        state_.speed_mps   = std::clamp(state_.speed_mps + accel_mps2 * dt_s,
                                        0.0f, kMaxSpeedMps);
        state_.lat_accel_mps2 = state_.speed_mps * state_.yaw_rate_rps;

        // Publish derived signals on the SDV message bus
        publishSignals();
        return state_;
    }

    [[nodiscard]] const VehicleState& state() const noexcept { return state_; }

    // Distance to the fixed obstacle in the world frame.
    [[nodiscard]] float obstacleDistanceM() const noexcept {
        const float dx = cfg_.obstacle_x_m - state_.x_m;
        const float dy = cfg_.obstacle_y_m - state_.y_m;
        return std::sqrt(dx * dx + dy * dy);
    }

    // Speed as type-safe unit.
    [[nodiscard]] units::Kmh speedKmh() const noexcept {
        return units::unit_cast<units::Kmh>(units::Mps{state_.speed_mps});
    }

    void reset() noexcept { state_ = VehicleState{}; }

private:
    void publishSignals() {
        const uint64_t ts_ns = static_cast<uint64_t>(state_.timestamp_ns++);

        // Publish speed as SpeedSample-compatible float (services pick it up)
        // The VehicleSpeedService normally reads from CAN; in simulation mode
        // we publish directly so ADAS functions can consume the physics output.
        bus_.publish("sim.vehicle.speed_mps", state_.speed_mps, ts_ns);
        bus_.publish("sim.vehicle.yaw_rate",  state_.yaw_rate_rps, ts_ns);
        bus_.publish("sim.vehicle.lat_accel", state_.lat_accel_mps2, ts_ns);
        bus_.publish("sim.obstacle.distance", obstacleDistanceM(), ts_ns);
    }

    middleware::MessageBus& bus_;
    Config cfg_;
    VehicleState state_{};
};

// ─── Scenario helpers ─────────────────────────────────────────────────────────

// Drive at constant speed for duration_s, return final state.
inline VehicleState runConstantSpeed(VehicleModel& model,
                                     float target_mps, float duration_s,
                                     float dt_s = 0.01f) {
    VehicleState s{};
    for (float t = 0.0f; t < duration_s; t += dt_s) {
        const float err = target_mps - s.speed_mps;
        s = model.step(dt_s, std::clamp(err * 5.0f, -5.0f, 5.0f), 0.0f);
    }
    return s;
}

}  // namespace sdv::services::dynamics
