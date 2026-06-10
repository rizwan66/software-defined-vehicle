// =============================================================================
// services/vehicle-signals/vehicle_speed_service.hpp
//
// Production-grade version of the classic "VehicleSpeedService" example.
// Responsibilities:
//   * cyclically read raw CAN frames from the bus interface
//   * decode the speed signal (ID 0x123, bytes 0..1 LE, scale 0.01 km/h)
//   * run plausibility checks (range + rate-of-change)
//   * track signal timeout and attach a quality flag to every sample
//   * publish a typed SpeedSample on the middleware message bus
//
// Design notes (the things interviewers ask about):
//   * Dependency injection of ICanInterface -> unit-testable without hardware
//   * No blocking calls in the cycle -> deterministic 10 ms task period
//   * Last-valid-value substitution with degraded quality, never stale data
//     silently presented as fresh (relevant for ASIL decomposition arguments)
// =============================================================================
#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <thread>

#include "drivers/can/can_interface.hpp"
#include "middleware/communication/message_bus.hpp"

namespace sdv::services {

enum class SignalQuality : uint8_t {
    Valid,        // fresh, plausible value
    Degraded,     // last valid value substituted (timeout or implausible frame)
    Invalid       // no valid value available since startup
};

struct SpeedSample {
    float kmh{0.0f};
    SignalQuality quality{SignalQuality::Invalid};
    uint64_t timestamp_ns{0};
};

class VehicleSpeedService {
public:
    static constexpr uint32_t kSpeedCanId = 0x123;
    static constexpr float kScale = 0.01f;              // km/h per bit
    static constexpr float kMaxPlausibleKmh = 400.0f;   // range check
    static constexpr float kMaxJumpKmhPerCycle = 15.0f; // ~150 km/h/s at 10 ms
    static constexpr std::chrono::milliseconds kSignalTimeout{200};
    static constexpr std::chrono::milliseconds kCyclePeriod{10};

    VehicleSpeedService(drivers::ICanInterface& can,
                        middleware::MessageBus& bus)
        : can_(can), bus_(bus) {}

    ~VehicleSpeedService() { stop(); }

    void start() {
        running_.store(true);
        worker_ = std::thread([this] { runLoop(); });
    }

    void stop() {
        running_.store(false);
        if (worker_.joinable()) worker_.join();
    }

    // Single execution of the cyclic task. Public so unit tests can drive the
    // service deterministically without threads.
    void cycle() {
        const auto now = std::chrono::steady_clock::now();

        // Drain pending frames; keep the newest speed frame. Bounded drain so
        // a babbling-idiot node cannot starve the cycle.
        bool got_frame = false;
        float decoded_kmh = 0.0f;
        for (int i = 0; i < 64; ++i) {
            auto frame = can_.read();
            if (!frame) break;
            if (frame->id != kSpeedCanId || frame->dlc < 2) continue;
            const uint16_t raw = static_cast<uint16_t>(frame->data[0]) |
                                 (static_cast<uint16_t>(frame->data[1]) << 8);
            decoded_kmh = static_cast<float>(raw) * kScale;
            got_frame = true;
        }

        if (got_frame && plausible(decoded_kmh)) {
            last_valid_kmh_ = decoded_kmh;
            last_valid_time_ = now;
            has_valid_ = true;
            quality_.store(SignalQuality::Valid);
        } else if (has_valid_ && (now - last_valid_time_) <= kSignalTimeout) {
            quality_.store(SignalQuality::Degraded);  // substitute last valid
        } else {
            quality_.store(has_valid_ ? SignalQuality::Degraded
                                      : SignalQuality::Invalid);
            if (has_valid_ && (now - last_valid_time_) > kSignalTimeout) {
                has_valid_ = false;
                quality_.store(SignalQuality::Invalid);
            }
        }

        publish(now);
    }

    SpeedSample latest() const {
        return SpeedSample{last_valid_kmh_, quality_.load(), 0};
    }

private:
    void runLoop() {
        while (running_.load()) {
            const auto t0 = std::chrono::steady_clock::now();
            cycle();
            std::this_thread::sleep_until(t0 + kCyclePeriod);
        }
    }

    bool plausible(float kmh) const {
        if (kmh < 0.0f || kmh > kMaxPlausibleKmh) return false;
        if (has_valid_ &&
            std::abs(kmh - last_valid_kmh_) > kMaxJumpKmhPerCycle) {
            return false;  // physically impossible acceleration -> reject
        }
        return true;
    }

    void publish(std::chrono::steady_clock::time_point now) {
        SpeedSample s;
        s.kmh = has_valid_ ? last_valid_kmh_ : 0.0f;
        s.quality = quality_.load();
        s.timestamp_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                now.time_since_epoch())
                .count());
        bus_.publish("vehicle.speed", s, s.timestamp_ns);
    }

    drivers::ICanInterface& can_;
    middleware::MessageBus& bus_;

    std::thread worker_;
    std::atomic<bool> running_{false};
    std::atomic<SignalQuality> quality_{SignalQuality::Invalid};

    float last_valid_kmh_{0.0f};
    bool has_valid_{false};
    std::chrono::steady_clock::time_point last_valid_time_{};
};

}  // namespace sdv::services
