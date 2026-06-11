// =============================================================================
// services/vehicle-signals/vehicle_speed_service.hpp
//
// Production-grade vehicle speed service with C++20 modernisation:
//   * std::jthread (cooperative stop_token cancellation) replaces std::thread
//   * KalmanFilter for CAN noise reduction
//   * Prometheus-style metrics (frames_received, plausibility_rejects, ...)
//   * Structured logging via sdv_log
//   * std::span for buffer-safe frame draining
//
// Signal pipeline:
//   CAN frame → decode → plausibility → Kalman filter → quality → publish
//
// Signal quality levels:
//   Valid    — fresh, plausible, filtered value
//   Degraded — last-valid substituted (timeout or implausible)
//   Invalid  — no valid value since startup
//
// Design invariants (interviewers look for these):
//   * Dependency injection of ICanInterface (testable, hardware-agnostic)
//   * No blocking calls inside cycle() (deterministic timing)
//   * Plausibility rejection BEFORE the Kalman filter (garbage-in → reject)
//   * std::jthread destructor requests cancellation automatically — no manual stop()
// =============================================================================
#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <span>
#include <thread>

#include "drivers/can/can_interface.hpp"
#include "middleware/communication/message_bus.hpp"
#include "middleware/logging/sdv_log.hpp"
#include "middleware/metrics/metrics_collector.hpp"
#include "services/vehicle-signals/kalman_filter.hpp"

namespace sdv::services {

enum class SignalQuality : uint8_t {
    Valid,        // fresh, plausible, Kalman-filtered value
    Degraded,     // last valid value substituted (timeout or implausible frame)
    Invalid       // no valid value available since startup
};

struct SpeedSample {
    // Original fields kept in original order for backward compatibility.
    float kmh{0.0f};
    SignalQuality quality{SignalQuality::Invalid};
    uint64_t timestamp_ns{0};
    // C++20 additions — new fields appended so existing aggregate inits still compile.
    float kmh_filtered{0.0f};        // Kalman-filtered estimate
    float filter_convergence{0.0f};  // 0 = cold start, 1 = converged
};

class VehicleSpeedService {
public:
    static constexpr uint32_t kSpeedCanId          = 0x123;
    static constexpr float    kScale               = 0.01f;        // km/h per bit
    static constexpr float    kMaxPlausibleKmh     = 400.0f;
    static constexpr float    kMaxJumpKmhPerCycle  = 15.0f;        // ~150 km/h/s @ 10 ms
    static constexpr std::chrono::milliseconds kSignalTimeout{200};
    static constexpr std::chrono::milliseconds kCyclePeriod{10};
    static constexpr int kMaxFramesDrained = 64;                    // babbling-idiot guard

    VehicleSpeedService(drivers::ICanInterface& can,
                        middleware::MessageBus& bus)
        : can_(can), bus_(bus),
          m_frames_(metrics::counter("sdv.speed.frames_received",
                                     "CAN frames received with speed ID")),
          m_rejects_(metrics::counter("sdv.speed.plausibility_rejects",
                                      "Frames rejected by plausibility check")),
          m_timeout_(metrics::counter("sdv.speed.timeouts",
                                      "Signal timeout events")),
          m_speed_gauge_(metrics::gauge("sdv.speed.kmh",
                                       "Current vehicle speed (km/h)"))
    {}

    // std::jthread destructor calls request_stop() + join() — no manual stop needed.
    ~VehicleSpeedService() = default;

    // Start the cyclic 10 ms task on a jthread.
    void start() {
        worker_ = std::jthread([this](std::stop_token tok) { runLoop(tok); });
    }

    // Cooperative cancellation: jthread destructor does this automatically,
    // but callers can also trigger it explicitly.
    void stop() { worker_.request_stop(); }

    // Single cycle — public for deterministic unit testing without threads.
    void cycle() {
        const auto now = std::chrono::steady_clock::now();

        bool got_frame = false;
        float decoded_kmh = 0.0f;

        // Drain up to kMaxFramesDrained frames, keep the last speed frame.
        for (int i = 0; i < kMaxFramesDrained; ++i) {
            auto frame = can_.read();
            if (!frame) break;
            if (frame->id != kSpeedCanId || frame->dlc < 2) continue;
            m_frames_.inc();
            const uint16_t raw = static_cast<uint16_t>(frame->data[0]) |
                                 (static_cast<uint16_t>(frame->data[1]) << 8);
            decoded_kmh = static_cast<float>(raw) * kScale;
            got_frame = true;
        }

        // ── Plausibility → Kalman → quality ──────────────────────────
        if (got_frame && plausible(decoded_kmh)) {
            // First valid frame: seed the filter to avoid cold-start transient.
            if (!has_valid_) kalman_.seed(decoded_kmh);
            last_valid_kmh_ = decoded_kmh;
            last_valid_time_ = now;
            has_valid_ = true;
            quality_.store(SignalQuality::Valid);
        } else {
            if (got_frame) m_rejects_.inc();

            const auto since_valid = now - last_valid_time_;
            if (has_valid_ && since_valid <= kSignalTimeout) {
                quality_.store(SignalQuality::Degraded);
            } else if (has_valid_) {
                m_timeout_.inc();
                has_valid_ = false;
                quality_.store(SignalQuality::Invalid);
                SDV_LOG_WARN("VehicleSpeedService", "speed signal timeout");
            } else {
                quality_.store(SignalQuality::Invalid);
            }
        }

        // Always run the Kalman filter — it coasts on the last estimate when
        // there is no new measurement (substitution mode).
        const float filtered = kalman_.update(has_valid_ ? last_valid_kmh_ : kalman_.estimate());
        m_speed_gauge_.set(filtered);

        publishSample(now, filtered);
    }

    [[nodiscard]] SpeedSample latest() const {
        return SpeedSample{
            last_valid_kmh_,
            quality_.load(),
            0,
            kalman_.estimate(),
            kalman_.convergence()
        };
    }

private:
    void runLoop(std::stop_token tok) {
        SDV_LOG_INFO("VehicleSpeedService", "cyclic task started (period %d ms)",
                     static_cast<int>(kCyclePeriod.count()));
        while (!tok.stop_requested()) {
            const auto t0 = std::chrono::steady_clock::now();
            cycle();
            std::this_thread::sleep_until(t0 + kCyclePeriod);
        }
        SDV_LOG_INFO("VehicleSpeedService", "cyclic task stopped");
    }

    [[nodiscard]] bool plausible(float kmh) const noexcept {
        if (kmh < 0.0f || kmh > kMaxPlausibleKmh) return false;
        if (has_valid_ && std::abs(kmh - last_valid_kmh_) > kMaxJumpKmhPerCycle)
            return false;
        return true;
    }

    void publishSample(std::chrono::steady_clock::time_point now, float filtered) {
        const uint64_t ts_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                now.time_since_epoch()).count());
        SpeedSample s{
            has_valid_ ? last_valid_kmh_ : 0.0f,
            quality_.load(),
            ts_ns,
            filtered,
            kalman_.convergence()
        };
        bus_.publish("vehicle.speed", s, ts_ns);
    }

    drivers::ICanInterface& can_;
    middleware::MessageBus& bus_;

    // Metrics references (obtained once at construction, never copied).
    metrics::Counter& m_frames_;
    metrics::Counter& m_rejects_;
    metrics::Counter& m_timeout_;
    metrics::Gauge&   m_speed_gauge_;

    std::jthread worker_;
    std::atomic<SignalQuality> quality_{SignalQuality::Invalid};
    KalmanFilter kalman_{KalmanFilter::Params{1.0f, 4.0f, 100.0f}};

    float last_valid_kmh_{0.0f};
    bool  has_valid_{false};
    std::chrono::steady_clock::time_point last_valid_time_{};
};

}  // namespace sdv::services
