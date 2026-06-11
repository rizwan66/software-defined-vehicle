// =============================================================================
// services/vehicle-signals/kalman_filter.hpp
//
// Generic scalar Kalman filter for vehicle signal noise reduction.
//
// The discrete-time scalar Kalman filter is the optimal linear estimator for
// a process corrupted by additive Gaussian noise.  Applied to vehicle speed
// from a CAN bus it smooths out ADC quantisation noise, CAN retransmissions,
// and minor sensor jitter without introducing the fixed lag of a moving average.
//
// State model (constant-velocity):
//   x[k] = x[k-1] + w[k]       (process: speed changes slowly)
//   z[k] = x[k]   + v[k]       (measurement: CAN decoded speed)
//   Q = process noise variance  (tune: how fast can speed change legitimately?)
//   R = measurement noise var.  (tune: how noisy is the CAN signal?)
//
// Equations:
//   Predict:  x_pred = x_prev
//             P_pred = P_prev + Q
//   Update:   K = P_pred / (P_pred + R)          (Kalman gain)
//             x = x_pred + K * (z - x_pred)
//             P = (1 - K) * P_pred
//
// API is intentionally trivial so it composes easily with the speed service.
// =============================================================================
#pragma once

#include <cstdint>

namespace sdv::services {

class KalmanFilter {
public:
    struct Params {
        // Q — how much the true speed can change between cycles.
        // Small Q → filter trusts the model more (lag on real changes).
        // Large Q → filter trusts measurements more (less filtering).
        float process_noise_q{1.0f};        // km/h² per cycle

        // R — measurement noise variance.  Increase for noisier CAN links.
        float measurement_noise_r{4.0f};    // km/h²

        // Initial estimate error covariance.
        float initial_p{100.0f};
    };

    // Default constructor — uses sensible vehicle-speed defaults.
    KalmanFilter() noexcept : params_(), p_(params_.initial_p) {}

    explicit KalmanFilter(Params p) noexcept
        : params_(p), p_(p.initial_p) {}

    // Feed one measurement, return the filtered estimate.
    // Calling convention: invoke every cycle, even when substituting
    // last-valid (pass the same value; the filter will coast).
    [[nodiscard]] float update(float measurement) noexcept {
        // ── Predict ──────────────────────────────────────────────────
        const float p_pred = p_ + params_.process_noise_q;

        // ── Update ───────────────────────────────────────────────────
        const float k = p_pred / (p_pred + params_.measurement_noise_r);
        x_ = x_ + k * (measurement - x_);
        p_ = (1.0f - k) * p_pred;

        return x_;
    }

    // Seed the filter with a known-good starting value (e.g., first valid
    // CAN frame).  Avoids the cold-start transient where the filter has to
    // pull in from 0.
    void seed(float initial_value) noexcept {
        x_ = initial_value;
        p_ = params_.initial_p;
    }

    [[nodiscard]] float estimate()      const noexcept { return x_; }
    [[nodiscard]] float errorCovariance() const noexcept { return p_; }

    // How well the filter has converged: 0 = cold, 1 = fully converged.
    [[nodiscard]] float convergence() const noexcept {
        const float ideal_p = params_.process_noise_q * params_.measurement_noise_r /
                              (params_.process_noise_q + params_.measurement_noise_r);
        return (ideal_p > 0.0f)
            ? (1.0f - (p_ - ideal_p) / (params_.initial_p - ideal_p + 1e-6f))
            : 1.0f;
    }

private:
    Params params_;
    float x_{0.0f};   // state estimate (km/h)
    float p_;          // estimate error covariance
};

}  // namespace sdv::services
