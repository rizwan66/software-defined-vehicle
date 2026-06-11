// =============================================================================
// apps/adas/blind_spot.hpp
//
// Blind Spot Detection (BSD) + Lane Change Assist (LCA) function.
//
// Subscribes to:
//   bsd.left_sensor    (BsdSensorReading: object_detected, confidence, range_m)
//   bsd.right_sensor   (BsdSensorReading)
//   vehicle.speed      (SpeedSample)
//   driver.turn_signal (TurnSignalState: Left, Right, Off)
//
// Publishes:
//   bsd.warning        (BsdWarning: side, type, confidence)
//
// Warning types:
//   VISUAL_ONLY    — object in blind spot, no lane change imminent
//   AUDIBLE        — object in blind spot + driver signalled a lane change
//   HAPTIC         — object in blind spot + driver has started lane change (urgent)
//
// Safety / SOTIF:
//   * BSD deactivates below kMinSpeedKmh (not useful in slow traffic)
//   * Sensor confidence < kMinConfidence → no warning (false-positive avoidance)
//   * Warning hysteresis: object must be detected for kDebounceFrames consecutive
//     cycles before a warning is issued, and absent for kClearFrames before clearing.
//   * In production this maps to ISO 17387 Blind Spot Monitoring systems.
// =============================================================================
#pragma once

#include <atomic>
#include <cstdint>

#include "middleware/communication/message_bus.hpp"
#include "services/vehicle-signals/vehicle_speed_service.hpp"

namespace sdv::apps::adas {

// ─── Data types ──────────────────────────────────────────────────────────────

struct BsdSensorReading {
    bool  object_detected{false};
    float confidence{0.0f};       // [0,1]
    float range_m{0.0f};          // distance to detected object
    uint64_t timestamp_ns{0};
};

enum class TurnSignalState : uint8_t { Off, Left, Right };

enum class BsdSide : uint8_t { Left, Right };

enum class BsdWarningType : uint8_t {
    None,
    VisualOnly,   // object present, no lane change intent
    Audible,      // turn signal active
    Haptic        // lane change already underway (urgent)
};

struct BsdWarning {
    BsdSide        side{BsdSide::Left};
    BsdWarningType type{BsdWarningType::None};
    float          confidence{0.0f};
    float          range_m{0.0f};
    uint64_t       timestamp_ns{0};
};

// ─── BSD function ────────────────────────────────────────────────────────────

class BlindSpotDetection {
public:
    static constexpr float    kMinSpeedKmh    = 10.0f;
    static constexpr float    kMinConfidence  = 0.6f;
    static constexpr uint8_t  kDebounceFrames = 3;
    static constexpr uint8_t  kClearFrames    = 5;

    explicit BlindSpotDetection(middleware::MessageBus& bus) : bus_(bus) {
        speed_sub_ = bus_.subscribe(
            "vehicle.speed", [this](const middleware::Sample& s) {
                const auto sp = std::any_cast<services::SpeedSample>(s.payload);
                speed_kmh_.store(sp.kmh);
                speed_valid_.store(sp.quality == services::SignalQuality::Valid);
            });
        left_sub_ = bus_.subscribe(
            "bsd.left_sensor", [this](const middleware::Sample& s) {
                process(BsdSide::Left,
                        std::any_cast<BsdSensorReading>(s.payload),
                        s.timestamp_ns);
            });
        right_sub_ = bus_.subscribe(
            "bsd.right_sensor", [this](const middleware::Sample& s) {
                process(BsdSide::Right,
                        std::any_cast<BsdSensorReading>(s.payload),
                        s.timestamp_ns);
            });
        signal_sub_ = bus_.subscribe(
            "driver.turn_signal", [this](const middleware::Sample& s) {
                turn_signal_.store(std::any_cast<TurnSignalState>(s.payload));
            });
    }

    ~BlindSpotDetection() {
        bus_.unsubscribe("vehicle.speed",      speed_sub_);
        bus_.unsubscribe("bsd.left_sensor",    left_sub_);
        bus_.unsubscribe("bsd.right_sensor",   right_sub_);
        bus_.unsubscribe("driver.turn_signal", signal_sub_);
    }

    [[nodiscard]] bool isActive()    const noexcept { return speed_valid_.load() &&
                                                             speed_kmh_.load() >= kMinSpeedKmh; }
    [[nodiscard]] bool isSafeState() const noexcept { return !isActive(); }

    [[nodiscard]] BsdWarningType lastLeftWarning()  const noexcept { return left_warning_.load(); }
    [[nodiscard]] BsdWarningType lastRightWarning() const noexcept { return right_warning_.load(); }

private:
    struct SideState {
        uint8_t detect_count{0};
        uint8_t clear_count{0};
        bool    active{false};
    };

    void process(BsdSide side, const BsdSensorReading& reading, uint64_t ts_ns) {
        if (!isActive()) {
            left_warning_.store(BsdWarningType::None);
            right_warning_.store(BsdWarningType::None);
            return;
        }

        SideState& ss = (side == BsdSide::Left) ? left_state_ : right_state_;
        const TurnSignalState signal = turn_signal_.load();

        // ── Debounce ─────────────────────────────────────────────────
        if (reading.object_detected && reading.confidence >= kMinConfidence) {
            ss.clear_count = 0;
            if (ss.detect_count < kDebounceFrames) ++ss.detect_count;
        } else {
            ss.detect_count = 0;
            if (ss.clear_count < kClearFrames) ++ss.clear_count;
            else ss.active = false;
        }

        if (ss.detect_count >= kDebounceFrames) ss.active = true;

        // ── Warning level ─────────────────────────────────────────────
        BsdWarningType wtype = BsdWarningType::None;
        if (ss.active) {
            const bool signalling =
                (side == BsdSide::Left  && signal == TurnSignalState::Left) ||
                (side == BsdSide::Right && signal == TurnSignalState::Right);
            wtype = signalling ? BsdWarningType::Audible : BsdWarningType::VisualOnly;
        }

        if (side == BsdSide::Left)  left_warning_.store(wtype);
        else                         right_warning_.store(wtype);

        if (wtype != BsdWarningType::None) {
            bus_.publish("bsd.warning",
                         BsdWarning{side, wtype, reading.confidence, reading.range_m, ts_ns},
                         ts_ns);
        }
    }

    middleware::MessageBus& bus_;
    middleware::MessageBus::SubscriptionId speed_sub_{0};
    middleware::MessageBus::SubscriptionId left_sub_{0};
    middleware::MessageBus::SubscriptionId right_sub_{0};
    middleware::MessageBus::SubscriptionId signal_sub_{0};

    std::atomic<float>           speed_kmh_{0.0f};
    std::atomic<bool>            speed_valid_{false};
    std::atomic<TurnSignalState> turn_signal_{TurnSignalState::Off};
    std::atomic<BsdWarningType>  left_warning_{BsdWarningType::None};
    std::atomic<BsdWarningType>  right_warning_{BsdWarningType::None};

    SideState left_state_{};
    SideState right_state_{};
};

}  // namespace sdv::apps::adas
