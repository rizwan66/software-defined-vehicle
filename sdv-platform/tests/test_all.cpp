// =============================================================================
// tests/test_all.cpp
//
// Comprehensive test suite for the SDV platform.
// Organised into 9 sections:
//   1.  Cryptography (SHA-256, HMAC, constant-time compare)
//   2.  CAN interface (Mock + SocketCAN non-Linux stub)
//   3.  MessageBus (pub/sub, threading behaviour)
//   4.  VehicleSpeedService (decode, plausibility, quality state machine)
//   5.  AEB function (TTC decisions, safe-state degradation)
//   6.  DTC store (debounce, status bits, readback)
//   7.  OTA manager (full pipeline, rejection paths, state machine)
//   8.  HealthMonitor integration (signal → DTC → health gate)
//   9.  End-to-end integration (CAN → MessageBus → AEB, OTA gate)
//
// Runner: dependency-free assert-based runner.  In production this would be
// GoogleTest / Catch2 wired into CI.
// =============================================================================

#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <thread>

#include "apps/adas/aeb_function.hpp"
#include "apps/diagnostics/dtc_store.hpp"
#include "drivers/can/can_interface.hpp"
#include "middleware/communication/message_bus.hpp"
#include "security/crypto/sha256.hpp"
#include "services/ota/ota_manager.hpp"
#include "services/vehicle-health/health_monitor.hpp"
#include "services/vehicle-signals/vehicle_speed_service.hpp"

namespace fs = std::filesystem;
using namespace sdv;
using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Test runner
// ---------------------------------------------------------------------------
static int g_failures = 0;
static int g_total    = 0;

#define CHECK(cond)                                                           \
    do {                                                                      \
        ++g_total;                                                            \
        if (!(cond)) {                                                        \
            ++g_failures;                                                     \
            std::printf("  FAIL  %s:%d  %s\n", __FILE__, __LINE__, #cond);   \
        }                                                                     \
    } while (0)

// Convenience: reset the singleton bus before every test that uses it.
static void resetBus() { middleware::MessageBus::instance().reset(); }

// =============================================================================
// SECTION 1 — Cryptography
// =============================================================================

static void test_sha256_known_vectors() {
    std::printf("test_sha256_known_vectors\n");
    // FIPS 180-4 test vector: sha256("abc")
    std::vector<uint8_t> abc{'a', 'b', 'c'};
    CHECK(security::Sha256::toHex(security::Sha256::digest(abc)) ==
          "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    // sha256("")
    CHECK(security::Sha256::toHex(security::Sha256::digest({})) ==
          "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
}

static void test_sha256_streaming_equals_oneshot() {
    std::printf("test_sha256_streaming_equals_oneshot\n");
    // Feeding data in multiple chunks must give the same digest as one call.
    const std::string msg = "The quick brown fox jumps over the lazy dog";
    std::vector<uint8_t> full(msg.begin(), msg.end());

    const auto oneshot = security::Sha256::digest(full);

    security::Sha256 s;
    s.update(reinterpret_cast<const uint8_t*>(msg.data()), 10);
    s.update(reinterpret_cast<const uint8_t*>(msg.data()) + 10, msg.size() - 10);
    const auto streamed = s.finish();

    CHECK(oneshot == streamed);
}

static void test_sha256_single_byte_change_avalanche() {
    std::printf("test_sha256_single_byte_change_avalanche\n");
    std::vector<uint8_t> a = {'h', 'e', 'l', 'l', 'o'};
    std::vector<uint8_t> b = {'h', 'e', 'l', 'l', 'p'};  // one byte differs
    CHECK(security::Sha256::digest(a) != security::Sha256::digest(b));
}

static void test_hmac_deterministic() {
    std::printf("test_hmac_deterministic\n");
    std::vector<uint8_t> key{'k', 'e', 'y'};
    std::vector<uint8_t> msg{'m', 's', 'g'};
    CHECK(security::hmacSha256(key, msg) == security::hmacSha256(key, msg));
}

static void test_hmac_key_sensitivity() {
    std::printf("test_hmac_key_sensitivity\n");
    std::vector<uint8_t> key1{'k', '1'};
    std::vector<uint8_t> key2{'k', '2'};
    std::vector<uint8_t> msg{'m', 's', 'g'};
    // Different keys must produce different MACs.
    CHECK(security::hmacSha256(key1, msg) != security::hmacSha256(key2, msg));
}

static void test_hmac_message_sensitivity() {
    std::printf("test_hmac_message_sensitivity\n");
    std::vector<uint8_t> key{'k', 'e', 'y'};
    std::vector<uint8_t> msg1{'h', 'e', 'l', 'l', 'o'};
    std::vector<uint8_t> msg2{'h', 'e', 'l', 'l', 'p'};
    CHECK(security::hmacSha256(key, msg1) != security::hmacSha256(key, msg2));
}

static void test_hmac_long_key_hashed() {
    std::printf("test_hmac_long_key_hashed\n");
    // A key longer than 64 bytes should be hashed internally — must not crash
    // and must still be deterministic.
    std::vector<uint8_t> long_key(100, 0xAB);
    std::vector<uint8_t> msg{'d', 'a', 't', 'a'};
    const auto mac1 = security::hmacSha256(long_key, msg);
    const auto mac2 = security::hmacSha256(long_key, msg);
    CHECK(mac1 == mac2);
}

static void test_hmac_empty_message() {
    std::printf("test_hmac_empty_message\n");
    std::vector<uint8_t> key{'k'};
    std::vector<uint8_t> empty{};
    const auto mac = security::hmacSha256(key, empty);
    // Must produce a 32-byte result and be deterministic.
    CHECK(mac == security::hmacSha256(key, empty));
}

static void test_constant_time_equal_same() {
    std::printf("test_constant_time_equal_same\n");
    std::array<uint8_t, 32> a{}, b{};
    for (size_t i = 0; i < 32; ++i) a[i] = b[i] = static_cast<uint8_t>(i);
    CHECK(security::constantTimeEqual(a, b));
}

static void test_constant_time_equal_differs_by_one_bit() {
    std::printf("test_constant_time_equal_differs_by_one_bit\n");
    std::array<uint8_t, 32> a{}, b{};
    for (size_t i = 0; i < 32; ++i) a[i] = b[i] = static_cast<uint8_t>(i);
    b[31] ^= 0x01;  // flip last bit
    CHECK(!security::constantTimeEqual(a, b));
}

static void test_constant_time_equal_all_zeros_vs_one() {
    std::printf("test_constant_time_equal_all_zeros_vs_one\n");
    std::array<uint8_t, 32> a{};
    std::array<uint8_t, 32> b{};
    b[0] = 1;
    CHECK(!security::constantTimeEqual(a, b));
}

// =============================================================================
// SECTION 2 — CAN Interface
// =============================================================================

static void test_mock_can_alternates_frames() {
    std::printf("test_mock_can_alternates_frames\n");
    drivers::MockCanInterface can;
    can.setSpeedKmh(80.0f);
    can.setObstacleDistanceM(50.0f);

    // Tick 0 → speed frame (0x123), tick 1 → obstacle frame (0x250)
    auto f0 = can.read();
    auto f1 = can.read();
    CHECK(f0.has_value());
    CHECK(f1.has_value());
    CHECK(f0->id == 0x123);
    CHECK(f1->id == 0x250);
}

static void test_mock_can_speed_encoding_accuracy() {
    std::printf("test_mock_can_speed_encoding_accuracy\n");
    drivers::MockCanInterface can;
    can.setSpeedKmh(100.0f);

    // Read until we get the speed frame.
    drivers::CanFrame speed_frame{};
    for (int i = 0; i < 4; ++i) {
        auto f = can.read();
        if (f && f->id == 0x123) { speed_frame = *f; break; }
    }
    CHECK(speed_frame.id == 0x123);
    CHECK(speed_frame.dlc == 2);

    // 100.0 km/h / 0.01 scale = 10000 = 0x2710
    const uint16_t raw = static_cast<uint16_t>(speed_frame.data[0]) |
                         (static_cast<uint16_t>(speed_frame.data[1]) << 8);
    const float decoded = static_cast<float>(raw) * 0.01f;
    CHECK(decoded > 99.9f && decoded < 100.1f);
}

static void test_mock_can_zero_speed() {
    std::printf("test_mock_can_zero_speed\n");
    drivers::MockCanInterface can;
    can.setSpeedKmh(0.0f);

    drivers::CanFrame speed_frame{};
    for (int i = 0; i < 4; ++i) {
        auto f = can.read();
        if (f && f->id == 0x123) { speed_frame = *f; break; }
    }
    CHECK(speed_frame.id == 0x123);
    const uint16_t raw = static_cast<uint16_t>(speed_frame.data[0]) |
                         (static_cast<uint16_t>(speed_frame.data[1]) << 8);
    CHECK(raw == 0);
}

static void test_mock_can_obstacle_encoding_accuracy() {
    std::printf("test_mock_can_obstacle_encoding_accuracy\n");
    drivers::MockCanInterface can;
    can.setObstacleDistanceM(25.5f);

    drivers::CanFrame obs_frame{};
    for (int i = 0; i < 4; ++i) {
        auto f = can.read();
        if (f && f->id == 0x250) { obs_frame = *f; break; }
    }
    CHECK(obs_frame.id == 0x250);
    const uint16_t raw = static_cast<uint16_t>(obs_frame.data[0]) |
                         (static_cast<uint16_t>(obs_frame.data[1]) << 8);
    const float decoded = static_cast<float>(raw) * 0.01f;
    CHECK(decoded > 25.4f && decoded < 25.6f);
}

static void test_mock_can_always_open_and_write_ok() {
    std::printf("test_mock_can_always_open_and_write_ok\n");
    drivers::MockCanInterface can;
    CHECK(can.isOpen());
    drivers::CanFrame f{};
    CHECK(can.write(f));  // mock write is a no-op but returns true
}

static void test_socket_can_non_linux_stub() {
    std::printf("test_socket_can_non_linux_stub\n");
    // On non-Linux builds (macOS) the socket is never opened.
    drivers::SocketCanInterface can("nonexistent0");
#ifndef __linux__
    CHECK(!can.isOpen());
    CHECK(!can.read().has_value());
    CHECK(!can.write({}));
#else
    // On Linux, a non-existent interface returns fd < 0.
    CHECK(!can.isOpen());
    CHECK(!can.read().has_value());
    CHECK(!can.write({}));
#endif
}

static void test_can_frame_timestamp_is_set() {
    std::printf("test_can_frame_timestamp_is_set\n");
    drivers::MockCanInterface can;
    auto f = can.read();
    CHECK(f.has_value());
    CHECK(f->timestamp_ns > 0);
}

// =============================================================================
// SECTION 3 — MessageBus
// =============================================================================

static void test_message_bus_pub_sub() {
    std::printf("test_message_bus_pub_sub\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();

    int received = 0;
    auto id = bus.subscribe("t", [&](const middleware::Sample& s) {
        received += std::any_cast<int>(s.payload);
    });
    bus.publish("t", 21, 0);
    bus.publish("t", 21, 0);
    CHECK(received == 42);

    bus.unsubscribe("t", id);
    bus.publish("t", 100, 0);
    CHECK(received == 42);  // no change after unsubscribe
}

static void test_message_bus_multiple_subscribers_all_receive() {
    std::printf("test_message_bus_multiple_subscribers_all_receive\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();

    int a = 0, b = 0, c = 0;
    bus.subscribe("x", [&](const middleware::Sample&) { ++a; });
    bus.subscribe("x", [&](const middleware::Sample&) { ++b; });
    bus.subscribe("x", [&](const middleware::Sample&) { ++c; });

    bus.publish("x", 1, 0);
    CHECK(a == 1);
    CHECK(b == 1);
    CHECK(c == 1);
}

static void test_message_bus_unsubscribe_one_of_two() {
    std::printf("test_message_bus_unsubscribe_one_of_two\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();

    int a = 0, b = 0;
    auto id_a = bus.subscribe("q", [&](const middleware::Sample&) { ++a; });
    bus.subscribe("q", [&](const middleware::Sample&) { ++b; });

    bus.publish("q", 1, 0);
    CHECK(a == 1 && b == 1);

    bus.unsubscribe("q", id_a);
    bus.publish("q", 1, 0);
    CHECK(a == 1);  // a unsubscribed — no second increment
    CHECK(b == 2);  // b still active
}

static void test_message_bus_no_subscribers_no_crash() {
    std::printf("test_message_bus_no_subscribers_no_crash\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();
    // Must not crash when publishing to a topic with no subscribers.
    bus.publish("empty-topic", 42, 0);
    CHECK(true);
}

static void test_message_bus_topic_isolation() {
    std::printf("test_message_bus_topic_isolation\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();

    int a_count = 0, b_count = 0;
    bus.subscribe("topic.a", [&](const middleware::Sample&) { ++a_count; });
    bus.subscribe("topic.b", [&](const middleware::Sample&) { ++b_count; });

    bus.publish("topic.a", 1, 0);
    CHECK(a_count == 1);
    CHECK(b_count == 0);  // topic.b subscriber must not fire

    bus.publish("topic.b", 1, 0);
    CHECK(a_count == 1);
    CHECK(b_count == 1);
}

static void test_message_bus_unsubscribe_invalid_id_no_crash() {
    std::printf("test_message_bus_unsubscribe_invalid_id_no_crash\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();
    bus.unsubscribe("nonexistent-topic", 9999);
    CHECK(true);
}

static void test_message_bus_reset_clears_all_subscriptions() {
    std::printf("test_message_bus_reset_clears_all_subscriptions\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();

    int count = 0;
    bus.subscribe("r", [&](const middleware::Sample&) { ++count; });
    bus.publish("r", 1, 0);
    CHECK(count == 1);

    bus.reset();
    bus.publish("r", 1, 0);
    CHECK(count == 1);  // subscription was cleared by reset()
}

static void test_message_bus_timestamp_forwarded() {
    std::printf("test_message_bus_timestamp_forwarded\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();

    uint64_t received_ts = 0;
    bus.subscribe("ts-test", [&](const middleware::Sample& s) {
        received_ts = s.timestamp_ns;
    });
    bus.publish("ts-test", 0, 0xDEADBEEFCAFEBABEULL);
    CHECK(received_ts == 0xDEADBEEFCAFEBABEULL);
}

// =============================================================================
// SECTION 4 — VehicleSpeedService
// =============================================================================

static void test_speed_service_decode_and_plausibility() {
    std::printf("test_speed_service_decode_and_plausibility\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();

    drivers::MockCanInterface can;
    services::VehicleSpeedService svc(can, bus);

    can.setSpeedKmh(50.0f);
    svc.cycle();
    svc.cycle();  // mock alternates; two cycles guarantee a speed frame
    CHECK(svc.latest().quality == services::SignalQuality::Valid);
    CHECK(svc.latest().kmh > 49.9f && svc.latest().kmh < 50.1f);

    // Implausible jump (50 → 300 km/h in one 10 ms cycle) must be rejected.
    can.setSpeedKmh(300.0f);
    svc.cycle();
    svc.cycle();
    CHECK(svc.latest().kmh < 51.0f);  // last valid value retained
}

static void test_speed_service_starts_invalid() {
    std::printf("test_speed_service_starts_invalid\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();
    drivers::MockCanInterface can;
    services::VehicleSpeedService svc(can, bus);
    // Before any cycle: no valid value has ever been seen.
    CHECK(svc.latest().quality == services::SignalQuality::Invalid);
}

static void test_speed_service_zero_speed_is_valid() {
    std::printf("test_speed_service_zero_speed_is_valid\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();
    drivers::MockCanInterface can;
    services::VehicleSpeedService svc(can, bus);

    can.setSpeedKmh(0.0f);
    svc.cycle();
    svc.cycle();
    CHECK(svc.latest().quality == services::SignalQuality::Valid);
    CHECK(svc.latest().kmh < 0.5f);
}

static void test_speed_service_boundary_400_kmh_valid() {
    std::printf("test_speed_service_boundary_400_kmh_valid\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();
    drivers::MockCanInterface can;
    services::VehicleSpeedService svc(can, bus);

    // 400 km/h is at the range limit — must be accepted.
    can.setSpeedKmh(400.0f);
    svc.cycle();
    svc.cycle();
    CHECK(svc.latest().quality == services::SignalQuality::Valid);
    CHECK(svc.latest().kmh > 399.0f);
}

static void test_speed_service_above_400_kmh_rejected() {
    std::printf("test_speed_service_above_400_kmh_rejected\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();
    drivers::MockCanInterface can;
    services::VehicleSpeedService svc(can, bus);

    can.setSpeedKmh(401.0f);
    svc.cycle();
    svc.cycle();
    // No valid frame ever received — quality stays Invalid.
    CHECK(svc.latest().quality == services::SignalQuality::Invalid);
}

static void test_speed_service_wrong_can_id_ignored() {
    std::printf("test_speed_service_wrong_can_id_ignored\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();
    drivers::MockCanInterface can;
    services::VehicleSpeedService svc(can, bus);

    // Set CAN to obstacle frames only: we read obstacle (0x250) frames which
    // the speed service ignores.  The mock still alternates, but we can verify
    // quality stays Invalid if we set an implausible speed so the speed frames
    // are rejected and distance frames are irrelevant.
    can.setSpeedKmh(9999.0f);        // all speed frames will be out-of-range
    for (int i = 0; i < 10; ++i) svc.cycle();
    CHECK(svc.latest().quality != services::SignalQuality::Valid);
}

static void test_speed_service_signal_recovery_after_implausible() {
    std::printf("test_speed_service_signal_recovery_after_implausible\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();
    drivers::MockCanInterface can;
    services::VehicleSpeedService svc(can, bus);

    // Establish a valid baseline.
    can.setSpeedKmh(60.0f);
    svc.cycle(); svc.cycle();
    CHECK(svc.latest().quality == services::SignalQuality::Valid);

    // Implausible spike: quality must degrade.
    can.setSpeedKmh(500.0f);
    svc.cycle(); svc.cycle();
    CHECK(svc.latest().quality != services::SignalQuality::Valid);

    // Return to a plausible value close to the last valid (60 km/h → 70 km/h,
    // delta = 10 km/h < 15 km/h threshold) — quality should recover to Valid.
    can.setSpeedKmh(70.0f);
    svc.cycle(); svc.cycle();
    CHECK(svc.latest().quality == services::SignalQuality::Valid);
    CHECK(svc.latest().kmh > 69.0f && svc.latest().kmh < 71.0f);
}

static void test_speed_service_signal_timeout_to_invalid() {
    std::printf("test_speed_service_signal_timeout_to_invalid\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();
    drivers::MockCanInterface can;
    services::VehicleSpeedService svc(can, bus);

    // Establish a valid sample.
    can.setSpeedKmh(50.0f);
    svc.cycle(); svc.cycle();
    CHECK(svc.latest().quality == services::SignalQuality::Valid);

    // After > 200 ms with only implausible frames the signal should expire.
    can.setSpeedKmh(9999.0f);  // all speed frames out-of-range
    std::this_thread::sleep_for(220ms);
    svc.cycle(); svc.cycle();
    CHECK(svc.latest().quality == services::SignalQuality::Invalid);
}

static void test_speed_service_publishes_on_bus() {
    std::printf("test_speed_service_publishes_on_bus\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();
    drivers::MockCanInterface can;
    services::VehicleSpeedService svc(can, bus);

    int publish_count = 0;
    bus.subscribe("vehicle.speed", [&](const middleware::Sample&) {
        ++publish_count;
    });

    can.setSpeedKmh(80.0f);
    svc.cycle();
    CHECK(publish_count == 1);
    svc.cycle();
    CHECK(publish_count == 2);
}

static void test_speed_service_published_payload_matches_latest() {
    std::printf("test_speed_service_published_payload_matches_latest\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();
    drivers::MockCanInterface can;
    services::VehicleSpeedService svc(can, bus);

    services::SpeedSample last{};
    bus.subscribe("vehicle.speed", [&](const middleware::Sample& s) {
        last = std::any_cast<services::SpeedSample>(s.payload);
    });

    can.setSpeedKmh(123.45f);
    svc.cycle(); svc.cycle();
    CHECK(last.kmh > 123.0f && last.kmh < 124.0f);
    CHECK(last.quality == services::SignalQuality::Valid);
}

// =============================================================================
// SECTION 5 — AEB Function
// =============================================================================

static void test_aeb_ttc_decisions() {
    std::printf("test_aeb_ttc_decisions\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();
    apps::adas::AebFunction aeb(bus);

    // 72 km/h = 20 m/s. Obstacle at 100 m → TTC 5 s → no braking.
    services::SpeedSample sp{72.0f, services::SignalQuality::Valid, 1};
    bus.publish("vehicle.speed", sp, 1);
    bus.publish("obstacle.distance", 100.0f, 2);
    CHECK(aeb.lastDecision() == apps::adas::BrakeLevel::None);

    // 40 m → TTC 2.0 s → partial braking.
    bus.publish("obstacle.distance", 40.0f, 3);
    CHECK(aeb.lastDecision() == apps::adas::BrakeLevel::Partial);

    // 20 m → TTC 1.0 s → full braking.
    bus.publish("obstacle.distance", 20.0f, 4);
    CHECK(aeb.lastDecision() == apps::adas::BrakeLevel::Full);

    // Invalid speed quality → AEB must degrade to safe state.
    services::SpeedSample bad{72.0f, services::SignalQuality::Degraded, 5};
    bus.publish("vehicle.speed", bad, 5);
    CHECK(aeb.lastDecision() == apps::adas::BrakeLevel::None);
}

static void test_aeb_below_minimum_speed_inactive() {
    std::printf("test_aeb_below_minimum_speed_inactive\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();
    apps::adas::AebFunction aeb(bus);

    // 5 km/h is below the 10 km/h activation threshold.
    // Even with a very close obstacle the AEB must not activate.
    services::SpeedSample slow{5.0f, services::SignalQuality::Valid, 1};
    bus.publish("vehicle.speed", slow, 1);
    bus.publish("obstacle.distance", 2.0f, 2);
    CHECK(aeb.lastDecision() == apps::adas::BrakeLevel::None);
}

static void test_aeb_exactly_minimum_speed_activates() {
    std::printf("test_aeb_exactly_minimum_speed_activates\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();
    apps::adas::AebFunction aeb(bus);

    // 10 km/h = exactly at threshold; with close obstacle → Full.
    // 10 km/h = 2.778 m/s. Obstacle at 2 m → TTC ≈ 0.72 s → Full.
    services::SpeedSample sp{10.0f, services::SignalQuality::Valid, 1};
    bus.publish("vehicle.speed", sp, 1);
    bus.publish("obstacle.distance", 2.0f, 2);
    CHECK(aeb.lastDecision() == apps::adas::BrakeLevel::Full);
}

static void test_aeb_full_brake_just_below_15s_threshold() {
    std::printf("test_aeb_full_brake_just_below_15s_threshold\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();
    apps::adas::AebFunction aeb(bus);

    // 36 km/h = 10 m/s. Obstacle at 14 m → TTC 1.4 s < 1.5 → Full.
    services::SpeedSample sp{36.0f, services::SignalQuality::Valid, 1};
    bus.publish("vehicle.speed", sp, 1);
    bus.publish("obstacle.distance", 14.0f, 2);
    CHECK(aeb.lastDecision() == apps::adas::BrakeLevel::Full);
}

static void test_aeb_partial_brake_at_exactly_15s_boundary() {
    std::printf("test_aeb_partial_brake_at_exactly_15s_boundary\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();
    apps::adas::AebFunction aeb(bus);

    // 36 km/h = 10 m/s. Obstacle at 15 m → TTC 1.5 s.
    // 1.5 < 1.5 is false; 1.5 < 2.5 is true → Partial (not Full).
    services::SpeedSample sp{36.0f, services::SignalQuality::Valid, 1};
    bus.publish("vehicle.speed", sp, 1);
    bus.publish("obstacle.distance", 15.0f, 2);
    CHECK(aeb.lastDecision() == apps::adas::BrakeLevel::Partial);
}

static void test_aeb_none_at_exactly_25s_boundary() {
    std::printf("test_aeb_none_at_exactly_25s_boundary\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();
    apps::adas::AebFunction aeb(bus);

    // 36 km/h = 10 m/s. Obstacle at 25 m → TTC 2.5 s.
    // 2.5 < 2.5 is false → None.
    services::SpeedSample sp{36.0f, services::SignalQuality::Valid, 1};
    bus.publish("vehicle.speed", sp, 1);
    bus.publish("obstacle.distance", 25.0f, 2);
    CHECK(aeb.lastDecision() == apps::adas::BrakeLevel::None);
}

static void test_aeb_zero_distance_safe_state() {
    std::printf("test_aeb_zero_distance_safe_state\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();
    apps::adas::AebFunction aeb(bus);

    // dist = 0 fails the `dist > 0.0f` guard → no actuation (sensor error path).
    services::SpeedSample sp{72.0f, services::SignalQuality::Valid, 1};
    bus.publish("vehicle.speed", sp, 1);
    bus.publish("obstacle.distance", 0.0f, 2);
    CHECK(aeb.lastDecision() == apps::adas::BrakeLevel::None);
}

static void test_aeb_very_close_obstacle_full_brake() {
    std::printf("test_aeb_very_close_obstacle_full_brake\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();
    apps::adas::AebFunction aeb(bus);

    // 72 km/h = 20 m/s. Obstacle at 0.1 m → TTC 0.005 s → Full.
    services::SpeedSample sp{72.0f, services::SignalQuality::Valid, 1};
    bus.publish("vehicle.speed", sp, 1);
    bus.publish("obstacle.distance", 0.1f, 2);
    CHECK(aeb.lastDecision() == apps::adas::BrakeLevel::Full);
}

static void test_aeb_invalid_quality_is_safe_state() {
    std::printf("test_aeb_invalid_quality_is_safe_state\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();
    apps::adas::AebFunction aeb(bus);

    // Publish a close obstacle first, then send invalid-quality speed.
    bus.publish("obstacle.distance", 5.0f, 1);
    services::SpeedSample inv{80.0f, services::SignalQuality::Invalid, 2};
    bus.publish("vehicle.speed", inv, 2);
    CHECK(aeb.lastDecision() == apps::adas::BrakeLevel::None);
}

static void test_aeb_no_duplicate_publish_when_level_unchanged() {
    std::printf("test_aeb_no_duplicate_publish_when_level_unchanged\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();
    apps::adas::AebFunction aeb(bus);

    int brake_events = 0;
    bus.subscribe("brake.request", [&](const middleware::Sample&) { ++brake_events; });

    // First publish: None → Partial (change → 1 event).
    services::SpeedSample sp{72.0f, services::SignalQuality::Valid, 1};
    bus.publish("vehicle.speed", sp, 1);
    bus.publish("obstacle.distance", 40.0f, 2);  // TTC 2.0 s → Partial
    CHECK(brake_events == 1);

    // Second publish with identical inputs: Partial → Partial (no change → 0 new events).
    bus.publish("obstacle.distance", 40.0f, 3);
    CHECK(brake_events == 1);
}

static void test_aeb_ttc_value_reported_accurately() {
    std::printf("test_aeb_ttc_value_reported_accurately\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();
    apps::adas::AebFunction aeb(bus);

    // 72 km/h = 20 m/s. Obstacle at 40 m → TTC = 2.0 s.
    services::SpeedSample sp{72.0f, services::SignalQuality::Valid, 1};
    bus.publish("vehicle.speed", sp, 1);
    bus.publish("obstacle.distance", 40.0f, 2);
    CHECK(aeb.lastTtc() > 1.99f && aeb.lastTtc() < 2.01f);
}

// =============================================================================
// SECTION 6 — DTC Store
// =============================================================================

static void test_dtc_debounce_and_clear() {
    std::printf("test_dtc_debounce_and_clear\n");
    apps::diagnostics::DtcStore store;
    store.reportFailed(0xC12301, "speed invalid");
    store.reportFailed(0xC12301, "speed invalid");
    CHECK(store.readConfirmed().empty());  // below debounce threshold
    store.reportFailed(0xC12301, "speed invalid");
    CHECK(store.readConfirmed().size() == 1);  // confirmed at 3rd occurrence
    store.clearAll();
    CHECK(store.readAll().empty());
}

static void test_dtc_multiple_independent_codes() {
    std::printf("test_dtc_multiple_independent_codes\n");
    apps::diagnostics::DtcStore store;

    for (int i = 0; i < 3; ++i) store.reportFailed(0xAABBCC, "fault A");
    for (int i = 0; i < 3; ++i) store.reportFailed(0x112233, "fault B");
    for (int i = 0; i < 1; ++i) store.reportFailed(0x998877, "fault C");  // unconfirmed

    CHECK(store.readAll().size() == 3);
    CHECK(store.readConfirmed().size() == 2);  // C not confirmed yet
}

static void test_dtc_report_passed_nonexistent_no_crash() {
    std::printf("test_dtc_report_passed_nonexistent_no_crash\n");
    apps::diagnostics::DtcStore store;
    store.reportPassed(0xDEADBEEF);  // code never reported failed
    CHECK(store.readAll().empty());
}

static void test_dtc_unconfirmed_not_in_readconfirmed() {
    std::printf("test_dtc_unconfirmed_not_in_readconfirmed\n");
    apps::diagnostics::DtcStore store;
    store.reportFailed(0x001122, "marginal");
    store.reportFailed(0x001122, "marginal");  // 2 occurrences < threshold 3
    CHECK(!store.readAll().empty());
    CHECK(store.readConfirmed().empty());
}

static void test_dtc_status_bits_after_reportpassed() {
    std::printf("test_dtc_status_bits_after_reportpassed\n");
    apps::diagnostics::DtcStore store;

    for (int i = 0; i < 3; ++i) store.reportFailed(0xABCDEF, "err");
    store.reportPassed(0xABCDEF);

    const auto all = store.readAll();
    CHECK(all.size() == 1);
    const auto& e = all[0];
    // testFailed bit must be cleared.
    CHECK((e.status & apps::diagnostics::DtcStore::kTestFailed) == 0);
    // confirmedDTC bit must still be set (history preserved until clearAll).
    CHECK((e.status & apps::diagnostics::DtcStore::kConfirmed) != 0);
    // failedSinceClear bit must still be set.
    CHECK((e.status & apps::diagnostics::DtcStore::kFailedSinceClear) != 0);
}

static void test_dtc_occurrence_count_increments() {
    std::printf("test_dtc_occurrence_count_increments\n");
    apps::diagnostics::DtcStore store;
    for (int i = 0; i < 5; ++i) store.reportFailed(0x555555, "multi");
    const auto all = store.readAll();
    CHECK(all.size() == 1);
    CHECK(all[0].occurrence_count == 5);
}

static void test_dtc_cleared_after_clearall() {
    std::printf("test_dtc_cleared_after_clearall\n");
    apps::diagnostics::DtcStore store;
    for (int i = 0; i < 3; ++i) store.reportFailed(0xAAAAAA, "err");
    store.clearAll();
    CHECK(store.readAll().empty());
    CHECK(store.readConfirmed().empty());
}

static void test_dtc_confirmed_threshold_is_exactly_three() {
    std::printf("test_dtc_confirmed_threshold_is_exactly_three\n");
    apps::diagnostics::DtcStore store;
    store.reportFailed(0x010101, "e");
    CHECK(store.readConfirmed().empty());     // 1 < 3
    store.reportFailed(0x010101, "e");
    CHECK(store.readConfirmed().empty());     // 2 < 3
    store.reportFailed(0x010101, "e");
    CHECK(store.readConfirmed().size() == 1); // 3 == threshold
}

// =============================================================================
// SECTION 7 — OTA Manager
// =============================================================================

// Shared OTA test fixture: builds a properly signed package.
struct OtaFixture {
    fs::path root = fs::temp_directory_path() / "sdv-ota-test";
    std::vector<uint8_t> key{'d', 'e', 'm', 'o', '-', 'k', 'e', 'y'};

    OtaFixture() {
        fs::remove_all(root);
        fs::create_directories(root / "incoming");
    }

    fs::path makePackage(uint64_t version, bool tamper_manifest = false,
                         bool corrupt_payload = false,
                         const std::string& hw = "ECU-GW-001") {
        const fs::path payload = root / "incoming" / "firmware.bin";
        {
            std::ofstream f(payload, std::ios::binary);
            f << "FIRMWARE-v" << version;
        }
        std::ifstream pf(payload, std::ios::binary);
        std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(pf)),
                                   std::istreambuf_iterator<char>());
        const std::string digest =
            security::Sha256::toHex(security::Sha256::digest(bytes));

        services::ota::UpdateManifest m;
        m.name = "gateway-fw";
        m.hardware_id = hw;
        m.version = version;
        m.payload_file = "firmware.bin";
        m.payload_sha256 = digest;
        m.signature = security::Sha256::toHex(
            security::hmacSha256(key, m.canonicalBytes()));

        if (tamper_manifest) m.version += 100;
        if (corrupt_payload) {
            std::ofstream c(payload, std::ios::binary | std::ios::app);
            c << "CORRUPT";
        }

        const fs::path mpath = root / "incoming" / "manifest.json";
        std::ofstream out(mpath);
        out << "{\n"
            << "  \"name\": \"" << m.name << "\",\n"
            << "  \"hardware_id\": \"" << m.hardware_id << "\",\n"
            << "  \"version\": " << m.version << ",\n"
            << "  \"payload_file\": \"" << m.payload_file << "\",\n"
            << "  \"payload_sha256\": \"" << m.payload_sha256 << "\",\n"
            << "  \"signature\": \"" << m.signature << "\"\n"
            << "}\n";
        return mpath;
    }
};

static void test_ota_happy_path_commit() {
    std::printf("test_ota_happy_path_commit\n");
    OtaFixture fx;
    services::ota::OtaManager ota(fx.root, "ECU-GW-001", fx.key);
    CHECK(ota.activeSlot() == "a");
    CHECK(ota.installedVersion() == 0);

    auto r = ota.startUpdate(fx.makePackage(1),
                             fx.root / "incoming" / "firmware.bin");
    CHECK(r.ok);
    CHECK(ota.state() == services::ota::OtaState::PendingCommit);
    CHECK(fs::exists(fx.root / "slot_b" / "firmware.bin"));

    auto c = ota.commit();
    CHECK(c.ok);
    CHECK(ota.activeSlot() == "b");
    CHECK(ota.installedVersion() == 1);
}

static void test_ota_rejects_tampered_manifest() {
    std::printf("test_ota_rejects_tampered_manifest\n");
    OtaFixture fx;
    services::ota::OtaManager ota(fx.root, "ECU-GW-001", fx.key);
    auto r = ota.startUpdate(fx.makePackage(1, /*tamper=*/true),
                             fx.root / "incoming" / "firmware.bin");
    CHECK(!r.ok);
    CHECK(r.message.find("signature") != std::string::npos);
    CHECK(!fs::exists(fx.root / "slot_b" / "firmware.bin"));
}

static void test_ota_rejects_corrupt_payload() {
    std::printf("test_ota_rejects_corrupt_payload\n");
    OtaFixture fx;
    services::ota::OtaManager ota(fx.root, "ECU-GW-001", fx.key);
    auto r = ota.startUpdate(
        fx.makePackage(1, false, /*corrupt=*/true),
        fx.root / "incoming" / "firmware.bin");
    CHECK(!r.ok);
    CHECK(r.message.find("digest") != std::string::npos);
}

static void test_ota_anti_rollback() {
    std::printf("test_ota_anti_rollback\n");
    OtaFixture fx;
    services::ota::OtaManager ota(fx.root, "ECU-GW-001", fx.key);
    CHECK(ota.startUpdate(fx.makePackage(5),
                          fx.root / "incoming" / "firmware.bin").ok);
    CHECK(ota.commit().ok);

    auto r = ota.startUpdate(fx.makePackage(4),
                             fx.root / "incoming" / "firmware.bin");
    CHECK(!r.ok);
    CHECK(r.message.find("anti-rollback") != std::string::npos);
}

static void test_ota_rejects_wrong_hardware() {
    std::printf("test_ota_rejects_wrong_hardware\n");
    OtaFixture fx;
    services::ota::OtaManager ota(fx.root, "ECU-OTHER-999", fx.key);
    auto r = ota.startUpdate(fx.makePackage(1),  // built for ECU-GW-001
                             fx.root / "incoming" / "firmware.bin");
    CHECK(!r.ok);
    CHECK(r.message.find("hardware") != std::string::npos);
}

static void test_ota_version_equal_to_installed_rejected() {
    std::printf("test_ota_version_equal_to_installed_rejected\n");
    OtaFixture fx;
    services::ota::OtaManager ota(fx.root, "ECU-GW-001", fx.key);
    CHECK(ota.startUpdate(fx.makePackage(3),
                          fx.root / "incoming" / "firmware.bin").ok);
    CHECK(ota.commit().ok);

    // Re-installing the same version (version == installed) must be refused.
    auto r = ota.startUpdate(fx.makePackage(3),
                             fx.root / "incoming" / "firmware.bin");
    CHECK(!r.ok);
    CHECK(r.message.find("anti-rollback") != std::string::npos);
}

static void test_ota_rollback_returns_to_idle_active_slot_unchanged() {
    std::printf("test_ota_rollback_returns_to_idle_active_slot_unchanged\n");
    OtaFixture fx;
    services::ota::OtaManager ota(fx.root, "ECU-GW-001", fx.key);

    auto r = ota.startUpdate(fx.makePackage(1),
                             fx.root / "incoming" / "firmware.bin");
    CHECK(r.ok);
    CHECK(ota.state() == services::ota::OtaState::PendingCommit);

    auto rb = ota.rollback();
    CHECK(rb.ok);
    CHECK(ota.state() == services::ota::OtaState::Idle);
    CHECK(ota.activeSlot() == "a");          // active slot must not have moved
    CHECK(ota.installedVersion() == 0);      // version counter unchanged
    CHECK(!fs::exists(fx.root / "slot_b" / "firmware.bin"));  // slot wiped
}

static void test_ota_slot_alternates_over_two_updates() {
    std::printf("test_ota_slot_alternates_over_two_updates\n");
    OtaFixture fx;
    services::ota::OtaManager ota(fx.root, "ECU-GW-001", fx.key);

    // First update: a → b
    CHECK(ota.startUpdate(fx.makePackage(1),
                          fx.root / "incoming" / "firmware.bin").ok);
    CHECK(ota.commit().ok);
    CHECK(ota.activeSlot() == "b");
    CHECK(ota.installedVersion() == 1);

    // Second update: b → a
    CHECK(ota.startUpdate(fx.makePackage(2),
                          fx.root / "incoming" / "firmware.bin").ok);
    CHECK(ota.commit().ok);
    CHECK(ota.activeSlot() == "a");
    CHECK(ota.installedVersion() == 2);
}

static void test_ota_double_startupdate_rejected() {
    std::printf("test_ota_double_startupdate_rejected\n");
    OtaFixture fx;
    services::ota::OtaManager ota(fx.root, "ECU-GW-001", fx.key);

    // Leave the OTA in PendingCommit state.
    CHECK(ota.startUpdate(fx.makePackage(1),
                          fx.root / "incoming" / "firmware.bin").ok);
    CHECK(ota.state() == services::ota::OtaState::PendingCommit);

    // A second startUpdate call while in PendingCommit must be rejected.
    auto r = ota.startUpdate(fx.makePackage(2),
                             fx.root / "incoming" / "firmware.bin");
    CHECK(!r.ok);
    // State must remain PendingCommit — the first staged update is preserved.
    CHECK(ota.state() == services::ota::OtaState::PendingCommit);
}

static void test_ota_commit_without_pending_returns_error() {
    std::printf("test_ota_commit_without_pending_returns_error\n");
    OtaFixture fx;
    services::ota::OtaManager ota(fx.root, "ECU-GW-001", fx.key);
    auto r = ota.commit();  // nothing staged
    CHECK(!r.ok);
    CHECK(ota.state() == services::ota::OtaState::Idle);
}

static void test_ota_rollback_without_pending_returns_error() {
    std::printf("test_ota_rollback_without_pending_returns_error\n");
    OtaFixture fx;
    services::ota::OtaManager ota(fx.root, "ECU-GW-001", fx.key);
    auto r = ota.rollback();  // nothing staged
    CHECK(!r.ok);
}

static void test_ota_missing_manifest_file_returns_error() {
    std::printf("test_ota_missing_manifest_file_returns_error\n");
    OtaFixture fx;
    services::ota::OtaManager ota(fx.root, "ECU-GW-001", fx.key);
    auto r = ota.startUpdate(fx.root / "incoming" / "does_not_exist.json",
                             fx.root / "incoming" / "firmware.bin");
    CHECK(!r.ok);
    CHECK(ota.state() == services::ota::OtaState::Idle);
}

static void test_ota_missing_payload_file_returns_error() {
    std::printf("test_ota_missing_payload_file_returns_error\n");
    OtaFixture fx;
    services::ota::OtaManager ota(fx.root, "ECU-GW-001", fx.key);
    // Create a valid manifest but point to a non-existent payload.
    const fs::path mpath = fx.makePackage(1);
    auto r = ota.startUpdate(mpath,
                             fx.root / "incoming" / "does_not_exist.bin");
    CHECK(!r.ok);
    CHECK(ota.state() == services::ota::OtaState::Idle);
}

static void test_ota_parse_manifest_valid_json() {
    std::printf("test_ota_parse_manifest_valid_json\n");
    const std::string json = R"({
        "name": "gw-fw",
        "hardware_id": "ECU-01",
        "version": 42,
        "payload_file": "fw.bin",
        "payload_sha256": "aabbcc",
        "signature": "ddeeff"
    })";
    auto m = services::ota::OtaManager::parseManifest(json);
    CHECK(m.has_value());
    CHECK(m->name == "gw-fw");
    CHECK(m->hardware_id == "ECU-01");
    CHECK(m->version == 42);
    CHECK(m->payload_file == "fw.bin");
}

static void test_ota_parse_manifest_empty_returns_nullopt() {
    std::printf("test_ota_parse_manifest_empty_returns_nullopt\n");
    CHECK(!services::ota::OtaManager::parseManifest("").has_value());
    CHECK(!services::ota::OtaManager::parseManifest("{}").has_value());
}

static void test_ota_parse_manifest_missing_field_returns_nullopt() {
    std::printf("test_ota_parse_manifest_missing_field_returns_nullopt\n");
    // All required fields present except "signature".
    const std::string json = R"({
        "name": "gw-fw",
        "hardware_id": "ECU-01",
        "version": 1,
        "payload_file": "fw.bin",
        "payload_sha256": "aabb"
    })";
    CHECK(!services::ota::OtaManager::parseManifest(json).has_value());
}

static void test_ota_installed_version_persists_across_instances() {
    std::printf("test_ota_installed_version_persists_across_instances\n");
    OtaFixture fx;
    {
        services::ota::OtaManager ota(fx.root, "ECU-GW-001", fx.key);
        CHECK(ota.startUpdate(fx.makePackage(7),
                              fx.root / "incoming" / "firmware.bin").ok);
        CHECK(ota.commit().ok);
    }
    // Create a new OtaManager instance pointing at the same root.
    // The persisted state files must be read correctly.
    services::ota::OtaManager ota2(fx.root, "ECU-GW-001", fx.key);
    CHECK(ota2.installedVersion() == 7);
    CHECK(ota2.activeSlot() == "b");
}

// =============================================================================
// SECTION 8 — HealthMonitor Integration
// =============================================================================

static void test_health_monitor_valid_signal_reports_healthy() {
    std::printf("test_health_monitor_valid_signal_reports_healthy\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();
    apps::diagnostics::DtcStore dtcs;
    services::HealthMonitor monitor(bus, dtcs);

    CHECK(!monitor.systemHealthy());  // no signal yet

    services::SpeedSample valid{80.0f, services::SignalQuality::Valid, 1};
    bus.publish("vehicle.speed", valid, 1);
    CHECK(monitor.systemHealthy());
}

static void test_health_monitor_invalid_signal_reports_unhealthy() {
    std::printf("test_health_monitor_invalid_signal_reports_unhealthy\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();
    apps::diagnostics::DtcStore dtcs;
    services::HealthMonitor monitor(bus, dtcs);

    // First make it healthy.
    bus.publish("vehicle.speed",
                services::SpeedSample{80.0f, services::SignalQuality::Valid, 1}, 1);
    CHECK(monitor.systemHealthy());

    // Degrade the signal.
    bus.publish("vehicle.speed",
                services::SpeedSample{0.0f, services::SignalQuality::Invalid, 2}, 2);
    CHECK(!monitor.systemHealthy());
}

static void test_health_monitor_recovery_from_invalid() {
    std::printf("test_health_monitor_recovery_from_invalid\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();
    apps::diagnostics::DtcStore dtcs;
    services::HealthMonitor monitor(bus, dtcs);

    bus.publish("vehicle.speed",
                services::SpeedSample{0.0f, services::SignalQuality::Invalid, 1}, 1);
    CHECK(!monitor.systemHealthy());

    bus.publish("vehicle.speed",
                services::SpeedSample{60.0f, services::SignalQuality::Valid, 2}, 2);
    CHECK(monitor.systemHealthy());
}

static void test_health_monitor_triggers_dtc_on_invalid_signal() {
    std::printf("test_health_monitor_triggers_dtc_on_invalid_signal\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();
    apps::diagnostics::DtcStore dtcs;
    services::HealthMonitor monitor(bus, dtcs);

    // Three invalid samples → DTC confirmed (debounce threshold = 3).
    for (int i = 0; i < 3; ++i) {
        bus.publish("vehicle.speed",
                    services::SpeedSample{0.0f, services::SignalQuality::Invalid,
                                         static_cast<uint64_t>(i)},
                    static_cast<uint64_t>(i));
    }
    CHECK(!dtcs.readConfirmed().empty());
    CHECK(dtcs.readConfirmed()[0].code ==
          services::HealthMonitor::kDtcSpeedSignalInvalid);
}

static void test_health_monitor_clears_dtc_on_valid_signal() {
    std::printf("test_health_monitor_clears_dtc_on_valid_signal\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();
    apps::diagnostics::DtcStore dtcs;
    services::HealthMonitor monitor(bus, dtcs);

    // Confirm the DTC.
    for (int i = 0; i < 3; ++i) {
        bus.publish("vehicle.speed",
                    services::SpeedSample{0.0f, services::SignalQuality::Invalid,
                                         static_cast<uint64_t>(i)},
                    static_cast<uint64_t>(i));
    }
    CHECK(!dtcs.readConfirmed().empty());

    // Valid sample: testFailed bit must be cleared.
    bus.publish("vehicle.speed",
                services::SpeedSample{80.0f, services::SignalQuality::Valid, 10}, 10);

    const auto all = dtcs.readAll();
    CHECK(!all.empty());
    CHECK((all[0].status & apps::diagnostics::DtcStore::kTestFailed) == 0);
}

static void test_health_monitor_degraded_signal_is_unhealthy() {
    std::printf("test_health_monitor_degraded_signal_is_unhealthy\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();
    apps::diagnostics::DtcStore dtcs;
    services::HealthMonitor monitor(bus, dtcs);

    bus.publish("vehicle.speed",
                services::SpeedSample{50.0f, services::SignalQuality::Degraded, 1}, 1);
    // Degraded quality is not Valid → must report unhealthy.
    CHECK(!monitor.systemHealthy());
}

// =============================================================================
// SECTION 9 — End-to-End Integration
// =============================================================================

static void test_integration_can_to_aeb_full_pipeline() {
    std::printf("test_integration_can_to_aeb_full_pipeline\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();

    drivers::MockCanInterface can;
    can.setSpeedKmh(72.0f);   // 20 m/s

    services::VehicleSpeedService speed(can, bus);
    apps::adas::AebFunction aeb(bus);

    // Drive the speed service directly (no background thread for determinism).
    speed.cycle(); speed.cycle();  // two cycles → guaranteed speed frame seen

    // Manually publish obstacle distance (as the perception stack would).
    const uint64_t ts = 1000;
    bus.publish("obstacle.distance", 40.0f, ts);  // TTC = 40/20 = 2.0 s → Partial

    CHECK(aeb.lastDecision() == apps::adas::BrakeLevel::Partial);

    // Bring obstacle very close → Full.
    bus.publish("obstacle.distance", 5.0f, ts + 1);  // TTC = 5/20 = 0.25 s → Full
    CHECK(aeb.lastDecision() == apps::adas::BrakeLevel::Full);
}

static void test_integration_aeb_safe_after_speed_degrades() {
    std::printf("test_integration_aeb_safe_after_speed_degrades\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();

    drivers::MockCanInterface can;
    can.setSpeedKmh(80.0f);

    services::VehicleSpeedService speed(can, bus);
    apps::adas::AebFunction aeb(bus);

    // Establish valid speed + close obstacle → Full brake.
    speed.cycle(); speed.cycle();
    bus.publish("obstacle.distance", 10.0f, 1);
    CHECK(aeb.lastDecision() == apps::adas::BrakeLevel::Full);

    // Degrade speed signal — AEB must revert to safe state.
    can.setSpeedKmh(9999.0f);
    std::this_thread::sleep_for(220ms);  // let the 200 ms timeout expire
    speed.cycle(); speed.cycle();
    CHECK(aeb.lastDecision() == apps::adas::BrakeLevel::None);
}

static void test_integration_health_gates_ota_commit() {
    std::printf("test_integration_health_gates_ota_commit\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();

    apps::diagnostics::DtcStore dtcs;
    services::HealthMonitor monitor(bus, dtcs);

    OtaFixture fx;
    services::ota::OtaManager ota(fx.root, "ECU-GW-001", fx.key);

    CHECK(ota.startUpdate(fx.makePackage(1),
                          fx.root / "incoming" / "firmware.bin").ok);

    // System unhealthy → simulate rollback decision.
    bus.publish("vehicle.speed",
                services::SpeedSample{0.0f, services::SignalQuality::Invalid, 1}, 1);
    CHECK(!monitor.systemHealthy());
    if (!monitor.systemHealthy()) {
        auto rb = ota.rollback();
        CHECK(rb.ok);
        CHECK(ota.activeSlot() == "a");  // rolled back, active slot unchanged
    }

    // Now healthy → second update should commit successfully.
    bus.publish("vehicle.speed",
                services::SpeedSample{60.0f, services::SignalQuality::Valid, 2}, 2);
    CHECK(monitor.systemHealthy());

    CHECK(ota.startUpdate(fx.makePackage(2),
                          fx.root / "incoming" / "firmware.bin").ok);
    if (monitor.systemHealthy()) {
        auto c = ota.commit();
        CHECK(c.ok);
        CHECK(ota.activeSlot() == "b");
    }
}

static void test_integration_speed_service_thread_safety() {
    std::printf("test_integration_speed_service_thread_safety\n");
    resetBus();
    auto& bus = middleware::MessageBus::instance();

    drivers::MockCanInterface can;
    can.setSpeedKmh(60.0f);

    services::VehicleSpeedService speed(can, bus);

    std::atomic<int> sample_count{0};
    bus.subscribe("vehicle.speed", [&](const middleware::Sample&) {
        ++sample_count;
    });

    // Run the service in its background thread for a short duration.
    speed.start();
    std::this_thread::sleep_for(60ms);  // ~6 cycles at 10 ms period
    speed.stop();

    CHECK(sample_count.load() >= 4);  // at least 4 publishes in 60 ms
    CHECK(speed.latest().quality == services::SignalQuality::Valid);
}

// =============================================================================
// Main
// =============================================================================
int main() {
    std::printf("=== SDV Platform Test Suite ===\n\n");

    // -- Section 1: Cryptography --
    std::printf("[ Cryptography ]\n");
    test_sha256_known_vectors();
    test_sha256_streaming_equals_oneshot();
    test_sha256_single_byte_change_avalanche();
    test_hmac_deterministic();
    test_hmac_key_sensitivity();
    test_hmac_message_sensitivity();
    test_hmac_long_key_hashed();
    test_hmac_empty_message();
    test_constant_time_equal_same();
    test_constant_time_equal_differs_by_one_bit();
    test_constant_time_equal_all_zeros_vs_one();

    // -- Section 2: CAN Interface --
    std::printf("\n[ CAN Interface ]\n");
    test_mock_can_alternates_frames();
    test_mock_can_speed_encoding_accuracy();
    test_mock_can_zero_speed();
    test_mock_can_obstacle_encoding_accuracy();
    test_mock_can_always_open_and_write_ok();
    test_socket_can_non_linux_stub();
    test_can_frame_timestamp_is_set();

    // -- Section 3: MessageBus --
    std::printf("\n[ MessageBus ]\n");
    test_message_bus_pub_sub();
    test_message_bus_multiple_subscribers_all_receive();
    test_message_bus_unsubscribe_one_of_two();
    test_message_bus_no_subscribers_no_crash();
    test_message_bus_topic_isolation();
    test_message_bus_unsubscribe_invalid_id_no_crash();
    test_message_bus_reset_clears_all_subscriptions();
    test_message_bus_timestamp_forwarded();

    // -- Section 4: VehicleSpeedService --
    std::printf("\n[ VehicleSpeedService ]\n");
    test_speed_service_starts_invalid();
    test_speed_service_decode_and_plausibility();
    test_speed_service_zero_speed_is_valid();
    test_speed_service_boundary_400_kmh_valid();
    test_speed_service_above_400_kmh_rejected();
    test_speed_service_wrong_can_id_ignored();
    test_speed_service_signal_recovery_after_implausible();
    test_speed_service_publishes_on_bus();
    test_speed_service_published_payload_matches_latest();
    test_speed_service_signal_timeout_to_invalid();  // ~220 ms sleep

    // -- Section 5: AEB Function --
    std::printf("\n[ AEB Function ]\n");
    test_aeb_ttc_decisions();
    test_aeb_below_minimum_speed_inactive();
    test_aeb_exactly_minimum_speed_activates();
    test_aeb_full_brake_just_below_15s_threshold();
    test_aeb_partial_brake_at_exactly_15s_boundary();
    test_aeb_none_at_exactly_25s_boundary();
    test_aeb_zero_distance_safe_state();
    test_aeb_very_close_obstacle_full_brake();
    test_aeb_invalid_quality_is_safe_state();
    test_aeb_no_duplicate_publish_when_level_unchanged();
    test_aeb_ttc_value_reported_accurately();

    // -- Section 6: DTC Store --
    std::printf("\n[ DTC Store ]\n");
    test_dtc_debounce_and_clear();
    test_dtc_multiple_independent_codes();
    test_dtc_report_passed_nonexistent_no_crash();
    test_dtc_unconfirmed_not_in_readconfirmed();
    test_dtc_status_bits_after_reportpassed();
    test_dtc_occurrence_count_increments();
    test_dtc_cleared_after_clearall();
    test_dtc_confirmed_threshold_is_exactly_three();

    // -- Section 7: OTA Manager --
    std::printf("\n[ OTA Manager ]\n");
    test_ota_happy_path_commit();
    test_ota_rejects_tampered_manifest();
    test_ota_rejects_corrupt_payload();
    test_ota_anti_rollback();
    test_ota_rejects_wrong_hardware();
    test_ota_version_equal_to_installed_rejected();
    test_ota_rollback_returns_to_idle_active_slot_unchanged();
    test_ota_slot_alternates_over_two_updates();
    test_ota_double_startupdate_rejected();
    test_ota_commit_without_pending_returns_error();
    test_ota_rollback_without_pending_returns_error();
    test_ota_missing_manifest_file_returns_error();
    test_ota_missing_payload_file_returns_error();
    test_ota_parse_manifest_valid_json();
    test_ota_parse_manifest_empty_returns_nullopt();
    test_ota_parse_manifest_missing_field_returns_nullopt();
    test_ota_installed_version_persists_across_instances();

    // -- Section 8: HealthMonitor --
    std::printf("\n[ HealthMonitor ]\n");
    test_health_monitor_valid_signal_reports_healthy();
    test_health_monitor_invalid_signal_reports_unhealthy();
    test_health_monitor_recovery_from_invalid();
    test_health_monitor_triggers_dtc_on_invalid_signal();
    test_health_monitor_clears_dtc_on_valid_signal();
    test_health_monitor_degraded_signal_is_unhealthy();

    // -- Section 9: Integration --
    std::printf("\n[ Integration ]\n");
    test_integration_can_to_aeb_full_pipeline();
    test_integration_aeb_safe_after_speed_degrades();  // ~220 ms sleep
    test_integration_health_gates_ota_commit();
    test_integration_speed_service_thread_safety();     // ~60 ms sleep

    // Summary
    std::printf("\n─────────────────────────────────────────\n");
    std::printf("Ran %d checks.\n", g_total);
    if (g_failures == 0) {
        std::printf("ALL TESTS PASSED\n");
        return 0;
    }
    std::printf("%d FAILURE(S)\n", g_failures);
    return 1;
}
