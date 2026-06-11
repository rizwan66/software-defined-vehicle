// =============================================================================
// apps/demo/main_v2.cpp
//
// SDV Platform — Wave 2 Enhancement Demo (C++23 + new subsystems)
//
//   [1]  E2E Protection (ISO 26262 Part 7)  — CRC + sequence counter
//   [2]  Lock-free SPSC Ring Buffer         — ISR-safe CAN frame queue
//   [3]  Type-safe Physical Units           — km/h, m/s, m, s literals
//   [4]  ara::phm Alive Supervision         — cyclic task watchdog
//   [5]  Vehicle Dynamics Model             — kinematic bicycle simulation
//   [6]  SOVD Diagnostics (ISO 22900-6)    — JSON REST diagnostic service
//   [7]  std::expected OTA Pipeline         — monadic C++23 error handling
//   [8]  std::flat_map MessageBus           — cache-friendly pub/sub
//
// Build: make run-demo-v2
// =============================================================================
#include <chrono>
#include <expected>
#include <iomanip>
#include <iostream>
#include <thread>

#include "apps/diagnostics/dtc_store.hpp"
#include "drivers/can/can_interface.hpp"
#include "middleware/ara_phm/watchdog.hpp"
#include "middleware/communication/message_bus.hpp"
#include "middleware/e2e/e2e_protection.hpp"
#include "middleware/logging/sdv_log.hpp"
#include "middleware/metrics/metrics_collector.hpp"
#include "middleware/spsc/ring_buffer.hpp"
#include "middleware/units/physical_units.hpp"
#include "services/diagnostics/sovd_service.hpp"
#include "services/dynamics/vehicle_model.hpp"
#include "services/ota/ota_pipeline.hpp"
#include "services/vehicle-signals/vehicle_speed_service.hpp"

using namespace sdv;
using namespace sdv::units::literals;
using namespace std::chrono_literals;

static void section(const char* title) {
    std::cout << "\n╔══════════════════════════════════════════════════════╗\n"
              << "║  " << title << "\n"
              << "╚══════════════════════════════════════════════════════╝\n";
}

// ─── [1] E2E Protection ───────────────────────────────────────────────────────

static void demo_e2e() {
    section("[1] E2E Protection — ISO 26262 Part 7 (CRC-8 + sequence counter)");

    e2e::E2eProfile1 e2e_tx;
    e2e::E2eProfile1 e2e_rx;

    // Simulate 5 normal transmissions
    uint8_t speed_payload[2] = {0xD0, 0x07};  // 2000 * 0.01 = 20.00 km/h
    std::cout << "  Normal transmissions:\n";
    for (int i = 0; i < 5; ++i) {
        auto hdr = e2e_tx.protect(e2e::kDataIdVehicleSpeed,
                                  std::span(speed_payload));
        auto status = e2e_rx.check(e2e::kDataIdVehicleSpeed,
                                   std::span(speed_payload), hdr);
        std::cout << "    frame " << i << ": CRC=0x" << std::hex << std::setw(2)
                  << std::setfill('0') << static_cast<int>(hdr.crc)
                  << std::dec << "  cnt=" << static_cast<int>(hdr.counter)
                  << "  status=" << e2e::toString(status) << "\n";
    }

    // Simulate bit flip (corruption)
    auto hdr = e2e_tx.protect(e2e::kDataIdVehicleSpeed, std::span(speed_payload));
    hdr.crc ^= 0x01;  // flip one CRC bit
    auto s = e2e_rx.check(e2e::kDataIdVehicleSpeed, std::span(speed_payload), hdr);
    std::cout << "  Corrupted frame: status=" << e2e::toString(s) << " ← detected!\n";

    // Simulate lost frame (counter gap)
    auto hdr2 = e2e_tx.protect(e2e::kDataIdVehicleSpeed, std::span(speed_payload));
    e2e_tx.protect(e2e::kDataIdVehicleSpeed, std::span(speed_payload)); // send but not received
    auto hdr4 = e2e_tx.protect(e2e::kDataIdVehicleSpeed, std::span(speed_payload));
    e2e_rx.check(e2e::kDataIdVehicleSpeed, std::span(speed_payload), hdr2);
    s = e2e_rx.check(e2e::kDataIdVehicleSpeed, std::span(speed_payload), hdr4);
    std::cout << "  Lost frame (gap 2):  status=" << e2e::toString(s) << " ← detected!\n";

    // Wrong Data ID (routing error)
    e2e::E2eProfile1 wrong_tx;
    e2e::E2eProfile1 wrong_rx;
    auto hdr_tx = wrong_tx.protect(0xDEAD, std::span(speed_payload));  // wrong ID
    s = wrong_rx.check(e2e::kDataIdVehicleSpeed, std::span(speed_payload), hdr_tx);
    std::cout << "  Wrong Data ID:       status=" << e2e::toString(s) << " ← detected!\n";
}

// ─── [2] Lock-free SPSC Ring Buffer ──────────────────────────────────────────

static void demo_spsc() {
    section("[2] Lock-free SPSC Ring Buffer — ISR-safe, wait-free CAN frame queue");

    spsc::RingBuffer<drivers::CanFrame, 256> queue;

    // Simulate ISR: producer pushes 10 frames
    std::cout << "  Producer (ISR side): pushing 10 CAN frames...\n";
    int pushed = 0;
    for (int i = 0; i < 10; ++i) {
        drivers::CanFrame frame;
        frame.id  = 0x123;
        frame.dlc = 2;
        frame.data[0] = static_cast<uint8_t>((i * 100) & 0xFF);
        frame.data[1] = static_cast<uint8_t>((i * 100) >> 8);
        frame.timestamp_ns = static_cast<uint64_t>(i * 10'000'000);
        if (queue.push(frame)) ++pushed;
    }
    std::cout << "    pushed=" << pushed << "  queue size=" << queue.size() << "\n";

    // Consumer: pop all frames
    std::cout << "  Consumer (app task): draining queue...\n";
    int popped = 0;
    while (auto frame = queue.pop()) {
        const uint16_t raw = frame->data[0] | (frame->data[1] << 8);
        const float kmh = raw * 0.01f;
        std::cout << "    frame " << popped++ << ": id=0x"
                  << std::hex << frame->id << std::dec
                  << "  speed=" << kmh << " km/h\n";
    }
    std::cout << "  Queue empty after drain: " << (queue.empty() ? "yes" : "no") << "\n";

    // Show overflow protection
    std::cout << "  Fill test: capacity=" << queue.kCapacity << " frames\n";
    int filled = 0;
    drivers::CanFrame f{}; f.id = 0x999;
    while (queue.push(f)) ++filled;
    std::cout << "    filled=" << filled << " (queue full=" << queue.full() << ")\n";
}

// ─── [3] Type-safe Physical Units ────────────────────────────────────────────

static void demo_units() {
    section("[3] Type-safe Physical Units — zero-overhead km/h, m/s, m, s literals");

    using namespace units;
    using namespace units::literals;

    // User-defined literals
    auto speed = 120.0_kmh;
    auto dist  = 60.0_m;

    std::cout << "  120 km/h in m/s: " << to_mps(speed).value << " m/s\n";
    std::cout << "  33.3 m/s in km/h: " << to_kmh(Mps{33.3f}).value << " km/h\n";

    // TTC using type-safe function — cannot accidentally pass km/h as m/s
    auto closing = to_mps(speed);  // must convert explicitly
    auto ttc_val = ttc(dist, closing);
    std::cout << "  TTC at 120 km/h, 60 m gap: " << ttc_val.value << " s\n";

    // Compile-time safety demonstration
    // ttc(dist, speed);  // ← would NOT compile: Speed<Kmh> ≠ Speed<Mps>
    std::cout << "  unit_cast<Mps>(0.3_g): "
              << unit_cast<MpsSquared>(0.3_g).value << " m/s²\n";

    // Arithmetic within same unit
    auto total_dist = 60.0_m + 40.0_m;
    std::cout << "  60m + 40m = " << total_dist.value << " m\n";
}

// ─── [4] ara::phm Alive Supervision ──────────────────────────────────────────

static void demo_phm() {
    section("[4] ara::phm Alive Supervision — cyclic task watchdog");

    using namespace ara_phm;
    auto& wd = AliveWatchdog::instance();
    wd.reset();

    // Register two tasks
    bool speed_fault_fired = false;
    wd.registerTask("VehicleSpeedService", AliveConfig{
        .expected_period  = 10ms,
        .max_missed_alives = 3,
        .on_failure = [&](std::string_view task) {
            speed_fault_fired = true;
            std::cout << "  [PHM] FAULT: " << task << " missed too many alives!\n";
        }
    });
    wd.registerTask("HealthMonitor", AliveConfig{.expected_period = 20ms});

    wd.startSupervision();

    // Task runs normally for 5 cycles
    for (int i = 0; i < 5; ++i) {
        wd.checkpoint("VehicleSpeedService");
        wd.checkpoint("HealthMonitor");
        std::this_thread::sleep_for(12ms);
    }
    std::cout << "  After 5 normal cycles: VehicleSpeedService="
              << toString(wd.status("VehicleSpeedService")) << "\n";

    // Task stops checking in (simulate hang)
    std::cout << "  VehicleSpeedService stops checking in...\n";
    std::this_thread::sleep_for(80ms);  // supervisor fires after ~3 missed windows
    std::cout << "  Status after silence: "
              << toString(wd.status("VehicleSpeedService")) << "\n";
    std::cout << "  Fault callback fired: " << (speed_fault_fired ? "yes" : "no") << "\n";

    wd.stopSupervision();
}

// ─── [5] Vehicle Dynamics Model ───────────────────────────────────────────────

static void demo_dynamics() {
    section("[5] Kinematic Bicycle Vehicle Model — physics-based ADAS simulation");

    auto& bus = middleware::MessageBus::instance();
    services::dynamics::VehicleModel model(bus, {
        .wheelbase_m  = 2.7f,
        .obstacle_x_m = 200.0f,
        .obstacle_y_m = 0.0f
    });

    std::cout << "  Accelerating from 0 to 100 km/h (10 s simulation):\n";
    float t = 0.0f;
    const float dt = 0.1f;
    while (t < 5.0f) {
        auto s = model.step(dt, 3.0f /*m/s² accel*/, 0.0f);
        if (static_cast<int>(t * 10) % 10 == 0) {
            std::cout << "    t=" << std::fixed << std::setprecision(1) << t
                      << "s  x=" << std::setprecision(1) << s.x_m
                      << "m  speed=" << std::setprecision(1) << model.speedKmh().value
                      << "km/h  dist_to_obstacle=" << std::setprecision(1)
                      << model.obstacleDistanceM() << "m\n";
        }
        t += dt;
    }

    // Lane change: apply steering for 2 s
    std::cout << "  Lane change (δ=0.05 rad for 2 s):\n";
    t = 0.0f;
    while (t < 2.0f) {
        auto s = model.step(dt, 0.0f, 0.05f);
        if (static_cast<int>(t * 10) % 5 == 0) {
            std::cout << "    t=" << std::fixed << std::setprecision(1) << t
                      << "s  y=" << std::setprecision(2) << s.y_m
                      << "m  yaw_rate=" << std::setprecision(3) << s.yaw_rate_rps
                      << "rad/s  lat_accel=" << std::setprecision(3)
                      << s.lat_accel_mps2 << "m/s²\n";
        }
        t += dt;
    }
}

// ─── [6] SOVD Diagnostics ────────────────────────────────────────────────────

static void demo_sovd() {
    section("[6] SOVD Service (ISO 22900-6) — JSON REST-style vehicle diagnostics");

    apps::diagnostics::DtcStore dtcs;
    services::diagnostics::SovdService sovd(dtcs, {
        .vin              = "WBA12345678901234",
        .ecu_name         = "SDV-GW-001",
        .hardware_id      = "ECU-GW-001-REV-C",
        .software_version = "2.1.0",
        .software_build_date = "2026-06-11",
        .supplier         = "ACME Automotive GmbH"
    });

    // Register live data endpoints
    static float speed_kmh = 95.3f;
    static float battery_v = 12.4f;
    sovd.registerLiveData("vehicle_speed",   {"km/h",   []{ return speed_kmh; }, "Vehicle speed"});
    sovd.registerLiveData("battery_voltage", {"V",      []{ return battery_v; }, "12V battery"});
    sovd.registerLiveData("engine_temp",     {"°C",     []{ return 87.2; },      "Coolant temp"});

    // Inject a DTC
    dtcs.reportFailed(0xC12301, "Vehicle speed signal invalid");
    dtcs.reportFailed(0xC12301, "Vehicle speed signal invalid");
    dtcs.reportFailed(0xC12301, "Vehicle speed signal invalid");

    std::cout << "  GET /capabilities:\n" << sovd.getCapabilities() << "\n\n";
    std::cout << "  GET /properties:\n"   << sovd.getProperties()   << "\n\n";
    std::cout << "  GET /data/live/vehicle_speed:\n"
              << sovd.getLiveData("vehicle_speed") << "\n\n";
    std::cout << "  GET /data/live (all):\n" << sovd.getAllLiveData() << "\n\n";
    std::cout << "  GET /faults:\n" << sovd.getFaults() << "\n\n";
    std::cout << "  DELETE /faults → " << sovd.clearFaults() << "\n";
    std::cout << "  GET /faults (after clear):\n" << sovd.getFaults() << "\n";
}

// ─── [7] std::expected OTA Pipeline ──────────────────────────────────────────

static void demo_expected_ota() {
    section("[7] std::expected OTA Pipeline — C++23 monadic error handling");
    using namespace services::ota::pipeline;

    OtaPipeline pipe(
        "/tmp/sdv-ota-demo",
        "ECU-GW-001",
        {'d', 'e', 'm', 'o', '-', 'k', 'e', 'y'});

    std::cout << "  Monadic happy path (.and_then chain):\n";
    auto result = pipe.performUpdate(
        "/tmp/sdv-ota-demo/incoming/manifest.json",
        "/tmp/sdv-ota-demo/incoming/firmware.bin",
        true  // health check passes
    );

    // C++23 Expected: operator bool, operator*, .error()
    if (result) {
        std::cout << "  OK: " << *result << "\n";
        std::cout << "  Active slot: slot_" << pipe.activeSlot()
                  << "  version: " << pipe.installedVersion() << "\n";
    } else {
        const auto& err = result.error();
        std::cout << "  FAIL [" << errorName(err.code) << "]: " << err.detail << "\n";
    }

    std::cout << "\n  Tampered manifest (expected failure → typed error code):\n";
    auto bad = pipe.performUpdate(
        "/tmp/sdv-ota-demo/incoming/manifest_tampered.json",
        "/tmp/sdv-ota-demo/incoming/firmware.bin",
        true);

    if (!bad) {
        const auto& err = bad.error();
        std::cout << "  Rejected! ErrorCode::" << errorName(err.code)
                  << "\n  Detail: " << err.detail << "\n";
    }

    std::cout << "\n  .or_else() recovery — no try/catch, no if-chain:\n";
    auto recovered = pipe.performUpdate(
        "/tmp/sdv-ota-demo/incoming/manifest_tampered.json",
        "/tmp/sdv-ota-demo/incoming/firmware.bin",
        true)
        .or_else([](const OtaError& e) -> Expected<std::string> {
            // In production: log to DTC, notify FOTA server, schedule retry
            return std::string("update deferred — ") + std::string(errorName(e.code));
        });
    std::cout << "  or_else result: " << *recovered << "\n";
}

// ─── [8] flat_map MessageBus ──────────────────────────────────────────────────

static void demo_flatmap_bus() {
    section("[8] std::flat_map MessageBus — cache-friendly contiguous subscriber table");

    auto& bus = middleware::MessageBus::instance();

    // Subscribe to several topics to populate the flat_map
    std::vector<middleware::MessageBus::SubscriptionId> ids;
    const char* topics[] = {"sensor.a", "sensor.b", "sensor.c",
                             "control.x", "control.y", "diag.z"};
    int received = 0;
    for (const char* t : topics) {
        ids.push_back(bus.subscribe(t, [&](const middleware::Sample&) {
            ++received;
        }));
    }
    std::cout << "  Subscribed to " << sizeof(topics)/sizeof(*topics)
              << " topics (std::flat_map stores keys contiguously)\n";

    // Publish to all topics
    for (const char* t : topics)
        bus.publish(std::string(t), 42.0f, 0ULL);
    std::cout << "  Published to all " << sizeof(topics)/sizeof(*topics)
              << " topics — received=" << received << "\n";

    // Unsubscribe
    for (size_t i = 0; i < ids.size(); ++i)
        bus.unsubscribe(topics[i], ids[i]);
    std::cout << "  Unsubscribed all — flat_map resorted in-place\n";
}

// ─── Main ────────────────────────────────────────────────────────────────────

int main() {
    sdv::log::Logger::instance().setMinLevel(sdv::log::Level::Warn);

    demo_e2e();
    demo_spsc();
    demo_units();
    demo_phm();
    demo_dynamics();
    demo_sovd();
    demo_expected_ota();
    demo_flatmap_bus();

    section("Wave 2 Demo complete — all systems operational");
    return 0;
}
