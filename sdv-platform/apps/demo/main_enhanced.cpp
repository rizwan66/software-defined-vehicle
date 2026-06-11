// =============================================================================
// apps/demo/main_enhanced.cpp
//
// Enhanced end-to-end SDV platform demo — showcases all modern additions:
//
//   [1] Kalman-filtered vehicle speed  + metrics
//   [2] AEB (enhanced with filtered speed)
//   [3] Lane Keeping Assist (LKA)
//   [4] Adaptive Cruise Control (ACC)
//   [5] Blind Spot Detection (BSD)
//   [6] ara::com service discovery (skeleton → proxy)
//   [7] Prometheus metrics snapshot
//   [8] OTA update cycle (unchanged + tamper rejection)
//
// Build:  see sdv-platform/Makefile  `make run-demo-enhanced`
// =============================================================================
#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

#include "apps/adas/adaptive_cruise.hpp"
#include "apps/adas/aeb_function.hpp"
#include "apps/adas/blind_spot.hpp"
#include "apps/adas/lane_keeping.hpp"
#include "apps/diagnostics/dtc_store.hpp"
#include "drivers/can/can_interface.hpp"
#include "middleware/ara_com/service_registry.hpp"
#include "middleware/communication/message_bus.hpp"
#include "middleware/logging/sdv_log.hpp"
#include "middleware/metrics/metrics_collector.hpp"
#include "services/ota/ota_manager.hpp"
#include "services/vehicle-health/health_monitor.hpp"
#include "services/vehicle-signals/vehicle_speed_service.hpp"

using namespace sdv;
using namespace std::chrono_literals;

// ─── Helpers ─────────────────────────────────────────────────────────────────

static const char* brakeName(apps::adas::BrakeLevel l) {
    switch (l) {
        case apps::adas::BrakeLevel::None:    return "NONE";
        case apps::adas::BrakeLevel::Partial: return "PARTIAL";
        case apps::adas::BrakeLevel::Full:    return "FULL";
    }
    return "?";
}

static const char* warningName(apps::adas::BsdWarningType t) {
    switch (t) {
        case apps::adas::BsdWarningType::None:       return "none";
        case apps::adas::BsdWarningType::VisualOnly:  return "VISUAL";
        case apps::adas::BsdWarningType::Audible:     return "AUDIBLE";
        case apps::adas::BsdWarningType::Haptic:      return "HAPTIC";
    }
    return "?";
}

static void separator(const char* title) {
    std::cout << "\n╔══════════════════════════════════════════════════════╗\n";
    std::cout << "║  " << title << "\n";
    std::cout << "╚══════════════════════════════════════════════════════╝\n";
}

// ─── Main ────────────────────────────────────────────────────────────────────

int main() {
    sdv::log::Logger::instance().setMinLevel(sdv::log::Level::Info);
    auto& bus = middleware::MessageBus::instance();

    // ── [1] Speed service + Kalman filter ─────────────────────────────────
    separator("[1] Kalman-Filtered Vehicle Speed + Metrics");

    drivers::MockCanInterface can;
    can.setSpeedKmh(0.0f);

    apps::diagnostics::DtcStore dtcs;
    services::VehicleSpeedService speed(can, bus);
    services::HealthMonitor health(bus, dtcs);

    bus.subscribe("vehicle.speed", [](const middleware::Sample& s) {
        const auto sp = std::any_cast<services::SpeedSample>(s.payload);
        std::cout << std::fixed << std::setprecision(2)
                  << "  speed raw=" << sp.kmh
                  << "  filtered=" << sp.kmh_filtered
                  << "  convergence=" << sp.filter_convergence
                  << "  quality=" << (sp.quality == services::SignalQuality::Valid ? "Valid"
                                   : sp.quality == services::SignalQuality::Degraded ? "Degraded"
                                   : "Invalid")
                  << "\n";
    });

    // Ramp speed 0 → 100 km/h over 10 cycles to show Kalman convergence.
    for (float kmh = 0.0f; kmh <= 100.0f; kmh += 10.0f) {
        can.setSpeedKmh(kmh);
        speed.cycle();
    }

    // ── [2] AEB ────────────────────────────────────────────────────────────
    separator("[2] Autonomous Emergency Braking (AEB)");

    apps::adas::AebFunction aeb(bus);

    bus.subscribe("brake.request", [](const middleware::Sample& s) {
        const auto req = std::any_cast<apps::adas::BrakeRequest>(s.payload);
        std::cout << "  [AEB] brake=" << brakeName(req.level)
                  << "  TTC=" << std::fixed << std::setprecision(2) << req.ttc_s << "s\n";
    });

    can.setSpeedKmh(100.0f);
    for (float d = 120.0f; d >= 15.0f; d -= 20.0f) {
        can.setObstacleDistanceM(d);
        speed.cycle();
        const uint64_t ts = static_cast<uint64_t>(
            std::chrono::steady_clock::now().time_since_epoch().count());
        bus.publish("obstacle.distance", d, ts);
        std::cout << "  obstacle=" << d << "m  ego=100km/h\n";
        std::this_thread::sleep_for(20ms);
    }

    // ── [3] Lane Keeping Assist ────────────────────────────────────────────
    separator("[3] Lane Keeping Assist (LKA)");

    apps::adas::LaneKeepingAssist lka(bus);

    bus.subscribe("lka.steering_torque", [](const middleware::Sample& s) {
        const auto req = std::any_cast<apps::adas::SteeringTorque>(s.payload);
        std::cout << "  [LKA] torque=" << std::fixed << std::setprecision(2)
                  << req.torque_nm << "Nm  active=" << req.lka_active << "\n";
    });

    can.setSpeedKmh(100.0f);
    speed.cycle();  // publish fresh speed at 100 km/h (LKA requires ≥60)

    const uint64_t lka_ts = static_cast<uint64_t>(
        std::chrono::steady_clock::now().time_since_epoch().count());

    // Scenario: drifting right by 0.3 m, heading error 0.02 rad
    apps::adas::LaneData lane{0.3f, 0.02f, 0.9f, lka_ts};
    bus.publish("lane.data", lane, lka_ts);
    std::cout << "  lane: lateral_error=+0.3m  heading=+0.02rad  confidence=0.9\n";

    // Driver override: high steering torque suppresses LKA
    bus.publish("driver.steering_torque", 4.0f, lka_ts);
    lane = {0.3f, 0.02f, 0.9f, lka_ts};
    bus.publish("lane.data", lane, lka_ts);
    std::cout << "  driver override (4 Nm) — LKA suppressed\n";

    // Restore: driver releases wheel
    bus.publish("driver.steering_torque", 0.0f, lka_ts);
    bus.publish("lane.data", lane, lka_ts);

    // ── [4] Adaptive Cruise Control ────────────────────────────────────────
    separator("[4] Adaptive Cruise Control (ACC)");

    apps::adas::AdaptiveCruiseControl acc(bus, 120.0f);

    bus.subscribe("acc.throttle_request", [](const middleware::Sample& s) {
        const auto req = std::any_cast<apps::adas::AccRequest>(s.payload);
        if (!req.active) return;
        std::cout << "  [ACC] state=" << (req.brake > 0 ? "braking" : "throttle")
                  << "  throttle=" << std::fixed << std::setprecision(2) << req.throttle
                  << "  brake=" << req.brake
                  << "  gap=" << req.actual_gap_s << "s\n";
    });

    can.setSpeedKmh(100.0f);
    speed.cycle();

    // No lead vehicle: speed-only control (accelerate to 120)
    const uint64_t acc_ts1 = static_cast<uint64_t>(
        std::chrono::steady_clock::now().time_since_epoch().count());
    bus.publish("radar.lead_vehicle",
                apps::adas::LeadVehicle{0.0f, 0.0f, false, acc_ts1}, acc_ts1);
    std::cout << "  no lead vehicle — tracking 120 km/h set-point\n";

    // Lead vehicle at 60 m, 5 m/s relative closing speed (2.16 s gap)
    const uint64_t acc_ts2 = static_cast<uint64_t>(
        std::chrono::steady_clock::now().time_since_epoch().count());
    bus.publish("radar.lead_vehicle",
                apps::adas::LeadVehicle{60.0f, -5.0f, true, acc_ts2}, acc_ts2);
    std::cout << "  lead vehicle at 60m closing @ 5m/s — following mode\n";

    // ── [5] Blind Spot Detection ────────────────────────────────────────────
    separator("[5] Blind Spot Detection (BSD)");

    apps::adas::BlindSpotDetection bsd(bus);

    bus.subscribe("bsd.warning", [](const middleware::Sample& s) {
        const auto w = std::any_cast<apps::adas::BsdWarning>(s.payload);
        std::cout << "  [BSD] side=" << (w.side == apps::adas::BsdSide::Left ? "LEFT" : "RIGHT")
                  << "  warning=" << warningName(w.type)
                  << "  confidence=" << std::fixed << std::setprecision(2) << w.confidence
                  << "  range=" << w.range_m << "m\n";
    });

    // Keep speed at 100 km/h (already established) — BSD is active above 10 km/h.
    // Note: a sudden jump from 100→80 in one 10 ms cycle exceeds the 15 km/h
    // plausibility limit and would degrade the speed signal, deactivating BSD
    // (the correct safety response). We stay at 100 km/h to show BSD working.
    speed.cycle();

    const uint64_t bsd_ts = static_cast<uint64_t>(
        std::chrono::steady_clock::now().time_since_epoch().count());

    // Object in left blind spot, driver not signalling
    apps::adas::BsdSensorReading left_obj{true, 0.85f, 4.5f, bsd_ts};
    for (int i = 0; i < 4; ++i)
        bus.publish("bsd.left_sensor", left_obj, bsd_ts);

    // Driver signals left → warning escalates to AUDIBLE
    bus.publish("driver.turn_signal", apps::adas::TurnSignalState::Left, bsd_ts);
    bus.publish("bsd.left_sensor", left_obj, bsd_ts);

    // ── [6] ara::com service discovery ────────────────────────────────────
    separator("[6] AUTOSAR ara::com Service Discovery");

    {
        ara_com::ServiceId spd_svc_id{"VehicleSpeedService", 0x0101, 0x0001};

        // Skeleton (server side)
        ara_com::ServiceSkeleton skeleton(spd_svc_id);
        skeleton.declareEvent("SpeedEvent");
        skeleton.offerService();
        std::cout << "  Skeleton: VehicleSpeedService offered (SOME/IP 0x0101/0x0001)\n";

        // Proxy (client side) — finds the service
        auto proxy_opt = ara_com::ServiceProxy::find(spd_svc_id, 200);
        if (proxy_opt) {
            proxy_opt->subscribe<services::SpeedSample>(
                "SpeedEvent", [](const services::SpeedSample& sp) {
                    std::cout << "  Proxy received SpeedEvent: "
                              << std::fixed << std::setprecision(1)
                              << sp.kmh_filtered << " km/h (filtered)\n";
                });
            std::cout << "  Proxy: found and subscribed to SpeedEvent\n";

            // Skeleton sends a sample
            services::SpeedSample sample{
                .kmh = 100.0f,
                .quality = services::SignalQuality::Valid,
                .timestamp_ns = 0,
                .kmh_filtered = 99.2f,
                .filter_convergence = 0.95f
            };
            skeleton.sendEvent("SpeedEvent", sample);
        }
        skeleton.stopOfferService();
    }

    // ── [7] Prometheus metrics snapshot ────────────────────────────────────
    separator("[7] Prometheus Metrics Snapshot");
    std::cout << metrics::MetricsRegistry::instance().snapshot();

    // ── [8] OTA update cycle ───────────────────────────────────────────────
    separator("[8] OTA Update Cycle");

    services::ota::OtaManager ota("/tmp/sdv-ota-demo", "ECU-GW-001",
                                  {'d', 'e', 'm', 'o', '-', 'k', 'e', 'y'});
    std::cout << "  active slot: slot_" << ota.activeSlot()
              << "  installed version: " << ota.installedVersion() << "\n";

    auto r = ota.startUpdate("/tmp/sdv-ota-demo/incoming/manifest.json",
                             "/tmp/sdv-ota-demo/incoming/firmware.bin");
    std::cout << "  startUpdate: " << (r.ok ? "OK   " : "FAIL ") << r.message << "\n";
    if (r.ok) {
        auto c = ota.commit();
        std::cout << "  commit:      " << (c.ok ? "OK   " : "FAIL ") << c.message << "\n";
    }

    auto bad = ota.startUpdate("/tmp/sdv-ota-demo/incoming/manifest_tampered.json",
                               "/tmp/sdv-ota-demo/incoming/firmware.bin");
    std::cout << "  tampered pkg: " << (bad.ok ? "ACCEPTED (BUG!) " : "rejected — ")
              << bad.message << "\n";

    separator("Demo complete");
    return 0;
}
