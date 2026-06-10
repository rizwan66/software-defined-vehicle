// =============================================================================
// apps/demo/main.cpp
//
// End-to-end demo:
//   1. Start the vehicle speed service on a mock CAN bus.
//   2. AEB function reacts as an obstacle gets closer (TTC drops).
//   3. Health monitor reports DTCs when the speed signal degrades.
//   4. Full OTA cycle: verify signed package -> install to inactive slot
//      -> health check -> commit. Then a tampered package is rejected.
// =============================================================================
#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

#include "apps/adas/aeb_function.hpp"
#include "apps/diagnostics/dtc_store.hpp"
#include "drivers/can/can_interface.hpp"
#include "middleware/communication/message_bus.hpp"
#include "services/ota/ota_manager.hpp"
#include "services/vehicle-health/health_monitor.hpp"
#include "services/vehicle-signals/vehicle_speed_service.hpp"

using namespace sdv;
using namespace std::chrono_literals;

static const char* levelName(apps::adas::BrakeLevel l) {
    switch (l) {
        case apps::adas::BrakeLevel::None:    return "NONE";
        case apps::adas::BrakeLevel::Partial: return "PARTIAL";
        case apps::adas::BrakeLevel::Full:    return "FULL";
    }
    return "?";
}

int main() {
    auto& bus = middleware::MessageBus::instance();

    std::cout << "=== SDV platform demo ===\n\n[1] Vehicle signals + ADAS\n";

    drivers::MockCanInterface can;
    can.setSpeedKmh(80.0f);
    can.setObstacleDistanceM(120.0f);

    apps::diagnostics::DtcStore dtcs;
    services::VehicleSpeedService speed(can, bus);
    services::HealthMonitor health(bus, dtcs);
    apps::adas::AebFunction aeb(bus);

    bus.subscribe("brake.request", [](const middleware::Sample& s) {
        const auto req = std::any_cast<apps::adas::BrakeRequest>(s.payload);
        std::cout << "    [AEB] brake request: " << levelName(req.level)
                  << " (TTC " << std::fixed << std::setprecision(2)
                  << req.ttc_s << " s)\n";
    });

    speed.start();

    // Obstacle approaches: 120 m -> 20 m at constant 80 km/h (22.2 m/s).
    for (float d = 120.0f; d >= 20.0f; d -= 25.0f) {
        can.setObstacleDistanceM(d);
        // The speed service publishes vehicle.speed; obstacle distance comes
        // from a (mock) perception stack here:
        const uint64_t ts = static_cast<uint64_t>(
            std::chrono::steady_clock::now().time_since_epoch().count());
        bus.publish("obstacle.distance", d, ts);
        std::cout << "  obstacle at " << d << " m, speed "
                  << speed.latest().kmh << " km/h\n";
        std::this_thread::sleep_for(60ms);
    }

    speed.stop();

    std::cout << "\n[2] Diagnostics (DTC memory)\n";
    // Provoke a confirmed DTC: speed signal implausible.
    can.setSpeedKmh(9999.0f);  // > 400 km/h plausibility limit
    for (int i = 0; i < 25; ++i) speed.cycle();
    for (const auto& d : dtcs.readAll()) {
        std::cout << "    DTC 0x" << std::hex << std::uppercase << d.code
                  << std::dec << " [" << d.description
                  << "] status=0x" << std::hex << int(d.status) << std::dec
                  << " count=" << d.occurrence_count << "\n";
    }

    std::cout << "\n[3] OTA update cycle\n";
    services::ota::OtaManager ota("/tmp/sdv-ota-demo", "ECU-GW-001",
                                  {'d', 'e', 'm', 'o', '-', 'k', 'e', 'y'});
    std::cout << "    active slot: slot_" << ota.activeSlot()
              << ", installed version: " << ota.installedVersion() << "\n";

    auto r = ota.startUpdate("/tmp/sdv-ota-demo/incoming/manifest.json",
                             "/tmp/sdv-ota-demo/incoming/firmware.bin");
    std::cout << "    startUpdate: " << (r.ok ? "OK  " : "FAIL ") << r.message
              << "\n";
    if (r.ok) {
        const bool healthy = true;  // post-boot health check result
        if (healthy) {
            auto c = ota.commit();
            std::cout << "    commit:      " << (c.ok ? "OK  " : "FAIL ")
                      << c.message << "\n";
        } else {
            auto rb = ota.rollback();
            std::cout << "    rollback:    " << rb.message << "\n";
        }
    }

    auto bad = ota.startUpdate("/tmp/sdv-ota-demo/incoming/manifest_tampered.json",
                               "/tmp/sdv-ota-demo/incoming/firmware.bin");
    std::cout << "    tampered pkg: " << (bad.ok ? "ACCEPTED (BUG!) " : "rejected — ")
              << bad.message << "\n";

    std::cout << "\n=== demo complete ===\n";
    return 0;
}
