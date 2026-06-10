// =============================================================================
// services/vehicle-health/health_monitor.hpp
//
// Watches signal quality on the bus and maps degradations to DTCs.
// In a full stack this also feeds the OTA module's post-update health check
// ("is the vehicle healthy after first boot on the new image?").
// =============================================================================
#pragma once

#include <atomic>

#include "apps/diagnostics/dtc_store.hpp"
#include "middleware/communication/message_bus.hpp"
#include "services/vehicle-signals/vehicle_speed_service.hpp"

namespace sdv::services {

class HealthMonitor {
public:
    static constexpr uint32_t kDtcSpeedSignalInvalid = 0xC12301;

    HealthMonitor(middleware::MessageBus& bus,
                  apps::diagnostics::DtcStore& dtcs)
        : bus_(bus), dtcs_(dtcs) {
        sub_ = bus_.subscribe("vehicle.speed", [this](const middleware::Sample& s) {
            const auto sample = std::any_cast<SpeedSample>(s.payload);
            if (sample.quality == SignalQuality::Valid) {
                dtcs_.reportPassed(kDtcSpeedSignalInvalid);
                healthy_.store(true);
            } else {
                dtcs_.reportFailed(kDtcSpeedSignalInvalid,
                                   "Vehicle speed signal invalid/degraded");
                healthy_.store(false);
            }
        });
    }

    ~HealthMonitor() { bus_.unsubscribe("vehicle.speed", sub_); }

    // Used by the OTA module as the commit/rollback criterion.
    bool systemHealthy() const { return healthy_.load(); }

private:
    middleware::MessageBus& bus_;
    apps::diagnostics::DtcStore& dtcs_;
    middleware::MessageBus::SubscriptionId sub_{0};
    std::atomic<bool> healthy_{false};
};

}  // namespace sdv::services
