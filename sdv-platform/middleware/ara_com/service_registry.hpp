// =============================================================================
// middleware/ara_com/service_registry.hpp
//
// AUTOSAR Adaptive ara::com — service discovery simulation.
//
// In a real AUTOSAR Adaptive platform, ara::com provides:
//   * Service-oriented communication (SOME/IP transport)
//   * Skeleton (server) / Proxy (client) code-generated from ARXML
//   * OfferService / FindService / StartFindService APIs
//   * Event, Method, Field abstractions
//
// This header simulates that programming model in-process so application code
// follows the same patterns and can be ported to a generated ara::com binding
// with minimal changes (replace the registry calls with real SOME/IP stubs).
//
// Supported operations:
//   skeleton.offerService()      — makes a service visible
//   proxy  = ServiceProxy::find() — blocks until service is available
//   proxy.event.subscribe()      — subscribe to events from the server
//
// Mapping to production:
//   ServiceRegistry ↔ ara::com::FindService / OfferService
//   EventSkeleton   ↔ ara::com::skeleton::event<T>::Send()
//   EventProxy      ↔ ara::com::proxy::event<T>::Subscribe() / GetNewSamples()
// =============================================================================
#pragma once

#include <any>
#include <condition_variable>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace sdv::ara_com {

// ─── Service descriptor ──────────────────────────────────────────────────────

struct ServiceId {
    std::string name;        // logical name  (e.g. "VehicleSpeedService")
    uint16_t    service_id;  // SOME/IP service ID
    uint16_t    instance_id; // SOME/IP instance ID

    bool operator<(const ServiceId& o) const noexcept {
        return std::tie(service_id, instance_id) <
               std::tie(o.service_id, o.instance_id);
    }
};

// ─── Event channel (skeleton → proxy) ────────────────────────────────────────

// An event endpoint that the skeleton publishes samples into.
// Proxy subscribers receive copies via their registered callbacks.
class EventChannel {
public:
    using Handler = std::function<void(const std::any&)>;

    // Skeleton calls this to send an event sample.
    void send(std::any value) {
        std::vector<Handler> snapshot;
        {
            std::lock_guard lk(mu_);
            snapshot = subscribers_;
        }
        for (const auto& h : snapshot) h(value);
    }

    // Proxy calls this during subscribe().
    void addSubscriber(Handler h) {
        std::lock_guard lk(mu_);
        subscribers_.push_back(std::move(h));
    }

private:
    std::mutex mu_;
    std::vector<Handler> subscribers_;
};

// ─── Service registry ─────────────────────────────────────────────────────────

class ServiceRegistry {
public:
    static ServiceRegistry& instance() {
        static ServiceRegistry reg;
        return reg;
    }

    // Called by a skeleton to make itself visible.
    void offerService(ServiceId id,
                      std::shared_ptr<std::map<std::string, EventChannel>> events) {
        std::lock_guard lk(mu_);
        offered_[id] = std::move(events);
        cv_.notify_all();
    }

    // Called by a skeleton when it shuts down.
    void stopOfferService(const ServiceId& id) {
        std::lock_guard lk(mu_);
        offered_.erase(id);
    }

    // Blocking find — waits up to timeout_ms for the service to appear.
    // Returns nullptr on timeout (proxy should handle gracefully).
    std::shared_ptr<std::map<std::string, EventChannel>>
    findService(const ServiceId& id, int timeout_ms = 1000) {
        std::unique_lock lk(mu_);
        cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms), [&] {
            return offered_.count(id) > 0;
        });
        auto it = offered_.find(id);
        return (it != offered_.end()) ? it->second : nullptr;
    }

    void reset() {
        std::lock_guard lk(mu_);
        offered_.clear();
    }

private:
    ServiceRegistry() = default;
    std::mutex mu_;
    std::condition_variable cv_;
    std::map<ServiceId, std::shared_ptr<std::map<std::string, EventChannel>>> offered_;
};

// ─── Base skeleton ────────────────────────────────────────────────────────────

// Derive from this to publish a service.  Call offerService() in the subclass
// constructor, stopOfferService() in the destructor (or override shutdown()).
class ServiceSkeleton {
public:
    explicit ServiceSkeleton(ServiceId id)
        : id_(std::move(id)),
          events_(std::make_shared<std::map<std::string, EventChannel>>()) {}

    void offerService() {
        ServiceRegistry::instance().offerService(id_, events_);
    }
    void stopOfferService() {
        ServiceRegistry::instance().stopOfferService(id_);
    }

    // Send an event by name.
    template <typename T>
    void sendEvent(std::string_view name, const T& value) {
        auto it = events_->find(std::string(name));
        if (it != events_->end()) it->second.send(std::any(value));
    }

    // Register an event name so proxies can subscribe before the first send.
    void declareEvent(std::string name) {
        events_->try_emplace(std::move(name));
    }

protected:
    ServiceId id_;
    std::shared_ptr<std::map<std::string, EventChannel>> events_;
};

// ─── Base proxy ──────────────────────────────────────────────────────────────

// Find a service and subscribe to its events.
class ServiceProxy {
public:
    static std::optional<ServiceProxy> find(const ServiceId& id,
                                            int timeout_ms = 1000) {
        auto events = ServiceRegistry::instance().findService(id, timeout_ms);
        if (!events) return std::nullopt;
        return ServiceProxy(std::move(events));
    }

    // Subscribe to a typed event.  Callback fires on every sample.
    template <typename T>
    bool subscribe(std::string_view event_name,
                   std::function<void(const T&)> handler) {
        auto it = events_->find(std::string(event_name));
        if (it == events_->end()) return false;
        it->second.addSubscriber([h = std::move(handler)](const std::any& a) {
            if (a.type() == typeid(T)) h(std::any_cast<const T&>(a));
        });
        return true;
    }

private:
    explicit ServiceProxy(std::shared_ptr<std::map<std::string, EventChannel>> e)
        : events_(std::move(e)) {}

    std::shared_ptr<std::map<std::string, EventChannel>> events_;
};

}  // namespace sdv::ara_com
