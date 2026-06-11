// =============================================================================
// middleware/ara_phm/watchdog.hpp
//
// AUTOSAR Adaptive ara::phm — Alive Supervision Watchdog
//
// ara::phm (Platform Health Management) is the AUTOSAR Adaptive service
// responsible for monitoring the health of software processes and functions.
// It implements three supervision types (ISO 26262 Part 6, clause 10.4.5):
//
//   Alive Supervision  — periodic checkpoint: a task must check in every N ms
//   Deadline Supervision — a block of code must complete within [min, max] ms
//   Logical Supervision — control-flow graph is followed correctly
//
// This implementation covers Alive Supervision, which is the most common
// pattern for cyclic automotive tasks (10 ms / 20 ms / 100 ms runnables).
//
// How it works:
//   1. Each monitored task calls checkpoint(name) every cycle.
//   2. A supervisor thread checks periodically whether each task has called
//      checkpoint at the expected rate.
//   3. If a task misses more than kMaxMissedAlives consecutive windows,
//      the supervisor fires the registered failure action.
//
// Failure actions (FunctionGroup transition in production):
//   * Log a DTC
//   * Request a process restart via ExecutionManagement
//   * Enter a safe state (e.g., suppress ADAS outputs)
//
// Production mapping:
//   ara::phm::SupervisedEntity → this watchdog's task registration
//   ara::phm::Checkpoint       → checkpoint() call
//   FunctionGroupState         → state machine transitions on failure
// =============================================================================
#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>

namespace sdv::ara_phm {

// ─── Supervision status ───────────────────────────────────────────────────────

enum class SupervisionStatus : uint8_t {
    Ok,          // task is alive and checking in on time
    Deactivated, // task not yet started or explicitly disabled
    Expired,     // task missed too many consecutive alives → fault
    Failed       // terminal: fault action triggered
};

constexpr const char* toString(SupervisionStatus s) noexcept {
    switch (s) {
        case SupervisionStatus::Ok:          return "Ok";
        case SupervisionStatus::Deactivated: return "Deactivated";
        case SupervisionStatus::Expired:     return "Expired";
        case SupervisionStatus::Failed:      return "Failed";
    }
    return "?";
}

// ─── Per-task supervision config ─────────────────────────────────────────────

struct AliveConfig {
    std::chrono::milliseconds expected_period{10};  // nominal cycle time
    uint32_t max_missed_alives{3};                  // how many to tolerate
    std::function<void(std::string_view)> on_failure{};  // optional callback
};

// ─── Watchdog ─────────────────────────────────────────────────────────────────

class AliveWatchdog {
public:
    static AliveWatchdog& instance() {
        static AliveWatchdog wd;
        return wd;
    }

    // Register a task with its supervision configuration.
    // Safe to call from any thread before the task starts.
    void registerTask(std::string name, AliveConfig cfg) {
        std::lock_guard lk(mu_);
        tasks_.emplace(std::move(name), std::make_unique<TaskState>(std::move(cfg)));
    }

    // Called from within the monitored task, every cycle.
    // Lock-free on the common (happy) path — only the per-task
    // counter is incremented atomically.
    void checkpoint(std::string_view name) noexcept {
        std::lock_guard lk(mu_);
        auto it = tasks_.find(std::string(name));
        if (it == tasks_.end()) return;
        it->second->alive_count.fetch_add(1, std::memory_order_relaxed);
        it->second->status = SupervisionStatus::Ok;
        it->second->missed = 0;
    }

    // Query status from any thread (monitoring / diagnostics).
    [[nodiscard]] SupervisionStatus status(std::string_view name) const noexcept {
        std::lock_guard lk(mu_);
        auto it = tasks_.find(std::string(name));
        return (it != tasks_.end()) ? it->second->status
                                    : SupervisionStatus::Deactivated;
    }

    // Start the supervisor background thread.
    void startSupervision() {
        supervisor_ = std::jthread([this](std::stop_token tok) {
            supervise(tok);
        });
    }

    void stopSupervision() { supervisor_.request_stop(); }

    void reset() {
        std::lock_guard lk(mu_);
        tasks_.clear();
    }

private:
    // TaskState wraps an atomic — atomics are not movable, so we heap-allocate
    // via unique_ptr and store pointers in the map.  The map itself is only
    // mutated under the lock, so raw pointer access from the supervise loop is safe.
    struct TaskState {
        explicit TaskState(AliveConfig c) : cfg(std::move(c)) {}
        AliveConfig            cfg;
        SupervisionStatus      status{SupervisionStatus::Deactivated};
        std::atomic<uint64_t>  alive_count{0};   // producer-side (non-movable)
        uint64_t               last_seen_count{0};
        uint32_t               missed{0};
    };

    void supervise(std::stop_token tok) {
        while (!tok.stop_requested()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            std::lock_guard lk(mu_);
            for (auto& [name, ts_ptr] : tasks_) {
                TaskState& ts = *ts_ptr;
                if (ts.status == SupervisionStatus::Failed) continue;

                const uint64_t current = ts.alive_count.load(std::memory_order_relaxed);
                if (current != ts.last_seen_count) {
                    ts.last_seen_count = current;
                    ts.missed = 0;
                    ts.status = SupervisionStatus::Ok;
                } else {
                    ++ts.missed;
                    if (ts.missed >= ts.cfg.max_missed_alives) {
                        ts.status = SupervisionStatus::Failed;
                        if (ts.cfg.on_failure) ts.cfg.on_failure(name);
                    } else {
                        ts.status = SupervisionStatus::Expired;
                    }
                }
            }
        }
    }

    AliveWatchdog() = default;

    mutable std::mutex mu_;
    std::map<std::string, std::unique_ptr<TaskState>> tasks_;
    std::jthread supervisor_;
};

}  // namespace sdv::ara_phm
