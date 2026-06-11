// =============================================================================
// middleware/metrics/metrics_collector.hpp
//
// Prometheus-style in-process metrics for SDV platform observability.
//
// Metric types (matching Prometheus conventions):
//   Counter — monotonically increasing integer (messages received, errors, ...)
//   Gauge   — arbitrary double that can rise or fall (speed km/h, temp, ...)
//   Histogram — bucket-based latency/value distribution
//
// Design:
//   * All metric operations are lock-free (atomics) for counters / gauges.
//   * A single global MetricsRegistry holds all named metrics.
//   * Snapshot() produces a Prometheus text-exposition-format string,
//     ready to serve on an HTTP /metrics endpoint.
//   * The registry is safe to query from any thread at any time.
//
// Production mapping:
//   This in-process registry maps to Prometheus C++ client library or
//   AUTOSAR Adaptive ara::phm / DLT log statistics in a real ECU.
// =============================================================================
#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace sdv::metrics {

// ─── Counter ─────────────────────────────────────────────────────────────────

class Counter {
public:
    explicit Counter(std::string help = {}) : help_(std::move(help)) {}

    void inc(uint64_t n = 1) noexcept {
        value_.fetch_add(n, std::memory_order_relaxed);
    }
    [[nodiscard]] uint64_t get() const noexcept {
        return value_.load(std::memory_order_relaxed);
    }
    [[nodiscard]] const std::string& help() const noexcept { return help_; }

private:
    std::atomic<uint64_t> value_{0};
    std::string help_;
};

// ─── Gauge ───────────────────────────────────────────────────────────────────

class Gauge {
public:
    explicit Gauge(std::string help = {}) : help_(std::move(help)) {}

    void set(double v) noexcept {
        // Store double atomically via a union trick (IEEE 754 / lock-free on x86)
        std::atomic_store_explicit(&value_, v, std::memory_order_relaxed);
    }
    void inc(double n = 1.0) noexcept { set(get() + n); }
    void dec(double n = 1.0) noexcept { set(get() - n); }

    [[nodiscard]] double get() const noexcept {
        return std::atomic_load_explicit(&value_, std::memory_order_relaxed);
    }
    [[nodiscard]] const std::string& help() const noexcept { return help_; }

private:
    std::atomic<double> value_{0.0};
    std::string help_;
};

// ─── Histogram ───────────────────────────────────────────────────────────────

// Fixed upper-bounds buckets; values beyond the last bucket go to +Inf.
template <size_t N>
class Histogram {
public:
    explicit Histogram(std::array<double, N> bounds, std::string help = {})
        : bounds_(bounds), help_(std::move(help)) {
        bucket_counts_.fill(0);
    }

    void observe(double value) noexcept {
        sum_.fetch_add(static_cast<uint64_t>(value * 1000),
                       std::memory_order_relaxed);  // store as micro-units
        count_.fetch_add(1, std::memory_order_relaxed);
        for (size_t i = 0; i < N; ++i) {
            if (value <= bounds_[i]) {
                bucket_counts_[i].fetch_add(1, std::memory_order_relaxed);
                return;
            }
        }
        inf_count_.fetch_add(1, std::memory_order_relaxed);
    }

    struct Snapshot {
        std::array<uint64_t, N> buckets;
        uint64_t inf_count;
        double   sum;
        uint64_t count;
        std::array<double, N> bounds;
    };

    [[nodiscard]] Snapshot snapshot() const noexcept {
        Snapshot s;
        for (size_t i = 0; i < N; ++i)
            s.buckets[i] = bucket_counts_[i].load(std::memory_order_relaxed);
        s.inf_count = inf_count_.load(std::memory_order_relaxed);
        s.sum       = sum_.load(std::memory_order_relaxed) / 1000.0;
        s.count     = count_.load(std::memory_order_relaxed);
        s.bounds    = bounds_;
        return s;
    }
    [[nodiscard]] const std::string& help() const noexcept { return help_; }

private:
    std::array<double, N> bounds_;
    std::array<std::atomic<uint64_t>, N> bucket_counts_{};
    std::atomic<uint64_t> inf_count_{0};
    std::atomic<uint64_t> sum_{0};
    std::atomic<uint64_t> count_{0};
    std::string help_;
};

// ─── Registry ────────────────────────────────────────────────────────────────

class MetricsRegistry {
public:
    static MetricsRegistry& instance() {
        static MetricsRegistry reg;
        return reg;
    }

    Counter& counter(std::string_view name, std::string help = {}) {
        std::lock_guard lk(mu_);
        auto [it, _] = counters_.try_emplace(std::string(name), std::move(help));
        return it->second;
    }

    Gauge& gauge(std::string_view name, std::string help = {}) {
        std::lock_guard lk(mu_);
        auto [it, _] = gauges_.try_emplace(std::string(name), std::move(help));
        return it->second;
    }

    // Produce Prometheus text exposition format.
    [[nodiscard]] std::string snapshot() const {
        std::ostringstream out;
        std::lock_guard lk(mu_);
        for (const auto& [name, c] : counters_) {
            if (!c.help().empty())
                out << "# HELP " << name << ' ' << c.help() << '\n';
            out << "# TYPE " << name << " counter\n";
            out << name << ' ' << c.get() << '\n';
        }
        for (const auto& [name, g] : gauges_) {
            if (!g.help().empty())
                out << "# HELP " << name << ' ' << g.help() << '\n';
            out << "# TYPE " << name << " gauge\n";
            out << name << ' ' << g.get() << '\n';
        }
        return out.str();
    }

    void reset() {
        std::lock_guard lk(mu_);
        counters_.clear();
        gauges_.clear();
    }

private:
    MetricsRegistry() = default;
    mutable std::mutex mu_;
    std::map<std::string, Counter> counters_;
    std::map<std::string, Gauge>   gauges_;
};

// ─── Convenience free functions ───────────────────────────────────────────────

inline Counter& counter(std::string_view name, std::string help = {}) {
    return MetricsRegistry::instance().counter(name, std::move(help));
}
inline Gauge& gauge(std::string_view name, std::string help = {}) {
    return MetricsRegistry::instance().gauge(name, std::move(help));
}

}  // namespace sdv::metrics
