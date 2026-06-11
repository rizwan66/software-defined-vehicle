// =============================================================================
// middleware/logging/sdv_log.hpp
//
// Structured, thread-safe logging for the SDV platform.
//
// Design:
//   - Five severity levels: Debug < Info < Warn < Error < Fatal
//   - Each log line: [<elapsed_ms>] [LEVEL] [Component] message
//   - Global minimum level filter (set at startup or test time)
//   - Pluggable sink (default: stderr); tests can redirect to a buffer
//   - Macros carry __FILE__/__LINE__ with zero overhead when filtered
//
// In a production AUTOSAR Adaptive stack this maps to ara::log.
// =============================================================================
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <mutex>
#include <string>
#include <string_view>

namespace sdv::log {

// ─── Severity levels ─────────────────────────────────────────────────────────

enum class Level : uint8_t { Debug = 0, Info, Warn, Error, Fatal };

constexpr std::string_view levelTag(Level l) noexcept {
    switch (l) {
        case Level::Debug: return "DEBUG";
        case Level::Info:  return "INFO ";
        case Level::Warn:  return "WARN ";
        case Level::Error: return "ERROR";
        case Level::Fatal: return "FATAL";
    }
    return "?????";
}

// ─── Logger singleton ────────────────────────────────────────────────────────

using Sink = std::function<void(std::string_view line)>;

class Logger {
public:
    static Logger& instance() {
        static Logger inst;
        return inst;
    }

    void setMinLevel(Level l) { min_level_.store(l, std::memory_order_relaxed); }
    Level minLevel() const   { return min_level_.load(std::memory_order_relaxed); }

    void setSink(Sink s) {
        std::lock_guard<std::mutex> lk(sink_mutex_);
        sink_ = std::move(s);
    }

    // Core write — called by the macros below.
    void write(Level level, std::string_view component,
               std::string_view file, int line, std::string_view msg) {
        if (level < min_level_.load(std::memory_order_relaxed)) return;

        const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_).count();

        char buf[512];
        int n = std::snprintf(buf, sizeof(buf),
            "[%8lld ms] [%s] [%-20s] %s  (%s:%d)\n",
            static_cast<long long>(elapsed_ms),
            levelTag(level).data(),
            component.data(), msg.data(),
            file.data(), line);

        std::string_view view(buf, (n > 0) ? static_cast<size_t>(n) : 0);

        std::lock_guard<std::mutex> lk(sink_mutex_);
        sink_(view);
    }

private:
    Logger() : start_(std::chrono::steady_clock::now()) {
        sink_ = [](std::string_view line) {
            std::fwrite(line.data(), 1, line.size(), stderr);
        };
    }

    std::chrono::steady_clock::time_point start_;
    std::atomic<Level> min_level_{Level::Info};
    std::mutex sink_mutex_;
    Sink sink_;
};

// ─── Helper (avoids repeated strlen in hot paths) ─────────────────────────────

inline void _write(Level l, std::string_view comp,
                   std::string_view file, int line, const std::string& msg) {
    Logger::instance().write(l, comp, file, line, msg);
}

}  // namespace sdv::log

// ─── Convenience macros ───────────────────────────────────────────────────────

// Usage: SDV_LOG_INFO("VehicleSpeedService", "speed {} km/h", kmh);
// (printf-style formatting via snprintf)

// C++20 __VA_OPT__ replaces the GNU ##__VA_ARGS__ extension.
// Usage: SDV_LOG_INFO("Component", "value is %d", val);
//        SDV_LOG_INFO("Component", "no args message");

#define SDV_LOG_DEBUG(comp, fmt, ...) \
    do { \
        if (::sdv::log::Logger::instance().minLevel() <= ::sdv::log::Level::Debug) { \
            char _buf[256]; \
            std::snprintf(_buf, sizeof(_buf), fmt __VA_OPT__(,) __VA_ARGS__); \
            ::sdv::log::_write(::sdv::log::Level::Debug, comp, __FILE__, __LINE__, _buf); \
        } \
    } while(0)

#define SDV_LOG_INFO(comp, fmt, ...) \
    do { \
        char _buf[256]; \
        std::snprintf(_buf, sizeof(_buf), fmt __VA_OPT__(,) __VA_ARGS__); \
        ::sdv::log::_write(::sdv::log::Level::Info, comp, __FILE__, __LINE__, _buf); \
    } while(0)

#define SDV_LOG_WARN(comp, fmt, ...) \
    do { \
        char _buf[256]; \
        std::snprintf(_buf, sizeof(_buf), fmt __VA_OPT__(,) __VA_ARGS__); \
        ::sdv::log::_write(::sdv::log::Level::Warn, comp, __FILE__, __LINE__, _buf); \
    } while(0)

#define SDV_LOG_ERROR(comp, fmt, ...) \
    do { \
        char _buf[256]; \
        std::snprintf(_buf, sizeof(_buf), fmt __VA_OPT__(,) __VA_ARGS__); \
        ::sdv::log::_write(::sdv::log::Level::Error, comp, __FILE__, __LINE__, _buf); \
    } while(0)

#define SDV_LOG_FATAL(comp, fmt, ...) \
    do { \
        char _buf[256]; \
        std::snprintf(_buf, sizeof(_buf), fmt __VA_OPT__(,) __VA_ARGS__); \
        ::sdv::log::_write(::sdv::log::Level::Fatal, comp, __FILE__, __LINE__, _buf); \
    } while(0)
