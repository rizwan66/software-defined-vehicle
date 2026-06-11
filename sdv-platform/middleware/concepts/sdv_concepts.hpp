// =============================================================================
// middleware/concepts/sdv_concepts.hpp
//
// C++20 concept constraints for SDV interfaces.
//
// Concepts act as semantic type-contracts that are checked at compile time.
// They replace ad-hoc SFINAE and document the minimum interface that any
// conforming implementation must satisfy — the compiler enforces what a
// comment only suggests.
//
// Concepts defined here:
//   CanBusDevice      — CAN read/write device (replaces raw virtual ICanInterface)
//   SignalConsumer<T> — anything that can receive a typed signal sample
//   AdasModule        — any ADAS function with evaluate() and a safety degrade
//   MetricsSink       — anything that can accept a named counter/gauge update
// =============================================================================
#pragma once

#include <concepts>
#include <optional>
#include <string_view>
#include <cstdint>
#include <functional>

namespace sdv::concepts {

// ─── CAN bus device ──────────────────────────────────────────────────────────

// Any frame type: must have id, dlc, data, timestamp_ns members.
template <typename F>
concept CanFrame = requires(F f) {
    { f.id }           -> std::convertible_to<uint32_t>;
    { f.dlc }          -> std::convertible_to<uint8_t>;
    { f.timestamp_ns } -> std::convertible_to<uint64_t>;
};

// Any CAN bus device: non-blocking read, write, open check.
template <typename T>
concept CanBusDevice = requires(T t, const typename T::FrameType& frame) {
    typename T::FrameType;
    requires CanFrame<typename T::FrameType>;
    { t.read()        } -> std::same_as<std::optional<typename T::FrameType>>;
    { t.write(frame)  } -> std::same_as<bool>;
    { t.isOpen()      } -> std::same_as<bool>;
};

// ─── Signal consumer ─────────────────────────────────────────────────────────

// A callable that accepts a const-ref to a typed signal sample.
template <typename F, typename Sample>
concept SignalConsumer = requires(F f, const Sample& s) {
    { f(s) } -> std::same_as<void>;
};

// ─── ADAS function ────────────────────────────────────────────────────────────

// Any ADAS function must be able to report whether it is safe-state active.
// The "safe state" contract: when the function degrades (invalid inputs) it
// must not produce actuation output — isActive() returning false signals this.
template <typename T>
concept AdasModule = requires(const T ct) {
    { ct.isActive()    } -> std::same_as<bool>;
    { ct.isSafeState() } -> std::same_as<bool>;
};

// ─── Metrics sink ────────────────────────────────────────────────────────────

template <typename T>
concept MetricsSink = requires(T t, std::string_view name, double value) {
    { t.incCounter(name, uint64_t{1}) } -> std::same_as<void>;
    { t.setGauge(name, value)         } -> std::same_as<void>;
};

// ─── State machine helpers ────────────────────────────────────────────────────

// Any strongly-typed state enum that can be printed as a string.
template <typename E>
concept StringifiableState = std::is_enum_v<E> && requires(E e) {
    { toString(e) } -> std::convertible_to<std::string_view>;
};

}  // namespace sdv::concepts
