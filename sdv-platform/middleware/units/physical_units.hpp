// =============================================================================
// middleware/units/physical_units.hpp
//
// Zero-overhead type-safe physical units for automotive software.
//
// The problem this solves:
//   The Mars Climate Orbiter was lost because one module sent Newton·seconds
//   and another expected pound-force·seconds. In automotive:
//     float speed = 100.0f;          // km/h? m/s? mph?
//     float ttc = distance / speed;  // units wrong → wrong TTC → wrong braking
//
//   With this header, a function accepting Speed<Mps> will NOT compile if you
//   pass Speed<Kmh>. The conversion must be explicit. Zero runtime cost —
//   the wrapper is a single float with no vtable, no heap, no overhead.
//
// Design:
//   Unit<Value, Tag, Num, Den> — strong typedef parameterized by:
//     * Value: underlying numeric type (float, double)
//     * Tag:   unique empty struct that names the physical quantity
//     * Num/Den: SI ratio relative to the base unit
//
// Units defined:
//   Speed:    Kmh, Mps, Mph
//   Distance: Meters, Kilometers, Millimeters
//   Time:     Seconds, Milliseconds, Nanoseconds
//   Force:    Newtons, Kilonewtons
//   Angle:    Radians, Degrees
//   Accel:    MpsSquared, Gs
//
// Usage:
//   void setSpeed(Speed<Kmh> s);
//   setSpeed(Speed<Kmh>{120.0f});           // OK
//   setSpeed(Speed<Mps>{33.3f});            // compile error — explicit cast needed
//   setSpeed(unit_cast<Kmh>(Speed<Mps>{33.3f}));  // explicit, compiles
//
// Operators:
//   +, -, *, / within same unit type
//   Comparison operators
//   to<OtherUnit>() conversion method
// =============================================================================
#pragma once

#include <cmath>
#include <concepts>
#include <cstdint>
#include <ratio>
#include <type_traits>

namespace sdv::units {

// ─── Core unit template ───────────────────────────────────────────────────────

// Ratio of this unit relative to the SI base unit.
// e.g. km/h → base m/s: Num=1, Den=36  (1 km/h = 1/3.6 m/s = 10/36 m/s)

template <typename ValueT, typename Tag,
          std::intmax_t Num = 1, std::intmax_t Den = 1>
struct Unit {
    using value_type = ValueT;
    using tag_type   = Tag;

    static_assert(Den != 0, "Denominator must be non-zero");
    static_assert(std::is_arithmetic_v<ValueT>);

    ValueT value{};

    constexpr Unit() = default;
    explicit constexpr Unit(ValueT v) noexcept : value(v) {}

    // Arithmetic within same unit type
    constexpr Unit operator+(Unit rhs) const noexcept { return Unit{value + rhs.value}; }
    constexpr Unit operator-(Unit rhs) const noexcept { return Unit{value - rhs.value}; }
    constexpr Unit operator*(ValueT s)  const noexcept { return Unit{value * s}; }
    constexpr Unit operator/(ValueT s)  const noexcept { return Unit{value / s}; }
    constexpr Unit operator-()          const noexcept { return Unit{-value}; }

    constexpr Unit& operator+=(Unit rhs) noexcept { value += rhs.value; return *this; }
    constexpr Unit& operator-=(Unit rhs) noexcept { value -= rhs.value; return *this; }

    // Comparison
    constexpr bool operator==(Unit rhs) const noexcept { return value == rhs.value; }
    constexpr bool operator!=(Unit rhs) const noexcept { return value != rhs.value; }
    constexpr bool operator< (Unit rhs) const noexcept { return value <  rhs.value; }
    constexpr bool operator<=(Unit rhs) const noexcept { return value <= rhs.value; }
    constexpr bool operator> (Unit rhs) const noexcept { return value >  rhs.value; }
    constexpr bool operator>=(Unit rhs) const noexcept { return value >= rhs.value; }

    constexpr Unit abs() const noexcept { return Unit{std::abs(value)}; }

    // Ratio constants — used by unit_cast
    static constexpr double kToBase = static_cast<double>(Num) / static_cast<double>(Den);
};

// ─── Unit conversion ──────────────────────────────────────────────────────────

// Convert between two units that share the same Tag (same physical quantity).
// Fails to compile if tags differ — you cannot cast Speed to Distance.
template <typename ToUnit, typename FromUnit>
    requires std::is_same_v<typename ToUnit::tag_type, typename FromUnit::tag_type>
constexpr ToUnit unit_cast(FromUnit from) noexcept {
    // from_value * (from.Num/from.Den) = base_value
    // to_value   = base_value / (to.Num/to.Den)
    //            = from_value * (from.Num/from.Den) * (to.Den/to.Num)
    constexpr double factor = FromUnit::kToBase / ToUnit::kToBase;
    return ToUnit{static_cast<typename ToUnit::value_type>(from.value * factor)};
}

// ─── Speed ────────────────────────────────────────────────────────────────────

struct SpeedTag {};

// Base: m/s. Ratios express how many m/s one unit equals.
using Mps = Unit<float, SpeedTag, 1,   1>;     // 1 m/s  → 1 m/s
using Kmh = Unit<float, SpeedTag, 10,  36>;    // 1 km/h → 10/36 m/s
using Mph = Unit<float, SpeedTag, 1397, 3125>; // 1 mph  ≈ 0.44704 m/s

// Convenient named conversions
[[nodiscard]] constexpr Mps  to_mps(Kmh v) noexcept { return unit_cast<Mps>(v); }
[[nodiscard]] constexpr Kmh  to_kmh(Mps v) noexcept { return unit_cast<Kmh>(v); }

// ─── Distance ─────────────────────────────────────────────────────────────────

struct DistanceTag {};

using Meters      = Unit<float, DistanceTag, 1,    1>;
using Kilometers  = Unit<float, DistanceTag, 1000, 1>;
using Millimeters = Unit<float, DistanceTag, 1,    1000>;

// ─── Time ────────────────────────────────────────────────────────────────────

struct TimeTag {};

using Seconds      = Unit<float, TimeTag, 1,          1>;
using Milliseconds = Unit<float, TimeTag, 1,          1000>;
using Nanoseconds  = Unit<float, TimeTag, 1,          1'000'000'000>;

// ─── Acceleration ─────────────────────────────────────────────────────────────

struct AccelTag {};

using MpsSquared = Unit<float, AccelTag, 1, 1>;
using Gs         = Unit<float, AccelTag, 981, 100>;  // 1 g ≈ 9.81 m/s²

[[nodiscard]] constexpr MpsSquared to_mps2(Gs g) noexcept { return unit_cast<MpsSquared>(g); }

// ─── Angle ────────────────────────────────────────────────────────────────────

struct AngleTag {};

// Base: radians.
using Radians = Unit<float, AngleTag, 1, 1>;
// Degrees: π/180 radians ≈ 314159/18000000  (use 1/57295 as approx)
using Degrees = Unit<float, AngleTag, 1, 5730>;  // 1° ≈ 1/57.30 rad

// ─── Force / Torque ──────────────────────────────────────────────────────────

struct ForceTag {};
using Newtons     = Unit<float, ForceTag, 1,    1>;
using Kilonewtons = Unit<float, ForceTag, 1000, 1>;

// ─── Named literals ───────────────────────────────────────────────────────────
// Allows writing: 120.0_kmh, 33.3_mps, 0.3_g

namespace literals {
constexpr Kmh       operator""_kmh(long double v) noexcept { return Kmh{static_cast<float>(v)}; }
constexpr Mps       operator""_mps(long double v) noexcept { return Mps{static_cast<float>(v)}; }
constexpr Meters    operator""_m  (long double v) noexcept { return Meters{static_cast<float>(v)}; }
constexpr Seconds   operator""_s  (long double v) noexcept { return Seconds{static_cast<float>(v)}; }
constexpr Gs        operator""_g  (long double v) noexcept { return Gs{static_cast<float>(v)}; }
constexpr Radians   operator""_rad(long double v) noexcept { return Radians{static_cast<float>(v)}; }
constexpr Newtons   operator""_N  (long double v) noexcept { return Newtons{static_cast<float>(v)}; }

// Integer overloads
constexpr Kmh       operator""_kmh(unsigned long long v) noexcept { return Kmh{static_cast<float>(v)}; }
constexpr Mps       operator""_mps(unsigned long long v) noexcept { return Mps{static_cast<float>(v)}; }
constexpr Meters    operator""_m  (unsigned long long v) noexcept { return Meters{static_cast<float>(v)}; }
constexpr Seconds   operator""_s  (unsigned long long v) noexcept { return Seconds{static_cast<float>(v)}; }
}  // namespace literals

// ─── Physics helpers ─────────────────────────────────────────────────────────

// Time-to-collision: TTC = distance / closing_speed (both in SI base units)
[[nodiscard]] constexpr Seconds ttc(Meters dist, Mps closing_speed) noexcept {
    return Seconds{closing_speed.value > 1e-4f ? dist.value / closing_speed.value : 999.0f};
}

}  // namespace sdv::units
