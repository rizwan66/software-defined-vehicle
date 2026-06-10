// =============================================================================
// drivers/can/can_interface.hpp
//
// CAN bus abstraction. Two implementations:
//   * SocketCanInterface — real Linux SocketCAN (can0 / vcan0)
//   * MockCanInterface   — deterministic frames for development & unit tests
//
// Applications and services depend only on ICanInterface, which keeps them
// testable on a developer laptop and portable to real hardware.
// =============================================================================
#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <optional>
#include <string>

namespace sdv::drivers {

struct CanFrame {
    uint32_t id{0};                  // 11-bit standard or 29-bit extended ID
    uint8_t dlc{0};                  // data length code (0..8 for Classic CAN)
    std::array<uint8_t, 8> data{};   // payload
    uint64_t timestamp_ns{0};        // monotonic receive timestamp
};

class ICanInterface {
public:
    virtual ~ICanInterface() = default;

    // Non-blocking read. Returns std::nullopt when no frame is pending.
    virtual std::optional<CanFrame> read() = 0;

    // Returns false on transmit failure (bus-off, buffer full, ...).
    virtual bool write(const CanFrame& frame) = 0;

    virtual bool isOpen() const = 0;
};

// -----------------------------------------------------------------------------
// Mock implementation: synthesizes a realistic vehicle speed signal
// (CAN ID 0x123, bytes 0..1, little-endian, scale 0.01 km/h) plus an
// obstacle distance signal (CAN ID 0x250, scale 0.01 m).
// -----------------------------------------------------------------------------
class MockCanInterface final : public ICanInterface {
public:
    std::optional<CanFrame> read() override;
    bool write(const CanFrame& frame) override { (void)frame; return true; }
    bool isOpen() const override { return true; }

    // Test hooks
    void setSpeedKmh(float v) { speed_kmh_.store(v); }
    void setObstacleDistanceM(float d) { obstacle_m_.store(d); }

private:
    std::atomic<float> speed_kmh_{0.0f};
    std::atomic<float> obstacle_m_{200.0f};
    uint32_t tick_{0};
};

// -----------------------------------------------------------------------------
// SocketCAN implementation (Linux). Use with a real interface ("can0") or a
// virtual one for integration tests:
//   sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
// -----------------------------------------------------------------------------
class SocketCanInterface final : public ICanInterface {
public:
    explicit SocketCanInterface(const std::string& ifname);
    ~SocketCanInterface() override;

    std::optional<CanFrame> read() override;
    bool write(const CanFrame& frame) override;
    bool isOpen() const override { return fd_ >= 0; }

private:
    int fd_{-1};
};

}  // namespace sdv::drivers
