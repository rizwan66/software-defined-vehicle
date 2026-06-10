// =============================================================================
// drivers/can/can_interface.cpp
// =============================================================================
#include "drivers/can/can_interface.hpp"

#include <chrono>
#include <cmath>
#include <cstring>

#ifdef __linux__
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace sdv::drivers {

namespace {
uint64_t nowNs() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}
}  // namespace

// ---------------------------------------------------------------------------
// MockCanInterface
// ---------------------------------------------------------------------------
std::optional<CanFrame> MockCanInterface::read() {
    // Alternate between the speed frame (0x123) and obstacle frame (0x250)
    // to emulate a periodic bus schedule.
    CanFrame f;
    f.timestamp_ns = nowNs();
    if (tick_++ % 2 == 0) {
        const float kmh = speed_kmh_.load();
        const auto raw = static_cast<uint16_t>(
            std::min(65535L, std::lround(kmh / 0.01f)));
        f.id = 0x123;
        f.dlc = 2;
        f.data[0] = static_cast<uint8_t>(raw & 0xFF);         // LSB
        f.data[1] = static_cast<uint8_t>((raw >> 8) & 0xFF);  // MSB
    } else {
        const float m = obstacle_m_.load();
        const auto raw = static_cast<uint16_t>(
            std::min(65535L, std::lround(m / 0.01f)));
        f.id = 0x250;
        f.dlc = 2;
        f.data[0] = static_cast<uint8_t>(raw & 0xFF);
        f.data[1] = static_cast<uint8_t>((raw >> 8) & 0xFF);
    }
    return f;
}

// ---------------------------------------------------------------------------
// SocketCanInterface
// ---------------------------------------------------------------------------
#ifdef __linux__
SocketCanInterface::SocketCanInterface(const std::string& ifname) {
    fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0) return;

    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
    if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
        ::close(fd_);
        fd_ = -1;
        return;
    }

    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(fd_);
        fd_ = -1;
        return;
    }

    // Non-blocking reads so the service main loop is never stalled by the bus.
    const int flags = ::fcntl(fd_, F_GETFL, 0);
    ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
}

SocketCanInterface::~SocketCanInterface() {
    if (fd_ >= 0) ::close(fd_);
}

std::optional<CanFrame> SocketCanInterface::read() {
    if (fd_ < 0) return std::nullopt;
    struct can_frame raw {};
    const ssize_t n = ::read(fd_, &raw, sizeof(raw));
    if (n != static_cast<ssize_t>(sizeof(raw))) return std::nullopt;

    CanFrame f;
    f.id = raw.can_id & CAN_EFF_MASK;
    f.dlc = raw.can_dlc;
    std::memcpy(f.data.data(), raw.data, raw.can_dlc);
    f.timestamp_ns = nowNs();
    return f;
}

bool SocketCanInterface::write(const CanFrame& frame) {
    if (fd_ < 0) return false;
    struct can_frame raw {};
    raw.can_id = frame.id;
    raw.can_dlc = frame.dlc;
    std::memcpy(raw.data, frame.data.data(), frame.dlc);
    return ::write(fd_, &raw, sizeof(raw)) == static_cast<ssize_t>(sizeof(raw));
}
#else
// Non-Linux stub so the project still compiles on other hosts.
SocketCanInterface::SocketCanInterface(const std::string&) {}
SocketCanInterface::~SocketCanInterface() = default;
std::optional<CanFrame> SocketCanInterface::read() { return std::nullopt; }
bool SocketCanInterface::write(const CanFrame&) { return false; }
#endif

}  // namespace sdv::drivers
