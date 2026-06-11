// =============================================================================
// middleware/e2e/e2e_protection.hpp
//
// End-to-End (E2E) Communication Protection — ISO 26262 Part 7 / AUTOSAR E2E
//
// Why E2E matters:
//   A single bit flip in a vehicle speed CAN frame can make a plausible value
//   (say 28 km/h → 540 km/h) that passes range checks and fools an AEB.
//   E2E wraps every safety-critical sample with a CRC and a sequence counter
//   so the receiver can detect:
//     * Bit errors / frame corruption   (CRC mismatch)
//     * Lost frames / sender silent     (counter gap > 1)
//     * Repeated / stale frames         (counter unchanged)
//     * Wrong source / routing error    (Data ID mismatch)
//
// Profiles implemented (AUTOSAR E2E Library, SWS_E2EXf_00002):
//   Profile 1 — 8-bit CRC, 4-bit counter, 2-byte overhead (ASIL-B capable)
//   Profile 2 — 8-bit CRC, 8-bit counter, 3-byte overhead (ASIL-D capable)
//
// Usage:
//   E2eHeader<Profile1> hdr;
//   hdr.protect(payload_bytes, kSpeedDataId);   // transmit side
//   auto status = hdr.check(payload_bytes, kSpeedDataId);  // receive side
//   if (status != E2eStatus::Ok) { /* degrade signal */ }
//
// Production mapping:
//   AUTOSAR E2EXf transformer wraps this in a SOME/IP PDU transformer.
//   On Classic AUTOSAR it sits in the COM layer.
// =============================================================================
#pragma once

#include <array>
#include <cstdint>
#include <span>

namespace sdv::e2e {

// ─── Status codes ─────────────────────────────────────────────────────────────

enum class E2eStatus : uint8_t {
    Ok,              // CRC correct, counter increment as expected
    WrongCrc,        // Data corrupted or wrong Data ID
    WrongSequence,   // Counter gap > 1 — frame(s) lost
    Repeated,        // Counter unchanged — duplicate or stale frame
    NoNewData,       // Receiver called before any frame arrived
    Error            // General protocol error
};

constexpr const char* toString(E2eStatus s) noexcept {
    switch (s) {
        case E2eStatus::Ok:            return "Ok";
        case E2eStatus::WrongCrc:      return "WrongCrc";
        case E2eStatus::WrongSequence: return "WrongSequence";
        case E2eStatus::Repeated:      return "Repeated";
        case E2eStatus::NoNewData:     return "NoNewData";
        case E2eStatus::Error:         return "Error";
    }
    return "?";
}

// ─── CRC-8 (SAE J1850 / AUTOSAR) ─────────────────────────────────────────────

namespace detail {

// CRC-8/SAE-J1850 polynomial 0x1D, used by AUTOSAR E2E Profile 1 & 2.
constexpr uint8_t kCrcPoly = 0x1D;

inline constexpr std::array<uint8_t, 256> buildCrc8Table() noexcept {
    std::array<uint8_t, 256> tbl{};
    for (uint16_t i = 0; i < 256; ++i) {
        uint8_t crc = static_cast<uint8_t>(i);
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x80u) ? ((crc << 1) ^ kCrcPoly) : (crc << 1);
        tbl[i] = crc;
    }
    return tbl;
}

inline constexpr auto kCrc8Table = buildCrc8Table();

constexpr uint8_t crc8(std::span<const uint8_t> data, uint8_t init = 0xFF) noexcept {
    uint8_t crc = init;
    for (uint8_t b : data)
        crc = kCrc8Table[crc ^ b];
    return crc ^ 0xFF;  // final XOR
}

// Include Data ID bytes in the CRC so routing errors are detected.
constexpr uint8_t crc8WithDataId(uint16_t data_id,
                                  std::span<const uint8_t> payload) noexcept {
    const uint8_t id_bytes[2] = {static_cast<uint8_t>(data_id & 0xFF),
                                  static_cast<uint8_t>(data_id >> 8)};
    uint8_t crc = crc8(std::span(id_bytes), 0xFF);
    return crc8(payload, crc ^ 0xFF);  // continue CRC over payload
}

}  // namespace detail

// ─── Profile 1 (2-byte overhead: CRC-8 + 4-bit counter) ──────────────────────

struct Profile1Header {
    uint8_t crc{0};
    uint8_t counter{0};  // only low 4 bits used

    static constexpr uint8_t kCounterMax = 0x0F;
    static constexpr uint8_t kCounterMask = 0x0F;
};

class E2eProfile1 {
public:
    // Transmit side: fill `header` in-place, return it (attach to the frame).
    Profile1Header protect(uint16_t data_id,
                           std::span<const uint8_t> payload) noexcept {
        Profile1Header hdr;
        hdr.counter = (tx_counter_++ & Profile1Header::kCounterMask);
        // CRC covers: data_id (16-bit LE) + counter nibble + payload
        const uint8_t counter_byte = hdr.counter;
        uint8_t crc = detail::crc8(std::span(&counter_byte, 1), 0xFF);
        crc = detail::crc8WithDataId(data_id, payload) ^ crc;  // chain
        hdr.crc = crc;
        return hdr;
    }

    // Receive side: verify the header against incoming payload.
    E2eStatus check(uint16_t data_id,
                    std::span<const uint8_t> payload,
                    const Profile1Header& hdr) noexcept {
        // ── CRC check ──────────────────────────────────────────────
        const uint8_t counter_byte = hdr.counter & Profile1Header::kCounterMask;
        uint8_t expected = detail::crc8(std::span(&counter_byte, 1), 0xFF);
        expected = detail::crc8WithDataId(data_id, payload) ^ expected;
        if (expected != hdr.crc) return E2eStatus::WrongCrc;

        // ── Counter check ───────────────────────────────────────────
        if (!has_first_) {
            has_first_ = true;
            rx_last_counter_ = hdr.counter & Profile1Header::kCounterMask;
            return E2eStatus::Ok;
        }
        const uint8_t expected_cnt = (rx_last_counter_ + 1) & Profile1Header::kCounterMask;
        const uint8_t rx_cnt = hdr.counter & Profile1Header::kCounterMask;

        if (rx_cnt == rx_last_counter_) return E2eStatus::Repeated;
        if (rx_cnt != expected_cnt)     return E2eStatus::WrongSequence;

        rx_last_counter_ = rx_cnt;
        return E2eStatus::Ok;
    }

    void reset() noexcept { tx_counter_ = 0; has_first_ = false; }

private:
    uint8_t tx_counter_{0};
    uint8_t rx_last_counter_{0};
    bool    has_first_{false};
};

// ─── Profile 2 (3-byte overhead: CRC-8 + 8-bit counter) ──────────────────────

struct Profile2Header {
    uint8_t crc{0};
    uint8_t counter{0};
    uint8_t data_id_nibble{0};  // lower nibble of high byte of Data ID
};

class E2eProfile2 {
public:
    Profile2Header protect(uint16_t data_id,
                           std::span<const uint8_t> payload) noexcept {
        Profile2Header hdr;
        hdr.counter = tx_counter_++;
        hdr.data_id_nibble = static_cast<uint8_t>((data_id >> 8) & 0x0F);
        // CRC over [counter | data_id_nibble | payload]
        const uint8_t prefix[2] = {hdr.counter, hdr.data_id_nibble};
        uint8_t crc = detail::crc8(std::span(prefix));
        crc = detail::crc8(payload, crc ^ 0xFF);
        // Include full data_id in CRC for routing protection
        const uint8_t id[2] = {static_cast<uint8_t>(data_id), static_cast<uint8_t>(data_id >> 8)};
        hdr.crc = detail::crc8(std::span(id), crc ^ 0xFF);
        return hdr;
    }

    E2eStatus check(uint16_t data_id,
                    std::span<const uint8_t> payload,
                    const Profile2Header& hdr) noexcept {
        const uint8_t prefix[2] = {hdr.counter, hdr.data_id_nibble};
        uint8_t expected = detail::crc8(std::span(prefix));
        expected = detail::crc8(payload, expected ^ 0xFF);
        const uint8_t id[2] = {static_cast<uint8_t>(data_id), static_cast<uint8_t>(data_id >> 8)};
        expected = detail::crc8(std::span(id), expected ^ 0xFF);
        if (expected != hdr.crc) return E2eStatus::WrongCrc;

        // Check data_id nibble
        if (hdr.data_id_nibble != static_cast<uint8_t>((data_id >> 8) & 0x0F))
            return E2eStatus::WrongCrc;

        if (!has_first_) {
            has_first_ = true;
            rx_last_ = hdr.counter;
            return E2eStatus::Ok;
        }
        const uint8_t expected_cnt = static_cast<uint8_t>(rx_last_ + 1);
        if (hdr.counter == rx_last_)     return E2eStatus::Repeated;
        if (hdr.counter != expected_cnt) return E2eStatus::WrongSequence;
        rx_last_ = hdr.counter;
        return E2eStatus::Ok;
    }

    void reset() noexcept { tx_counter_ = 0; has_first_ = false; }

private:
    uint8_t tx_counter_{0};
    uint8_t rx_last_{0};
    bool    has_first_{false};
};

// ─── Well-known Data IDs for SDV signals ─────────────────────────────────────

inline constexpr uint16_t kDataIdVehicleSpeed  = 0x0101;
inline constexpr uint16_t kDataIdBrakeRequest  = 0x0201;
inline constexpr uint16_t kDataIdSteeringTorq  = 0x0301;
inline constexpr uint16_t kDataIdLeadVehicle   = 0x0401;

}  // namespace sdv::e2e
