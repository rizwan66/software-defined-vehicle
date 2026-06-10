// =============================================================================
// security/crypto/sha256.hpp
//
// Self-contained SHA-256 (FIPS 180-4). No external dependencies, so the OTA
// module compiles anywhere. In a series-production ECU this would be replaced
// by the HSM/SHE crypto driver or mbedTLS/OpenSSL via a Crypto Service
// Manager (CSM) abstraction — the call sites stay identical.
// =============================================================================
#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace sdv::security {

class Sha256 {
public:
    Sha256() { reset(); }

    void reset() {
        h_ = {0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au,
              0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u};
        buf_len_ = 0;
        total_len_ = 0;
    }

    void update(const uint8_t* data, size_t len) {
        total_len_ += len;
        while (len > 0) {
            const size_t take = std::min(len, kBlockSize - buf_len_);
            std::memcpy(buf_.data() + buf_len_, data, take);
            buf_len_ += take;
            data += take;
            len -= take;
            if (buf_len_ == kBlockSize) {
                processBlock(buf_.data());
                buf_len_ = 0;
            }
        }
    }

    std::array<uint8_t, 32> finish() {
        const uint64_t bit_len = total_len_ * 8;
        const uint8_t pad = 0x80;
        update(&pad, 1);
        total_len_ -= 1;  // padding does not count toward message length
        const uint8_t zero = 0x00;
        while (buf_len_ != 56) {
            update(&zero, 1);
            total_len_ -= 1;
        }
        for (int i = 7; i >= 0; --i) {
            const uint8_t b = static_cast<uint8_t>((bit_len >> (i * 8)) & 0xFF);
            buf_[buf_len_++] = b;
            if (buf_len_ == kBlockSize) {
                processBlock(buf_.data());
                buf_len_ = 0;
            }
        }
        std::array<uint8_t, 32> out{};
        for (int i = 0; i < 8; ++i) {
            out[i * 4 + 0] = static_cast<uint8_t>((h_[i] >> 24) & 0xFF);
            out[i * 4 + 1] = static_cast<uint8_t>((h_[i] >> 16) & 0xFF);
            out[i * 4 + 2] = static_cast<uint8_t>((h_[i] >> 8) & 0xFF);
            out[i * 4 + 3] = static_cast<uint8_t>(h_[i] & 0xFF);
        }
        return out;
    }

    static std::array<uint8_t, 32> digest(const std::vector<uint8_t>& data) {
        Sha256 s;
        if (!data.empty()) s.update(data.data(), data.size());
        return s.finish();
    }

    static std::string toHex(const std::array<uint8_t, 32>& d) {
        static const char* hex = "0123456789abcdef";
        std::string out;
        out.reserve(64);
        for (uint8_t b : d) {
            out.push_back(hex[b >> 4]);
            out.push_back(hex[b & 0x0F]);
        }
        return out;
    }

private:
    static constexpr size_t kBlockSize = 64;

    static uint32_t rotr(uint32_t x, uint32_t n) {
        return (x >> n) | (x << (32 - n));
    }

    void processBlock(const uint8_t* p) {
        static const uint32_t K[64] = {
            0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b,
            0x59f111f1, 0x923f82a4, 0xab1c5ed5, 0xd807aa98, 0x12835b01,
            0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7,
            0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
            0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152,
            0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147,
            0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc,
            0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
            0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819,
            0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116, 0x1e376c08,
            0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f,
            0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
            0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};

        uint32_t w[64];
        for (int i = 0; i < 16; ++i) {
            w[i] = (static_cast<uint32_t>(p[i * 4]) << 24) |
                   (static_cast<uint32_t>(p[i * 4 + 1]) << 16) |
                   (static_cast<uint32_t>(p[i * 4 + 2]) << 8) |
                   static_cast<uint32_t>(p[i * 4 + 3]);
        }
        for (int i = 16; i < 64; ++i) {
            const uint32_t s0 =
                rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
            const uint32_t s1 =
                rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16] + s0 + w[i - 7] + s1;
        }

        uint32_t a = h_[0], b = h_[1], c = h_[2], d = h_[3];
        uint32_t e = h_[4], f = h_[5], g = h_[6], h = h_[7];

        for (int i = 0; i < 64; ++i) {
            const uint32_t S1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
            const uint32_t ch = (e & f) ^ (~e & g);
            const uint32_t t1 = h + S1 + ch + K[i] + w[i];
            const uint32_t S0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
            const uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
            const uint32_t t2 = S0 + maj;
            h = g; g = f; f = e; e = d + t1;
            d = c; c = b; b = a; a = t1 + t2;
        }

        h_[0] += a; h_[1] += b; h_[2] += c; h_[3] += d;
        h_[4] += e; h_[5] += f; h_[6] += g; h_[7] += h;
    }

    std::array<uint32_t, 8> h_{};
    std::array<uint8_t, kBlockSize> buf_{};
    size_t buf_len_{0};
    uint64_t total_len_{0};
};

// ---------------------------------------------------------------------------
// HMAC-SHA256 — used here to sign/verify OTA manifests.
// Production note: real OTA pipelines use asymmetric signatures
// (Ed25519 / ECDSA-P256, X.509 chains per Uptane), so the private key never
// leaves the OEM signing backend. HMAC keeps this reference self-contained
// while exercising the identical verify-before-install control flow.
// ---------------------------------------------------------------------------
inline std::array<uint8_t, 32> hmacSha256(const std::vector<uint8_t>& key,
                                          const std::vector<uint8_t>& msg) {
    constexpr size_t B = 64;
    std::vector<uint8_t> k = key;
    if (k.size() > B) {
        auto d = Sha256::digest(k);
        k.assign(d.begin(), d.end());
    }
    k.resize(B, 0x00);

    std::vector<uint8_t> ipad(B), opad(B);
    for (size_t i = 0; i < B; ++i) {
        ipad[i] = k[i] ^ 0x36;
        opad[i] = k[i] ^ 0x5c;
    }

    Sha256 inner;
    inner.update(ipad.data(), B);
    if (!msg.empty()) inner.update(msg.data(), msg.size());
    const auto inner_digest = inner.finish();

    Sha256 outer;
    outer.update(opad.data(), B);
    outer.update(inner_digest.data(), inner_digest.size());
    return outer.finish();
}

// Constant-time comparison — prevents timing side channels on MAC checks.
inline bool constantTimeEqual(const std::array<uint8_t, 32>& a,
                              const std::array<uint8_t, 32>& b) {
    uint8_t diff = 0;
    for (size_t i = 0; i < 32; ++i) diff |= static_cast<uint8_t>(a[i] ^ b[i]);
    return diff == 0;
}

}  // namespace sdv::security
