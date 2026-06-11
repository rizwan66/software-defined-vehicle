// =============================================================================
// services/ota/ota_pipeline.hpp
//
// OTA Update Pipeline using C++23 std::expected<T, E>
//
// std::expected is the C++23 standard for monadic error handling.
// It encodes either a success value OR an error, without exceptions.
// The monadic operations (.and_then, .transform, .or_else) allow
// sequential pipeline steps to be chained cleanly:
//
//   auto result = parseManifest(json_str)
//     .and_then(verifySignature)
//     .and_then(checkHardwareId)
//     .and_then(enforceAntiRollback)
//     .and_then(verifyPayloadDigest)
//     .transform([](auto& m) { return m.name; });
//
// Compare with the old {bool ok, string message} approach:
//   OtaResult r1 = parseManifest(...);
//   if (!r1.ok) return r1;
//   OtaResult r2 = verifySignature(...);
//   if (!r2.ok) return r2;
//   ... (pyramid of if-chains)
//
// Error type (OtaError):
//   Tagged union distinguishing WHY something failed — the caller can
//   decide whether to retry, rollback, or escalate without parsing strings.
//
// This file wraps the existing OtaManager with a std::expected API, keeping
// backward compatibility with all existing tests while showcasing C++23.
// =============================================================================
#pragma once

#include <expected>
#include <filesystem>
#include <string>
#include <string_view>

#include "services/ota/ota_manager.hpp"

namespace sdv::services::ota::pipeline {

// ─── Typed error enumeration ─────────────────────────────────────────────────

enum class OtaErrorCode : uint8_t {
    ParseFailed,           // manifest JSON malformed
    InvalidSignature,      // HMAC-SHA256 verification failed
    HardwareMismatch,      // manifest targets different hardware_id
    AntiRollback,          // new version ≤ installed version
    PayloadDigestMismatch, // SHA-256 of firmware.bin doesn't match manifest
    IoError,               // file not found, disk full, etc.
    InvalidState,          // startUpdate called while not in Idle state
};

struct OtaError {
    OtaErrorCode code;
    std::string  detail;  // human-readable context (for logging / DTC)
};

// Expected types used throughout the pipeline
template <typename T = void>
using Expected = std::expected<T, OtaError>;

// makeErr returns std::unexpected<OtaError>, which is implicitly convertible
// to Expected<T> for any T — the compiler infers T from the function return type.
// This avoids having to template makeErr on T.
inline std::unexpected<OtaError> makeErr(OtaErrorCode c, std::string d) {
    return std::unexpected(OtaError{c, std::move(d)});
}

// ─── Pipeline wrapper ────────────────────────────────────────────────────────

class OtaPipeline {
public:
    OtaPipeline(std::filesystem::path root_dir,
                std::string hardware_id,
                std::vector<uint8_t> key)
        : mgr_(std::move(root_dir), std::move(hardware_id), std::move(key)) {}

    // Full update pipeline expressed as a monadic chain.
    // Each step only runs if the previous succeeded.
    Expected<std::string> startUpdate(
        const std::filesystem::path& manifest_path,
        const std::filesystem::path& payload_path)
    {
        // Validate files exist first (IO boundary check)
        if (!std::filesystem::exists(manifest_path))
            return makeErr(OtaErrorCode::IoError,
                           "manifest not found: " + manifest_path.string());
        if (!std::filesystem::exists(payload_path))
            return makeErr(OtaErrorCode::IoError,
                           "payload not found: " + payload_path.string());

        // Delegate to OtaManager and convert legacy OtaResult → Expected
        return liftResult(mgr_.startUpdate(manifest_path, payload_path))
            .transform([](const std::string& msg) {
                return "update staged: " + msg;
            });
    }

    // Commit: runs only after a successful startUpdate + health check.
    Expected<std::string> commit() {
        return liftResult(mgr_.commit());
    }

    // Rollback: revert if the health check failed.
    Expected<std::string> rollback() {
        return liftResult(mgr_.rollback());
    }

    // Full validated cycle using monadic chaining — no if-chains.
    Expected<std::string> performUpdate(
        const std::filesystem::path& manifest_path,
        const std::filesystem::path& payload_path,
        bool health_check_passes)
    {
        return startUpdate(manifest_path, payload_path)
            .and_then([&](std::string staged_msg) -> Expected<std::string> {
                if (!health_check_passes)
                    return rollback()
                        .transform([](auto msg) {
                            return "health check failed, rolled back: " + msg;
                        });
                return commit()
                    .transform([&staged_msg](auto commit_msg) {
                        return staged_msg + " | " + commit_msg;
                    });
            });
    }

    [[nodiscard]] OtaState   state()            const { return mgr_.state(); }
    [[nodiscard]] std::string activeSlot()      const { return mgr_.activeSlot(); }
    [[nodiscard]] uint64_t   installedVersion() const { return mgr_.installedVersion(); }

private:
    // Map legacy OtaResult{bool, string} to Expected<string, OtaError>.
    static Expected<std::string> liftResult(const OtaResult& r) {
        if (r.ok) return r.message;
        return translateError(r.message);
    }

    // Best-effort error code extraction from legacy string messages.
    static Expected<std::string> translateError(const std::string& msg) {
        if (msg.find("signature") != std::string::npos)
            return makeErr(OtaErrorCode::InvalidSignature, msg);
        if (msg.find("hardware") != std::string::npos)
            return makeErr(OtaErrorCode::HardwareMismatch, msg);
        if (msg.find("rollback") != std::string::npos || msg.find("version") != std::string::npos)
            return makeErr(OtaErrorCode::AntiRollback, msg);
        if (msg.find("digest") != std::string::npos || msg.find("SHA") != std::string::npos)
            return makeErr(OtaErrorCode::PayloadDigestMismatch, msg);
        if (msg.find("state") != std::string::npos)
            return makeErr(OtaErrorCode::InvalidState, msg);
        return makeErr(OtaErrorCode::IoError, msg);
    }

    OtaManager mgr_;
};

// ─── Error description helper ─────────────────────────────────────────────────

constexpr std::string_view errorName(OtaErrorCode c) noexcept {
    switch (c) {
        case OtaErrorCode::ParseFailed:           return "ParseFailed";
        case OtaErrorCode::InvalidSignature:      return "InvalidSignature";
        case OtaErrorCode::HardwareMismatch:      return "HardwareMismatch";
        case OtaErrorCode::AntiRollback:          return "AntiRollback";
        case OtaErrorCode::PayloadDigestMismatch: return "PayloadDigestMismatch";
        case OtaErrorCode::IoError:               return "IoError";
        case OtaErrorCode::InvalidState:          return "InvalidState";
    }
    return "Unknown";
}

}  // namespace sdv::services::ota::pipeline
