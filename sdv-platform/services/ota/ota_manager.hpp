// =============================================================================
// services/ota/ota_manager.hpp
//
// Over-the-Air update module implementing the patterns expected in a real
// SDV stack (Uptane-inspired, UNECE R156 aligned):
//
//   * signed manifest, verified BEFORE anything touches a slot
//   * payload integrity via SHA-256 digest pinned in the manifest
//   * hardware-ID compatibility check
//   * anti-rollback: monotonically increasing version counter
//   * A/B slot installation: download+install into the INACTIVE slot,
//     active slot stays untouched until an explicit commit after a
//     successful health check ("first boot OK")
//   * automatic rollback path if the new image fails its health check
//
// State machine:
//   IDLE -> VERIFYING -> INSTALLING -> PENDING_COMMIT -> IDLE (commit)
//                                   \-> IDLE (rollback)
//   any error -> FAILED (terminal until reset)
// =============================================================================
#pragma once

#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace sdv::services::ota {

enum class OtaState : uint8_t {
    Idle,
    Verifying,
    Installing,
    PendingCommit,
    Failed
};

const char* toString(OtaState s);

struct UpdateManifest {
    std::string name;
    std::string hardware_id;
    uint64_t version{0};            // monotonic counter (anti-rollback)
    std::string payload_file;       // relative file name of the image
    std::string payload_sha256;     // hex digest of the payload
    std::string signature;          // HMAC-SHA256 hex over canonical fields

    // Canonical byte string that the signature covers. Field order is fixed;
    // never sign "whatever the parser produced".
    std::vector<uint8_t> canonicalBytes() const;
};

struct OtaResult {
    bool ok{false};
    std::string message;
};

class OtaManager {
public:
    // root_dir layout:
    //   root/slot_a/ , root/slot_b/   -> firmware slots
    //   root/state/active_slot        -> "a" or "b"
    //   root/state/installed_version  -> decimal counter
    OtaManager(std::filesystem::path root_dir,
               std::string hardware_id,
               std::vector<uint8_t> verification_key);

    // Full pipeline: parse + verify manifest, verify payload digest,
    // install into the inactive slot. On success state == PendingCommit.
    OtaResult startUpdate(const std::filesystem::path& manifest_path,
                          const std::filesystem::path& payload_path);

    // Called after the new software passed its post-boot health check.
    // Switches the active slot marker and persists the new version counter.
    OtaResult commit();

    // Called when the health check failed: discard the staged slot.
    OtaResult rollback();

    OtaState state() const { return state_; }
    std::string activeSlot() const;
    uint64_t installedVersion() const;

    // Parses a flat JSON manifest (string and integer values only).
    static std::optional<UpdateManifest> parseManifest(const std::string& json);

private:
    OtaResult fail(const std::string& why);
    OtaResult verifyManifest(const UpdateManifest& m) const;
    OtaResult verifyPayload(const UpdateManifest& m,
                            const std::filesystem::path& payload_path) const;
    OtaResult installToInactiveSlot(const UpdateManifest& m,
                                    const std::filesystem::path& payload_path);

    std::filesystem::path slotPath(const std::string& slot) const;
    std::string inactiveSlot() const;
    void writeStateFile(const std::string& name, const std::string& value) const;
    std::string readStateFile(const std::string& name,
                              const std::string& fallback) const;

    std::filesystem::path root_;
    std::string hardware_id_;
    std::vector<uint8_t> key_;
    OtaState state_{OtaState::Idle};
    UpdateManifest staged_{};
};

}  // namespace sdv::services::ota
