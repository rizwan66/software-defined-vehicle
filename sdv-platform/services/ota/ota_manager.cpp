// =============================================================================
// services/ota/ota_manager.cpp
// =============================================================================
#include "ota_manager.hpp"

#include <fstream>
#include <sstream>

#include "security/crypto/sha256.hpp"

namespace fs = std::filesystem;

namespace sdv::services::ota {

const char* toString(OtaState s) {
    switch (s) {
        case OtaState::Idle:          return "IDLE";
        case OtaState::Verifying:     return "VERIFYING";
        case OtaState::Installing:    return "INSTALLING";
        case OtaState::PendingCommit: return "PENDING_COMMIT";
        case OtaState::Failed:        return "FAILED";
    }
    return "?";
}

// ---------------------------------------------------------------------------
// Manifest
// ---------------------------------------------------------------------------
std::vector<uint8_t> UpdateManifest::canonicalBytes() const {
    // Fixed field order, '\n'-separated. The signature covers exactly this.
    std::ostringstream os;
    os << "name=" << name << '\n'
       << "hardware_id=" << hardware_id << '\n'
       << "version=" << version << '\n'
       << "payload_file=" << payload_file << '\n'
       << "payload_sha256=" << payload_sha256 << '\n';
    const std::string s = os.str();
    return {s.begin(), s.end()};
}

// Minimal flat-JSON parser: {"key": "value", "key2": 123}
// Deliberately strict and tiny — no nesting, no arrays, no escapes beyond \".
std::optional<UpdateManifest> OtaManager::parseManifest(const std::string& json) {
    std::map<std::string, std::string> kv;
    size_t i = 0;
    auto skipWs = [&] { while (i < json.size() && std::isspace(static_cast<unsigned char>(json[i]))) ++i; };
    auto readString = [&]() -> std::optional<std::string> {
        if (i >= json.size() || json[i] != '"') return std::nullopt;
        ++i;
        std::string out;
        while (i < json.size() && json[i] != '"') {
            if (json[i] == '\\' && i + 1 < json.size() && json[i + 1] == '"') {
                out.push_back('"');
                i += 2;
            } else {
                out.push_back(json[i++]);
            }
        }
        if (i >= json.size()) return std::nullopt;
        ++i;  // closing quote
        return out;
    };

    skipWs();
    if (i >= json.size() || json[i] != '{') return std::nullopt;
    ++i;
    while (true) {
        skipWs();
        if (i < json.size() && json[i] == '}') break;
        auto key = readString();
        if (!key) return std::nullopt;
        skipWs();
        if (i >= json.size() || json[i] != ':') return std::nullopt;
        ++i;
        skipWs();
        std::string value;
        if (i < json.size() && json[i] == '"') {
            auto v = readString();
            if (!v) return std::nullopt;
            value = *v;
        } else {  // bare number
            while (i < json.size() && (std::isdigit(static_cast<unsigned char>(json[i])))) {
                value.push_back(json[i++]);
            }
            if (value.empty()) return std::nullopt;
        }
        kv[*key] = value;
        skipWs();
        if (i < json.size() && json[i] == ',') { ++i; continue; }
        if (i < json.size() && json[i] == '}') break;
        return std::nullopt;
    }

    UpdateManifest m;
    try {
        m.name = kv.at("name");
        m.hardware_id = kv.at("hardware_id");
        m.version = std::stoull(kv.at("version"));
        m.payload_file = kv.at("payload_file");
        m.payload_sha256 = kv.at("payload_sha256");
        m.signature = kv.at("signature");
    } catch (...) {
        return std::nullopt;  // missing field or bad number
    }
    return m;
}

// ---------------------------------------------------------------------------
// OtaManager
// ---------------------------------------------------------------------------
OtaManager::OtaManager(fs::path root_dir, std::string hardware_id,
                       std::vector<uint8_t> verification_key)
    : root_(std::move(root_dir)),
      hardware_id_(std::move(hardware_id)),
      key_(std::move(verification_key)) {
    fs::create_directories(root_ / "slot_a");
    fs::create_directories(root_ / "slot_b");
    fs::create_directories(root_ / "state");
    if (!fs::exists(root_ / "state" / "active_slot")) {
        writeStateFile("active_slot", "a");
    }
    if (!fs::exists(root_ / "state" / "installed_version")) {
        writeStateFile("installed_version", "0");
    }
}

OtaResult OtaManager::startUpdate(const fs::path& manifest_path,
                                  const fs::path& payload_path) {
    if (state_ != OtaState::Idle) {
        return {false, std::string("update already in progress (state=") +
                           toString(state_) + ")"};
    }
    state_ = OtaState::Verifying;

    std::ifstream mf(manifest_path);
    if (!mf) return fail("cannot open manifest: " + manifest_path.string());
    std::stringstream ss;
    ss << mf.rdbuf();

    auto manifest = parseManifest(ss.str());
    if (!manifest) return fail("manifest parse error");

    if (auto r = verifyManifest(*manifest); !r.ok) return fail(r.message);
    if (auto r = verifyPayload(*manifest, payload_path); !r.ok) return fail(r.message);

    state_ = OtaState::Installing;
    if (auto r = installToInactiveSlot(*manifest, payload_path); !r.ok) {
        return fail(r.message);
    }

    staged_ = *manifest;
    state_ = OtaState::PendingCommit;
    return {true, "staged version " + std::to_string(manifest->version) +
                      " into slot_" + inactiveSlot() + ", awaiting health check"};
}

OtaResult OtaManager::verifyManifest(const UpdateManifest& m) const {
    // 1. Signature FIRST — nothing else in the manifest is trusted before this.
    std::array<uint8_t, 32> expected{};
    {
        const auto mac = security::hmacSha256(key_, m.canonicalBytes());
        expected = mac;
    }
    if (m.signature.size() != 64) return {false, "signature: bad length"};
    std::array<uint8_t, 32> provided{};
    for (size_t i = 0; i < 32; ++i) {
        auto nibble = [](char c) -> int {
            if (c >= '0' && c <= '9') return c - '0';
            if (c >= 'a' && c <= 'f') return c - 'a' + 10;
            if (c >= 'A' && c <= 'F') return c - 'A' + 10;
            return -1;
        };
        const int hi = nibble(m.signature[i * 2]);
        const int lo = nibble(m.signature[i * 2 + 1]);
        if (hi < 0 || lo < 0) return {false, "signature: not hex"};
        provided[i] = static_cast<uint8_t>((hi << 4) | lo);
    }
    if (!security::constantTimeEqual(expected, provided)) {
        return {false, "signature verification FAILED — rejecting package"};
    }

    // 2. Hardware compatibility.
    if (m.hardware_id != hardware_id_) {
        return {false, "hardware mismatch: package for '" + m.hardware_id +
                           "', this ECU is '" + hardware_id_ + "'"};
    }

    // 3. Anti-rollback.
    if (m.version <= installedVersion()) {
        return {false, "anti-rollback: package version " +
                           std::to_string(m.version) +
                           " <= installed version " +
                           std::to_string(installedVersion())};
    }
    return {true, "manifest ok"};
}

OtaResult OtaManager::verifyPayload(const UpdateManifest& m,
                                    const fs::path& payload_path) const {
    std::ifstream f(payload_path, std::ios::binary);
    if (!f) return {false, "cannot open payload: " + payload_path.string()};

    security::Sha256 sha;
    std::vector<char> buf(64 * 1024);  // streamed — images can be gigabytes
    while (f) {
        f.read(buf.data(), static_cast<std::streamsize>(buf.size()));
        const auto n = f.gcount();
        if (n > 0) sha.update(reinterpret_cast<const uint8_t*>(buf.data()),
                              static_cast<size_t>(n));
    }
    const std::string actual = security::Sha256::toHex(sha.finish());
    if (actual != m.payload_sha256) {
        return {false, "payload digest mismatch (corrupt or tampered download)"};
    }
    return {true, "payload ok"};
}

OtaResult OtaManager::installToInactiveSlot(const UpdateManifest& m,
                                            const fs::path& payload_path) {
    const fs::path target = slotPath(inactiveSlot()) / m.payload_file;
    std::error_code ec;
    fs::copy_file(payload_path, target, fs::copy_options::overwrite_existing, ec);
    if (ec) return {false, "install failed: " + ec.message()};
    return {true, "installed"};
}

OtaResult OtaManager::commit() {
    if (state_ != OtaState::PendingCommit) {
        return {false, "nothing staged to commit"};
    }
    const std::string new_active = inactiveSlot();
    // Order matters: persist the version BEFORE flipping the slot so a power
    // loss between the two writes can never leave an old version marked
    // newest (fails safe: re-update is possible, rollback is not).
    writeStateFile("installed_version", std::to_string(staged_.version));
    writeStateFile("active_slot", new_active);
    state_ = OtaState::Idle;
    return {true, "committed — active slot is now slot_" + new_active +
                      ", version " + std::to_string(staged_.version)};
}

OtaResult OtaManager::rollback() {
    if (state_ != OtaState::PendingCommit && state_ != OtaState::Failed) {
        return {false, "nothing to roll back"};
    }
    std::error_code ec;
    fs::remove_all(slotPath(inactiveSlot()), ec);
    fs::create_directories(slotPath(inactiveSlot()));
    state_ = OtaState::Idle;
    return {true, "rolled back — active slot slot_" + activeSlot() +
                      " untouched"};
}

OtaResult OtaManager::fail(const std::string& why) {
    state_ = OtaState::Failed;
    // A failed verify/install must not brick retries: clear to Idle so the
    // backend can push a corrected package. The active slot was never touched.
    state_ = OtaState::Idle;
    return {false, why};
}

std::string OtaManager::activeSlot() const {
    return readStateFile("active_slot", "a");
}

std::string OtaManager::inactiveSlot() const {
    return activeSlot() == "a" ? "b" : "a";
}

uint64_t OtaManager::installedVersion() const {
    try {
        return std::stoull(readStateFile("installed_version", "0"));
    } catch (...) {
        return 0;
    }
}

fs::path OtaManager::slotPath(const std::string& slot) const {
    return root_ / ("slot_" + slot);
}

void OtaManager::writeStateFile(const std::string& name,
                                const std::string& value) const {
    std::ofstream f(root_ / "state" / name, std::ios::trunc);
    f << value;
}

std::string OtaManager::readStateFile(const std::string& name,
                                      const std::string& fallback) const {
    std::ifstream f(root_ / "state" / name);
    if (!f) return fallback;
    std::string v;
    std::getline(f, v);
    return v.empty() ? fallback : v;
}

}  // namespace sdv::services::ota
