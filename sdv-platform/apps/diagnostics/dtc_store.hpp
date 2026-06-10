// =============================================================================
// apps/diagnostics/dtc_store.hpp
//
// Minimal DTC (Diagnostic Trouble Code) memory, modeled after the UDS
// (ISO 14229) status-byte concept. Real ECUs expose this through services
// 0x19 (ReadDTCInformation) and 0x14 (ClearDiagnosticInformation) over
// DoIP/DoCAN; here it is a plain in-memory store with the same semantics:
//
//   testFailed            bit 0 — fault currently present
//   confirmedDTC          bit 3 — fault was qualified (debounced)
//   testFailedSinceClear  bit 5 — occurred at least once since last clear
// =============================================================================
#pragma once

#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace sdv::apps::diagnostics {

struct DtcEntry {
    uint32_t code{0};         // e.g. 0xC12301 — vehicle speed signal invalid
    uint8_t status{0};
    std::string description;
    uint32_t occurrence_count{0};
};

class DtcStore {
public:
    static constexpr uint8_t kTestFailed = 0x01;
    static constexpr uint8_t kConfirmed = 0x08;
    static constexpr uint8_t kFailedSinceClear = 0x20;
    static constexpr uint32_t kConfirmThreshold = 3;  // debounce counter

    void reportFailed(uint32_t code, const std::string& description) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto& e = dtcs_[code];
        e.code = code;
        e.description = description;
        e.status |= kTestFailed | kFailedSinceClear;
        if (++e.occurrence_count >= kConfirmThreshold) e.status |= kConfirmed;
    }

    void reportPassed(uint32_t code) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = dtcs_.find(code);
        if (it == dtcs_.end()) return;
        it->second.status &= static_cast<uint8_t>(~kTestFailed);
        it->second.occurrence_count = 0;
    }

    // UDS 0x14 ClearDiagnosticInformation equivalent.
    void clearAll() {
        std::lock_guard<std::mutex> lock(mutex_);
        dtcs_.clear();
    }

    std::vector<DtcEntry> readAll() const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<DtcEntry> out;
        out.reserve(dtcs_.size());
        for (const auto& [_, e] : dtcs_) out.push_back(e);
        return out;
    }

    std::vector<DtcEntry> readConfirmed() const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<DtcEntry> out;
        for (const auto& [_, e] : dtcs_) {
            if (e.status & kConfirmed) out.push_back(e);
        }
        return out;
    }

private:
    mutable std::mutex mutex_;
    std::map<uint32_t, DtcEntry> dtcs_;
};

}  // namespace sdv::apps::diagnostics
