// =============================================================================
// services/diagnostics/sovd_service.hpp
//
// SOVD — Service-Oriented Vehicle Diagnostics (ISO 22900-6)
//
// SOVD is the successor to classic UDS (ISO 14229) for AUTOSAR Adaptive ECUs.
// Instead of binary protocol frames over CAN/DoIP, SOVD exposes a REST-like
// JSON API over HTTP/2 (or HTTPS) — making it tool-agnostic and cloud-ready.
//
// Key differences from classic UDS:
//   UDS:  0x22 ReadDataByIdentifier → binary frame → decode with ODX/PDXF
//   SOVD: GET /data/live/vehicle_speed → {"value": 120.3, "unit": "km/h"}
//
//   UDS:  0x14 ClearDiagnosticInformation → send 0x14 0xFF 0xFF 0xFF
//   SOVD: DELETE /faults → 200 OK
//
// This implementation provides the DATA LAYER only — no HTTP server, no TLS.
// In production, an ASIO/crow/Pistache server wraps these endpoints.
// The SovdService just serialises its state to JSON strings that can be served.
//
// Endpoints simulated:
//   GET  /properties           → ECU identity (VIN, SW version, HW version)
//   GET  /data/live/{name}     → current live data value
//   GET  /faults               → all active DTCs as JSON array
//   DELETE /faults             → clear all DTCs
//   GET  /capabilities         → what this ECU supports
//
// Production mapping:
//   SOVD HIM (Hosted Integration Model) on AUTOSAR Adaptive
//   ISO 22900-6 client/server API
// =============================================================================
#pragma once

#include <cstdio>
#include <functional>
#include <map>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "apps/diagnostics/dtc_store.hpp"

namespace sdv::services::diagnostics {

// ─── Live data accessor ───────────────────────────────────────────────────────

struct LiveDataPoint {
    std::string unit;                     // "km/h", "°C", "V", "bool", ...
    std::function<double()> read;         // live getter
    std::string description;
};

// ─── ECU properties ───────────────────────────────────────────────────────────

struct EcuProperties {
    std::string vin{"WBA00000000000000"};
    std::string ecu_name{"SDV-GW-001"};
    std::string hardware_id{"ECU-GW-001-REV-C"};
    std::string software_version{"2.0.0"};
    std::string software_build_date{"2026-06-11"};
    std::string supplier{"ACME Automotive GmbH"};
};

// ─── SOVD Service ─────────────────────────────────────────────────────────────

class SovdService {
public:
    explicit SovdService(apps::diagnostics::DtcStore& dtcs,
                         EcuProperties props = {})
        : dtcs_(dtcs), props_(std::move(props)) {}

    // Register a live data endpoint: GET /data/live/{name}
    void registerLiveData(std::string name, LiveDataPoint point) {
        live_data_.emplace(std::move(name), std::move(point));
    }

    // ── GET /properties ──────────────────────────────────────────────────────
    [[nodiscard]] std::string getProperties() const {
        std::ostringstream j;
        j << "{\n"
          << "  \"vin\":              \"" << props_.vin             << "\",\n"
          << "  \"ecu_name\":         \"" << props_.ecu_name        << "\",\n"
          << "  \"hardware_id\":      \"" << props_.hardware_id     << "\",\n"
          << "  \"software_version\": \"" << props_.software_version << "\",\n"
          << "  \"build_date\":       \"" << props_.software_build_date << "\",\n"
          << "  \"supplier\":         \"" << props_.supplier        << "\"\n"
          << "}";
        return j.str();
    }

    // ── GET /data/live/{name} ─────────────────────────────────────────────────
    [[nodiscard]] std::string getLiveData(std::string_view name) const {
        auto it = live_data_.find(std::string(name));
        if (it == live_data_.end())
            return R"({"error": "not_found", "message": "Unknown data point"})";

        char vbuf[32];
        std::snprintf(vbuf, sizeof(vbuf), "%.4g", it->second.read());

        std::ostringstream j;
        j << "{\n"
          << "  \"name\":        \"" << name              << "\",\n"
          << "  \"value\":       "   << vbuf              << ",\n"
          << "  \"unit\":        \"" << it->second.unit   << "\",\n"
          << "  \"description\": \"" << it->second.description << "\"\n"
          << "}";
        return j.str();
    }

    // ── GET /data/live (all points) ───────────────────────────────────────────
    [[nodiscard]] std::string getAllLiveData() const {
        std::ostringstream j;
        j << "[\n";
        bool first = true;
        for (const auto& [name, dp] : live_data_) {
            if (!first) j << ",\n";
            first = false;
            char vbuf[32];
            std::snprintf(vbuf, sizeof(vbuf), "%.4g", dp.read());
            j << "  {\"name\": \"" << name << "\", "
              << "\"value\": " << vbuf << ", "
              << "\"unit\": \"" << dp.unit << "\"}";
        }
        j << "\n]";
        return j.str();
    }

    // ── GET /faults ───────────────────────────────────────────────────────────
    [[nodiscard]] std::string getFaults() const {
        const auto all = dtcs_.readAll();
        std::ostringstream j;
        j << "[\n";
        bool first = true;
        for (const auto& d : all) {
            if (!first) j << ",\n";
            first = false;
            char hex[16];
            std::snprintf(hex, sizeof(hex), "0x%06X", d.code);
            j << "  {\n"
              << "    \"dtc_code\":        \"" << hex            << "\",\n"
              << "    \"description\":     \"" << d.description  << "\",\n"
              << "    \"status\":          " << static_cast<int>(d.status) << ",\n"
              << "    \"occurrence_count\": " << d.occurrence_count << "\n"
              << "  }";
        }
        j << "\n]";
        return j.str();
    }

    // ── DELETE /faults ────────────────────────────────────────────────────────
    std::string clearFaults() {
        dtcs_.clearAll();
        return R"({"status": "ok", "message": "All DTCs cleared"})";
    }

    // ── GET /capabilities ─────────────────────────────────────────────────────
    [[nodiscard]] std::string getCapabilities() const {
        std::ostringstream j;
        j << "{\n"
          << "  \"protocol\": \"SOVD/1.0\",\n"
          << "  \"standard\": \"ISO 22900-6\",\n"
          << "  \"capabilities\": [\n"
          << "    \"properties\",\n"
          << "    \"faults\",\n"
          << "    \"data/live\"\n"
          << "  ],\n"
          << "  \"live_data_points\": [\n";
        bool first = true;
        for (const auto& [name, dp] : live_data_) {
            if (!first) j << ",\n";
            first = false;
            j << "    {\"name\": \"" << name << "\", \"unit\": \"" << dp.unit << "\"}";
        }
        j << "\n  ]\n}";
        return j.str();
    }

private:
    apps::diagnostics::DtcStore& dtcs_;
    EcuProperties props_;
    std::map<std::string, LiveDataPoint> live_data_;
};

}  // namespace sdv::services::diagnostics
