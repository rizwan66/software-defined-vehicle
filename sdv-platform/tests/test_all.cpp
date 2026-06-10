// =============================================================================
// tests/test_all.cpp
//
// Dependency-free unit tests (tiny assert-based runner; in a real project
// this is GoogleTest/Catch2 wired into CI).
// =============================================================================
#include <cassert>
#include <cstdio>
#include <filesystem>
#include <fstream>

#include "apps/adas/aeb_function.hpp"
#include "apps/diagnostics/dtc_store.hpp"
#include "drivers/can/can_interface.hpp"
#include "middleware/communication/message_bus.hpp"
#include "security/crypto/sha256.hpp"
#include "services/ota/ota_manager.hpp"
#include "services/vehicle-signals/vehicle_speed_service.hpp"

namespace fs = std::filesystem;
using namespace sdv;

static int g_failures = 0;
#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            ++g_failures;                                                 \
            std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
        }                                                                 \
    } while (0)

// ---------------------------------------------------------------------------
static void test_sha256_known_vectors() {
    std::printf("test_sha256_known_vectors\n");
    // FIPS 180-4 test vector: sha256("abc")
    std::vector<uint8_t> abc{'a', 'b', 'c'};
    CHECK(security::Sha256::toHex(security::Sha256::digest(abc)) ==
          "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    // sha256("")
    CHECK(security::Sha256::toHex(security::Sha256::digest({})) ==
          "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
}

// ---------------------------------------------------------------------------
static void test_message_bus_pub_sub() {
    std::printf("test_message_bus_pub_sub\n");
    auto& bus = middleware::MessageBus::instance();
    bus.reset();

    int received = 0;
    auto id = bus.subscribe("t", [&](const middleware::Sample& s) {
        received += std::any_cast<int>(s.payload);
    });
    bus.publish("t", 21, 0);
    bus.publish("t", 21, 0);
    CHECK(received == 42);

    bus.unsubscribe("t", id);
    bus.publish("t", 100, 0);
    CHECK(received == 42);  // unchanged after unsubscribe
}

// ---------------------------------------------------------------------------
static void test_speed_service_decode_and_plausibility() {
    std::printf("test_speed_service_decode_and_plausibility\n");
    auto& bus = middleware::MessageBus::instance();
    bus.reset();

    drivers::MockCanInterface can;
    services::VehicleSpeedService svc(can, bus);

    can.setSpeedKmh(50.0f);
    svc.cycle();
    svc.cycle();  // mock alternates frames; two cycles guarantee a speed frame
    CHECK(svc.latest().quality == services::SignalQuality::Valid);
    CHECK(svc.latest().kmh > 49.9f && svc.latest().kmh < 50.1f);

    // Implausible jump (50 -> 300 km/h within one cycle) must be rejected:
    can.setSpeedKmh(300.0f);
    svc.cycle();
    svc.cycle();
    CHECK(svc.latest().kmh < 51.0f);  // last valid value retained
}

// ---------------------------------------------------------------------------
static void test_aeb_ttc_decisions() {
    std::printf("test_aeb_ttc_decisions\n");
    auto& bus = middleware::MessageBus::instance();
    bus.reset();

    apps::adas::AebFunction aeb(bus);

    // 72 km/h = 20 m/s. Obstacle at 100 m -> TTC 5 s -> no braking.
    services::SpeedSample sp{72.0f, services::SignalQuality::Valid, 1};
    bus.publish("vehicle.speed", sp, 1);
    bus.publish("obstacle.distance", 100.0f, 2);
    CHECK(aeb.lastDecision() == apps::adas::BrakeLevel::None);

    // Obstacle at 40 m -> TTC 2.0 s -> partial braking.
    bus.publish("obstacle.distance", 40.0f, 3);
    CHECK(aeb.lastDecision() == apps::adas::BrakeLevel::Partial);

    // Obstacle at 20 m -> TTC 1.0 s -> full braking.
    bus.publish("obstacle.distance", 20.0f, 4);
    CHECK(aeb.lastDecision() == apps::adas::BrakeLevel::Full);

    // Invalid speed quality -> AEB must degrade to safe state (no actuation).
    services::SpeedSample bad{72.0f, services::SignalQuality::Degraded, 5};
    bus.publish("vehicle.speed", bad, 5);
    CHECK(aeb.lastDecision() == apps::adas::BrakeLevel::None);
}

// ---------------------------------------------------------------------------
static void test_dtc_debounce_and_clear() {
    std::printf("test_dtc_debounce_and_clear\n");
    apps::diagnostics::DtcStore store;
    store.reportFailed(0xC12301, "speed invalid");
    store.reportFailed(0xC12301, "speed invalid");
    CHECK(store.readConfirmed().empty());  // below debounce threshold
    store.reportFailed(0xC12301, "speed invalid");
    CHECK(store.readConfirmed().size() == 1);  // confirmed at 3rd occurrence
    store.clearAll();
    CHECK(store.readAll().empty());
}

// ---------------------------------------------------------------------------
struct OtaFixture {
    fs::path root = fs::temp_directory_path() / "sdv-ota-test";
    std::vector<uint8_t> key{'d', 'e', 'm', 'o', '-', 'k', 'e', 'y'};

    OtaFixture() {
        fs::remove_all(root);
        fs::create_directories(root / "incoming");
    }

    // Builds a signed package; returns manifest path. tamper=true edits the
    // version AFTER signing (signature must then fail).
    fs::path makePackage(uint64_t version, bool tamper = false,
                         bool corrupt_payload = false) {
        const fs::path payload = root / "incoming" / "firmware.bin";
        {
            std::ofstream f(payload, std::ios::binary);
            f << "FIRMWARE-IMAGE-v" << version;
        }
        std::ifstream f(payload, std::ios::binary);
        std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(f)),
                                   std::istreambuf_iterator<char>());
        const std::string digest =
            security::Sha256::toHex(security::Sha256::digest(bytes));

        services::ota::UpdateManifest m;
        m.name = "gateway-fw";
        m.hardware_id = "ECU-GW-001";
        m.version = version;
        m.payload_file = "firmware.bin";
        m.payload_sha256 = digest;
        m.signature = security::Sha256::toHex(
            security::hmacSha256(key, m.canonicalBytes()));

        if (tamper) m.version += 100;  // attacker edit after signing
        if (corrupt_payload) {
            std::ofstream c(payload, std::ios::binary | std::ios::app);
            c << "BITFLIP";
        }

        const fs::path mpath = root / "incoming" / "manifest.json";
        std::ofstream out(mpath);
        out << "{\n"
            << "  \"name\": \"" << m.name << "\",\n"
            << "  \"hardware_id\": \"" << m.hardware_id << "\",\n"
            << "  \"version\": " << m.version << ",\n"
            << "  \"payload_file\": \"" << m.payload_file << "\",\n"
            << "  \"payload_sha256\": \"" << m.payload_sha256 << "\",\n"
            << "  \"signature\": \"" << m.signature << "\"\n"
            << "}\n";
        return mpath;
    }
};

static void test_ota_happy_path_commit() {
    std::printf("test_ota_happy_path_commit\n");
    OtaFixture fx;
    services::ota::OtaManager ota(fx.root, "ECU-GW-001", fx.key);
    CHECK(ota.activeSlot() == "a");
    CHECK(ota.installedVersion() == 0);

    auto r = ota.startUpdate(fx.makePackage(1),
                             fx.root / "incoming" / "firmware.bin");
    CHECK(r.ok);
    CHECK(ota.state() == services::ota::OtaState::PendingCommit);
    CHECK(fs::exists(fx.root / "slot_b" / "firmware.bin"));  // inactive slot

    auto c = ota.commit();
    CHECK(c.ok);
    CHECK(ota.activeSlot() == "b");
    CHECK(ota.installedVersion() == 1);
}

static void test_ota_rejects_tampered_manifest() {
    std::printf("test_ota_rejects_tampered_manifest\n");
    OtaFixture fx;
    services::ota::OtaManager ota(fx.root, "ECU-GW-001", fx.key);
    auto r = ota.startUpdate(fx.makePackage(1, /*tamper=*/true),
                             fx.root / "incoming" / "firmware.bin");
    CHECK(!r.ok);
    CHECK(r.message.find("signature") != std::string::npos);
    CHECK(!fs::exists(fx.root / "slot_b" / "firmware.bin"));  // never touched
}

static void test_ota_rejects_corrupt_payload() {
    std::printf("test_ota_rejects_corrupt_payload\n");
    OtaFixture fx;
    services::ota::OtaManager ota(fx.root, "ECU-GW-001", fx.key);
    auto r = ota.startUpdate(
        fx.makePackage(1, /*tamper=*/false, /*corrupt_payload=*/true),
        fx.root / "incoming" / "firmware.bin");
    CHECK(!r.ok);
    CHECK(r.message.find("digest") != std::string::npos);
}

static void test_ota_anti_rollback() {
    std::printf("test_ota_anti_rollback\n");
    OtaFixture fx;
    services::ota::OtaManager ota(fx.root, "ECU-GW-001", fx.key);
    auto r1 = ota.startUpdate(fx.makePackage(5),
                              fx.root / "incoming" / "firmware.bin");
    CHECK(r1.ok);
    CHECK(ota.commit().ok);
    CHECK(ota.installedVersion() == 5);

    // Older (or equal) version must be refused — downgrade attack.
    auto r2 = ota.startUpdate(fx.makePackage(4),
                              fx.root / "incoming" / "firmware.bin");
    CHECK(!r2.ok);
    CHECK(r2.message.find("anti-rollback") != std::string::npos);
}

static void test_ota_rejects_wrong_hardware() {
    std::printf("test_ota_rejects_wrong_hardware\n");
    OtaFixture fx;
    services::ota::OtaManager ota(fx.root, "ECU-OTHER-999", fx.key);
    auto r = ota.startUpdate(fx.makePackage(1),
                             fx.root / "incoming" / "firmware.bin");
    CHECK(!r.ok);
    CHECK(r.message.find("hardware") != std::string::npos);
}

// ---------------------------------------------------------------------------
int main() {
    test_sha256_known_vectors();
    test_message_bus_pub_sub();
    test_speed_service_decode_and_plausibility();
    test_aeb_ttc_decisions();
    test_dtc_debounce_and_clear();
    test_ota_happy_path_commit();
    test_ota_rejects_tampered_manifest();
    test_ota_rejects_corrupt_payload();
    test_ota_anti_rollback();
    test_ota_rejects_wrong_hardware();

    if (g_failures == 0) {
        std::printf("\nALL TESTS PASSED\n");
        return 0;
    }
    std::printf("\n%d FAILURE(S)\n", g_failures);
    return 1;
}
