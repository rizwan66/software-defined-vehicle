# SDV Platform — Architecture & Interview Documentation

This document explains the design of the reference project and, in the second half, covers the questions most commonly asked in SDV / automotive software interviews, anchored to concrete code in this repository so every answer can be backed by something you actually built.

---

## 1. Architecture overview

The stack follows the classic SDV layering. Each layer only depends downward, and applications never touch hardware directly.

```
┌───────────────────────────────────────────────────────────┐
│  Application layer                                        │
│  apps/adas (AEB)        apps/diagnostics (DTC store)      │
└───────────────▲───────────────────────▲───────────────────┘
                │ subscribe/publish     │ report/read
┌───────────────┴───────────────────────┴───────────────────┐
│  Vehicle services                                         │
│  vehicle-signals (speed)  vehicle-health   ota            │
└───────────────▲───────────────────────────────────────────┘
                │ typed signals on topics
┌───────────────┴───────────────────────────────────────────┐
│  Middleware                                               │
│  communication/message_bus  (stand-in for SOME/IP / DDS)  │
└───────────────▲───────────────────────────────────────────┘
                │ ICanInterface (DI)
┌───────────────┴───────────────────────────────────────────┐
│  Drivers / hardware abstraction                           │
│  drivers/can: SocketCanInterface | MockCanInterface       │
└───────────────────────────────────────────────────────────┘
        security/crypto cuts across services (OTA, future SecOC)
```

### Signal data flow (speed → braking decision)

```
CAN frame 0x123 ──▶ VehicleSpeedService.cycle()
                      ├─ decode (LE uint16 × 0.01 km/h)
                      ├─ plausibility (range 0..400, max Δ per cycle)
                      ├─ timeout tracking (200 ms)
                      └─ publish SpeedSample{kmh, quality} on "vehicle.speed"
                              │
            ┌─────────────────┴──────────────────┐
            ▼                                    ▼
     AebFunction                          HealthMonitor
     TTC = dist / v                       quality ≠ Valid → DTC 0xC12301
     <1.5 s → FULL                        quality = Valid → test passed
     <2.5 s → PARTIAL
     quality ≠ Valid → NO actuation (safe state)
```

### OTA update flow

```
Backend (tools/generate_update_package.py)        Vehicle (OtaManager)
  build payload                                     startUpdate()
  sha256(payload) ──▶ manifest                        1. verify signature   ◀── nothing trusted before this
  sign canonical manifest (HMAC) ──▶ signature        2. hardware_id match
                                                      3. anti-rollback (version > installed)
                                                      4. sha256(payload) == manifest digest
                                                      5. install into INACTIVE slot
                                                    state = PENDING_COMMIT
                                                    ── reboot, health check ──
                                                    healthy → commit()   : flip active slot, persist version
                                                    sick    → rollback() : wipe staged slot, old image untouched
```

---

## 2. Key design decisions (and why)

**Dependency injection of `ICanInterface`.** `VehicleSpeedService` takes the bus interface by reference. This single decision makes the entire signal chain unit-testable on a laptop (`MockCanInterface`) and portable to hardware (`SocketCanInterface`) without changing one line of service code. It is the same principle behind AUTOSAR's MCAL/driver abstraction.

**Signal quality is part of the data, not a side channel.** Every `SpeedSample` carries `Valid | Degraded | Invalid`. Consumers must make an explicit decision about degraded data. The AEB function demonstrates the correct consumer behavior: anything other than `Valid` ⇒ no actuation. A stale speed silently presented as fresh is exactly the kind of fault that turns into an unjustified full-braking event.

**Plausibility before acceptance.** The speed service rejects values outside 0–400 km/h and jumps larger than ~150 km/h/s. A single corrupted frame (bit flips happen on real buses despite CRC) therefore cannot propagate into an ADAS decision; the service substitutes the last valid value with `Degraded` quality and lets the timeout decide when even that is no longer defensible.

**Publish outside the lock.** `MessageBus::publish` snapshots the subscriber list under the mutex and invokes callbacks after releasing it. A slow or misbehaving subscriber can therefore not block publishers or deadlock the system — the in-process equivalent of "a slow DDS reader must not stall the writer".

**OTA verifies before it touches anything.** The order in `OtaManager` is deliberate: signature → hardware ID → anti-rollback → payload digest → only then write to the *inactive* slot. The active, running software is never modified until `commit()`, and `commit()` persists the version counter *before* flipping the slot, so a power loss between the two writes fails safe (re-update possible, downgrade not).

**Anti-rollback as a monotonic counter.** Without it, an attacker with a validly signed *old* package (which once shipped, so it is signed!) could downgrade the vehicle to a version with known vulnerabilities. This is a frequently-missed point and a favorite interview follow-up.

**Constant-time MAC comparison.** `constantTimeEqual` avoids early-exit byte comparison; a timing oracle on signature checks is a textbook side channel.

---

## 3. Mapping to production technologies

| Concept here | AUTOSAR Classic | AUTOSAR Adaptive / POSIX SDV |
|---|---|---|
| `MessageBus` topics | RTE sender/receiver ports, COM signals | `ara::com` events, SOME/IP or DDS binding |
| `VehicleSpeedService::cycle()` 10 ms | OS task, runnable mapped via RTE | deterministic execution mgmt / cyclic thread |
| `ICanInterface` | CanIf + MCAL CAN driver | SocketCAN / vendor driver behind a service |
| `DtcStore` | Dem (Diagnostic Event Manager) + Dcm | diagnostic manager over DoIP |
| `OtaManager` | FOTA via reprogramming (UDS 0x34/0x36/0x37) | UCM (Update & Configuration Mgmt), Uptane |
| `security/crypto` | CSM + SHE/HSM | crypto API, TLS, SecOC for on-bus auth |

---

## 4. Interview Q&A

### Architecture & middleware

**Q: AUTOSAR Classic vs. Adaptive — when do you use which?**
Classic is statically configured, runs on OSEK-class RTOSes on microcontrollers, everything (tasks, signals, memory) is fixed at build time — ideal for hard-real-time, safety-critical, resource-constrained ECUs (braking, powertrain). Adaptive is POSIX-based (typically Linux/QNX on high-performance SoCs), service-oriented (`ara::com`), supports dynamic deployment and updates — built for domain/zonal controllers running ADAS, connectivity, and OTA-updatable functions. Real vehicles use both: Classic at the actuator edge, Adaptive in the compute core, bridged by gateways.

**Q: SOME/IP vs. DDS vs. MQTT?**
SOME/IP is the automotive SOA protocol: service discovery, RPC, events over Ethernet, deeply integrated with AUTOSAR — strong inside the vehicle. DDS is data-centric pub/sub with rich QoS (deadline, liveliness, durability), broker-less peer discovery — favored for high-rate sensor/ADAS data and used by ROS 2. MQTT is broker-based, lightweight, ideal for vehicle-to-cloud telemetry, wrong for intra-vehicle real-time data (broker = single point of failure and latency). In this project, `MessageBus` plays the SOME/IP/DDS role in-process; the OTA backend channel would be MQTT or HTTPS.

**Q: Why CAN at all if we have automotive Ethernet?**
Cost, determinism, and legacy. CAN(-FD) is cheap, robust, well-understood, and sufficient for most body/powertrain signals. Ethernet earns its cost where bandwidth is needed (cameras, lidar, OTA, diagnostics). Zonal architectures use Ethernet as backbone and CAN/LIN as last-meter networks — which is why the gateway/translation role (what `VehicleSpeedService` does in miniature: raw frame → typed, quality-annotated service signal) matters so much.

**Q: What is a zonal architecture?**
Instead of ~100 function-specific ECUs, the vehicle is divided into physical zones, each with a zonal controller aggregating local I/O, connected via an Ethernet backbone to a few central HPCs that run the actual software. Benefits: wiring harness reduction (kg and €), software centralization (enabler for SDV/OTA), scalability. Challenge: mixed-criticality on shared compute → hypervisors, freedom-from-interference arguments (ISO 26262 part 6).

### Signals & safety

**Q: How do you handle a sensor signal you don't trust?**
Exactly like `VehicleSpeedService`: range check, rate-of-change (gradient) check, timeout supervision, and an explicit quality attribute traveling with the value. Consumers define their degradation behavior per quality level. For ASIL-rated signals, add E2E protection (CRC + alive counter per AUTOSAR E2E profiles) so corruption *between* sender and receiver is also detected.

**Q: What does "fail-safe" vs. "fail-operational" mean for your AEB?**
This AEB is fail-safe: on invalid input it withdraws to "no actuation" — acceptable because the driver remains the fallback. A level-3+ system must be fail-operational: it needs redundant channels (e.g. 2-out-of-3 or dual-channel with comparator) because there is no driver to hand over to. That distinction drives the hardware architecture (redundant power, redundant compute) far more than the software.

**Q: ISO 26262 in one minute, applied to this code?**
Hazard analysis (HARA) classifies "unjustified full braking at high speed" by severity/exposure/controllability → ASIL. The speed-signal path inherits the ASIL of the functions consuming it. Code-level consequences you can point to here: deterministic cycle (no unbounded loops — see the bounded frame drain), defensive input handling, defined degraded states, no dynamic allocation in the cyclic path, and unit tests for the failure modes, not just the happy path.

### OTA & cybersecurity

**Q: Walk me through a secure OTA update.**
Backend signs a manifest binding name, hardware ID, version, and payload digest (in production: asymmetric keys in an HSM, Uptane's split director/image repositories). Vehicle downloads, then — order matters — verifies the signature first, checks hardware compatibility, enforces anti-rollback, verifies the payload digest, installs into the inactive A/B slot, reboots into it, runs a health check, and only then commits. Any failure before commit leaves the running system untouched; failure after reboot triggers rollback to the proven slot. `services/ota/ota_manager.cpp` implements every one of these steps and `tests/test_all.cpp` proves the negative cases (tampered manifest, corrupt payload, downgrade, wrong hardware).

**Q: Why A/B instead of in-place update?**
Atomicity and rollback. In-place update + power loss = brick. With A/B, the old image is physically untouched until the new one has proven itself; the worst case after any failure is "boot the old slot". Cost: 2× storage — which is why some ECUs use compressed single-bank schemes with a recovery bootloader instead, trading robustness for flash size.

**Q: What do UNECE R155 and R156 require?**
R155: a certified Cybersecurity Management System — the OEM must demonstrate risk analysis, secured development, monitoring, and incident response across the vehicle lifecycle (ISO/SAE 21434 is the implementation standard). R156: a Software Update Management System — every update must be tracked (which software is in which vehicle), integrity-protected, and assessed for type-approval relevance before deployment. Anti-rollback, signed packages, and the audit trail of versions in this project are exactly the technical primitives those regulations assume.

**Q: How would you secure in-vehicle communication itself?**
SecOC (AUTOSAR Secure Onboard Communication): truncated MAC + freshness counter appended to selected PDUs, keys held in HSM/SHE. On Ethernet: TLS/DTLS or MACsec, plus network segmentation (VLANs, gateway firewalls) so the infotainment domain can never speak directly to the braking domain. Plus secure boot anchoring the whole chain: every layer verifies the next before executing it — conceptually the same verify-before-use pattern as the OTA manifest check.

### Testing & process

**Q: How do you test software like this without a vehicle?**
Layered: unit tests with injected mocks (this repo: `MockCanInterface`, deterministic `cycle()` calls — no threads, no sleeps, no flakiness); SIL with virtual buses (vcan0 + replayed CAN logs); HIL with the real ECU against simulated plant models; then test track. The design choice that enables the cheap lower layers is dependency injection — if the service constructed its own SocketCAN socket internally, none of this would be possible.

**Q: Why is `cycle()` public?**
Deliberate testability seam. Tests drive the cyclic task synchronously and assert behavior deterministically. The threaded `start()/stop()` wrapper is trivial and tested implicitly; the logic lives in the function that needs no clock to test.

**Q: What would you change first to make this production-ready?**
1. Replace HMAC with asymmetric signatures and move verification keys/operations into an HSM. 2. Replace the in-process bus with a real binding (vsomeip or CycloneDDS) and add E2E protection on safety signals. 3. Persist DTCs to NvM and expose them via a UDS/DoIP stack. 4. Static allocation + MISRA/cert-C compliance pass + static analysis (Polyspace/Coverity) on the safety path. 5. CI with the test suite, sanitizers (ASan/TSan), and fuzzing on the manifest parser — any parser facing external input is an attack surface.

---

## 5. File-by-file tour (for walking someone through the code)

| File | One-line pitch |
|---|---|
| `middleware/communication/message_bus.hpp` | Lock-snapshot pub/sub; the SOA backbone in 100 lines |
| `drivers/can/can_interface.*` | Hardware abstraction; SocketCAN + deterministic mock |
| `services/vehicle-signals/vehicle_speed_service.hpp` | The pasted 10-line example, grown into a defensible service |
| `services/vehicle-health/health_monitor.hpp` | Signal quality → DTC; feeds the OTA health check |
| `services/ota/ota_manager.*` | Verify-then-install state machine, A/B slots, anti-rollback |
| `security/crypto/sha256.hpp` | FIPS-vector-tested SHA-256, HMAC, constant-time compare |
| `apps/adas/aeb_function.hpp` | TTC logic + the safe-degradation pattern |
| `apps/diagnostics/dtc_store.hpp` | UDS-style status bytes with debounce |
| `tools/generate_update_package.py` | The backend half of the OTA trust chain |
| `tests/test_all.cpp` | 10 tests; half of them are attacks that must fail |
