# SDV Platform — Complete Technical Guide

> **Software Defined Vehicle Platform** — C++17, embedded/Linux target  
> Covers every module: architecture, data-flow diagrams, flowcharts, concept explanations, and security model.

---

## Table of Contents

1. [What is an SDV?](#1-what-is-an-sdv)
2. [System Architecture](#2-system-architecture)
3. [Layer-by-Layer Breakdown](#3-layer-by-layer-breakdown)
4. [Module Deep Dives](#4-module-deep-dives)
   - [CAN Driver](#41-can-driver)
   - [Message Bus](#42-message-bus-middleware)
   - [Vehicle Speed Service](#43-vehicle-speed-service)
   - [AEB Function](#44-aeb-autonomous-emergency-braking)
   - [Health Monitor & DTC Store](#45-health-monitor--dtc-store)
   - [OTA Manager](#46-ota-manager)
   - [SHA-256 / HMAC Security](#47-sha-256--hmac-security)
5. [End-to-End Data Flow](#5-end-to-end-data-flow)
6. [Flowcharts](#6-flowcharts)
   - [Vehicle Speed Service Cycle](#61-vehicle-speed-service-cycle)
   - [AEB Decision Logic](#62-aeb-decision-logic)
   - [OTA State Machine](#63-ota-state-machine)
   - [DTC Debounce Logic](#64-dtc-debounce-logic)
   - [Message Bus Pub/Sub](#65-message-bus-pubsub-flow)
7. [Security Model](#7-security-model)
8. [Key Automotive Concepts](#8-key-automotive-concepts)
9. [Build & Test Reference](#9-build--test-reference)

---

## 1. What is an SDV?

A **Software Defined Vehicle** (SDV) moves intelligence that was previously locked in dozens of isolated Electronic Control Units (ECUs) into a centralized software platform running on a high-performance compute domain controller.

```
Traditional Vehicle                    Software Defined Vehicle
─────────────────────────              ───────────────────────────────────
  ECU-1 (Engine Mgmt)                    ┌─────────────────────────────┐
  ECU-2 (Transmission)                   │  High-Performance Compute   │
  ECU-3 (ABS/ESC)          ─────►        │  Domain Controller          │
  ECU-4 (ADAS Camera)                    │  ┌─────┐  ┌──────┐ ┌─────┐ │
  ECU-5 (Telematics)                     │  │ADAS │  │ OTA  │ │Diag │ │
  ECU-6 (Body Control)                   │  └─────┘  └──────┘ └─────┘ │
  ...30+ more ECUs                       │  Shared OS  │  Middleware   │
                                         └─────────────┴───────────────┘
 Each ECU = proprietary HW+SW                  │         │
 Update = dealer visit                   CAN / LIN / Ethernet to remaining
                                         zone controllers and actuators
```

**Key benefits:**
- OTA (Over-the-Air) updates — no dealer visit required
- New features deployable like smartphone apps
- Central safety monitoring across all vehicle domains
- Reduced hardware BOM (bill of materials)

This platform implements the **core middleware layer** of such a system: signal acquisition, processing, safety functions, diagnostics, and secure OTA.

---

## 2. System Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         SDV PLATFORM STACK                              │
├─────────────────────────────────────────────────────────────────────────┤
│  APPLICATION LAYER                                                       │
│  ┌──────────────────────┐   ┌──────────────────────────────────────┐    │
│  │   AEB Function        │   │   Demo App (main.cpp)                │    │
│  │   apps/adas/          │   │   apps/demo/                         │    │
│  │   - TTC calculation   │   │   - Wires all modules together       │    │
│  │   - BrakeRequest pub  │   │   - Simulates obstacle scenario      │    │
│  └──────────────────────┘   └──────────────────────────────────────┘    │
│  ┌──────────────────────┐                                                │
│  │   DTC Store           │                                               │
│  │   apps/diagnostics/   │                                               │
│  │   - UDS DTC memory    │                                               │
│  │   - Debounce counter  │                                               │
│  └──────────────────────┘                                                │
├─────────────────────────────────────────────────────────────────────────┤
│  SERVICE LAYER                                                           │
│  ┌──────────────────────┐  ┌───────────────────┐  ┌──────────────────┐  │
│  │ VehicleSpeedService  │  │  HealthMonitor    │  │  OtaManager      │  │
│  │ services/vehicle-    │  │  services/        │  │  services/ota/   │  │
│  │ signals/             │  │  vehicle-health/  │  │                  │  │
│  │ - CAN frame decode   │  │  - Signal watch   │  │ - Manifest parse │  │
│  │ - Plausibility check │  │  - DTC trigger    │  │ - HMAC verify    │  │
│  │ - Quality tracking   │  │  - Health report  │  │ - Payload SHA256 │  │
│  │ - 10ms cycle         │  │                   │  │ - A/B slot mgmt  │  │
│  └──────────────────────┘  └───────────────────┘  └──────────────────┘  │
├─────────────────────────────────────────────────────────────────────────┤
│  MIDDLEWARE LAYER                                                        │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │   MessageBus  (middleware/communication/)                         │   │
│  │   Thread-safe singleton pub/sub broker                            │   │
│  │   Topics: vehicle.speed | obstacle.distance | brake.request       │   │
│  └──────────────────────────────────────────────────────────────────┘   │
├─────────────────────────────────────────────────────────────────────────┤
│  SECURITY LAYER                                                          │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │   SHA-256 + HMAC-SHA256  (security/crypto/)                       │   │
│  │   Self-contained FIPS 180-4 — no external deps                    │   │
│  │   Used by OTA manifest verification                               │   │
│  └──────────────────────────────────────────────────────────────────┘   │
├─────────────────────────────────────────────────────────────────────────┤
│  DRIVER LAYER                                                            │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │   ICanInterface  (drivers/can/)                                   │   │
│  │   ┌─────────────────────────┐   ┌──────────────────────────────┐ │   │
│  │   │  MockCanInterface       │   │  SocketCanInterface          │ │   │
│  │   │  - In-memory simulation │   │  - Linux SocketCAN (can0)    │ │   │
│  │   │  - Speed + obstacle     │   │  - Non-blocking O_NONBLOCK   │ │   │
│  │   │  - Thread-safe atomics  │   │  - vcan0 for integration     │ │   │
│  │   └─────────────────────────┘   └──────────────────────────────┘ │   │
│  └──────────────────────────────────────────────────────────────────┘   │
├─────────────────────────────────────────────────────────────────────────┤
│  PHYSICAL / VIRTUAL HARDWARE                                             │
│  ┌───────────────────────────────────────────────────────────────┐      │
│  │  CAN Bus  (ISO 11898)                                          │      │
│  │  ID 0x123 — Vehicle Speed   (2 bytes, LE, scale 0.01 km/h)    │      │
│  │  ID 0x250 — Obstacle Dist.  (2 bytes, LE, scale 0.01 m)       │      │
│  └───────────────────────────────────────────────────────────────┘      │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 3. Layer-by-Layer Breakdown

| Layer | Role | Files |
|---|---|---|
| **Driver** | Hardware abstraction — isolates the rest of the stack from real CAN hardware | `drivers/can/can_interface.hpp/.cpp` |
| **Middleware** | In-process pub/sub broker — decouples publishers from subscribers | `middleware/communication/message_bus.hpp` |
| **Services** | Cyclic processing, signal conditioning, OTA state machine | `services/vehicle-signals/`, `services/vehicle-health/`, `services/ota/` |
| **Applications** | High-level functions (safety, diagnostics) that consume processed signals | `apps/adas/`, `apps/diagnostics/` |
| **Security** | Cryptographic primitives used by OTA | `security/crypto/sha256.hpp` |

**Dependency rule:** Each layer may only reference layers **below** it. Applications never talk to drivers directly; services never talk to other services via direct calls — they communicate through the MessageBus.

---

## 4. Module Deep Dives

### 4.1 CAN Driver

#### What is CAN?

**Controller Area Network (CAN)** is the dominant in-vehicle serial bus protocol (ISO 11898). It was designed in the 1980s by Bosch for automotive use and is characterized by:

- **Differential signaling** — resistant to electromagnetic noise
- **Multi-master broadcast** — any node can send; all nodes receive every message
- **Arbitration by ID** — lower ID = higher priority; no collisions
- **11-bit or 29-bit message ID** — classic CAN uses 11-bit
- **Up to 8 bytes payload** — called the Data Length Code (DLC)
- **Up to 1 Mbit/s** — classic CAN (CAN-FD goes higher)

#### CAN Frame Structure

```
 ┌────────┬─────┬──────────────────────┬─────┐
 │  SOF   │ ID  │       Data           │ CRC │
 │ 1 bit  │11 b │  0–8 bytes (DLC)     │ ... │
 └────────┴─────┴──────────────────────┴─────┘

 This platform uses two IDs:

 ID 0x123  — Vehicle Speed
 ┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
 │Byte 0│Byte 1│  --  │  --  │  --  │  --  │  --  │  --  │
 │ LSB  │ MSB  │      │      │      │      │      │      │
 └──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
 raw_value = Byte0 | (Byte1 << 8)    (little-endian uint16)
 speed_kmh = raw_value × 0.01

 ID 0x250  — Obstacle Distance (same encoding, unit = meters)
```

#### Interface Design (Dependency Injection)

```
              «interface»
           ICanInterface
          ┌─────────────┐
          │ read()       │  ◄── returns std::optional<CanFrame>
          │ write()      │       (std::nullopt = no frame pending)
          │ isOpen()     │
          └─────┬────────┘
                │ implements
       ┌────────┴──────────┐
       │                   │
MockCanInterface    SocketCanInterface
(dev/test)          (Linux production)
```

The interface lets every service and test use `ICanInterface&` — you swap `MockCanInterface` in for tests and `SocketCanInterface` on real hardware without changing a single line of application code.

---

### 4.2 Message Bus (Middleware)

#### What is Publish/Subscribe?

Pub/Sub is a messaging pattern where:
- **Publishers** emit data on named **topics** without knowing who listens
- **Subscribers** register callbacks on topics without knowing who publishes
- A **Broker** (the MessageBus) routes messages between them

This decouples the vehicle speed service from the AEB function — neither references the other.

#### Topics in this Platform

| Topic | Payload Type | Publisher | Subscribers |
|---|---|---|---|
| `vehicle.speed` | `SpeedSample` | VehicleSpeedService | AebFunction, HealthMonitor |
| `obstacle.distance` | `float` (meters) | Perception stack / demo | AebFunction |
| `brake.request` | `BrakeRequest` | AebFunction | Brake actuator / demo logger |

#### Thread Safety Design

```
Publisher Thread                     Subscriber Thread(s)
────────────────                     ─────────────────────
bus.publish("vehicle.speed", s)
  │
  ├─ LOCK mutex
  ├─ COPY subscriber list → snapshot     (fast — no alloc usually)
  ├─ UNLOCK mutex
  │
  └─ call each callback(sample)  ──────►  subscriber callback runs
     (outside the lock!)                  without holding bus mutex
                                          → slow subscribers cannot
                                            block publishers
                                          → no deadlock if subscriber
                                            calls publish() itself
```

The snapshot pattern is the critical insight: the lock is held only long enough to read the subscriber list, never while executing callbacks.

#### Production Equivalents

In a real SDV stack, this MessageBus would be replaced by:
- **SOME/IP** (vsomeip) — service-oriented middleware, network-transparent
- **DDS** (CycloneDDS / FastDDS) — real-time, distributed, QoS-configurable
- **AUTOSAR Adaptive ara::com** — the standardized binding over SOME/IP or DDS

The programming model (publish/subscribe on named topics) is **identical** — porting means swapping the bus implementation, not the application code.

---

### 4.3 Vehicle Speed Service

This service bridges raw CAN bytes into a semantically rich, quality-annotated signal that the rest of the platform can safely consume.

#### Signal Quality State Machine

```
       No valid value received
       since startup
              │
              ▼
         ┌─────────┐
         │ INVALID │ ◄──────────────────────────────┐
         └────┬────┘                                 │
              │ first plausible frame received        │ timeout (200 ms)
              ▼                                       │ exceeded with no
         ┌─────────┐  implausible jump or no frame    │ recovery
         │  VALID  │ ─────────────────────────────► ┌─────────┐
         │         │ ◄───────────────────────────── │DEGRADED │
         └─────────┘  valid frame resumes            └─────────┘
                                                       │
                                           last valid value substituted,
                                           quality flag = DEGRADED
```

#### Plausibility Checks

Two checks run on every decoded speed value:

1. **Range check**: `0 ≤ speed ≤ 400 km/h` — physically impossible values are rejected
2. **Rate-of-change check**: `|new - last| ≤ 15 km/h per 10ms cycle` — equivalent to ≤150 km/h/s acceleration — rejects bit-flip / babbling-node noise

If either check fails: the last valid value is retained (substitution) and quality is downgraded to `Degraded`.

#### Why This Matters for Safety (ASIL)

The AUTOSAR functional safety standard (ISO 26262) requires that safety-relevant signals carry **E2E protection** — end-to-end integrity, freshness, and quality. The `SignalQuality` enum is the simplified version of this concept. Systems that consume the speed signal (like AEB) **must check quality before acting** — this is a key ASIL decomposition argument.

---

### 4.4 AEB (Autonomous Emergency Braking)

#### Concept: Time-To-Collision (TTC)

```
   Ego Vehicle                           Obstacle
   ──────────►  speed v (m/s)           │ stationary
                                         │
   ◄─────────── distance d (m) ─────────►

   TTC = d / v         (seconds until impact at current speed)

   AEB Decision:
   ┌──────────────────────────────────────────────┐
   │  TTC ≥ 2.5 s  →  BrakeLevel::None            │
   │  TTC < 2.5 s  →  BrakeLevel::Partial (warn)  │
   │  TTC < 1.5 s  →  BrakeLevel::Full             │
   └──────────────────────────────────────────────┘
```

#### Safety Degradation (Critical Design Pattern)

```
   speed_valid == false   OR   speed < 10 km/h
           │                          │
           └──────────┬───────────────┘
                      ▼
              BrakeLevel::None     ← SAFE STATE
              (no actuation)

   This prevents "phantom braking" on the Autobahn at 200 km/h
   if the speed sensor fails — AEB would otherwise compute a tiny
   TTC and slam the brakes on a false obstacle.
```

This is the **fail-safe** principle: in any state of uncertainty about inputs, the system defaults to the action with the lowest hazard potential (no actuation is safer than incorrect actuation for AEB).

---

### 4.5 Health Monitor & DTC Store

#### What is a DTC?

A **Diagnostic Trouble Code (DTC)** is a standardized fault code stored in an ECU's non-volatile memory when a fault is detected. They are read by workshops using an OBD-II scanner over the **UDS protocol (ISO 14229)**.

DTC codes follow a format like `C1230` or hex `0xC12301`:
- First nibble indicates system: `C` = Chassis, `B` = Body, `P` = Powertrain, `U` = Network
- Remaining digits = manufacturer-defined fault identifier

#### DTC Status Byte (ISO 14229 model)

```
  Bit 7   Bit 6   Bit 5         Bit 4   Bit 3       Bit 2   Bit 1   Bit 0
  ─────   ─────   ───────       ─────   ─────────   ─────   ─────   ─────────
  warn-   test    testFailed-   unused  confirmed-  pend-   test    testFailed
  Ind.    notComp SinceClear            DTC         ing     notRun  (current)
                  (kFailedSinceClear=0x20)
                                        (kConfirmed=0x08)
                                                            (kTestFailed=0x01)
```

The platform implements three of these bits:
- `kTestFailed (0x01)` — fault currently active
- `kConfirmed (0x08)` — fault has occurred ≥3 times (debounced, reliable)
- `kFailedSinceClear (0x20)` — fault occurred at least once since last clear

#### Debounce Counter

```
  reportFailed() called:
  
  Call 1:  occurrence_count = 1   status = 0x21  (testFailed | failedSinceClear)
  Call 2:  occurrence_count = 2   status = 0x21
  Call 3:  occurrence_count = 3   status = 0x29  (+ confirmed bit 0x08 set)
  
  reportPassed() called:
  
  occurrence_count = 0   status &= ~kTestFailed  → bit 0 cleared
  (kConfirmed bit stays until clearAll() — fault history preserved)
```

#### HealthMonitor Integration

HealthMonitor subscribes to `vehicle.speed` and decides:
- Quality == Valid → `reportPassed(kDtcSpeedSignalInvalid)` → healthy = true
- Quality != Valid → `reportFailed(kDtcSpeedSignalInvalid, ...)` → healthy = false

The `systemHealthy()` return value is used by `OtaManager` as the commit/rollback criterion: if the vehicle is not healthy after booting on a new firmware image, the OTA is rolled back automatically.

---

### 4.6 OTA Manager

#### Background: Why OTA is Hard in Automotive

Unlike a smartphone app update, an automotive OTA failure can brick a vehicle at 100 mph. Requirements:
- **Integrity** — the downloaded image must not be corrupted or tampered
- **Authenticity** — the image must come from the OEM, not an attacker
- **Anti-rollback** — an attacker cannot force installation of an older, vulnerable version
- **Compatibility** — an image built for ECU model A cannot flash ECU model B
- **Atomic swap** — the active ECU software must remain untouched until the new image is verified healthy after a first boot

This platform implements all five using the **Uptane-inspired, UNECE R156-aligned** pattern.

#### A/B Slot Architecture

```
  Flash Storage Layout:
  ┌────────────────────────────────────────────────┐
  │  slot_a/    ← ACTIVE (currently running FW)    │
  │  slot_b/    ← INACTIVE (download target)       │
  │  state/                                         │
  │    active_slot        = "a"                     │
  │    installed_version  = "5"                     │
  └────────────────────────────────────────────────┘

  During update:
  1. Download + install → slot_b/  (slot_a untouched)
  2. First boot from slot_b, run health checks
  3a. Health OK  → write "installed_version=6" → write "active_slot=b"
  3b. Health FAIL → rm -rf slot_b/, stays on slot_a  (rollback)
```

The write order in `commit()` is deliberate: `installed_version` is written before `active_slot`. If power is lost between the two writes, the version counter records the new version but the active slot still points at the old image — a safe state that allows re-update.

#### Verification Chain

```
  Received: manifest.json + firmware.bin
                │
                ▼
  ┌─────────────────────────────────────────────┐
  │  Step 1: Parse manifest JSON                │
  │  {"name":..., "hardware_id":...,            │
  │   "version":6, "payload_sha256":"abc...",   │
  │   "signature":"def..."}                     │
  └────────────────────┬────────────────────────┘
                       │
                       ▼
  ┌─────────────────────────────────────────────┐
  │  Step 2: Verify HMAC-SHA256 signature       │
  │  canonical = "name=...\nhardware_id=...\n   │
  │              version=6\n..."                │
  │  expected_mac = HMAC(key, canonical)        │
  │  provided_mac = hex_decode(signature field) │
  │  constantTimeEqual(expected, provided)?     │
  │  NO  → REJECT (tampering detected)          │
  │  YES → continue                             │
  └────────────────────┬────────────────────────┘
                       │
                       ▼
  ┌─────────────────────────────────────────────┐
  │  Step 3: Hardware ID check                  │
  │  manifest.hardware_id == this_ecu_id?       │
  │  NO  → REJECT (wrong ECU target)            │
  └────────────────────┬────────────────────────┘
                       │
                       ▼
  ┌─────────────────────────────────────────────┐
  │  Step 4: Anti-rollback check                │
  │  manifest.version > installed_version?      │
  │  NO  → REJECT (downgrade attack)            │
  └────────────────────┬────────────────────────┘
                       │
                       ▼
  ┌─────────────────────────────────────────────┐
  │  Step 5: SHA-256 payload integrity          │
  │  sha256(firmware.bin) == manifest.sha256?   │
  │  NO  → REJECT (corrupt download)            │
  └────────────────────┬────────────────────────┘
                       │
                       ▼
  ┌─────────────────────────────────────────────┐
  │  Step 6: Install to inactive slot           │
  │  cp firmware.bin → slot_b/firmware.bin      │
  │  state = PendingCommit                      │
  └────────────────────┬────────────────────────┘
                       │
              (health check happens externally)
                       │
           ┌───────────┴───────────┐
           │ Healthy               │ Not healthy
           ▼                       ▼
        commit()               rollback()
    active_slot = b         rm -rf slot_b/
    version = 6             active_slot = a (unchanged)
    state = Idle            state = Idle
```

---

### 4.7 SHA-256 / HMAC Security

#### SHA-256 Algorithm Overview

SHA-256 (Secure Hash Algorithm 256-bit, FIPS 180-4) produces a 32-byte digest from arbitrary-length input. Properties:
- **One-way**: infeasible to find input from output
- **Collision-resistant**: infeasible to find two inputs with the same output
- **Avalanche effect**: 1-bit change in input → ~50% of output bits change

```
  Input message (any length)
  ┌──────────────────────────────────────┐
  │ M₁  │ M₂  │ M₃  │ ... │ Mₙ │padding│
  └──────┴─────┴─────┴─────┴────┴───────┘
         each block = 512 bits (64 bytes)
              │
              ▼
  ┌────────────────────────────────────────┐
  │  Initial hash values H₀..H₇ (FIPS)    │
  │  = first 32 bits of fractional parts  │
  │    of √2, √3, √5, √7 ...              │
  └────────────┬───────────────────────────┘
               │
   ┌───────────▼─────────────┐
   │  For each 64-byte block  │
   │  1. Expand to W[0..63]   │ ← message schedule
   │  2. 64 rounds of:         │
   │     Ch, Maj, Σ₀, Σ₁      │ ← bitwise mix functions
   │     T₁ = h+Σ₁+Ch+K[i]+W │
   │     T₂ = Σ₀+Maj          │
   │     rotate a..h registers │
   │  3. H += (a..h)           │ ← Davies-Meyer compression
   └───────────┬───────────────┘
               │ (loop until all blocks processed)
               ▼
  Output: H₀‖H₁‖H₂‖H₃‖H₄‖H₅‖H₆‖H₇ = 256-bit digest
```

#### HMAC-SHA256

HMAC (Hash-based Message Authentication Code) uses SHA-256 to produce a MAC that proves both **integrity** (content not altered) and **authenticity** (produced by someone who knows the key):

```
  HMAC(K, M) = SHA256( (K⊕opad) ‖ SHA256( (K⊕ipad) ‖ M ) )

  where:
    ipad = 0x36 repeated 64 times
    opad = 0x5c repeated 64 times
    ⊕    = XOR
    ‖    = concatenation
```

#### Constant-Time Comparison

```cpp
uint8_t diff = 0;
for (size_t i = 0; i < 32; ++i) diff |= a[i] ^ b[i];
return diff == 0;
```

A naive `memcmp` returns early on the first mismatch. An attacker can measure the time difference between a 1-byte match and a 32-byte match to **enumerate the MAC byte by byte** (timing side-channel). The constant-time loop **always runs all 32 iterations** regardless of where the first mismatch is, making every comparison indistinguishable by timing.

---

## 5. End-to-End Data Flow

This traces a single speed reading from raw bytes on the CAN bus to an AEB brake request.

```
  Physical World
  ──────────────
  Vehicle traveling at 80 km/h approaches obstacle at 35 m

  CAN Bus (hardware)
  ──────────────────
  ECU broadcasts frame:
    ID=0x123  DLC=2  data=[0x00, 0xDC]    ← 80.00 km/h
    (0xDC00 = 56320 decimal × 0.01 = 563.20? No:
     raw = 0x00 | (0xDC << 8) = 0xDC00 = 56320 × 0.01 = 563.2 km/h — wait
     Correct: 80 km/h / 0.01 = 8000 = 0x1F40
     data[0]=0x40, data[1]=0x1F)

  Driver Layer  (can_interface.cpp)
  ──────────────────────────────────
  SocketCanInterface::read()
    ::read(fd_, &raw_frame, sizeof(raw_frame))
    → CanFrame { id=0x123, dlc=2, data=[0x40,0x1F,...], timestamp_ns=... }

  Service Layer  (vehicle_speed_service.hpp)
  ──────────────────────────────────────────
  VehicleSpeedService::cycle()  [runs every 10 ms]
    can_.read()  → CanFrame
    id == 0x123 && dlc >= 2  ✓
    raw = 0x40 | (0x1F << 8) = 0x1F40 = 8000
    decoded_kmh = 8000 × 0.01 = 80.0 km/h

    plausible(80.0)?
      range check: 0 ≤ 80.0 ≤ 400  ✓
      rate check:  |80.0 - last_valid| ≤ 15  ✓  (assume prev 79.8)
    → last_valid_kmh_ = 80.0
    → quality = Valid

    bus_.publish("vehicle.speed",
                 SpeedSample{80.0, Valid, timestamp_ns}, ts)

  Middleware Layer  (message_bus.hpp)
  ────────────────────────────────────
  MessageBus::publish("vehicle.speed", sample, ts)
    lock → copy subscriber list → unlock
    invoke callback for AebFunction
    invoke callback for HealthMonitor

  Application Layer — AEB  (aeb_function.hpp)
  ─────────────────────────────────────────────
  AebFunction::evaluate(ts)  [triggered by vehicle.speed callback]
    kmh = 80.0, dist = 35.0, speed_valid = true
    kmh >= 10.0  ✓  (AEB active threshold)
    mps = 80.0 / 3.6 = 22.22 m/s
    ttc = 35.0 / 22.22 = 1.575 s

    ttc < 1.5?  NO
    ttc < 2.5?  YES  → level = Partial

    bus_.publish("brake.request",
                 BrakeRequest{Partial, 1.575s, ts}, ts)

  Application Layer — HealthMonitor
  ───────────────────────────────────
  HealthMonitor callback [also triggered by vehicle.speed]
    quality == Valid → dtcs_.reportPassed(kDtcSpeedSignalInvalid)
    healthy_ = true  (OTA commit gate stays green)

  Brake Actuator / Logger
  ───────────────────────
  Subscriber on "brake.request" receives:
    BrakeRequest { level=Partial, ttc=1.575s }
    → Hydraulic pressure applied to 60% of maximum
```

---

## 6. Flowcharts

### 6.1 Vehicle Speed Service Cycle

```
  VehicleSpeedService::cycle()   [called every 10 ms]
  ─────────────────────────────────────────────────────

  START
    │
    ▼
  Record now = steady_clock::now()
    │
    ▼
  ┌─────────────────────────────────────┐
  │  Drain CAN frames (max 64)          │
  │  for i in 0..63:                    │
  │    frame = can_.read()              │
  │    if !frame: break                 │
  │    if frame.id == 0x123 &&          │
  │       frame.dlc >= 2:               │
  │      raw = byte0 | (byte1<<8)       │
  │      decoded_kmh = raw * 0.01       │
  │      got_frame = true               │
  └─────────────────┬───────────────────┘
                    │
                    ▼
              got_frame?
             /          \
           YES            NO
            │              │
            ▼              │
      plausible(kmh)?       │
       /        \           │
     YES         NO         │
      │           │         │
      ▼           └────┐    │
  last_valid = kmh      │    │
  last_time  = now      │    │
  has_valid  = true     │    │
  quality = Valid       │    │
      │                │    │
      │         (implausible │
      │          or no frame)│
      │                │    │
      └────────►  has_valid? ◄┘
                  /       \
                YES         NO
                 │           │
         (now - last) ≤ 200ms?  quality = Invalid
                 /    \
               YES      NO
                │        │
         quality=      has_valid=false
         Degraded      quality=Invalid
         (substitute   (no more
          last_valid)   substitution)
                │
                ▼
           publish SpeedSample{kmh, quality, ts}
           on topic "vehicle.speed"
                │
              END
```

### 6.2 AEB Decision Logic

```
  AebFunction::evaluate()   [triggered on speed OR distance update]
  ─────────────────────────────────────────────────────────────────

  START
    │
    ▼
  Read atomics: kmh, dist_m, speed_valid
    │
    ▼
  speed_valid == true?
   /              \
 NO                YES
  │                 │
  │           kmh >= 10.0 km/h?
  │             /          \
  │           NO             YES
  │            │               │
  │            │         dist_m > 0.0?
  │            │           /       \
  │            │         NO         YES
  │            │          │           │
  │          level        │     mps = kmh / 3.6
  │          = None       │     ttc = dist / mps
  │            │          │           │
  └────────────┴──────────┘     ttc < 1.5s?
                │                 /        \
                │               YES          NO
                │                │            │
                │          level=Full    ttc < 2.5s?
                │                │         /      \
                │                │       YES        NO
                │                │        │          │
                │                │   level=Partial  level=None
                │                │        │          │
                └────────────────┴────────┴──────────┘
                                          │
                                          ▼
                                  level changed?
                                   /         \
                                 YES           NO
                                  │             │
                           publish              (no event —
                           brake.request        avoid flooding
                           on MessageBus        the bus)
                                  │
                                END
```

### 6.3 OTA State Machine

```
  ┌──────────────────────────────────────────────────────────────────┐
  │                    OTA STATE MACHINE                             │
  └──────────────────────────────────────────────────────────────────┘

  startUpdate() called
        │
        ▼
     ┌──────┐  state != Idle   ┌──────────────────────────┐
     │ IDLE │ ───────────────► │ return error "in progress"│
     └──┬───┘                  └──────────────────────────┘
        │ state = Idle
        ▼
  ┌───────────┐
  │ VERIFYING │  (state set immediately on entry)
  └─────┬─────┘
        │
        ├── Parse manifest JSON
        │     FAIL ──────────────────────────────► IDLE + return error
        │
        ├── Verify HMAC signature
        │     FAIL ──────────────────────────────► IDLE + return error
        │
        ├── Check hardware_id == this_ecu
        │     FAIL ──────────────────────────────► IDLE + return error
        │
        ├── Check version > installed_version
        │     FAIL ──────────────────────────────► IDLE + return error
        │
        └── SHA-256 payload digest check
              FAIL ──────────────────────────────► IDLE + return error
        │
        ▼ (all checks pass)
  ┌────────────┐
  │ INSTALLING │
  └─────┬──────┘
        │
        ├── cp payload → inactive_slot/firmware.bin
        │     FAIL ──────────────────────────────► IDLE + return error
        │
        ▼ (install OK)
  ┌────────────────┐
  │ PENDING_COMMIT │  ← startUpdate() returns OK here
  └───────┬────────┘    caller must run health checks
          │
    ┌─────┴──────┐
    │            │
    ▼            ▼
 commit()    rollback()
    │            │
    │  write      │  rm -rf inactive_slot/
    │  installed_ │  create_directories
    │  version    │
    │  write      │
    │  active_    │
    │  slot       │
    │            │
    ▼            ▼
  ┌──────┐    ┌──────┐
  │ IDLE │    │ IDLE │
  └──────┘    └──────┘
  (new slot    (old slot
   active)      still active,
                update wiped)
```

### 6.4 DTC Debounce Logic

```
  reportFailed(code, description)
  ────────────────────────────────

  START
    │
    ▼
  Lock mutex
    │
    ▼
  dtcs_[code].status |= testFailed (0x01)
  dtcs_[code].status |= failedSinceClear (0x20)
  dtcs_[code].occurrence_count++
    │
    ▼
  occurrence_count >= 3?
     /              \
   YES                NO
    │                  │
  status |= confirmed  │  (fault seen, but not yet
  (0x08)               │   reliable enough to be
    │                  │   "confirmed" — intermittent
    └──────────────────┘   fault filtering)
                │
              UNLOCK
                │
              END


  reportPassed(code)
  ──────────────────

  START → LOCK
    │
    ▼
  Find code in map
     /      \
  FOUND     NOT FOUND → UNLOCK → END
    │
  status &= ~testFailed   (clear bit 0, fault no longer active)
  occurrence_count = 0     (reset debounce counter)
  (kConfirmed bit stays — fault history preserved until clearAll())
    │
  UNLOCK → END


  readConfirmed()  — what OBD scanner reads
  ──────────────────────────────────────────

  Returns only entries where (status & kConfirmed) != 0
  i.e., faults that occurred ≥3 times (debounced, reliable)
```

### 6.5 Message Bus Pub/Sub Flow

```
  PUBLISHER THREAD                      SUBSCRIBER THREADS
  ─────────────────                     ──────────────────────────────────
                                        Thread A registered:
                                          id_A = bus.subscribe("t", cbA)

  bus.publish("t", value, ts)           Thread B registered:
       │                                  id_B = bus.subscribe("t", cbB)
       ▼
  [LOCK mutex]                          ← mutex acquired by publisher
  Find "t" in subscribers_ map
  Copy {id_A, cbA}, {id_B, cbB}
    into local `snapshot` vector
  [UNLOCK mutex]                        ← mutex released BEFORE calling cb

  Call snapshot[0].cb(sample)  ─────►  cbA(sample) runs in publisher thread
  Call snapshot[1].cb(sample)  ─────►  cbB(sample) runs in publisher thread

  NOTE: callbacks run sequentially in the publisher's thread context.
  For parallel delivery, each subscriber would need its own queue+thread.
  The snapshot ensures:
    ✓ No deadlock if cbA calls bus.publish() itself
    ✓ Slow cbB doesn't block other publishers
    ✓ bus.unsubscribe() mid-publish is safe (snapshot is a copy)
```

---

## 7. Security Model

```
  ┌──────────────────────────────────────────────────────────────────────┐
  │                        THREAT MODEL                                  │
  ├──────────────────────────────────────────────────────────────────────┤
  │                                                                       │
  │  Attacker capabilities assumed:                                       │
  │  • Can intercept and replay OTA update packages                       │
  │  • Can craft malicious firmware images                                │
  │  • Can attempt to downgrade to older, vulnerable firmware             │
  │  • Can target the wrong ECU with a valid package                      │
  │  • Has access to network but NOT the OEM signing key                  │
  │                                                                       │
  │  Attacker capabilities NOT assumed:                                   │
  │  • Compromise of the OEM backend signing server                       │
  │  • Physical access to flash memory (HSM protects key in production)   │
  │                                                                       │
  ├──────────────────────────────────────────────────────────────────────┤
  │                        CONTROLS                                       │
  ├────────────────────────┬─────────────────────────────────────────────┤
  │  Threat                │  Countermeasure                             │
  ├────────────────────────┼─────────────────────────────────────────────┤
  │  Tampered manifest     │  HMAC-SHA256 signature over canonical bytes │
  │                        │  verified BEFORE payload is touched         │
  ├────────────────────────┼─────────────────────────────────────────────┤
  │  Corrupt download      │  SHA-256 digest of payload verified against │
  │                        │  pinned value in signed manifest            │
  ├────────────────────────┼─────────────────────────────────────────────┤
  │  Wrong ECU target      │  hardware_id field in manifest must match   │
  │                        │  ECU's own ID (prevents cross-flashing)     │
  ├────────────────────────┼─────────────────────────────────────────────┤
  │  Downgrade attack      │  monotonic version counter; manifest must   │
  │                        │  have version > installed_version           │
  ├────────────────────────┼─────────────────────────────────────────────┤
  │  Timing side-channel   │  constantTimeEqual() used for MAC compare;  │
  │  on MAC verification   │  runs all 32 iterations unconditionally     │
  ├────────────────────────┼─────────────────────────────────────────────┤
  │  Power loss during     │  version written before active_slot flip;   │
  │  commit                │  safe failure mode: re-update possible      │
  ├────────────────────────┼─────────────────────────────────────────────┤
  │  Bricking on bad image │  A/B slots: active slot never touched until │
  │                        │  explicit commit() after health check       │
  └────────────────────────┴─────────────────────────────────────────────┘

  Production upgrade path (not in this codebase but noted):
  HMAC (symmetric, shared secret)  →  Ed25519 / ECDSA-P256 (asymmetric)
  Private key stays in OEM HSM; ECU holds only the public certificate.
  This is the Uptane / UNECE R156 requirement.
```

---

## 8. Key Automotive Concepts

### CAN Bus

| Property | Value |
|---|---|
| Standard | ISO 11898 |
| Topology | Multi-drop bus (all nodes connected) |
| Speed | Up to 1 Mbit/s (Classic CAN), up to 8 Mbit/s (CAN-FD) |
| Max payload | 8 bytes (Classic), 64 bytes (CAN-FD) |
| Error detection | CRC-15, bit stuffing, ACK field |
| Arbitration | Non-destructive bitwise AND (lower ID wins) |

### AUTOSAR

**AUTOSAR** (AUTomotive Open System ARchitecture) is a global partnership defining standardized software architectures for automotive ECUs:

- **AUTOSAR Classic** — deeply embedded ECUs (OSEK/VDX OS, 32KB–few MB)
- **AUTOSAR Adaptive** — high-performance compute domains (POSIX OS, ara::com, ara::exec)

The MessageBus in this platform mimics the **ara::com** pub/sub interface.

### ISO 26262 (Functional Safety)

Defines **ASIL** (Automotive Safety Integrity Level) A–D for safety-relevant functions. Higher ASIL = stricter development, verification, and runtime requirements.

The signal quality mechanism (`Valid / Degraded / Invalid`) and the AEB safe-state degradation are direct implementations of ISO 26262 requirements for:
- **Diagnostic coverage** — detect and indicate faults
- **Safe state** — transition to a defined safe condition on fault detection

### UDS / OBD-II

**UDS (Unified Diagnostic Services, ISO 14229)** is the protocol used by diagnostics equipment to communicate with ECUs. Key services:
- `0x19` — ReadDTCInformation (reads the DTC store)
- `0x14` — ClearDiagnosticInformation (maps to `dtcs.clearAll()`)
- `0x27` — SecurityAccess (authentication before OTA)
- `0x2E` — WriteDataByIdentifier (ECU configuration)

### Uptane / UNECE R156

**Uptane** is the automotive OTA security framework adopted by the UNECE R156 regulation (mandatory for new vehicles in EU, Japan, South Korea since 2022):
- Delegated trust hierarchy (root → targets → images)
- Separate repositories for metadata and images
- Director repository for per-vehicle targeting
- Image repository for OEM-wide integrity

This platform implements the **ECU-side verification pipeline** that Uptane mandates.

---

## 9. Build & Test Reference

### Directory Structure

```
sdv-platform/
├── CMakeLists.txt              ← CMake build definition
├── Makefile                    ← Convenience wrappers (make demo, make test)
├── README.md
├── apps/
│   ├── adas/
│   │   └── aeb_function.hpp    ← AEB (header-only)
│   ├── demo/
│   │   └── main.cpp            ← End-to-end demo executable
│   └── diagnostics/
│       └── dtc_store.hpp       ← DTC memory (header-only)
├── docs/
│   └── PROJECT_DOCUMENTATION.md
├── drivers/
│   └── can/
│       ├── can_interface.hpp   ← ICanInterface + Mock + SocketCAN
│       └── can_interface.cpp
├── middleware/
│   └── communication/
│       └── message_bus.hpp     ← Pub/sub broker (header-only)
├── security/
│   └── crypto/
│       └── sha256.hpp          ← SHA-256 + HMAC (header-only)
├── services/
│   ├── ota/
│   │   ├── ota_manager.hpp
│   │   └── ota_manager.cpp
│   ├── vehicle-health/
│   │   └── health_monitor.hpp  ← (header-only)
│   └── vehicle-signals/
│       └── vehicle_speed_service.hpp  ← (header-only)
├── tests/
│   └── test_all.cpp            ← Unit tests
└── tools/
    └── generate_update_package.py  ← Creates signed test packages
```

### Build Commands

```bash
# Configure
cmake -B build -DCMAKE_BUILD_TYPE=Release

# Build everything
cmake --build build

# Or with the Makefile wrapper:
make all       # build demo + tests
make demo      # run the demo
make test      # run unit tests
make clean     # remove build artifacts
```

### Test Coverage

| Test | What It Verifies |
|---|---|
| `test_sha256_known_vectors` | FIPS 180-4 known-answer tests for SHA-256 |
| `test_message_bus_pub_sub` | Subscribe, receive, unsubscribe on MessageBus |
| `test_speed_service_decode_and_plausibility` | CAN decode + implausible-jump rejection |
| `test_aeb_ttc_decisions` | None/Partial/Full brake levels at correct TTC thresholds |
| `test_aeb_ttc_decisions` (bad quality) | AEB degrades to safe state on invalid speed |
| `test_dtc_debounce_and_clear` | 3-occurrence confirm threshold + clearAll |
| `test_ota_happy_path_commit` | Full update → commit → slot flip + version bump |
| `test_ota_rejects_tampered_manifest` | Post-sign version edit → HMAC failure |
| `test_ota_rejects_corrupt_payload` | Bitflip in firmware.bin → SHA-256 mismatch |
| `test_ota_anti_rollback` | Version N → reject version N-1 |
| `test_ota_rejects_wrong_hardware` | Wrong ECU ID in manifest → rejection |

### Generating a Signed Update Package

```bash
python3 tools/generate_update_package.py \
  --key "demo-key" \
  --hw-id "ECU-GW-001" \
  --version 7 \
  --firmware my_firmware.bin \
  --out /tmp/sdv-pkg/
# Produces: /tmp/sdv-pkg/manifest.json + firmware.bin
```

### Virtual CAN Interface (Linux Integration Testing)

```bash
# Create a virtual CAN bus
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# Use in code
auto can = std::make_unique<sdv::drivers::SocketCanInterface>("vcan0");

# Send a test frame (80 km/h)
cansend vcan0 123#401F0000000000000   # ID=0x123, data=0x40 0x1F
```
