# SDV Platform — Reference Project

A compact, fully working reference implementation of a Software-Defined Vehicle software stack. Everything compiles with a plain C++17 toolchain, no external dependencies, and runs on any Linux machine — yet the architecture, interfaces, and failure handling mirror what production SDV stacks (AUTOSAR Adaptive, vsomeip/DDS-based platforms) actually do.

## What's inside

```
sdv-platform/
├── apps/
│   ├── adas/            AEB function: TTC-based braking decisions with safe degradation
│   ├── diagnostics/     UDS-style DTC store (debounce, confirm, clear)
│   └── demo/            End-to-end demo wiring everything together
├── services/
│   ├── vehicle-signals/ VehicleSpeedService: decode, plausibility, quality, timeout
│   ├── vehicle-health/  Health monitor mapping signal quality to DTCs
│   └── ota/             OTA manager: signed manifests, A/B slots, anti-rollback
├── middleware/
│   └── communication/   Thread-safe pub/sub message bus (SOME/IP / DDS stand-in)
├── drivers/
│   └── can/             ICanInterface + SocketCAN impl + deterministic mock
├── security/
│   └── crypto/          SHA-256, HMAC-SHA256, constant-time compare
├── tools/               Backend-side signed update package generator (Python)
├── tests/               10 unit/integration tests incl. security negative cases
└── docs/                Architecture documentation + interview preparation
```

## Quick start

```bash
make run-tests     # build & run the test suite
make run-demo      # generate a signed OTA package, then run the full demo
```

Requirements: g++ ≥ 9 (C++17), GNU make, Python 3. Linux recommended (SocketCAN); the project also compiles elsewhere with the SocketCAN stub.

## Demo output (abridged)

```
[1] Vehicle signals + ADAS
  obstacle at 70 m, speed 80 km/h
    [AEB] brake request: PARTIAL (TTC 2.02 s)
    [AEB] brake request: FULL    (TTC 0.90 s)

[2] Diagnostics
    DTC 0xC12301 [Vehicle speed signal invalid/degraded] status=0x29

[3] OTA update cycle
    startUpdate: OK  staged version 1 into slot_b, awaiting health check
    commit:      OK  committed — active slot is now slot_b, version 1
    tampered pkg: rejected — signature verification FAILED
```

## Trying it on a real (virtual) CAN bus

```bash
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
# replace MockCanInterface with SocketCanInterface("vcan0") in main.cpp
# inject frames:  cansend vcan0 123#E803   (0x03E8 = 1000 -> 10.00 km/h)
```

## Where the simplifications are (honesty section)

| This project | Production equivalent |
|---|---|
| In-process message bus | SOME/IP (vsomeip), DDS, or ara::com |
| HMAC-SHA256 manifest signature | Ed25519/ECDSA + X.509 chains, Uptane director/image repos, HSM-held keys |
| A/B slots as directories | Raw partitions + bootloader slot switching (e.g. RAUC, OSTree, custom BL) |
| In-memory DTC store | NvM-backed UDS stack reachable via DoIP (ISO 13400) |
| `std::thread` cyclic task | OS task with deadline monitoring / AUTOSAR RTE runnable |

See `docs/PROJECT_DOCUMENTATION.md` for the full architecture rationale and interview Q&A.
