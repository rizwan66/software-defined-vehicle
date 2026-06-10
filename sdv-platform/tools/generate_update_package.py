#!/usr/bin/env python3
"""
tools/generate_update_package.py

Backend side of the OTA flow: builds and signs an update package that
services/ota/OtaManager will accept.

  payload (firmware.bin)  -> sha256 pinned in manifest
  manifest canonical form -> HMAC-SHA256 signature (demo key)

Production equivalent: the OEM signing backend holds an asymmetric private
key in an HSM (Ed25519 / ECDSA-P256, Uptane director+image repos); the
vehicle only ever holds the public key. The canonicalization and
verify-before-install flow are identical.

Usage:
  python3 tools/generate_update_package.py \
      --out /tmp/sdv-ota-demo/incoming \
      --name gateway-fw --hw ECU-GW-001 --version 2
"""
import argparse
import hashlib
import hmac
import json
import os

DEMO_KEY = b"demo-key"  # must match the key provisioned in the demo ECU


def canonical_bytes(m: dict) -> bytes:
    return (
        f"name={m['name']}\n"
        f"hardware_id={m['hardware_id']}\n"
        f"version={m['version']}\n"
        f"payload_file={m['payload_file']}\n"
        f"payload_sha256={m['payload_sha256']}\n"
    ).encode()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--name", default="gateway-fw")
    ap.add_argument("--hw", default="ECU-GW-001")
    ap.add_argument("--version", type=int, default=1)
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)

    # 1. Payload — a stand-in firmware image.
    payload_name = "firmware.bin"
    payload_path = os.path.join(args.out, payload_name)
    with open(payload_path, "wb") as f:
        f.write(os.urandom(4096))
        f.write(f"FW {args.name} v{args.version}".encode())

    with open(payload_path, "rb") as f:
        digest = hashlib.sha256(f.read()).hexdigest()

    # 2. Signed manifest.
    manifest = {
        "name": args.name,
        "hardware_id": args.hw,
        "version": args.version,
        "payload_file": payload_name,
        "payload_sha256": digest,
    }
    manifest["signature"] = hmac.new(
        DEMO_KEY, canonical_bytes(manifest), hashlib.sha256
    ).hexdigest()

    with open(os.path.join(args.out, "manifest.json"), "w") as f:
        json.dump(manifest, f, indent=2)

    # 3. A tampered variant (version bumped after signing) to demonstrate
    #    that the vehicle rejects modified manifests.
    tampered = dict(manifest)
    tampered["version"] = manifest["version"] + 100  # attacker edit
    with open(os.path.join(args.out, "manifest_tampered.json"), "w") as f:
        json.dump(tampered, f, indent=2)

    print(f"package written to {args.out}")
    print(f"  payload sha256: {digest}")
    print(f"  signature:      {manifest['signature']}")


if __name__ == "__main__":
    main()
