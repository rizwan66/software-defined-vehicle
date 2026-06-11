# =============================================================================
# Multi-stage Dockerfile for the SDV Platform
#
# Stages:
#   builder  — compiles all targets with g++-12 and C++20
#   runtime  — minimal image containing only the demo binaries
#   test     — runs the full test suite (used in CI)
#
# Usage:
#   docker build -t sdv-platform .
#   docker run --rm sdv-platform                   # run enhanced demo
#   docker run --rm sdv-platform ./build/sdv_tests  # run test suite
# =============================================================================

# ── Stage 1: builder ──────────────────────────────────────────────────────────
FROM ubuntu:22.04 AS builder

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        g++-12 \
        cmake \
        make \
        python3 \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Use g++-12 as the default c++ compiler
RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-12 100 \
 && update-alternatives --install /usr/bin/c++ c++ /usr/bin/g++-12 100

WORKDIR /sdv

# Copy only what CMake needs first (better layer caching for headers-only changes)
COPY sdv-platform/CMakeLists.txt        sdv-platform/
COPY sdv-platform/drivers/              sdv-platform/drivers/
COPY sdv-platform/services/             sdv-platform/services/
COPY sdv-platform/middleware/           sdv-platform/middleware/
COPY sdv-platform/apps/                 sdv-platform/apps/
COPY sdv-platform/tests/               sdv-platform/tests/
COPY sdv-platform/tools/               sdv-platform/tools/

WORKDIR /sdv/sdv-platform

RUN cmake -B build \
          -DCMAKE_CXX_COMPILER=g++-12 \
          -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build --parallel "$(nproc)"

# ── Stage 2: runtime ──────────────────────────────────────────────────────────
FROM ubuntu:22.04 AS runtime

RUN apt-get update && apt-get install -y --no-install-recommends \
        libstdc++6 \
        python3 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /sdv/sdv-platform

COPY --from=builder /sdv/sdv-platform/build/sdv_demo          build/
COPY --from=builder /sdv/sdv-platform/build/sdv_demo_enhanced build/
COPY --from=builder /sdv/sdv-platform/build/sdv_tests         build/
COPY --from=builder /sdv/sdv-platform/tools/                  tools/

ENV OTA_ROOT=/tmp/sdv-ota-demo

CMD ["bash", "-c", \
     "python3 tools/generate_update_package.py \
        --out /tmp/sdv-ota-demo/incoming --version 1 && \
      rm -rf /tmp/sdv-ota-demo/slot_a /tmp/sdv-ota-demo/slot_b \
             /tmp/sdv-ota-demo/state && \
      ./build/sdv_demo_enhanced"]

# ── Stage 3: test ──────────────────────────────────────────────────────────────
FROM runtime AS test
CMD ["./build/sdv_tests"]
