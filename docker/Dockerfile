# Build environment for volumetric-contact: Ubuntu 24.04 + Drake (C++) + Project Chrono + MuJoCo.
# Drake no longer ships binaries for Ubuntu 22.04, so all C++ builds run here.
FROM ubuntu:24.04
LABEL org.opencontainers.image.source=https://github.com/dvilelaf/volumetric-contact

ARG DRAKE_VERSION=1.58.0
ARG DRAKE_DEB=drake-dev_${DRAKE_VERSION}-1_amd64-noble.deb
ARG CHRONO_VERSION=10.0.0

RUN apt-get update \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      build-essential ca-certificates ccache cmake curl git libeigen3-dev ninja-build \
 && cd /tmp \
 && curl -fsSLO https://github.com/RobotLocomotion/drake/releases/download/v${DRAKE_VERSION}/${DRAKE_DEB} \
 && curl -fsSLO https://github.com/RobotLocomotion/drake/releases/download/v${DRAKE_VERSION}/${DRAKE_DEB}.sha256 \
 && sha256sum -c ${DRAKE_DEB}.sha256 \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends ./${DRAKE_DEB} \
 && rm -rf /var/lib/apt/lists/* /tmp/${DRAKE_DEB}*

# Chrono core only. SIMD flags stay generic (no -march=native) so that its Eigen
# ABI matches the other translation units of the same executable.
RUN cd /tmp \
 && curl -fsSL -o chrono.tar.gz https://github.com/projectchrono/chrono/archive/refs/tags/${CHRONO_VERSION}.tar.gz \
 && mkdir chrono && tar xzf chrono.tar.gz -C chrono --strip-components=1 \
 && cmake -S chrono -B chrono-build -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/opt/chrono \
      -DCH_USE_SIMD=OFF -DUSE_SIMD=OFF \
      -DCH_ENABLE_OPENMP=OFF -DENABLE_OPENMP=OFF \
      -DBUILD_DEMOS=OFF -DBUILD_TESTING=OFF -DBUILD_BENCHMARKING=OFF \
      -DCH_BUILD_DEMOS=OFF -DCH_BUILD_TESTS=OFF -DCH_BUILD_BENCHMARKS=OFF \
 && cmake --build chrono-build \
 && cmake --install chrono-build \
 && rm -rf /tmp/chrono /tmp/chrono-build /tmp/chrono.tar.gz

ENV CMAKE_PREFIX_PATH=/opt/drake:/opt/chrono
ENV LD_LIBRARY_PATH=/opt/chrono/lib

# MuJoCo: the official release binaries (libmujoco.so and headers; no CMake
# config, see cmake/FindMuJoCo.cmake). Kept last so the layers above are reused.
ARG MUJOCO_VERSION=4.0.0
ARG MUJOCO_SHA256=3d5d2d96b6d286b758c75198aa7862590c960256626d3aac53ac5721b0a697f3
RUN curl -fsSL -o /tmp/mujoco.tar.gz https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz \
 && echo "${MUJOCO_SHA256}  /tmp/mujoco.tar.gz" | sha256sum -c - \
 && mkdir /opt/mujoco \
 && tar xzf /tmp/mujoco.tar.gz -C /opt/mujoco --strip-components=1 \
 && rm -rf /tmp/mujoco.tar.gz /opt/mujoco/bin /opt/mujoco/model /opt/mujoco/sample /opt/mujoco/simulate /opt/mujoco/wasm

ENV MUJOCO_ROOT=/opt/mujoco
ENV LD_LIBRARY_PATH=/opt/chrono/lib:/opt/mujoco/lib
