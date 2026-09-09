FROM ubuntu:24.04
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    build-essential cmake ninja-build clang clang-tidy python3 python3-pip \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /workspace
COPY . .
RUN cmake -S . -B build/docker -G Ninja -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build/docker --parallel \
    && ctest --test-dir build/docker --output-on-failure
CMD ["./build/docker/robotics_control_demo", "trajectory.csv"]
