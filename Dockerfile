# Use Ubuntu 22.04 as base image
FROM ubuntu:22.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    libboost-all-dev \
    pkg-config \
    git \
    curl \
    gnupg \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# Create workspace directory
WORKDIR /workspace

# Copy source code
COPY . /workspace/ddsm115_driver/

# Build the library
WORKDIR /workspace/ddsm115_driver
RUN mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    make install && \
    ldconfig

# Set up entry point
WORKDIR /workspace
CMD ["/bin/bash"]

# Labels for metadata
LABEL org.opencontainers.image.title="DDSM115 Driver Library"
LABEL org.opencontainers.image.description="C++ library for controlling DDSM115 servo motors via RS485"
LABEL org.opencontainers.image.source="https://github.com/your-username/ddsm115_driver"
LABEL org.opencontainers.image.licenses="MIT"
