# linux/amd64 SITL for native Rust modules: the PR's SITL runtime runs x86-64 Linux only.
FROM --platform=linux/amd64 ubuntu:22.04
ARG RUST_VERSION=1.99.1
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential git python3 python3-dev python3-pip python3-lxml rsync ca-certificates curl \
        pkg-config libxml2-dev libxslt1-dev procps \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir empy==3.3.4 future pexpect pyelftools pymavlink MAVProxy dronecan pytest \
    && pip3 freeze > /pip-freeze.txt
RUN curl -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal --default-toolchain ${RUST_VERSION} \
        --target x86_64-unknown-linux-gnu
ENV PATH=/root/.cargo/bin:$PATH
WORKDIR /work
