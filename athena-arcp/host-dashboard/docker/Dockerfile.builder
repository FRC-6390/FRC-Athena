FROM debian:bookworm

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_ID=1000
ARG GROUP_ID=1000
ARG BUN_VERSION=1.2.9

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    git \
    xz-utils \
    zip \
    unzip \
    build-essential \
    pkg-config \
    file \
    libgtk-3-dev \
    libwebkit2gtk-4.1-dev \
    libayatana-appindicator3-dev \
    librsvg2-dev \
    libssl-dev \
    libglib2.0-dev \
    patchelf \
    desktop-file-utils \
    fakeroot \
    rpm \
    dpkg-dev \
    binutils \
    python3 \
    && rm -rf /var/lib/apt/lists/*

RUN if [ "${GROUP_ID}" = "0" ]; then group_name="root"; else groupadd -g "${GROUP_ID}" builder; group_name="builder"; fi \
    && if [ "${USER_ID}" = "0" ]; then user_name="root"; else useradd -m -u "${USER_ID}" -g "${group_name}" -s /bin/bash builder; user_name="builder"; fi \
    && mkdir -p /workspace \
    && chown "${USER_ID}:${GROUP_ID}" /workspace || true

USER ${USER_ID}:${GROUP_ID}

ENV HOME=/home/builder
ENV CARGO_HOME=/home/builder/.cargo
ENV RUSTUP_HOME=/home/builder/.rustup
ENV PATH=/home/builder/.cargo/bin:/home/builder/.bun/bin:${PATH}

RUN curl https://sh.rustup.rs -sSf | sh -s -- -y --profile minimal \
    && rustup toolchain install stable \
    && rustup default stable

RUN curl -fsSL https://bun.sh/install | bash -s -- bun-v${BUN_VERSION}

WORKDIR /workspace

CMD ["bash", "-lc", "echo 'ARCP docker builder ready'"]
