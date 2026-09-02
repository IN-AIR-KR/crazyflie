FROM ubuntu:24.04

ARG UID=1000
ARG GID=1000

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential git curl ca-certificates make sudo \
    libusb-1.0-0 libusb-1.0-0-dev \
    libgl1 libglib2.0-0 libxkbcommon-x11-0 libxcb-cursor0 \
    libxcb-xinerama0 libxcb-icccm4 libxcb-image0 libxcb-keysyms1 \
    libxcb-randr0 libxcb-render-util0 libxcb-shape0 \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd -f plugdev \
    && groupadd -f dialout \
    && groupadd -f -g ${GID} dev \
    && useradd -m -u ${UID} -g ${GID} -G plugdev,dialout -s /bin/bash dev \
    && echo "dev ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER dev
WORKDIR /home/dev

# pixi drives the STM32 cross-toolchain (xpack-gcc-arm-none-eabi) and cfclient per crazyflie-firmware/pixi.toml
RUN curl -fsSL https://pixi.sh/install.sh | bash
ENV PATH="/home/dev/.pixi/bin:$PATH"

COPY --chown=dev:dev entrypoint.sh /home/dev/entrypoint.sh
USER root
RUN chmod +x /home/dev/entrypoint.sh
USER dev

WORKDIR /workspace/crazyflie-firmware

ENTRYPOINT ["/home/dev/entrypoint.sh"]
CMD ["bash"]
