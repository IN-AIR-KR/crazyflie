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
    libegl1 libfontconfig1 libfreetype6 libdbus-1-3 libpcsclite1 libpulse0 \
    && rm -rf /var/lib/apt/lists/*

# Ubuntu 24.04 base images ship a default "ubuntu" user/group at UID/GID 1000, which
# collides with the common host UID/GID we want for "dev" — remove it first if present.
RUN existing_user=$(getent passwd ${UID} | cut -d: -f1); \
    if [ -n "$existing_user" ]; then userdel -r "$existing_user" 2>/dev/null || userdel "$existing_user"; fi; \
    existing_group=$(getent group ${GID} | cut -d: -f1); \
    if [ -n "$existing_group" ]; then groupdel "$existing_group" 2>/dev/null || true; fi; \
    groupadd -f plugdev \
    && groupadd -f dialout \
    && groupadd -f -g ${GID} dev \
    && useradd -m -u ${UID} -g ${GID} -G plugdev,dialout -s /bin/bash dev \
    && echo "dev ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# No udev rule ships a Crazyradio permission fix in this repo, so /dev/bus/usb nodes
# keep whatever host-side ownership they had (often root:root). Re-run this inside the
# container any time you hot-plug the Crazyradio/Crazyflie after the container started.
RUN printf '#!/usr/bin/env bash\nsudo chmod -R o+rw /dev/bus/usb 2>/dev/null || true\n' > /usr/local/bin/fix-usb-perms \
    && chmod +x /usr/local/bin/fix-usb-perms

# Pre-create the cfclient config dir so the named volume mounted here at runtime
# inherits "dev" ownership on first use instead of defaulting to root.
RUN mkdir -p /home/dev/.config/cfclient && chown -R dev:dev /home/dev/.config

USER dev
WORKDIR /home/dev

# pixi drives the STM32 cross-toolchain (xpack-gcc-arm-none-eabi) and cfclient per crazyflie-firmware/pixi.toml
RUN curl -fsSL https://pixi.sh/install.sh | bash
ENV PATH="/home/dev/.pixi/bin:$PATH"

# The above PATH covers the `pixi` launcher itself; cfclient and friends live in the
# *project-local* env pixi installs per pixi.toml, which `pixi run <cmd>` activates
# on the fly. Putting it in PATH directly (an image ENV, not just .bashrc) means a
# plain `cfclient` works both interactively and via `docker compose exec firmware cfclient`.
ENV PATH="/workspace/crazyflie-firmware/.pixi/envs/default/bin:$PATH"

COPY --chown=dev:dev docker/firmware/entrypoint.sh /home/dev/entrypoint.sh
USER root
RUN chmod +x /home/dev/entrypoint.sh
USER dev

WORKDIR /workspace/crazyflie-firmware

ENTRYPOINT ["/home/dev/entrypoint.sh"]
CMD ["bash"]
