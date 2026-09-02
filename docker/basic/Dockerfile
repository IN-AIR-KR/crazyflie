FROM osrf/ros:jazzy-desktop

ARG UID=1000
ARG GID=1000

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential git curl sudo \
    python3-pip python3-colcon-common-extensions python3-rosdep \
    libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true && rosdep update

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

# Pre-create the workspace tree so the named volumes mounted at build/install/log
# (see docker/compose.yml) inherit "dev" ownership on first use instead of root.
RUN mkdir -p /workspace/cf_ws/src /workspace/cf_ws/build /workspace/cf_ws/install /workspace/cf_ws/log \
    && chown -R dev:dev /workspace/cf_ws

# cflib is imported by crazyswarm2's crazyflie_server_py / flash.py but is not declared
# as a dependency anywhere upstream (no requirements.txt, no package.xml <depend>) — pin it here.
COPY docker/basic/requirements.txt /tmp/requirements.txt
# --ignore-installed avoids pip trying (and failing) to uninstall apt-managed packages
# like PyYAML that ROS/rosdep already installed via dpkg without pip RECORD metadata.
RUN pip install --break-system-packages --ignore-installed -r /tmp/requirements.txt

COPY docker/basic/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# entrypoint.sh sources ROS/the workspace overlay only for its own PID-1 shell —
# `docker compose exec basic bash` starts a brand-new shell that never sees that,
# so `ros2`/colcon's ament_cmake lookup fail there unless we also source on login.
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /home/dev/.bashrc \
    && echo 'if [ -f /workspace/cf_ws/install/setup.bash ]; then source /workspace/cf_ws/install/setup.bash; fi' >> /home/dev/.bashrc

USER dev
WORKDIR /workspace/cf_ws

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
