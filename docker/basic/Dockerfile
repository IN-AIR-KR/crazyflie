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

RUN groupadd -f plugdev \
    && groupadd -f dialout \
    && groupadd -f -g ${GID} dev \
    && useradd -m -u ${UID} -g ${GID} -G plugdev,dialout -s /bin/bash dev \
    && echo "dev ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# cflib is imported by crazyswarm2's crazyflie_server_py / flash.py but is not declared
# as a dependency anywhere upstream (no requirements.txt, no package.xml <depend>) — pin it here.
COPY requirements.txt /tmp/requirements.txt
RUN pip install --break-system-packages -r /tmp/requirements.txt

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER dev
WORKDIR /workspace/cf_ws

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
