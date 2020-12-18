FROM microros/base:foxy
LABEL maintainer="Robert Wilbrandt <robert@stamm-wilbrandt.de>"
LABEL description="Everything needed to start using micro-ROS with the Zephyr RTOS"

ARG PLATFORM_BOARD
RUN test -n "$PLATFORM_BOARD"

WORKDIR /uros_ws

RUN . install/setup.sh \
 && apt update \
 && apt install -y --no-install-recommends git cmake ninja-build gperf \
      ccache dfu-util device-tree-compiler wget \
      python3-dev python3-pip python3-setuptools python3-tk python3-wheel xz-utils file \
      make gcc gcc-multilib g++-multilib libsdl2-dev \
 && ros2 run micro_ros_setup create_firmware_ws.sh zephyr $PLATFORM_BOARD
