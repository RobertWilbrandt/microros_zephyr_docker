FROM microros/base:foxy
LABEL maintainer="Robert Wilbrandt <robert@stamm-wilbrandt.de>"
LABEL description="Everything needed to start using micro-ROS with the Zephyr RTOS"

# This is a required parameter, so check that it is used
ARG PLATFORM_BOARD
RUN test -n "$PLATFORM_BOARD"

# This directory is set up in microros/base:foxy
WORKDIR /uros_ws

RUN . install/setup.sh \
 # Install zephyr dependencies as described in "Getting Started"
 && apt update \
 && apt install -y --no-install-recommends git cmake ninja-build gperf \
      ccache dfu-util device-tree-compiler wget \
      python3-dev python3-pip python3-setuptools python3-tk python3-wheel xz-utils file \
      make gcc gcc-multilib g++-multilib libsdl2-dev \
 # Set up actual micro-ROS workspace
 && ros2 run micro_ros_setup create_firmware_ws.sh zephyr $PLATFORM_BOARD

# Use --volume to map external apps into this container
ENV UROS_CUSTOM_APP_FOLDER=/uros_apps

# Set up custom entry point
COPY ./microros_zephyr_entrypoint.sh /microros_zephyr_entrypoint.sh
ENTRYPOINT ["/microros_zephyr_entrypoint.sh"]
CMD ["bash"]
