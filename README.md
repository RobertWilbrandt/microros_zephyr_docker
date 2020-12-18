microros_zephyr_docker
======================

This package supplies a docker image with everything needed to start using [micro-ROS](https://micro-ros.github.io/) with the [Zephyr RTOS](https://www.zephyrproject.org/).

**Disclaimer**: This package is still under construction and not yet ready for any serious use.

Building the image
------------------

You can build the image using

```console
$ git clone git@github.com:RobertWilbrandt/microros_zephyr_docker.git
$ docker build microros_zephyr_docker \
    --tag wilbrandt/microros_zephyr:foxy \
    --build-arg PLATFORM_BOARD=<Your Board>
```

The ```PLATFORM_BOARD``` argument specifies which board you are using. The format it uses depends on:
- If your board is already supported by *micro-ROS*, use the name shown in the [micro_ros_setup](https://github.com/micro-ROS/micro_ros_setup) supported platforms section.
- If your board is not yet supported by *micro-ROS* (but supported by *Zephyr*) and you just want to try out this configuration, use the name used by *Zephyr* (as found in its board side in the [documentation](https://docs.zephyrproject.org/latest/boards/index.html)). This will give you warnings during your build.

Running the image
-----------------

You can use this image just like any other docker image. Recommended usage is

```console
$ docker run -it --rm --privileged \
    --volume <absolute_path_to_your_apps_folder>:/uros_apps \
    wilbrandt/microros_zephyr:foxy
```

The effects of these flags are:
- ```-it```: Keeps ```STDIN``` open and creates a pseudo-TTY for you to interact with. With these two options you can use this like a normal shell.
- ```--rm```: Removes the container after it exits. You might not want to use this if you want to keep an internal state in the image and attach to it later on.
- ```--privileged```: This is needed to give the container all permissions needed to flash your microcontroller.
- ```--volume```: Maps your custom apps into the image. This has to be an absoulte path, and it should contain the directories of your custom apps.

Example Application
-------------------

Suppose you want to build one of the example applications distributed in [zephyr_apps](https://github.com/micro-ROS/zephyr_apps) as part of *micro-ROS*, for example *ping_pong*. You use the [STM32F429I-DISC1 discovery kit](https://docs.zephyrproject.org/latest/boards/arm/stm32f429i_disc1/doc/index.html).

The first step to achieve this consists of getting the application and building this docker image:
```console
$ git clone git@github.com:micro-ROS/zephyr_apps.git
$ git clone git@github.com:RobertWilbrandt/microros_zephyr_docker.git
$ docker build microros_zephyr_docker --tag wilbrandt/microros_zephyr:foxy \
    --build-arg PLATFORM_BOARD=stm32f429i_disc1
```

This will take a while, as a full ROS foxy image with a complete *micro-ROS* and *Zephyr* installation needs to be installed. When this is done, you can run the image like this:

```console
$ docker run -it --rm --privileged --volume $(pwd)/zephyr_apps/apps:/uros_apps \
    wilbrandt/microros_zephyr:foxy
```

Inside this image you can now configure, build and flash the application. The example board is connected using a serial transport (piped through the on-board debugger connected via USB) and the *Zephyr* board page tells us we want to use ```UART1```.

```console
$ ros2 run micro_ros_setup configure_firmware.sh ping_pong --transport serial --dev 1
$ ros2 run micro_ros_setup build_firmware.sh -f
$ ros2 run micro_ros_setup flash_firmware.sh
```

You have now flashed your *micro-ROS* image to your board! When developing, you might want to keep this docker image open (to prevent having to fully rebuild every time).

An easy way to connect to your MCU is to use the *micro-ROS* ```micro-ros-agent``` docker image:

```console
$ docker run -it --rm --privileged microros/micro-ros-agent:latest serial --dev /dev/ttyACM0
```

Make sure to replace the ```/dev/ttyACM0``` with your actual device.

Now you should be ready to try out your demo: Open two shells and enter:
```console
# Shell 1
$ source /opt/ros/foxy/setup.bash
$ ros2 topic echo /microROS/pong
```

```console
# Shell 2
$ source /opt/ros/foxy/setup.bash
$ ros2 topic pub -1 /microROS/ping std_msgs/msg/Header "frame_id: 'Hello World!'"
```

You should now see your message in the first shell.
