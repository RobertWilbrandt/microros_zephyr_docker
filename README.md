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
- ```--volume```: Maps your custom apps into the image. This has to be an absoulte path, and it should contain the directories of your custom apps. The toolchain inside the image expects your apps to be mapped to ```/uros_apps``` and will error out if it doesn't find any there.

Example Applications
--------------------

### Serial communication with real board

Suppose you want to build one of the example applications distributed in [zephyr_apps](https://github.com/micro-ROS/zephyr_apps) as part of *micro-ROS*, for example *ping_pong*. You use the [STM32F429I-DISC1 discovery kit](https://docs.zephyrproject.org/latest/boards/arm/stm32f429i_disc1/doc/index.html).

The first step to achieve this consists of getting the application and building this docker image:
```console
$ git clone git@github.com:micro-ROS/zephyr_apps.git
$ git clone git@github.com:RobertWilbrandt/microros_zephyr_docker.git
$ docker build microros_zephyr_docker --tag wilbrandt/microros_zephyr_stm32f429i_disc1:foxy \
    --build-arg PLATFORM_BOARD=stm32f429i_disc1
```

(Okay i am cheating a bit here, as the *micro-ROS* applications in ```zephyr_apps``` will check if the board is supported, which is not the case for ```stm32f429i_disc1```. You can just comment out the check in ```CMakeLists.txt``` though and i will try to get a patch in that removes this for basic applications)

This will take a while, as a full ROS foxy image with a complete *micro-ROS* and *Zephyr* installation needs to be installed. When this is done, you can run the image like this:

```console
$ docker run -it --rm --privileged --volume $(pwd)/zephyr_apps/apps:/uros_apps \
    wilbrandt/microros_zephyr_stm32f429i_disc1:foxy
```

Inside this image you can now configure, build and flash the application. The example board is connected using a serial transport (piped through the on-board debugger connected via USB) and the *Zephyr* board page tells us we want to use ```UART1```.

```console
# ros2 run micro_ros_setup configure_firmware.sh ping_pong --transport serial --dev 1
# ros2 run micro_ros_setup build_firmware.sh -f
# ros2 run micro_ros_setup flash_firmware.sh
```

You have now flashed your *micro-ROS* image to your board! When developing, you might want to keep this docker image open (to prevent having to fully rebuild every time).

An easy way to connect to your MCU is to use the *micro-ROS* ```micro-ros-agent``` docker image:

```console
$ docker run -it --rm --privileged microros/micro-ros-agent:foxy serial --dev /dev/ttyACM0
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

### UDP-based communication with host emulation

Zephyr also allows you to build an application right on your computer, emulating some of the driver interfaces seamlessly (even complex peripherals like displays, network stacks and BLE). If you don't have access to a microcontroller board right now but still want to take a look at *micro-ROS*, i encourage getting started using this. It can also be incredibly useful for prototyping and debugging embedded applications. More information on this can be found in the [Zephyr documentation](https://docs.zephyrproject.org/latest/boards/posix/native_posix/doc/index.html).

Using this host emulation is nearly identical to the above example. You start by cloning your application (in this case ```int32_publisher```) and building this docker image:

```console
$ git clone git@github.com:micro-ROS/zephyr_apps.git
$ git clone git@github.com:RobertWilbrandt/microros_zephyr_docker.git
$ docker build microros_zephyr_docker --tag wilbrandt/microros_zephyr_host:foxy \
    --build-arg PLATFORM_BOARD=host
```

After this, start the container. Inside it, you can now configure and build an image. Note that we need to replace the ```--privileged``` flag with ```--net=host```, as we are now communicationg via network.

```console
$ docker run -it --rm --net=host --volume $(pwd)/zephyr_apps/apps:/uros_apps \
    wilbrandt/microros_zephyr_stm32f429i_disc1:foxy
# ros2 run micro_ros_setup configure_firmware.sh int32_publisher --transport udp --ip 127.0.0.1 --port 8888
# ros2 run micro_ros_setup build_firmware.sh -f
```

Note that we did not yet start the application (using ```flash_firmware.sh```), as this will just error out when it doesn't find an agent running. We can (again) start an agent in a seperate container, also changing the ```--privileged``` with ```--net=host```:

```console
$ docker run -it --rm --net=host microros/micro-ros-agent:foxy udp4 --port 8888
```

Now go back to your first container and start the application:

```console
# ros2 run micro_ros_setup flash_firmware.sh
```

You should see some messages in the agent container. In a normal ros2 foxy machine, your can now check the functionality:

```console
$ source /opt/ros/foxy/setup.bash
$ ros2 topic echo /zephyr_int32_publisher
```
