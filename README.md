microros_zephyr_docker
======================

This package supplies a docker image with everything needed to start using [micro-ROS](https://micro-ros.github.io/) with the [Zephyr RTOS](https://www.zephyrproject.org/).

**Disclaimer**: This package is still under construction and not yet ready for any serious use.

Building the image
------------------

You can build the image using

```console
git clone git@github.com:RobertWilbrandt/microros_zephyr_docker.git
docker build microros_zephyr_docker --tag wilbrandt/microros_zephyr:foxy --build-arg PLATFORM_BOARD=<Your Board>
```

Running the image
-----------------

You can use this image just like any other docker image. Recommended usage is

```console
docker run -it --rm wilbrandt/microros_zephyr:foxy
```

The effects of these flags are:
- ```-it```: Keeps ```STDIN``` open and creates a pseudo-TTY for you to interact with. With these two options you can use this like a normal shell.
- ```--rm```: Removes the container after it exits. You might not want to use this if you want to keep an internal state in the image and attach to it later on.
