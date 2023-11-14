FROM mcr.microsoft.com/devcontainers/cpp:1-ubuntu-22.04

# [Optional] Uncomment this section to install additional packages.
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends unzip cmake build-essential wget ca-certificates \
    gdb-multiarch automake autoconf libtool libftdi-dev libusb-1.0-0-dev pkg-config clang-format libhidapi-dev

WORKDIR /opt

RUN wget -O arm-none-eabi-gcc.tar.xz "https://developer.arm.com/-/media/Files/downloads/gnu/13.2.rel1/binrel/arm-gnu-toolchain-13.2.rel1-x86_64-arm-none-eabi.tar.xz" \
    && tar -xvf arm-none-eabi-gcc.tar.xz \
    && mv arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-eabi arm-none-eabi-gcc \
    && rm -rf arm-none-eabi-gcc.tar.xz \
    
RUN export PATH=$PATH:/opt/arm-none-eabi-gcc/bin

WORKDIR /apps

RUN git clone -b master https://github.com/raspberrypi/pico-sdk.git --depth=1 && git -C pico-sdk submodule update --init
ENV PICO_SDK_PATH=/apps/pico-sdk

RUN cd /apps && git clone https://github.com/raspberrypi/picotool.git --depth=1 \
    && cd picotool && mkdir build && cd build && cmake ../ && make -j 8 && cp picotool /usr/local/bin

RUN cd /apps && git clone https://github.com/wtarreau/bootterm.git --depth=1 \
    && cd bootterm && make -j 8 install 

RUN cd /apps && wget https://github.com/sparkiebot/sparkie_uros_sdk/releases/latest/download/libmicroros.zip \
    && unzip libmicroros.zip && rm libmicroros.zip
ENV MICRO_ROS_PICO=/apps/libmicroros

ENV PATH "$PATH:/opt/arm-none-eabi-gcc/bin"

RUN usermod -aG plugdev vscode
