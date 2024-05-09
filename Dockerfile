FROM alpine:3.17.0

# Install toolchain
RUN apk update && \
    apk upgrade && \
    apk add git \
            python3 \
            py3-pip \
            cmake \
            build-base \
            libusb-dev \
            bsd-compat-headers \
            newlib-arm-none-eabi \
            gcc-arm-none-eabi \
            valgrind