#!/usr/bin/env sh

mkdir -p build
make CC_NATIVE=i686-w64-mingw32-gcc build/fernly-usb-loader
mv build/fernly-usb-loader build/fernly-usb-loader.exe

