#!/usr/bin/env bash
mkdir -p src_build
cp sketch.ino src_build/src_build.ino
./bin/arduino-cli compile --fqbn esp32:esp32:esp32 --build-path build src_build/src_build.ino
