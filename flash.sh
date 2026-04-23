#!/usr/bin/env bash
PORT=${1:-/dev/ttyUSB0}

echo "Building project..."
./build.sh

echo "Requesting sudo to set USB port permissions..."
sudo chmod 666 $PORT

echo "Flashing to $PORT..."
./bin/arduino-cli upload -p $PORT --fqbn esp32:esp32:esp32 --build-path build src_build/src_build.ino

echo "Flash complete!"

PORT=${1:-/dev/ttyUSB0}
BAUD=${2:-115200}

echo "Starting serial monitor on $PORT at $BAUD baud..."
echo "Press Ctrl+C to exit."
./bin/arduino-cli monitor -p $PORT -c $BAUD