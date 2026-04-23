#!/usr/bin/env bash
PORT=${1:-/dev/ttyUSB0}
BAUD=${2:-115200}

echo "Starting serial monitor on $PORT at $BAUD baud..."
echo "Press Ctrl+C to exit."
./bin/arduino-cli monitor -p $PORT -c $BAUD
