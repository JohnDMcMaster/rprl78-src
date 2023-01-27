#!/usr/bin/env bash

source env.sh

# Full sync is somewhat slow
# Consider something like
# ampy --port /dev/ttyACM0 --baud 115200 put probe.py
for f in $(ls *.py */*.py); do
    echo "Uploading $f..."
    ampy --port $PORT --baud $BAUD put $f
done

