#!/usr/bin/env bash
# About 35 sec
# Is there an "rsync" version thats quicker?

source env.sh

echo "Cleaning board..."
# is there a way to quickly nuke the whole board?
# $AMPY rmdir / 2>/dev/null || true
echo "Removing rl78/..."
$AMPY rmdir rl78 2>/dev/null || true
echo "Removing files..."
for f in $($AMPY ls); do
    echo "  $f..."
    $AMPY rm $f
done

echo "Begin upload"
$AMPY mkdir rl78

# Full sync is somewhat slow
# Consider something like
# ampy --port /dev/ttyACM0 --baud 115200 put probe.py
echo "Uploading scripts..."
for f in $(ls *.py); do
    echo "  $f..."
    $AMPY put $f
done

echo "Uploading rl78/..."
$AMPY put rl78/

echo "Done!"
