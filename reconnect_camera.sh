#!/bin/bash

echo "Attempting to reconnect GenX320 camera..."

# Reset USB port (requires sudo)
sudo usbreset $(lsusb | grep "OpenMV" | cut -d' ' -f6)

# Set permissions
sleep 2
if [ -e /dev/ttyACM0 ]; then
    sudo chmod 666 /dev/ttyACM0
    echo "Camera reconnected at /dev/ttyACM0"
else
    echo "Camera not found. Try physically unplugging and reconnecting."
fi
