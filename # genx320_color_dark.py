# genx320_color_dark.py

# This work is licensed under the MIT license.
# Copyright (c) 2013-2024 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# This example shows off using the genx320 event camera from Prophesee.

import sensor
import image
import time

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)  # Must always be grayscale.
sensor.set_framesize(sensor.B320X320)  # Must always be 320x320.
sensor.set_color_palette(image.PALETTE_EVT_LIGHT)  # Set color palette for event camera.

clock = time.clock()

while True:
    clock.tick()

    img = sensor.snapshot()
    # img.median(1) # noise cleanup.

    print(clock.fps())
