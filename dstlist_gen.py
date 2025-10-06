# genx320_color_dark with_tracking.py

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
sensor.set_color_palette(image.PALETTE_EVT_DARK)

clock = time.clock()
stabilizer=0
while True:
    clock.tick()

    img = sensor.snapshot()
    # img.median(1) # noise cleanup.

    blobs = img.find_blobs(
        [(10, 20, -10, 10, -20, 0)], invert=True, pixels_threshold=2, area_threshold=4, merge=True
    )
    jumpers=[]
    for blob in blobs:
        img.draw_rectangle(blob.rect(), color=(255, 0, 0))
        img.draw_cross(blob.cx(), blob.cy(), color=(0, 255, 0))
        if stabilizer>20:
            jumpers.append((blob.cx(),blob.cy()))
    print(clock.fps())
    if stabilizer>20:
        break
    stabilizer+=1
print(jumpers)