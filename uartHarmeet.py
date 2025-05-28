import sensor
import time
try:
    import pyb
except ImportError:
    pyb = None  # or handle accordingly if running off hardware

import image

# Sensor setup
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.B320X320)
sensor.set_color_palette(image.PALETTE_EVT_DARK)
sensor.ioctl(sensor.IOCTL_GENX320_SET_BIAS, sensor.GENX320_BIAS_REFR,1)
sensor.ioctl(sensor.IOCTL_GENX320_SET_BIAS, sensor.GENX320_BIAS_DIFF_ON, 20)
sensor.ioctl(sensor.IOCTL_GENX320_SET_BIAS, sensor.GENX320_BIAS_DIFF_OFF, 20)

sensor.set_framerate(60)
sensor.skip_frames(time=2000)

# UART setup
if pyb is not None:
    uart = pyb.UART(3, 19200)
else:
    uart = None

# --- Blob-based LED center detection using your reference blob creation ---
def detect_led_center_via_blobs(duration_s=2):
    start = time.ticks_ms()
    center = None
    max_blob = None
    max_area = 0
    while time.ticks_diff(time.ticks_ms(), start) < duration_s * 1000:
        img = sensor.snapshot()
        # Use a simple grayscale threshold for bright spots
        blobs = img.find_blobs(
            [(150, 255)], invert=False, pixels_threshold=10, area_threshold=10, merge=True
        )
        # print("Blobs found:", len(blobs) if blobs else 0)
        if blobs:
            # Sort blobs by area, largest first
            blob = max(blobs, key=lambda b: b.pixels())
            area = blob.pixels()
            if area > max_area:
                max_area = area
                max_blob = blob
    if max_blob:
        center = (int(max_blob.cx()), int(max_blob.cy()))
        # Draw for visualization (optional)
        x, y, w, h = max_blob.rect()
        img = sensor.snapshot()  # Ensure img is defined here
        img.draw_rectangle(x, y, w, h, color=(255, 0, 0))
        img.draw_cross(max_blob.cx(), max_blob.cy(), color=(0, 255, 0))
    return center

print("Detecting LED blob...")
led_center = detect_led_center_via_blobs(duration_s=2)
print("Detected LED center:", led_center)
if not led_center:
    print("WARNING: No LED blob detected! Using default center (160, 160).")
    led_center = (171,171)
px, py = led_center

# --- Frequency detection for the detected LED ---
def measure_led_frequency(led_center, duration_ms=500):
        px, py = led_center
        transition_times = []
        val_list=[]
        polarity_list = []
        transitionmomentlist = []
        prevValueList = []
        allvaluesafetr17milliseconds = []
        last_transition_time = 0

        # Estimate threshold from initial samples
        threshold_samples = []

        img = sensor.snapshot()
        first_val = img.get_pixel(px, py)
        img = sensor.snapshot()
        prev_val=img.get_pixel(px,py)
        if isinstance(first_val, tuple):
                first_val = first_val[0]
        if isinstance(prev_val, tuple):
                prev_val=prev_val[0]
        prev_polarity=None
        start = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start) < duration_ms:
            # print(time.ticks_diff(time.ticks_ms(), start))
            img = sensor.snapshot()
            now = time.ticks_ms()
            val = img.get_pixel(px, py)
            if isinstance(val, tuple):
                val = val[0]
            polarity = 1 if val >= prev_val else -1
            
            if prev_polarity is None:
                prev_polarity=polarity
                continue
            if polarity != prev_polarity:
                if last_transition_time == 0 or (now - last_transition_time) > 5:
                    transition_times.append(now)
                    # print(f"Transition at {now} ms: {val} (polarity: {polarity})")
                    last_transition_time = now
                    prevValueList.append(prev_val)
                    allvaluesafetr17milliseconds.append(val)
                    val_list.append(val)
                    polarity_list.append(polarity)
                    transitionmomentlist.append(time.ticks_diff(time.ticks_ms(), start))
            prev_polarity = polarity
            prev_val=val
        print("the values at these times",val_list)
        print("the previous values",prev_val)
        print("the transition times",transition_times)
        print("the polarities at these times",polarity_list)
        print("the transition moments",transitionmomentlist)
        print("the values after 17 milliseconds",allvaluesafetr17milliseconds)
        
        
        
        
        cycles = len(transition_times) // 2
        freq_hz = cycles / (duration_ms / 1000.0)
        freq_hz = freq_hz/2  # Adjust for half-cycle counting
        print(f"Transitions: {len(transition_times)} detected at {led_center}")
        print(f"Frequency detected: {freq_hz:.2f}")
        return freq_hz
    
    
# Measure frequency for the detected LED
freq = measure_led_frequency(led_center, duration_ms = 1000)

# freq += 5  # Add 5 Hz offset

# Send frequency via UART
msg = f"{freq:.2f}\n"
if uart is not None:
    uart.write(msg)
    print("Sent frequency:", msg.strip())
else:
    print("UART not available, frequency:", msg.strip())