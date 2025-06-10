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
# sensor.set_color_palette(image.PALETTE_EVT_DARK)
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
            [(200, 255)], invert=False, pixels_threshold=10, area_threshold=10, merge=True
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
        # Ensure img is defined here by taking a new snapshot
        img = sensor.snapshot()
        x, y, w, h = max_blob.rect()
        img.draw_rectangle(x, y, w, h, color=(255, 0, 0))
        img.draw_cross(max_blob.cx(), max_blob.cy(), color=(0, 255, 0))
    return center

print("Detecting LED blob...")
led_center = detect_led_center_via_blobs(duration_s=2)
print("Detected LED center:", led_center)
if not led_center:
    print("WARNING: No LED blob detected! Using default center (160, 160).")
    led_center = (160, 160)
px, py = led_center

# --- Frequency detection for the detected LED ---
def measure_led_frequency(led_center, duration_ms=100):  # Use 100ms for 10Hz/20Hz pair
    px, py = led_center
    transition_times = []
    transition_times2 = []
    prev_val = 0
    prev_val2 = 0
    prev_polarity = 0
    prev_polarity2 = 0
    last_transition_time = 0
    last_transition_time2 = 0

    # Estimate threshold from initial samples
    threshold_samples = []
    threshold_samples2 = []
    for _ in range(10):
        img = sensor.snapshot()
        val = img.get_pixel(px, py)
        val2 = img.get_pixel(px+5, py+5)
        if isinstance(val, tuple):
            val = val[0]
        if isinstance(val2, tuple):
            val2 = val2[0]
        threshold_samples.append(val)
        threshold_samples2.append(val2)
    if threshold_samples:
        min_val = min(threshold_samples)
        max_val = max(threshold_samples)
        if isinstance(min_val, tuple):
            min_val = min_val[0]
        if isinstance(max_val, tuple):
            max_val = max_val[0]
        threshold = (max_val + min_val) / 2.0
    else:
        threshold = 127
    if threshold_samples2:
        min_val2 = min(threshold_samples2)
        max_val2 = max(threshold_samples2)
        if isinstance(min_val2, tuple):
            min_val2 = min_val2[0]
        if isinstance(max_val2, tuple):
            max_val2 = max_val2[0]
        threshold2 = (max_val2 + min_val2) / 2.0
    else:
        threshold2 = 127

    img = sensor.snapshot()
    prev_val = img.get_pixel(px, py)
    prev_val2 = img.get_pixel(px+5, py+5)
    if isinstance(prev_val, tuple):
        prev_val = prev_val[0]
    if isinstance(prev_val2, tuple):
        prev_val2 = prev_val2[0]
    prev_polarity = 1 if prev_val > threshold else -1
    prev_polarity2 = 1 if prev_val2 > threshold2 else -1
    start = time.ticks_ms()

    while time.ticks_diff(time.ticks_ms(), start) < duration_ms:
        img = sensor.snapshot()
        now = time.ticks_ms()
        # Remove debug prints to reduce processing overhead
        val = img.get_pixel(px, py)
        val2 = img.get_pixel(px+5, py+5)
        if isinstance(val, tuple):
            val = val[0]
        if isinstance(val2, tuple):
            val2 = val2[0]
        polarity = 1 if val > threshold else -1
        polarity2 = 1 if val2 > threshold2 else -1
        if polarity != prev_polarity:
            if last_transition_time == 0 or (now - last_transition_time) > 5:
                transition_times.append(now)
                last_transition_time = now
        if polarity2 != prev_polarity2:
            if last_transition_time2 == 0 or (now - last_transition_time2) > 5:
                transition_times2.append(now)
                last_transition_time2 = now
        prev_polarity = polarity
        prev_polarity2 = polarity2

    # Add validation for minimum transitions
    min_transitions = max(2, (duration_ms / 100.0) * 2)  # Expect at least 2 transitions per 100ms
    if len(transition_times) < min_transitions:
        print(f"Warning: Too few transitions ({len(transition_times)}) for reliable measurement")
    
    cycles = len(transition_times) // 2
    freq_hz = cycles / (duration_ms / 1000.0)
    cycles2 = len(transition_times2) // 2
    freq_hz2 = cycles2 / (duration_ms / 1000.0)
    print(f"Transitions: {len(transition_times)} detected at {led_center}")
    print(f"Frequency detected: {freq_hz:.2f}")
    print(f"Transitions2: {len(transition_times2)} detected at ({px+5}, {py+5})")
    print(f"Frequency2 detected: {freq_hz2:.2f}")
    return freq_hz, freq_hz2

# Measure frequency with 100ms duration
freq, freq2 = measure_led_frequency(led_center, duration_ms=100) ## for the  10Hz/20Hz pair , 2 transitions detected for 10 Hz ,  because 100ms is the LCM of 100ms and 50ms
# and 4  transitions detected for 20Hz.
# freq += 5  # Add 5 Hz offset

# Send frequency via UART
# freq = 15
msg = f"{freq:.2f}\n"
if uart is not None:
    uart.write(msg)
    print("Sent frequency:", msg.strip())
else:
    print("UART not available, frequency:", msg.strip())