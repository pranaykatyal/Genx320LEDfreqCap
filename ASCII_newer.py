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
sensor.set_framerate(60)
sensor.skip_frames(time=2000)

# UART setup
if pyb is not None:
    uart = pyb.UART(3, 19200)
else:
    uart = None

# --- Blob-based LED center detection ---
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
        img = sensor.snapshot()
        x, y, w, h = max_blob.rect()
        img.draw_rectangle(x, y, w, h, color=(255, 0, 0))
        img.draw_cross(max_blob.cx(), max_blob.cy(), color=(0, 255, 0))
    return center

# --- Robust frequency detection based on transition count ---
def measure_led_frequency_robust(led_center, duration_ms=100):
    px, py = led_center
    transition_times = []
    prev_polarity = 0
    last_transition_time = 0

    # Estimate threshold from initial samples
    threshold_samples = []
    for _ in range(10):
        img = sensor.snapshot()
        val = img.get_pixel(px, py)
        if isinstance(val, tuple):
            val = val[0]
        threshold_samples.append(val)
    
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

    # Initialize with first reading
    img = sensor.snapshot()
    prev_val = img.get_pixel(px, py)
    if isinstance(prev_val, tuple):
        prev_val = prev_val[0]
    prev_polarity = 1 if prev_val > threshold else -1
    
    start = time.ticks_ms()

    while time.ticks_diff(time.ticks_ms(), start) < duration_ms:
        img = sensor.snapshot()
        now = time.ticks_ms()
        val = img.get_pixel(px, py)
        if isinstance(val, tuple):
            val = val[0]
        
        polarity = 1 if val > threshold else -1
        
        # Detect transition
        if polarity != prev_polarity:
            if last_transition_time == 0 or (now - last_transition_time) > 5:
                transition_times.append(now)
                last_transition_time = now
        
        prev_polarity = polarity

    # Robust frequency determination based on transition count
    transition_count = len(transition_times)
    
    # Logic: 1-2 transitions = 10Hz, 3-4 transitions = 20Hz
    if transition_count <= 2:
        detected_freq = 10.0
        freq_class = "10Hz"
    elif transition_count >= 3:
        detected_freq = 20.0
        freq_class = "20Hz"
    else:
        detected_freq = 10.0  # Default to 10Hz for edge cases
        freq_class = "10Hz (default)"
    
    print(f"Transitions: {transition_count} detected at {led_center} -> {freq_class}")
    
    return detected_freq

# --- Convert frequency list to binary ---
def frequencies_to_binary(freq_list):
    """Convert frequency list to binary (20Hz = 1, 10Hz = 0)"""
    binary_list = []
    for freq in freq_list:
        # More robust frequency classification
        if freq < 5:  # Handle very low frequencies as errors
            print(f"Warning: Very low frequency {freq:.1f} Hz detected, treating as 10 Hz")
            binary_list.append(0)
        elif abs(freq - 20) < abs(freq - 10):
            binary_list.append(1)  # Closer to 20Hz = 1
        else:
            binary_list.append(0)  # Closer to 10Hz = 0
    return binary_list

# --- Convert binary list to ASCII ---
def binary_to_ascii(binary_list):
    """Convert binary list to ASCII characters"""
    ascii_chars = []
    
    # Process in groups of 8 bits
    for i in range(0, len(binary_list), 8):
        byte_bits = binary_list[i:i+8]
        
        # If we don't have 8 bits, pad with zeros
        while len(byte_bits) < 8:
            byte_bits.append(0)
        
        # Convert 8 bits to decimal
        decimal_value = 0
        for j, bit in enumerate(byte_bits):
            decimal_value += bit * (2 ** (7 - j))
        
        # Convert to ASCII character (if printable)
        if 32 <= decimal_value <= 126:  # Printable ASCII range
            ascii_chars.append(chr(decimal_value))
        else:
            ascii_chars.append(f"[{decimal_value}]")  # Non-printable characters
    
    return ascii_chars

# --- Synchronize with LED frequency changes ---
def wait_for_frequency_sync(led_center, timeout_ms=5000):
    """Wait for a frequency transition to synchronize timing"""
    print("Synchronizing with LED frequency changes...")
    
    # Get initial frequency using robust detection
    initial_freq = measure_led_frequency_robust(led_center, duration_ms=100)
    print(f"Initial frequency: {initial_freq:.1f} Hz")
    
    start_time = time.ticks_ms()
    
    # Wait for frequency to change (indicating start of new cycle)
    while time.ticks_diff(time.ticks_ms(), start_time) < timeout_ms:
        current_freq = measure_led_frequency_robust(led_center, duration_ms=100)
        
        # Check if frequency changed
        if current_freq != initial_freq:
            print(f"Frequency changed to {current_freq:.1f} Hz - synchronized!")
            return True
    
    print("Timeout waiting for frequency change - proceeding anyway")
    return False

def validate_uart_frame(bits):
    """Validate UART frame with start, parity, and stop bits"""
    if len(bits) < 11:  # Need at least 11 bits for a complete frame
        return None
    
    # Check start bit (should be 0)
    if bits[0] != 0:
        print("Invalid start bit")
        return None
        
    # Check stop bit (should be 1)
    if bits[10] != 1:
        print("Invalid stop bit")
        return None
        
    # Check parity (even parity)
    data_bits = bits[1:9]
    parity_bit = bits[9]
    bit_count = sum(data_bits)
    expected_parity = 0 if (bit_count % 2 == 0) else 1
    
    if parity_bit != expected_parity:
        print("Parity check failed")
        return None
    
    # Convert data bits to byte
    byte = 0
    for i, bit in enumerate(data_bits):
        byte |= (bit << i)
    
    return byte

# --- Main frequency monitoring function ---
def monitor_led_frequencies(led_center, window_duration_s=1, sample_interval_ms=100):
    """
    Monitor LED frequencies for a fixed window (in seconds).
    Samples are separated by sample_interval_ms, and stops after window_duration_s.
    Prints the time (ms) at which each bit was sampled.
    Only records a new bit if the frequency changes from the previous sample.
    """
    print(f"Monitoring LED at {led_center} for {window_duration_s} second(s)...")
    wait_for_frequency_sync(led_center)
    frequency_list = []
    timestamps = []
    start = time.ticks_ms()
    sample_num = 0
    prev_freq = None
    while True:
        now = time.ticks_ms()
        elapsed = time.ticks_diff(now, start)
        if elapsed >= window_duration_s * 1000:
            break
        freq = measure_led_frequency_robust(led_center, duration_ms=100)
        if freq != prev_freq:
            frequency_list.append(freq)
            timestamps.append(elapsed)
            print(f"Bit {sample_num}: {freq:.2f} Hz at {elapsed} ms")
            sample_num += 1
            prev_freq = freq
        # Calculate next sample time based on start + N * interval
        next_sample_time = start + sample_num * sample_interval_ms
        while True:
            current = time.ticks_ms()
            if time.ticks_diff(current, next_sample_time) >= 0:
                break
            time.sleep_ms(1)
    binary_list = frequencies_to_binary(frequency_list)
    print("Raw frame bits:", binary_list)
    print("Timestamps (ms):", timestamps)
    return frequency_list, binary_list, timestamps

# --- Main execution ---
print("Detecting LED blob...")
led_center = detect_led_center_via_blobs(duration_s=2)
print("Detected LED center:", led_center)

if not led_center:
    print("WARNING: No LED blob detected! Using default center (160, 160).")
    led_center = (160, 160)

print("\nStarting frequency monitoring...")
frequencies, binary_data, timestamps = monitor_led_frequencies(
    led_center, 
    window_duration_s=1, 
    sample_interval_ms=100
)

# Send results via UART
if uart is not None:
    result_msg = f"FREQ:{frequencies}\nBIN:{binary_data}\nTIME:{timestamps}\n"
    uart.write(result_msg)
else:
    print("UART not available")
print("=== FINAL RESULTS ===")
print("Frequencies:", [f"{freq:.1f}" for freq in frequencies])
print("Binary:", binary_data)
print("Timestamps (ms):", timestamps)
if len(timestamps) > 1:
    diffs = [timestamps[i] - timestamps[i-1] for i in range(1, len(timestamps))]
    print("Time differences (ms):", diffs)
else:
    print("Time differences (ms): []")