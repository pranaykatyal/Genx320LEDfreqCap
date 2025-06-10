import sensor
import time
try:
    import pyb
except ImportError:
    pyb = None  # or handle accordingly if running off hardware

import image

class Agent:
    def __init__(self,id,freq,timeperiod,stepsize,flag,neighbors):
        self.freq=freq
        self.timeperiod=timeperiod
        self.stepsize=stepsize
        self.flag=flag
        self.neighbors=neighbors
        self.id=id

    def update(self):
        frequencies=[]
        for a in self.neighbors:
            frequencies.append(self.measure_led_frequency(a))
        rate=sum(frequencies)-(len(frequencies)*self.freq)
        self.freq+=self.stepsize*rate

    def measure_led_frequency(self,led_center, duration_ms=2000):
        px, py = led_center
        transition_times = []
        prev_val = 0
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
            if polarity != prev_polarity:
                if last_transition_time == 0 or (now - last_transition_time) > 5:
                    transition_times.append(now)
                    last_transition_time = now
            prev_polarity = polarity

        cycles = len(transition_times) // 2
        freq_hz = cycles / (duration_ms / 1000.0)
        print(f"Transitions: {len(transition_times)} detected at {led_center}")
        print(f"Frequency detected: {freq_hz:.2f}")
        return freq_hz
