import sensor
import time
try:
    import pyb
except ImportError:
    pyb = None  # or handle accordingly if running off hardware

import image
# from agent import Agent
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
class agent:
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
        return self.freq

    def measure_led_frequency(self,led_center, duration_ms=1000):
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
        # print(f"Transitions: {len(transition_times)} detected at {led_center}")
        # print(f"Frequency detected: {freq_hz:.2f}")
        return freq_hz




# Initialize agents
# agentA = Agent(1,4,12,0.2,0,[(194,139),(226,152)])
# agentB = Agent(2,10,12,0.2,8,[(226,152),(193,147)])
agentC =  agent(3,22,12,1,4,[(87,154),(88,164)]) # agent(3, frequency, timeperiod, stepsize, flag, neighbors)

agent_list = [agentC]

# Define agent locations (dummy coordinates for now)
# You should replace this with actual dynamic tracking
# agent_locations = {
#     0: [(20, 30), (40, 50)],  # Locations visible to agentA (e.g., B and C)
#     1: [(60, 70), (20, 30)],  # Locations visible to agentB (e.g., C and A)
#     2: [(40, 50), (60, 70)]   # Locations visible to agentC (e.g., A and B)
# }




# Time tracking
last_time = time.ticks_ms()

while True:
    now = time.ticks_ms()
    
    # Update every second (1000 ms)
    if time.ticks_diff(now, last_time) > 1000:
        last_time = now

        # Update each agent's flag
        for agent in agent_list:
            agent.flag += 1

        # Update agents whose flag == timeperiod
        for idx, agent in enumerate(agent_list):
            if agent.flag >= agent.timeperiod:
                agent.flag = 0
                freq=agent.update()
                # Send frequency via UART
                msg = f"{freq:.2f}\n"
                if uart is not None:
                    uart.write(msg)
                    # print("Sent frequency:", msg.strip())
                else:
                    pass
                # print("UART not available, frequency:", msg.strip())