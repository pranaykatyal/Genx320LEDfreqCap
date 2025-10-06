import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default


import numpy as np

from event_camera_msgs.msg import EventPacket
from event_camera_py import Decoder as EventPacketDecoder

from custom_messages.msg import Blob

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import cv2 as cv
import time
class Tracker(Node):

    def __init__(self):

        super().__init__('tracker')

        self.subscription = self.create_subscription(
            EventPacket,
            '/event_camera/events',
            self.listener_callback,
            qos_profile_sensor_data)
        
        self.decoder = EventPacketDecoder()

        self.indices = None

        self.window_size = 1.0 # 0.1 # ignore all data more than 0.1 seconds old
        self.B = 1
        self.threshold = 0.45 # 0.6 # can't ge at or lower than 0.5. Otherwise a single event will cause a blob to form

        self.t = 0

        self.blobs = {}

        self.create_timer(0.001, self.calculate_indices)

        self.bridge = CvBridge()
        self.img_pub = self.create_publisher(Image, "/blob_image", qos_profile_system_default)
        self.contour_pub = self.create_publisher(Image, "/contour_image", qos_profile_system_default)

        self.blob_pub = self.create_publisher(Blob, "/blobs", qos_profile_system_default)

    def listener_callback(self, msg: EventPacket):
        # self.get_logger().info(f"Got EventPacket with {len(msg.events)} events")
        if self.indices is None:
            self.indices = np.zeros((msg.height, msg.width))
            self.blobs = np.zeros((msg.height, msg.width), dtype=np.uint8)
            self.data = {}

        self.decoder.decode(msg)
        cd_events = self.decoder.get_cd_events()
        last=time.time()
        for event in cd_events:
            x = event["x"]
            y = event["y"]
            p = event["p"]
            if p == 0:
                p = -1
            t = event["t"] / 1e6
            self.t = max(self.t, t)
            if (y, x) not in self.data.keys():
                #self.get_logger().info(f"  the pixel coords are {x,y}")
                self.data[(y, x)] = [[], []]
            self.data[(y, x)][0].append(p)
            self.data[(y, x)][1].append(t)
            # self.get_logger().info(f"the pixel is xy {x},{y},{t},{p}")
        now=time.time()
          
        
    def calculate_indices(self):
        last=time.time()
        if self.indices is not None:
            for (y, x) in self.data.keys():
                i = 0
                for i in range(len(self.data[(y, x)][1])):
                    if self.t - self.data[(y, x)][1][i] <= self.window_size:
                        break
                self.data[(y, x)][0] = self.data[(y, x)][0][i:]
                self.data[(y, x)][1] = self.data[(y, x)][1][i:]

                self.indices[y, x] = len(self.data[(y, x)][0]) / (self.B + np.abs(sum(self.data[(y, x)][0])))

            self.blobs = np.where(self.indices < self.threshold, 0, 255).astype(np.uint8)



            morph = self.blobs
            # morph = cv.morphologyEx(morph, cv.MORPH_OPEN, cv.getStructuringElement(cv.MORPH_RECT, (2,2)))
            # morph = cv.morphologyEx(morph, cv.MORPH_CLOSE, cv.getStructuringElement(cv.MORPH_RECT, (7,7)))



            ## try using erions and dilation with different sized kernels

            erode_kernel = np.ones((2,2),np.uint8)
            dilate_kernel = np.ones((7,7),np.uint8)

            morph = cv.erode(morph, erode_kernel, iterations=1)
            morph = cv.dilate(morph, dilate_kernel, iterations=1)

            new_msg = self.bridge.cv2_to_imgmsg(morph) # morph instead of self.blobs
            self.img_pub.publish(new_msg)
            contours, heierarchy = cv.findContours(morph, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

            ## Added by MC to plot contours and tracker
            contour_img = np.zeros((self.blobs.shape[0], self.blobs.shape[1]), dtype=np.uint8)

            for c in contours:
                M = cv.moments(c)
                if M["m00"] == 0:
                    continue
                x = int(M["m10"]/M["m00"])
                y = int(M["m01"]/M["m00"])
                area = cv.contourArea(c)
                radius = np.ceil(np.sqrt(area/np.pi))
                led_msg = Blob()
                led_msg.x = int(x)
                led_msg.y = int(y)
                led_msg.r = int(radius)
                #
                self.blob_pub.publish(led_msg)
                #self.get_logger().info(f"{x}{y} xy blob center")
                #self.get_logger().info(f"{self.data[(x,y)][0]}")
                ## Added by MC to plot contours and tracker
                cv.circle(contour_img, (x,y), int(radius), (255, 255, 255))
            now=time.time()
            #self.get_logger().info(f"the time to process this data is {(now-last)*1e3}") 
            ## Added by MC to plot contours and tracker
            new_msg = self.bridge.cv2_to_imgmsg(contour_img)
            self.contour_pub.publish(new_msg)



def main(args=None):
    rclpy.init(args=args)

    tracker = Tracker()

    rclpy.spin(tracker)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# def freq_calc(self):
#         if not self.led_centers or not self.data:
            
#             return
#         self.get_logger().info("atleast I came here")
        
        
#         for (y, x) in self.led_centers:
#             if (y, x) in self.data:
#                 polarities = self.data[(y, x)][0]
#                 times = self.data[(y, x)][1]

#                 if len(polarities) < 2:
#                     continue

#                 last_pole = polarities[-1]
#                 last_time = times[-1]

#                 # Find previous event with the same polarity
#                 for i in range(len(polarities) - 2, -1, -1):
#                     if polarities[i] == last_pole:
#                         delta = last_time - times[i]
#                         freq = 1000.0 / delta if delta > 0 else 0
#                         self.led_freqs[(y, x)] = freq
#                         self.get_logger().info(f"Frequency at LED center ({y},{x}): {freq:.2f} Hz")
#                         break
