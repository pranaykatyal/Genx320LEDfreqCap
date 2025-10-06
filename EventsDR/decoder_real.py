import pickle
import threading
import time

import cv2
import numpy as np
import rclpy
import rclpy.logging
from custom_messages.msg import Blob
from cv_bridge import CvBridge
from decoder.utils import decode_bits
from event_camera_msgs.msg import EventPacket
from event_camera_py import Decoder as EventPacketDecoder
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from sensor_msgs.msg import Image


class Decoder(Node):

    def __init__(self):
        super().__init__("decoder")

        # subscribe to messages from the blob tracker
        self.blob_sub = self.create_subscription(
            Blob, "/blobs", self.blob_callback, qos_profile_system_default
        )

        self.img_pub = self.create_publisher(
            Image, "/tracked_image", qos_profile_system_default
        )

        self.subscription = self.create_subscription(
            EventPacket,
            "/event_camera/events",
            self.event_callback,
            qos_profile_sensor_data,
        )


        ## Added by MC for tracker testing and debugging
        self.img_sub = self.create_subscription(
            Image, "/blob_image", self.blob_img_callback, qos_profile_system_default
        )
        self.blob_img = None

        ## Added by MC for tracker testing and debugging
        self.contour_img_sub = self.create_subscription(
            Image, "/contour_image", self.contour_img_callback, qos_profile_system_default
        )
        self.contour_img = None

        self.decoder = EventPacketDecoder()

        self._timers.append(self.create_timer(0.25, self.cleanup_callback))

        self.bridge = CvBridge()

        ## Parameters for tracking LEDs
        self.beta = 0.08
        self.N = 10
        self.staleness_threshold = 0.5 #20  # 0.05 # ignore this for now
        self.bin_size = 400

        ## Parameters for decoding messages
        self.base_frequency = 240

        # 1 start bit + 7-bit ASCII code + 1 parity bit + 2 end bits
        self.bit_per_letter = 11

        # Event camera contrast threshold ct
        self.ct = 0.1
        self.ct_tresh_on = 0.038
        self.ct_tresh_off = -0.038

        # Cut-off frequency for high-pass filter
        self.cut_freq = self.base_frequency / 3 * 2 * np.pi

        # Unit t for one bit
        self.unit_t = 1 / self.base_frequency

        self.centers: dict[int, np.ndarray] = {}
        self.distances: dict[int, list[float]] = {}
        self.radii: dict[int, float] = {}

        self.track_event_polarities: dict[int, list[float]] = {}
        self.track_event_times: dict[int, list[float]] = {}
        self.track_filtered_signals: dict[int, list[float]] = {}
        self.demod: dict[int, list[int]] = {}
        self.demod_t: dict[int, list[float]] = {}
        self.demod_prev: dict[int, int] = {}
        self.demod_t_prev: dict[int, float] = {}
        self.start_id: dict[int, int] = {}

        self.decoded_messages: dict[int, str] = {}

        self.track_ids: list[int] = []
        self.last_update_time: dict[int, float] = {}
        self.lock = threading.Lock()

        self.create_timer(1, self.decode_bits)

    def decode_bits(self):

        with self.lock:
            #self.get_logger().info(f"self tracks length{len(self.track_ids)}")
            for track_id in self.track_ids:

                if self.start_id[track_id] is None:
                    continue

                st = time.time()

                t_start = self.demod_t[track_id][self.start_id[track_id]]
                t2 = t_start + self.unit_t * self.bit_per_letter

                end_msg = self.demod_t[track_id][-1]

                output_bin = []
                output_t = []
                full_msg = ""
                while t2 < end_msg:

                    # search from right to left for falling edge
                    t2_right_lim = t2 + self.unit_t / 2
                    t2_left_lim = t2 - self.unit_t * 2
                    t2_right_lim_i = [
                        idx
                        for idx, t in enumerate(self.demod_t[track_id])
                        if t <= t2_right_lim
                    ][-1]
                    t2_left_lim_i = [
                        idx
                        for idx, t in enumerate(self.demod_t[track_id])
                        if t <= t2_left_lim
                    ][-1]
                    for i in range(t2_right_lim_i, t2_left_lim_i - 1, -1):
                        if (
                            i + 1 < len(self.demod[track_id])
                            and self.demod[track_id][i] == 1
                            and self.demod[track_id][i + 1] == -1
                        ):
                            t_end = self.demod_t[track_id][i + 1]
                            break
                        if i == t2_left_lim_i:
                            t_end = t2

                    # sample signal to get bits
                    bits = []
                    interval = (t_end - t_start) / self.bit_per_letter
                    self.get_logger().debug(
                        f"Decoding message from {t_start} to {t_end}"
                    )
                    for t_sample in np.linspace(
                        t_start + interval / 2,
                        t_end - interval / 2,
                        self.bit_per_letter,
                    ):
                        t_sample_i = [
                            idx
                            for idx, t in enumerate(self.demod_t[track_id])
                            if t <= t_sample
                        ][-1]
                        output_bin.append((1 + self.demod[track_id][t_sample_i]) / 2)
                        bits.append(
                            str(int((1 + self.demod[track_id][t_sample_i]) / 2))
                        )
                        output_t.append(t_sample)
                    #self.get_logger().info(f"Bits for this letter: {''.join(bits)}")
                    # decode current message
                    msg = decode_bits("".join(bits))
                    self.get_logger().info(f"{msg} this is the msg")
                    if msg is not None:
                        full_msg = full_msg + msg

                    # move on to next message
                    t_start = t_end
                    t2 = t_start + self.unit_t * self.bit_per_letter

                # now t_start is the start of the next message that we
                # weren't able to decode this time, so keep it and
                # everything after it
                # new start_id should be 0
                self.get_logger().debug(f"{self.demod_t[track_id]}")
                try:
                    idx = self.demod_t[track_id].index(t_start)
                except ValueError:
                    idx = -1
                self.demod_t[track_id] = self.demod_t[track_id][idx:]
                self.demod[track_id] = self.demod[track_id][idx:]
                self.start_id[track_id] = 0

                if len(full_msg) > 0:
                    self.decoded_messages[track_id] += full_msg
                    self.get_logger().info(
                        f"Decoded message from track {track_id}: {self.decoded_messages[track_id]}"
                    )

                self.get_logger().debug(
                    f"Time to decode message: {time.time() - st:.6f}"
                )
                self.get_logger().debug(
                    f"Num events per packet: {self.n_event}, used: {self.n_event_used}"
                )
                self.get_logger().debug(
                    f"Time to update events: {self.event_time:.6f}, useful: {self.useful_time:.6f}"
                )
                self.get_logger().debug(f"dist time: {self.dist_time:.6f}")
                self.get_logger().debug(f"decode time: {self.decode_time:.6f}")
                self.get_logger().debug(f"p: {self.p_time:.6f}, t: {self.t_time:.6f}")

                with open(
                    f"/home/harmeet/nonrf_ws/msg.pkl",
                    "wb",
                ) as f:
                    pickle.dump(self.decoded_messages[track_id], f)




    def cleanup_callback(self):
        cur_time = time.time()
        for track_id in self.track_ids:
            if cur_time - self.last_update_time[track_id] > self.staleness_threshold:
                self.delete_track(track_id)

    def add_track(self, center, radius):

        if len(self.track_ids) == 0:
            new_id = 0
        else:
            new_id = max(self.track_ids) + 1
        # self.get_logger().info(f"centre  raddii  . {center , radius,new_id}")
        #self.get_logger().info(f"the self ids{self.track_ids}")
        self.track_ids.append(new_id)
        self.centers[new_id] = np.array(center)
        self.radii[new_id] = 8 * radius  ## SEEME
        self.distances[new_id] = []
        self.last_update_time[new_id] = time.time()

        self.track_event_polarities[new_id] = []
        self.track_event_times[new_id] = []
        self.track_filtered_signals[new_id] = []
        self.demod[new_id] = []
        self.demod_t[new_id] = []
        self.start_id[new_id] = None
        self.demod_prev[new_id] = -1
        self.demod_t_prev[new_id] = 0
        self.decoded_messages[new_id] = ""

    def delete_track(self, track_id):

        self.centers.pop(track_id)
        self.distances.pop(track_id)
        self.radii.pop(track_id)
        self.last_update_time.pop(track_id)
        self.track_event_polarities.pop(track_id)
        self.track_event_times.pop(track_id)
        self.track_ids.remove(track_id)

    def blob_callback(self, msg: Blob):

        # self.get_logger().info("")

        cx = msg.x
        cy = msg.y
        radius = msg.r

        point = np.array([cx, cy])

        self.update_tracks_from_contour(point, radius)

    def event_callback(self, msg: EventPacket):
        with self.lock:
            st = time.time()
            n_event = 0
            n_event_used = 0
            self.p_time = 0
            self.t_time = 0
            decst = time.time()
            self.decoder.decode(msg)
            cd_events = self.decoder.get_cd_events()
            #self.get_logger().info(f"herr{cd_events}")
            ts = cd_events["t"] * 1e-6
            self.decode_time = time.time() - decst
            useful_time = 0
            dst = time.time()
            pos = np.array((cd_events["x"], cd_events["y"]))
            distances = {
                track_id: np.linalg.norm(
                    pos - self.centers[track_id].reshape((2, 1)), axis=0
                ).astype(np.float64)
                < self.radii[track_id]
                for track_id in self.track_ids
            }
            self.dist_time = time.time() - dst
            # self.get_logger().info(f"{distances}")

            for idx, event in enumerate(cd_events):
                n_event += 1
                for track_id in self.track_ids:
                    if distances[track_id][idx]:

                        # Convert time from microseconds to seconds
                        tst = time.time()
                        t = ts[idx]
                        # self.get_logger().info(f"{t}")
                        self.t_time += time.time() - tst

                        # Make sure this is either the first event for a track or it occurs after the latest event
                        if (
                            not self.track_event_times[track_id]
                            or t
                            > self.track_event_times[track_id][-1]  # + self.unit_t / 16
                        ):
                            n_event_used += 1
                            ust = time.time()

                            # We want polarity to be in {-1, 1} not {0, 1}
                            pst = time.time()
                            p = event["p"]
                            if p == 0:
                                p = -1
                            self.p_time += time.time() - pst
                            # Update the raw event stream for this track
                            self.track_event_polarities[track_id].append(p)
                            self.track_event_times[track_id].append(t)

                            # Update the filtered signal for this track
                            if len(self.track_filtered_signals[track_id]) == 0:
                                self.track_filtered_signals[track_id].append(
                                    p * self.ct
                                )
                            else:
                                dt = t - self.track_event_times[track_id][-2]
                                self.track_filtered_signals[track_id].append(
                                    np.exp(-self.cut_freq * dt)
                                    * self.track_filtered_signals[track_id][-1]
                                    + p * self.ct
                                )

                            # Check if we have crossed the threshold for switching the square wave
                            if (
                                self.track_filtered_signals[track_id][-1]
                                > self.ct_tresh_on
                                and self.demod_prev[track_id] == -1
                            ):
                                self.demod[track_id].append(1)
                                self.demod_t[track_id].append(t)
                                self.demod_prev[track_id] = 1
                                self.demod_t_prev[track_id] = t
                            elif (
                                self.track_filtered_signals[track_id][-1]
                                < self.ct_tresh_off
                                and self.demod_prev[track_id] == 1
                            ):
                                self.demod[track_id].append(-1)
                                self.demod_t[track_id].append(t)

                                if self.start_id[track_id] is None:
                                    self.start_id[track_id] = (
                                        len(self.demod[track_id]) - 1
                                    )

                                self.demod_prev[track_id] = -1
                                self.demod_t_prev[track_id] = t
                            useful_time += time.time() - ust
            # self.get_logger().info(f"Time to add events: {time.time() - st:.4f}")
            self.n_event = n_event
            self.n_event_used = n_event_used
            self.useful_time = useful_time
            self.event_time = time.time() - st

    def update_tracks_from_contour(self, blob_center, blob_radius):

        updated = False

        for track_id in self.track_ids:

            distance = np.linalg.norm(
                np.array(blob_center) - self.centers[track_id]
            ).astype(np.float64)

            if distance <= self.radii[track_id] + blob_radius:

                self.last_update_time[track_id] = time.time()

                # update center immediately
                self.centers[track_id] = (
                    blob_center  ##SEEME # self.beta * np.array(blob_center) + (1 - self.beta) * self.centers[track_id]
                )

                # increment distances and if we've reached N, update radii
                # self.distances[track_id].append(distance)
                # self.get_logger().info(f"things outside{self.radii[track_id]}")
                # self.get_logger().info(f"things outside  centreee{self.centers[track_id]}")
                # self.get_logger().info(f"Iam trackID{track_id}")

                if len(self.distances[track_id]) == self.N:
                    # self.radii[track_id] = 8 * np.mean(self.distances[track_id]).astype(np.float64)
                    self.radii[track_id] = (
                        0.5 * self.radii[track_id] + 0.5 * blob_radius
                    ).astype(
                        np.float64
                    )  # new track radius is mean of old track radius and new blob radius  ## SEEME
                    self.distances[track_id].clear()
                    self.get_logger().info(f"i  am here{self.radii[track_id]}")

                updated = True
                break

        if not updated:
            # self.get_logger().info("Adding track")
            self.add_track(blob_center, blob_radius)
        
        ## Added by MC
        if (self.blob_img is not None)  and (self.contour_img is not None):
            # self.write_tracks_frame(self.blob_img)
            self.write_tracks_frame(self.blob_img)

    def write_tracks_frame(self, cv_img):

        for track_id in self.track_ids:
            center = (int(self.centers[track_id][0]), int(self.centers[track_id][1]))
            cv2.circle(cv_img, center, int(self.radii[track_id]), (255, 255, 255))

        new_msg = self.bridge.cv2_to_imgmsg(cv_img)
        self.img_pub.publish(new_msg)

    ## Added by MC
    def blob_img_callback(self, blob_msg):

        # self.get_logger().info("Got blob")

        self.blob_img = self.bridge.imgmsg_to_cv2(blob_msg)

    ## Added by MC
    def contour_img_callback(self, contour_msg):

        # self.get_logger().info("Got contour")
        self.contour_img = self.bridge.imgmsg_to_cv2(contour_msg)


def main(args=None):
    rclpy.init(args=args)

    decoder = Decoder()

    rclpy.spin(decoder)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    decoder.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


#011100001110111001001101100001111011011101110110000111101111001111
###tracker -- coming and going 
###none msg
