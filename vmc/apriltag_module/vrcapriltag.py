# python standard library
import time
import threading
import os
from math import pi, cos, sin
import json
import socket
import warnings

# pip installed packages
import numpy as np
import transforms3d as t3d
from setproctitle import setproctitle
from colored import fore, back, style

# custom libraries
from apriltag_library import AprilTagVPS

from loguru import logger
from typing import Dict, List, Union, Any

import paho.mqtt.client as mqtt

# find the file path to this file
__location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))

warnings.simplefilter("ignore", np.RankWarning)


class VRCAprilTag(object):
    def __init__(self):
        self.default_config: dict = {
            "camera_params": {
                "HD": [584.3866, 583.3444, 661.2944, 320.7182],
                "SD": [284.2, 284.5, 326.0, 173.5],
                "160CSI": [397.7698, 397.5626, 313.2360, 183.425],
            },
            "detector": {
                "protocol": "argus",
                "video_device": "/dev/video0",
                "res": [640, 360],
                "camera_params": "160CSI",
                "tag_size": 0.174,
                "framerate": 5,
            },
            "cam": {
                "pos": [13, 0, 8.5],  # cm from FC
                "rpy": [
                    0,
                    0,
                    -pi / 2,
                ],  # cam x = body -y; cam y = body x, cam z = body z
            },
            "tag_truth": {"0": {"rpy": [0, 0, 0], "xyz": [0, 0, 0]}},
            "AT_UPDATE_FREQ": 5,
            "AT_HEARTBEAT_THRESH": 0.25,
        }
        self.topics = None
        self.at: Union[AprilTagVPS, None] = None  # AprilTag object
        self.tm = dict()

        self.setup_transforms()

        self.pos_array = {"n": [], "e": [], "d": [], "heading": [], "time": []}

        self.mqtt_host = "mqtt"
        self.mqtt_port = 18830

        # self.mqtt_user = "user"
        # self.mqtt_pass = "password"

        self.mqtt_client = mqtt.Client()
        # self.mqtt_client.username_pw_set(
        #     username=self.mqtt_user, password=self.mqtt_pass
        # )

        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        self.topic_prefix = "vrc/apriltag"
        self.topic_map = {}

    def on_message(
        self, client: mqtt.Client, userdata: Any, msg: mqtt.MQTTMessage
    ) -> None:
        try:
            logger.debug(f"{msg.topic}: {str(msg.payload)}")

            if msg.topic in self.topic_map:
                payload = json.loads(msg.payload)
                self.topic_map[msg.topic](payload)
        except Exception as e:
            logger.exception(f"Error handling message on {msg.topic}")

    def on_connect(
        self,
        client: mqtt.Client,
        userdata: Any,
        rc: int,
        properties: mqtt.Properties = None,
    ) -> None:
        logger.debug(f"Connected with result code {str(rc)}")
        for topic in self.topic_map.keys():
            logger.debug(f"Apriltag Module: Subscribed to: {topic}")
            client.subscribe(topic)

    def run_mqtt(self):
        self.mqtt_client.connect(host=self.mqtt_host, port=self.mqtt_port, keepalive=60)
        self.mqtt_client.loop_forever()

    def setup_transforms(self):
        rmat = t3d.euler.euler2mat(
            self.default_config["cam"]["rpy"][0],
            self.default_config["cam"]["rpy"][1],
            self.default_config["cam"]["rpy"][2],
            axes="rxyz",
        )  # type: ignore
        H_cam_aeroBody = t3d.affines.compose(self.default_config["cam"]["pos"], rmat, [1, 1, 1])  # type: ignore

        H_aeroBody_cam = np.linalg.inv(H_cam_aeroBody)
        self.tm["H_aeroBody_cam"] = H_aeroBody_cam

        for tag in self.default_config["tag_truth"]:  # type: ignore
            name = "tag" + tag
            tag_dat = self.default_config["tag_truth"][tag]  # type: ignore
            rmat = t3d.euler.euler2mat(tag_dat["rpy"][0], tag_dat["rpy"][1], tag_dat["rpy"][2], axes="rxyz")  # type: ignore
            tag_tf = t3d.affines.compose(tag_dat["xyz"], rmat, [1, 1, 1])  # type: ignore

            H_to_from = "H_" + name + "_aeroRef"
            self.tm[H_to_from] = tag_tf
            H_to_from = "H_" + name + "_cam"
            self.tm[H_to_from] = np.eye(4)

    def dist_to_tag(self, tag):
        """
        returns the scalar distance in the x-y plane to a tag
        """
        return float(np.linalg.norm([tag.pose_t[0][0] * 100, tag.pose_t[1][0] * 100]))

    def handle_tag(self, tag):
        """
        Calculates the distance, position, and heading of the drone in NED frame
        based on the tag detections.
        """
        distance = self.dist_to_tag(tag)
        tag_id = tag.tag_id
        error = tag.pose_err

        # if we have a location definition for the visible tag
        if str(tag.tag_id) in self.default_config["tag_truth"].keys():
            rpy = t3d.euler.mat2euler(tag.pose_R)
            R = t3d.euler.euler2mat(0, 0, rpy[2], axes="rxyz")
            H_tag_cam = t3d.affines.compose(
                [
                    tag.pose_t[0][0] * 100,
                    tag.pose_t[1][0] * 100,
                    tag.pose_t[2][0] * 100 * 2.54, #TODO why is this magic value here
                ],
                R,
                [1, 1, 1],
            )
            T, R, Z, S = t3d.affines.decompose44(H_tag_cam)

            name = "tag" + str(tag.tag_id)
            H_to_from = "H_" + name + "cam"
            self.tm[H_to_from] = H_tag_cam

            H_cam_tag = np.linalg.inv(H_tag_cam)

            H_cam_aeroRef = self.tm["H_" + name + "_aeroRef"].dot(H_cam_tag)

            H_aeroBody_aeroRef = H_cam_aeroRef.dot(self.tm["H_aeroBody_cam"])

            pos, R, Z, S = t3d.affines.decompose44(H_aeroBody_aeroRef)
            rpy = t3d.euler.mat2euler(R)
            heading = rpy[2]
            if heading < 0:
                heading += 2 * pi

            heading = np.rad2deg(heading)

            return tag_id, error, distance, pos, heading
        else:
            return tag_id, error, distance, None, None

    def publish_dict(self, topic, data):
        self.mqtt_client.publish(
            topic,
            json.dumps(data),
            retain=False,
            qos=0,
        )

    def publish_heartbeat(self, last_detection):
        heartbeat = {
            "last_loop_timestamp":time.time(),
            "last_detection_timestamp":last_detection
        }
        self.publish_dict(f"{self.topic_prefix}/heartbeat", heartbeat)

    def publish_updates(
        self,
        visible_tag_ids,
        best_error,
        best_position,
        best_heading,
        best_tag_id,
    ):

        # visible_tags
        tags = {"ids": visible_tag_ids}
        self.publish_dict(f"{self.topic_prefix}/visible_tags", tags)

        # selected positon
        position = {"n": best_position[0], "e": best_position[1], "d": best_position[2]}
        self.publish_dict(f"{self.topic_prefix}/selected/position/ned", position)

        # selected heading
        heading = {"degrees": best_heading}
        self.publish_dict(f"{self.topic_prefix}/selected/heading", heading)

        # selected id & error
        update = {"id": best_tag_id, "error": best_error}
        self.publish_dict(f"{self.topic_prefix}/selected", update)

    def loop(self):
        """
        need to return/publish

        List(all visible tags)
        closest (x-y) tag
        (x-y) distance to closest tag

        error
        position of drone based on tag with least error
        heading

        """
        current_timestamp = time.time()
        prev_timestamp = current_timestamp
        assert self.at  # make sure at is not none

        last_hearbeat = time.time()

        while True:

            current_tags = self.at.tags
            current_timestamp = self.at.tags_timestamp

            if current_timestamp - prev_timestamp > (
                1 / self.default_config["AT_UPDATE_FREQ"]
            ):
                
                if current_tags and (current_timestamp != prev_timestamp):

                    # handle the data
                    visible_tag_ids = []
                    closest_tag_id = -1
                    distance_to_closest_tag = 10000000
                    best_error = 10000000
                    best_position = None
                    best_heading = None
                    best_tag_id = -1

                    for tag in current_tags:
                        # get the details about the tag as well as convert to NED
                        tag_id, error, distance, position, heading = self.handle_tag(
                            tag
                        )

                        # update the visible list
                        visible_tag_ids.append(tag_id)

                        # update the closest tag
                        if distance < distance_to_closest_tag:
                            closest_tag_id = tag_id
                            distance_to_closest_tag = distance

                        # update the best tag (has to be one we have a mapping for)
                        if position is not None:
                            if error < best_error:
                                best_tag_id = tag_id
                                best_position = position
                                best_heading = heading
                                best_error = error

                    self.publish_updates(
                        visible_tag_ids,
                        best_error,
                        best_position,
                        best_heading,
                        best_tag_id,
                    )

            prev_timestamp = current_timestamp
            
            now = time.time()
            if (now - last_hearbeat > ( 1 / self.default_config["AT_UPDATE_FREQ"]) ):
                self.publish_heartbeat(current_timestamp)
                last_hearbeat = now
            
            time.sleep(0.01)

    def main(self):
        # tells the os what to name this process, for debugging
        setproctitle("AprilTagVPS_main")

        self.at = AprilTagVPS(
            protocol=self.default_config["detector"]["protocol"],
            video_device=self.default_config["detector"]["video_device"],
            res=self.default_config["detector"]["res"],
            camera_params=self.default_config["camera_params"][
                self.default_config["detector"]["camera_params"]
            ],
            tag_size=self.default_config["detector"]["tag_size"],
            framerate=self.default_config["detector"]["framerate"],
        )

        threads = []

        at_thread = threading.Thread(
            target=self.at.start, args=(), daemon=True, name="apriltag_main_thread"
        )
        threads.append(at_thread)

        transform_thread = threading.Thread(
            target=self.loop,
            args=(),
            daemon=True,
            name="apriltag_transform_and_publish_thread",
        )
        threads.append(transform_thread)

        mqtt_thread = threading.Thread(
            target=self.run_mqtt, args=(), daemon=True, name="apriltag_mqtt_thread"
        )
        threads.append(mqtt_thread)

        for thread in threads:
            thread.start()
            logger.debug(f"{fore.GREEN}AT: starting thread: {thread.name}{style.RESET}")  # type: ignore

        while True:
            time.sleep(0.1)


if __name__ == "__main__":
    atag = VRCAprilTag()
    atag.main()
