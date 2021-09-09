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
from loguru import logger


import subprocess


from typing import Dict, List, Union, Any

import paho.mqtt.client as mqtt

# find the file path to this file
__location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))

warnings.simplefilter("ignore", np.RankWarning)


# [{
#     "id": 0,
#     "pos": {
#         "x": -0.08439404,
#         "y": 0.34455082,
#         "z": 1.1740385
#     },
#     "rotation": [
#         [
#             -0.71274376,
#             -0.47412094,
#             -0.5169194
#         ],
#         [
#             0.054221954,
#             -0.7719936,
#             0.6333134
#         ],
#         [
#             -0.6993256,
#             0.4233618,
#             0.5759414
#         ]
#     ]
# }]

class VRCAprilTag(object):
    def __init__(self):
        self.default_config: dict = {
            "cam": {
                "pos": [13, 0, 8.5],  # cm from FC
                "rpy": [0, 0, -pi / 2,],  # cam x = body -y; cam y = body x, cam z = body z
            },
            "tag_truth": {"0": {"rpy": [0, 0, 0], "xyz": [0, 0, 0]}},
            "AT_UPDATE_FREQ": 5,
            "AT_HEARTBEAT_THRESH": 0.25,
        }

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

        self.topic_prefix = "vrc/apriltags"
        self.topic_map = {
            f"{self.topic_prefix}/raw":self.on_apriltag_message
        }

    def on_message(
        self, client: mqtt.Client, userdata: Any, msg: mqtt.MQTTMessage
    ) -> None:
        try:
            #logger.debug(f"{msg.topic}: {str(msg.payload)}")
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
            name = "tag_" + tag
            tag_dat = self.default_config["tag_truth"][tag]  # type: ignore
            rmat = t3d.euler.euler2mat(tag_dat["rpy"][0], tag_dat["rpy"][1], tag_dat["rpy"][2], axes="rxyz")  # type: ignore
            tag_tf = t3d.affines.compose(tag_dat["xyz"], rmat, [1, 1, 1])  # type: ignore

            H_to_from = "H_" + name + "_aeroRef"
            self.tm[H_to_from] = tag_tf
            H_to_from = "H_" + name + "_cam"
            self.tm[H_to_from] = np.eye(4)

    def on_apriltag_message(self, payload):
        tags = payload

        tag_list = []

        for tag in tags:
            id, distance, pos, heading = self.handle_tag(tag)

            tag = {
                "id":id,
                "horizontal_dist": distance,
                "pos":{
                    "x":pos[0],
                    "y":pos[1],
                    "z":pos[2]
                },
                "heading":heading
            }

            tag_list.append(tag) 

        self.mqtt_client.publish(f"{self.topic_prefix}/visible_tags", json.dumps(tag_list))


    

    def horizontal_dist_to_tag(self, tag: dict)->float:
        """
        returns the scalar distance in the x-y plane to a tag in centimeters
        """
        return float(np.linalg.norm([tag["pos"]["x"] * 100, tag["pos"]["y"] * 100]))

    def handle_tag(self, tag):
        """
        Calculates the distance, position, and heading of the drone in NED frame
        based on the tag detections.
        """
        distance = self.horizontal_dist_to_tag(tag)
        tag_id = tag["id"]

        # if we have a location definition for the visible tag
        if str(tag["id"]) in self.default_config["tag_truth"].keys():
            tag_rot = np.asarray(tag["rotation"])
            rpy = t3d.euler.mat2euler(tag_rot)
            R = t3d.euler.euler2mat(0, 0, rpy[2], axes="rxyz")
            H_tag_cam = t3d.affines.compose(
                [
                    tag["pos"]["x"] * 100,
                    tag["pos"]["y"] * 100,
                    tag["pos"]["z"] * 100
                ],
                R,
                [1, 1, 1],
            )
            T, R, Z, S = t3d.affines.decompose44(H_tag_cam)

            name = "tag_" + str(tag_id)
            H_to_from = "H_" + name + "_cam"
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

            return tag_id, distance, pos, heading
        else:
            return tag_id, distance, None, None

    def main(self):
        # tells the os what to name this process, for debugging
        setproctitle("AprilTagVPS_main")

        subprocess.Popen("./vrcapriltags", cwd="./c/build",shell=True)
        threads = []
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