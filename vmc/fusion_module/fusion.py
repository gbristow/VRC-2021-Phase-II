# python standard library
import os
import threading
import time
from math import atan2, pi
import json

# pip installed packages
import numpy as np
import pymap3d
from setproctitle import setproctitle
from colored import fore, back, style
from loguru import logger
import paho.mqtt.client as mqtt

from typing import Any

# find the file path to this file
__location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))

INTERRUPTED = False

class Fusion(object):
    def __init__(self, broker, config):
        # ========== VMC_CORE init ========================================================================== #
        self.config = {
            "package": "VMC_CORE",
            "threads":{
                "fuse_pos":{},
                "fuse_vel":{},
                "fuse_att":{},
                "status_prints": {
                    "init": "False",
                    "PRINT_FREQ": 2
                },
                "local_to_geo":{
                    "origin":{
                        "lat": 32.807650,
                        "lon": -97.157153,
                        "alt": 161.5
                    }
                },
                "assemble_hil_gps_message":{
                    "init": "True",
                    "hil_gps_constants":{
                        "fix_type": 3,
                        "eph": 20,
                        "epv": 5,
                        "satellites_visible": 13
                    }
                },
                "pos_sensor_check":{
                    "topics": {
                        "vmc.mocap": {
                            "timeout": 0.5,
                            "priority": 1
                        }
                    }
                },
                "aprilTag_resync_t265":{
                    "init": "True"
                }
            },
            "COURSE_THRESHOLD":10,
            "POS_DETLA_THRESHOLD": 10,
            "POS_D_THRESHOLD": 30,
            "HEADING_DELTA_THRESHOLD": 5,
            "AT_THRESH": 0.25,
            "T265_THRESH": 0.25,
            "AT_DERIV_THRESH": 10,
            "INIT_WAIT_TIME": 2
        }
        # ========== fusion init ========================================================================== #
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

        self.topic_prefix = "vrc/fusion"

        
        self.topic_map = {
            "vrc/vio/position/ned":self.fuse_pos,
            "vrc/vio/orientation/eul":self.fuse_att_euler,
            "vrc/vio/heading":self.fuse_att_heading,
            "vrc/vio/velocity/ned":self.fuse_vel,
            f"{self.topic_prefix}/pos":self.local_to_geo
        }

        self.primary_topic = None
        self.norm = None
        self.heading_delta = None
        self.deriv_norm = None

        logger.debug(f"{fore.LIGHT_CYAN_1} FUS: Object created! {style.RESET}") #type: ignore

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
        logger.debug(f" FUS: Connected with result code {str(rc)}")
        for topic in self.topic_map.keys():
            logger.debug(f"FUS: Subscribed to: {topic}")
            client.subscribe(topic)

    def local_to_geo(self, msg: dict) -> None:
        '''
        Callback for the fusion/pos topic. This method calculates the 
        geodetic location from an NED position and origin and publishes it.
        '''
        origin = [  self.config["origin"]["lat"],
                    self.config["origin"]["lon"],
                    self.config["origin"]["alt"]]

        try:
            ned = msg["ned"]
            lla = pymap3d.enu2geodetic( float(ned["e"])/100, # East   | Y
                                        float(ned["n"])/100, # North  | X
                                    -1 * float(ned["d"])/100, # Up     | Z
                                        origin[0], # Origin lat
                                        origin[1], # Origin lon
                                        origin[2], # Origin alt
                                        deg=True)

            geo_update = {
                "geodetic": {
                    "lat": lla[0],
                    "lon": lla[1],
                    "alt": lla[2]
                }
            }
            self.mqtt_client.publish(
                f"{self.topic_prefix}/geo",
                json.dumps(geo_update),
                retain=False,
                qos=0,
            )

        except Exception as e:
            logger.debug(f"{fore.RED} FUS: Error updating Geodetic location {style.RESET}") #type: ignore
            logger.debug(f"{fore.RED} FUS: {str(e)}") #type: ignore
            raise
                
    def fuse_pos(self, msg: dict) -> None:
        '''
        Callback for receiving pos data in NED reference frame from vio and publishes into a fusion/pos topic. 

        VRC doesnt have sophisticated fusion yet, so this just re-routes the message onto the fusion topic.

        '''
        try:
            #TODO - properly fill out this dict 
            pos_update = {
                    "n": msg["n"],
                    "e": msg["e"],
                    "d": msg["d"]
                }

            self.mqtt_client.publish(
                f"{self.topic_prefix}/pos/ned",
                json.dumps(pos_update),
                retain=False,
                qos=0,
            )

        except Exception as e:
            logger.debug(f"{fore.RED}FUS: Error fusing pos sources {str(e)}{style.RESET}") #type: ignore
            raise e

    def fuse_vel(self, msg: dict) -> None:
        '''
        Callback for receiving vel data in NED reference frame from vio and publishes into a fusion/vel topic. 

        VRC doesnt have sophisticated fusion yet, so this just re-routes the message onto the fusion topic.
        
        '''
        try:

            vmc_vel_update = {
                    "Vn": msg["n"],
                    "Ve": msg["e"],
                    "Vd": msg["d"]
                }

            self.mqtt_client.publish(
                f"{self.topic_prefix}/vel/ned",
                json.dumps(vmc_vel_update),
                retain=False,
                qos=0,
            )

            # compute groundspeed
            gs = np.linalg.norm([msg["Vn"], msg["Ve"]])
            groundspeed_update = { "groundspeed": gs }

            self.mqtt_client.publish(
                f"{self.topic_prefix}/vel/groundspeed",
                json.dumps(groundspeed_update),
                retain=False,
                qos=0,
            )

            # arctan gets real noisy when the values get small, so we just lock course
            # to heading when we aren't really moving
            if gs >= self.config["GROUNDSPEED_THRESHOLD"]:
                course = atan2(msg["Ve"], msg["Vn"])
                # wrap [-pi, pi] to [0, 360]
                if course < 0:
                    course += 2 * pi
                # rad to deg
                course = course * 180 / pi

                course_update = { "course": course }

                self.mqtt_client.publish(
                    f"{self.topic_prefix}/vel/course",
                    json.dumps(course_update),
                    retain=False,
                    qos=0,
                )

            m_per_s_2_ft_per_min = 196.85
            climb_rate_update = { "climb_rate": -1 * msg["Vd"] * m_per_s_2_ft_per_min}

            self.mqtt_client.publish(
                f"{self.topic_prefix}/vel/climbrate",
                json.dumps(climb_rate_update),
                retain=False,
                qos=0,
            )

        except Exception as e:
            logger.debug(f"{fore.RED}FUS: Error fusing vel sources {str(e)}{style.RESET}") #type: ignore
            raise e

    def fuse_att_quat(self, msg: dict) -> None:
        '''
        Callback for receiving quaternion att data in NED reference frame from vio and publishes into a fusion/att/quat topic. 

        VRC doesnt have sophisticated fusion yet, so this just re-routes the message onto the fusion topic.

        '''

        try:
            quat_update = {
                    "w": msg["w"],
                    "x": msg["x"],
                    "y": msg["y"],
                    "z": msg["z"]
                }

            self.mqtt_client.publish(
                f"{self.topic_prefix}/att/quat",
                json.dumps(quat_update),
                retain=False,
                qos=0,
            )
        except Exception as e:
            logger.debug(f"{fore.RED}FUS: Error fusing att/quat sources {str(e)}{style.RESET}") #type: ignore
            raise e
    
    def fuse_att_euler(self, msg:dict) -> None:
        '''
        Callback for receiving euler att data in NED reference frame from vio and publishes into a fusion/att/euler topic. 

        VRC doesnt have sophisticated fusion yet, so this just re-routes the message onto the fusion topic.

        '''

        try:
            euler_update = {
                "psi": msg["psi"],
                "theta": msg["theta"],
                "phi": msg["phi"]
            }

            self.mqtt_client.publish(
                f"{self.topic_prefix}/att/euler",
                json.dumps(euler_update),
                retain=False,
                qos=0,
            )
        except Exception as e:
            logger.debug(f"{fore.RED}FUS: Error fusing att/eul sources {str(e)}{style.RESET}") #type: ignore
            raise e

    def fuse_att_heading(self, msg: dict) -> None:
        '''
        Callback for receiving heading att data in NED reference frame from vio and publishes into a fusion/att/heading topic. 

        VRC doesnt have sophisticated fusion yet, so this just re-routes the message onto the fusion topic.

        '''

        try:
            heading_update = {
                "heading": msg["degrees"]
            }

            self.mqtt_client.publish(
                f"{self.topic_prefix}/att/heading",
                json.dumps(heading_update),
                retain=False,
                qos=0,
            )
        except Exception as e:
            logger.debug(f"{fore.RED}FUS: Error fusing att/heading sources {str(e)}{style.RESET}") #type: ignore
            raise e

    # def assemble_hil_gps_message(self, config=None):
    #     '''
    #     This code takes the pos data from the VMC and formats it into a special message that is exactly
    #     what the FCC needs to generate the hil_gps message (with heading)
    #     '''
    #     while not INTERRUPTED:
    #         time.sleep(0.01)
    #         try:
    #             if self.topics["vmc.pos"].message("geodetic").block_until_new:
    #                 lla = self.topics["vmc.pos"].message("geodetic").data

    #                 lat = int( lla["lat"] * 10000000) # convert to int32 format
    #                 lon = int( lla["lon"] * 10000000)  # convert to int32 format

    #                 if "hil_gps_constants" not in config.keys():
    #                     config["hil_gps_constants"] = {
    #                         "fix_type": 3,
    #                         "eph": 10,
    #                         "epv": 10,
    #                         "satellites_visible": 13
    #                     }
                    
    #                 # if lat / lon is 0, that means the ned -> lla conversion hasn't run yet, don't send that data to FCC
    #                 if lat != 0 and lon != 0:
    #                     hil_gps_update = {
    #                         "hil_gps":{
    #                             "time_usec": int(time.time() * 1000000),
    #                             "fix_type": int(config["hil_gps_constants"]["fix_type"]), # 3 - 3D fix
    #                             "lat": lat,
    #                             "lon": lon, 
    #                             "alt": int(self.topics["vmc.pos"].message("geodetic").data["alt"] * 1000), # convert m to mm
    #                             "eph": int(config["hil_gps_constants"]["eph"]), # cm
    #                             "epv": int(config["hil_gps_constants"]["epv"]), # cm
    #                             "vel": int(self.topics["vmc.vel"].message("groundspeed").data),
    #                             "vn": int(self.topics["vmc.vel"].message("speed").data["Vn"]),
    #                             "ve": int(self.topics["vmc.vel"].message("speed").data["Ve"]),
    #                             "vd": int(self.topics["vmc.vel"].message("speed").data["Vd"]),
    #                             "cog": int(self.topics["vmc.vel"].message("course").data * 100),
    #                             "satellites_visible": int(config["hil_gps_constants"]["satellites_visible"]),
    #                             "heading": int(self.topics["vmc.att"].message("heading").data * 100)
    #                         }
    #                     }
    #                     self.topics["vmc.gps"].pub([hil_gps_update])

    #         except Exception as e:
    #             print("FUS: Error assebling hil gps message!")
    #             print("FUS: ", e)
    #             continue

    # def status_prints(self, config=None):
        
        
    #     while self.primary_topic is None:
    #         time.sleep(.5)
            
    #     time.sleep(1)

    #     while not INTERRUPTED:

    #         t265_ned = self.topics["vmc.t265"].subtopic("pos").message("ned").data
    #         t265_heading = self.topics["vmc.t265"].subtopic("att").message("heading").data
    #         at_ned = self.topics["vrc.aprilTag"].message("ned").data
    #         at_heading = self.topics["vrc.aprilTag"].message("heading").data
    #         tags = self.topics["vrc.aprilTag"].message("tags").data
    #         AT_ts = self.topics["vrc.aprilTag"].message("last_loop").data
    #         norm = self.norm 
    #         heading_delta = self.heading_delta
    #         deriv_norm = self.deriv_norm


    #         # os.system('cls' if os.name=='nt' else 'clear -x')
    #         print(fore.GREY_0 + "========================================================================", style.RESET)

    #         try:
    #             now = time.time()

    #             if at_ned is not None and at_heading is not None:
    #                 if (now - self.topics["vrc.aprilTag"].message("ned").timestamp) > self.config["AT_THRESH"]:
    #                     color = fore.ORANGE_3
    #                 else:
    #                     color = fore.GREEN
    #             else:
    #                 at_ned["n"] = '-'
    #                 at_ned["e"] = '-'
    #                 at_ned["d"] = '-'
    #                 at_heading = '-'

    #             print(color + "AT:\tN: {:.3f}\tE: {:.3f}\tD: {:.3f}\tHeading: {:.3f}\t".format(
    #                     at_ned["n"], at_ned["e"], at_ned["d"], at_heading), style.RESET)

    #             if t265_ned is not None and t265_heading is not None:
    #                 if (now - self.topics["vmc.t265"].timestamp) > self.config["T265_THRESH"]:
    #                     color = fore.ORANGE_3
    #                 else:
    #                     color = fore.GREEN
                    
    #             print(color + "T265:\tN: {:.3f}\tE: {:.3f}\tD: {:.3f}\tHeading: {:.3f}\t".format(
    #                     t265_ned["n"], t265_ned["e"], t265_ned["d"], t265_heading), style.RESET)

    #             if norm is not None and heading_delta is not None and deriv_norm is not None:
    #                 if norm > self.config["POS_DETLA_THRESHOLD"]:
    #                     color1 = fore.ORANGE_3
    #                 else:
    #                     color1 = fore.GREEN
                    
    #                 if heading_delta > self.config["HEADING_DELTA_THRESHOLD"]:
    #                     color2 = fore.ORANGE_3
    #                 else:
    #                     color2 = fore.GREEN
                    
    #                 if deriv_norm > self.config["AT_DERIV_THRESH"]:
    #                     color3 = fore.ORANGE_3
    #                 else:
    #                     color3 = fore.GREEN

    #                 norm = "{:.3f}".format(norm)
    #                 heading_delta = "{:.3f}".format(heading_delta)
    #                 deriv_norm = "{:.3f}".format(deriv_norm)
                
    #             else:
    #                 color1 = fore.RED
    #                 color2 = fore.RED
    #                 color3 = fore.RED
    #                 norm = "-"
    #                 heading_delta = "-"
    #                 deriv_norm = "-"
                    
    #             print(color1 + "Norm: {},".format(norm) + color2 + "\tHeading Delta: {}".format(heading_delta) + color3 + "\tAT Deriv: {}".format(deriv_norm), style.RESET)

    #             if '-1' in tags:
    #                 color = fore.ORANGE_3
    #             else:
    #                 color = fore.GREEN
    #             print(color + "AT Tags: {}".format(tags), style.RESET)

    #             if now - AT_ts < self.config["AT_THRESH"]:
    #                 print(fore.GREEN + "AT Pipe Good!: {}".format(AT_ts), style.RESET)
    #             else:
    #                 print(fore.ORANGE_3 + "AT Pipe Bad!: {}".format(AT_ts), style.RESET)


    #             try:
    #                 time.sleep(1/self.config["PRINT_FREQ"])
    #             except:
    #                 time.sleep(1)
            
    #         except Exception as e:
    #             print(fore.RED + "FUS: Error printing: %s" % e, style.RESET)
    #             raise e

    # def aprilTag_resync_t265(self, config=None):
    #     '''
    #     Compares the pos data from april tags and t265, if both are valid and
    #     t265 drifts outside of a defined threshold, trigger a resync event 
    #     within t265.

    #     Subscribes
    #     -------------
    #     vmc.t265
    #     vrc.aprilTag

    #     Publishes
    #     -------------
    #     vmc.t265.resync
    #     '''
    #     try:
    #         time.sleep(1)

    #         init = "False"

    #         last_pos = [0,0,0]
    #         deriv = [0,0,0]
    #         then = time.time()

    #         while not INTERRUPTED:
    #             if self.topics["vrc.aprilTag"].message("ned").block_until_new:
    #                 now = time.time()

    #                 t265_ned = self.topics["vmc.t265"].subtopic("pos").message("ned").data
    #                 t265_heading = self.topics["vmc.t265"].subtopic("att").message("heading").data
    #                 at_ned = self.topics["vrc.aprilTag"].message("ned").data
    #                 at_heading = self.topics["vrc.aprilTag"].message("heading").data
                    
    #                 del_n = abs(at_ned["n"] - t265_ned["n"])
    #                 del_e = abs(at_ned["e"] - t265_ned["e"])
    #                 del_d = abs(at_ned["d"] - t265_ned["d"])

    #                 self.norm = np.linalg.norm([del_n, del_e, del_d])
    #                 self.heading_delta = abs(self.topics["vrc.aprilTag"].message("heading").data - self.topics["vmc.t265"].subtopic("att").message("heading").data)
    #                 if self.heading_delta > 180:
    #                     self.heading_delta = 360 - self.heading_delta
    #                 # print(fore.CYAN_2 + "FUS: norm: {}".format(self.norm), style.RESET)

    #                 for idx, val in enumerate(at_ned.keys()):
    #                     deriv[idx] = (at_ned[val] - last_pos[idx]) / (now - then)
    #                     last_pos[idx] = at_ned[val]

    #                 self.deriv_norm = np.linalg.norm(deriv)

                    
    #                 if (self.norm > self.config["POS_DETLA_THRESHOLD"] or abs(self.heading_delta) > self.config["HEADING_DELTA_THRESHOLD"]) and self.deriv_norm < self.config["AT_DERIV_THRESH"]:
    #                     # print(fore.YELLOW + "FUS: Resync Triggered! Delta= {}".format(norm), style.RESET)
    #                     updates = []



    #                     if del_d > self.config["POS_D_THRESHOLD"]:
    #                         # don't resync Z if del_d is too great, reject AT readings that are extraineous
    #                         at_ned["d"] = t265_ned["d"]
                        
    #                     resync_ned_update = {
    #                         "ned":{
    #                             "n": at_ned["n"],
    #                             "e": at_ned["e"],
    #                             "d": at_ned["d"]
    #                         }
    #                     }
    #                     updates.append(resync_ned_update)

    #                     resync_heading_update = {
    #                         "heading": self.topics["vrc.aprilTag"].message("heading").data
    #                     }
    #                     updates.append(resync_heading_update)

    #                     if init == "False":
    #                         init_update = {
    #                             "init": "True"
    #                         }
    #                         updates.append(init_update)

    #                     self.topics["vmc.t265.resync"].pub(updates)

    #                 then = now

    #     except Exception as e:
    #         print(fore.RED + "FUS: Error in t265 resync: %s" % e, style.RESET)
    #         raise e

    def main(self):
        # tells the os what to name this process, for debugging
        setproctitle("fusion_process")
        # allows for graceful shutdown of any child threads
        self.mqtt_client.connect(host=self.mqtt_host, port=self.mqtt_port, keepalive=60)
        self.mqtt_client.loop_forever()
  