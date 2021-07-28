# python standard library
import os
import math
import time
import threading
import typing
from math import pi, cos, sin

# pip installed packages
import pyrealsense2 as rs 
import numpy as np
import transforms3d as t3d
from setproctitle import setproctitle
from colored import fore, back, style

INTERRUPTED = False

class RS_CoordinateTransformation(object):
    '''
    This class handles all the coordinate transformations we need to use to get relevant data from the Intel Realsense T265 camera
    '''
    def __init__(self):
        self.tm = dict()

        sensor_pos_in_aeroBody = [17, 0, 8.5] #cm - ned
        sensor_att_in_aeroBody = [0, -pi/2, pi/2] #rad [-pi, pi]
        sensor_height_off_ground = 10 #cm

        H_aeroBody_T265Body = t3d.affines.compose(sensor_pos_in_aeroBody, t3d.euler.euler2mat(sensor_att_in_aeroBody[0], sensor_att_in_aeroBody[1], sensor_att_in_aeroBody[2], axes="rxyz"),[1,1,1])
        self.tm["H_aeroBody_T265Body"] = H_aeroBody_T265Body
        self.tm["H_T265Body_aeroBody"] = np.linalg.inv(H_aeroBody_T265Body)

        pos = sensor_pos_in_aeroBody
        pos[2] = -1 * sensor_height_off_ground
        H_aeroRef_T265Ref = t3d.affines.compose(pos, t3d.euler.euler2mat(sensor_att_in_aeroBody[0], sensor_att_in_aeroBody[1], sensor_att_in_aeroBody[2], axes="rxyz"),[1,1,1])
        self.tm["H_aeroRef_T265Ref"] = H_aeroRef_T265Ref

        H_aeroRefSync_aeroRef = np.eye(4)
        self.tm["H_aeroRefSync_aeroRef"] = H_aeroRefSync_aeroRef

        H_nwu_aeroRef = t3d.affines.compose([0, 0, 0], t3d.euler.euler2mat(pi, 0, 0), [1,1,1])
        self.tm["H_nwu_aeroRef"] = H_nwu_aeroRef

    def sync(self, heading_ref, pos_ref): 
        ''' 
        Computes offsets between t265 ref and "global" frames, to align coord. systems
        '''
        rad2deg = 180 / pi
        deg2rad = pi / 180

        # get current readings on where the aeroBody is, according to the sensor
        H = self.tm["H_aeroRef_aeroBody"]
        T, R, Z, S = t3d.affines.decompose44(H)
        eul = t3d.euler.mat2euler(R, axes="rxyz")
        
        ## Find the heading offset...
        heading = eul[2]

        # wrap heading in (0, 2*pi)
        if heading < 0:
            heading += 2 * pi
        
        # compute the difference between our global reference, and what our sensor is reading for heading
        heading_offset =  heading_ref - (heading * rad2deg) 
        print(fore.CYAN_2 + "T265: Resync: Heading Offset: ", heading_offset, style.RESET)

        # build a rotation matrix about the global Z axis to apply the heading offset we computed
        H_rot_correction = t3d.affines.compose([0,0,0], t3d.axangles.axangle2mat([0,0,1], heading_offset * deg2rad),[1,1,1])

        # apply the heading correction to the position data the T265 is providing
        H = H_rot_correction.dot(H)
        T, R, Z, S = t3d.affines.decompose44(H)
        eul = t3d.euler.mat2euler(R, axes="rxyz")

        ## Find the position offset
        pos_offset = [  pos_ref["n"]-T[0], 
                        pos_ref["e"]-T[1], 
                        pos_ref["d"]-T[2]]
        print(fore.CYAN_2 + "T265: Resync: Pos offset: ", pos_offset, style.RESET)

        # build a translation matrix that corrects the difference between where the sensor thinks we are and were our reference thinks we are
        H_aeroRefSync_aeroRef = t3d.affines.compose(pos_offset, H_rot_correction[:3, :3], [1,1,1])
        self.tm["H_aeroRefSync_aeroRef"] = H_aeroRefSync_aeroRef

    def transform_t265_to_global_ned(self, data):
        '''
        Takes in raw sensor data from the t265 frame, does the necessary transformations between the sensor, vehicle, and reference frames to 
        present the sensor data in the "global" NED reference frame. 

        Arguements:
        --------------------------
        data : T265 Frame data

        Returns:
        --------------------------
        pos: list 
            The NED position of the vehice. A 3 unit list [north, east, down]
        vel: list
            The NED velocities of the vehicle. A 3 unit list [Vn, Ve, Vd]
        rpy: list
            The euler representation of the vehicle attitude. A 3 unit list [roll, pitch, yaw]

        '''
        quaternion = [data.rotation.w, data.rotation.x, data.rotation.y, data.rotation.z]
        position = [data.translation.x * 100, data.translation.y * 100, data.translation.z * 100] # cm 
        velocity = np.transpose([data.velocity.x * 100, data.velocity.y * 100, data.velocity.z * 100, 0]) # cm/s

        H_T265Ref_T265Body = t3d.affines.compose(position, t3d.quaternions.quat2mat(quaternion), [1, 1, 1])
        self.tm["H_T265Ref_T265Body"] = H_T265Ref_T265Body

        H_aeroRef_aeroBody = self.tm["H_aeroRef_T265Ref"].dot(self.tm["H_T265Ref_T265Body"].dot(self.tm["H_T265Body_aeroBody"]))
        self.tm["H_aeroRef_aeroBody"] = H_aeroRef_aeroBody

        H_aeroRefSync_aeroBody = self.tm["H_aeroRefSync_aeroRef"].dot(H_aeroRef_aeroBody)
        self.tm["H_aeroRefSync_aeroBody"] = H_aeroRefSync_aeroBody

        T, R, Z, S = t3d.affines.decompose44(H_aeroRefSync_aeroBody)
        eul = t3d.euler.mat2euler(R, axes="rxyz")

        H_vel = self.tm["H_aeroRefSync_aeroRef"].dot(self.tm["H_aeroRef_T265Ref"])
        vel = np.transpose(H_vel.dot(velocity))

        # print("T265: N: {:.3f}\tE: {:.3f}\tD: {:.3f}\tR: {:.3f}\tP: {:.3f}\tY: {:.3f}\tVn: {:.3f}\tVe: {:.3f}\tVd: {:.3f}".format(
        #     translate[0], translate[1], translate[2], angles[0], angles[1], angles[2], vel[0], vel[1], vel[2]))

        return T, vel, eul

class RS_T265(object):
    '''
    Realsense T265 Tracking Camera interface. Manages pulling data off of the camera, tranforming the data into the coordinate system of the vehicle so it can be fused,
    and publishing that data to the vmc.t265 topic in the broker. The T265 sensor is synchronized with the global coordinate system whenever some absolute position
    is published to the vmc.t265.resync topic.
    '''
    def __init__(self):
        self.t265_pipe = None

    def get_rs_devices(self) -> typing.Dict:
        """ Get Serial numbers of connected RealSense devices"""
        rs_devices = {}
        rs_context = rs.context()
        for i in range(len(rs_context.devices)):
            rs_device = rs_context.devices[i]
            rs_device_name = rs_device.get_info(rs.camera_info.name)
            rs_device_sid  = rs_device.get_info(rs.camera_info.serial_number)
            if "T265" in rs_device_name:
                rs_devices["T265"] = rs_device_sid
            elif "D435I" in rs_device_name:
                rs_devices["D435I"] = rs_device_sid

        return rs_devices

    def setup_T265(self):
        try:
            # Reference to a post showing how to use multiple camera: https://github.com/IntelRealSense/librealsense/issues/1735
            rs_devices = self.get_rs_devices()

            t265_sid = rs_devices.get('T265', 0)
            if t265_sid == 0:
                raise ValueError("RealSense T265 not connected. Please connect & retry")
            
            context = rs.context()

            self.t265_pipe = rs.pipeline()
            t265_config = rs.config()

            t265_config.enable_device(t265_sid)
            t265_config.enable_stream(rs.stream.pose)
            self.t265_pipe.start(t265_config)

        except Exception as e:
                print(fore.RED + "T265: Error connecting to Realsense Camera: %s" % e, style.RESET)
                raise e

    def get_pipe_data(self):
        # Wait for the next set of frames from the camera
        frames = self.t265_pipe.wait_for_frames()

        # # Fetch pose frame
        pose = frames.get_pose_frame()
        
        if pose: # is not None
            data = pose.get_pose_data()

            return data
    
    def stop(self):
        try:
            self.t265_pipe.stop()
        except:
            print("Couldn't stop the pipe")

class VIOModule(object):
    def __init__(self):
        self.config = {
            "init_sync": "True",
            "cont_sync": "True",
            "T265_UPDATE_FREQ": 10,
        }
        self.coord_trans = RS_CoordinateTransformation()

    def handle_resync(self):
        # whenever new data is published to the t265 resync topic, we need to compute a new correction 
        # to compensate for sensor drift over time.
        if self.topics["vmc.t265"].subtopic("resync").message("ned").new and (self.config["init_sync"] == "True" or self.config["cont_sync"] == "True"):
            pos_ref = self.topics["vmc.t265"].subtopic("resync").message("ned").data
            heading_ref = self.topics["vmc.t265"].subtopic("resync").message("heading").data
            self.coord_trans.sync(heading_ref, pos_ref)
            self.config["init_sync"] = "False"

    def publish_updates(self, ned_pos, ned_vel, rpy):
        updates = []
        log_updates = []

        if not np.isnan(ned_pos).any():
            n = float(ned_pos[0])
            e = float(ned_pos[1])
            d = float(ned_pos[2])
            ned_update = {
                "ned":{
                    "n": n , # cm
                    "e": e , # cm
                    "d": d # cm
                }
            }
            updates.append(ned_update)

            log_updates.append({"t265_n": n})
            log_updates.append({"t265_e": e})
            log_updates.append({"t265_d": d})

        if not np.isnan(rpy).any():
            deg = [rad*180/pi for rad in rpy]
            eul_update = {
                "eul": {
                    "psi": rpy[0],
                    "theta": rpy[1],
                    "phi": rpy[2]
                }
            }
            updates.append(eul_update)
            # print(fore.CYAN_2 + "T265: Heading: {}".format(eul_update["eul"]["phi"]), style.RESET)
            
            heading = rpy[2]
            if heading < 0:
                heading += 2 * pi
            heading = np.rad2deg(heading)
            heading_update = {
                "heading": heading
            }
            updates.append(heading_update)
            # coord_trans.heading = rpy[2]

        if not np.isnan(ned_vel).any():
            vel_update = {
                "speed":{
                    "Vn": ned_vel[0],
                    "Ve": ned_vel[1],
                    "Vd": ned_vel[2]
                }
            }
            updates.append(vel_update)

        if updates:
            # print("T265: Pos: {}".format(ned_pos))
            self.topics["vmc.t265"].pub(updates)
            self.topics["log"].pub(log_updates)
        else:
            print("T265: Bad data")

    def run(self, t265_pipe, coord_trans):

        while not INTERRUPTED:
            data = self.get_pipe_data(t265_pipe)
            if data is not None:
                # collect data from the sensor and transform it into "global" NED frame
                ned_pos, ned_vel, rpy = coord_trans.transform_t265_to_global_ned(data)
                self.publish_updates(ned_pos, ned_vel, rpy)
                self.tracker_confidence = data.tracker_confidence
                self.mapper_confidence = data.mapper_confidence
            else:
                continue

            # #if we lost confidence in our readings, trigger a recalc of the offset
            # if self.tracker_confidence < 3 or self.mapper_confidence < 3:
            #     self.has_confidence_been_lost = True

            time.sleep(1/self.config["T265_UPDATE_FREQ"])