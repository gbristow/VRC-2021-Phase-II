import pyrealsense2 as rs 
from typing import Dict
from loguru import logger
from colored import fore, back, style

class T265(object):
    '''
    Realsense T265 Tracking Camera interface. Manages pulling data off of the camera for use by the transforms to get it in the correct reference frame.
    '''
    def __init__(self):
        self.pipe = None

    def get_rs_devices(self) -> Dict:
        """ Get Serial numbers of connected RealSense devices"""
        rs_devices = {}
        rs_context: rs.context = rs.context()
        for i in range(len(rs_context.devices)):
            rs_device = rs_context.devices[i]
            rs_device_name = rs_device.get_info(rs.camera_info.name)
            rs_device_sid  = rs_device.get_info(rs.camera_info.serial_number)
            if "T265" in rs_device_name:
                rs_devices["T265"] = rs_device_sid
            elif "D435I" in rs_device_name:
                rs_devices["D435I"] = rs_device_sid

        return rs_devices

    def setup(self) -> None:
        try:
            # Reference to a post showing how to use multiple camera: https://github.com/IntelRealSense/librealsense/issues/1735
            rs_devices = self.get_rs_devices()

            t265_sid = rs_devices.get('T265', 0)
            if t265_sid == 0:
                raise ValueError("RealSense T265 not connected. Please connect & retry")
            
            context = rs.context()

            self.pipe: rs.pipeline = rs.pipeline()
            t265_config = rs.config()

            t265_config.enable_device(t265_sid)
            t265_config.enable_stream(rs.stream.pose)
            self.pipe.start(t265_config)

        except Exception as e:
                logger.exception(f"{fore.RED}T265: Error connecting to Realsense Camera: {e}{style.RESET}")
                raise e

    def get_pipe_data(self) -> rs.pose:
        # Wait for the next set of frames from the camera
        frames = self.pipe.wait_for_frames()

        # # Fetch pose frame
        pose = frames.get_pose_frame()
        
        if pose: # is not None
            data = pose.get_pose_data()
            return data
    
    def stop(self) -> None:
        try:
            self.pipe.stop()
        except:
           logger.exception("Couldn't stop the pipe")