import asyncio
import math
import json
import queue
import time
import datetime
from typing import Any, Callable, List

import mavsdk
from mavsdk.action import ActionError
from mavsdk.geofence import Point, Polygon
from mavsdk.mission_raw import MissionItem, MissionRawError
from mavsdk.offboard import VelocityBodyYawspeed, VelocityNedYaw
from pymavlink import mavutil


class MAVMQTTBase(object):
    def __init__(self, client: MQTTClient, drone: mavsdk.System = None) -> None:
        self.drone = drone
        self.mqtt_client = client

    @decorators.try_except()
    def _publish_state_machine_event(self, name: str, payload: str = "") -> None:
        """
        Create and publish state machine event.
        """
        event = StateMachine_pb2.Event(name=name, payload=payload)
        event.timestamp.GetCurrentTime()  # type: ignore
        mqtt.publish_to_topic(self.mqtt_client, "events", event)

    async def async_queue_proto_action(
        self, queue_: queue.Queue, proto: type, action: Callable, frequency: int = 10
    ) -> None:
        """
        Creates a while loop that continously tries to pull a protobuf from a queue
        and do something with it at a set frequency.

        The given function needs to accept a single argument of the protobuf object
        and be async.

        Setting the frequency to 0 will always run the action.
        """
        last_time = time.time()

        # this particular design will constantly get messages from the queue,
        # even if they are not used, just to try and process them as fast
        # as possible to prevent the queue from filling

        while True:
            try:
                # get the next item from the queue
                data = queue_.get_nowait()
                # if the frequency is 0, or the time since our last run is greater
                # than the frequency, run
                if frequency == 0 or time.time() - last_time > (1 / frequency):
                    # convert to protobuf object
                    proto_obj = protobuf.from_bytes(data, proto)
                    # call function
                    await action(proto_obj)
                    # reset timer
                    last_time = time.time()
            except queue.Empty:
                # if the queue was empty, just wait
                await asyncio.sleep(0.01)
            except Exception as e:
                Logging.exception(
                    self, e, "Unexpected error in async_queue_proto_action"
                )


class FCC(MAVMQTTBase):
    def __init__(
        self,
        drone: mavsdk.System,
        client: MQTTClient,
        action_queue: queue.Queue,
        offboard_ned_queue: queue.Queue,
        offboard_body_queue: queue.Queue,
    ) -> None:
        super().__init__(client, drone)
        self.mission_api = MissionAPI(drone, client)

        # queues
        self.action_queue = action_queue
        self.offboard_ned_queue = offboard_ned_queue
        self.offboard_body_queue = offboard_body_queue

        # current state of offboard mode, acts as a backup for PX4
        self.offboard_enabled = False

        # telemetry
        self.in_air = False
        self.is_armed = False
        self.fcc_mode = "UNKNOWN"
        self.connected = False
        self.heading = 0.0

    async def connect(self) -> None:
        """
        Connect the Drone object.
        """
        await self.drone.connect(system_address="udp://:14541")

    # region ###################  T E L E M E T R Y ###########################

    async def telemetry_tasks(self) -> asyncio.Future:
        """
        Gathers the telemetry tasks
        """
        return asyncio.gather(
            self.connected_status_telemetry(),
            self.battery_telemetry(),
            self.in_air_telemetry(),
            self.is_armed_telemetry(),
            self.flight_mode_telemetry(),
            self.landed_state_telemetry(),
            self.position_ned_telemetry(),
            self.position_lla_telemetry(),
            self.home_lla_telemetry(),
            self.attitude_euler_telemetry(),
            self.velocity_ned_telemetry(),
        )

    @decorators.async_try_except()
    async def connected_status_telemetry(self) -> None:
        """
        Runs the connected_status telemetry loop
        """
        was_connected = False
        flip_time = time.time()
        debounce_time = 2

        Logging.normal(self, f"connected_status loop started")
        async for connection_status in self.drone.core.connection_state():
            connected = connection_status.is_connected
            now = time.time()
            should_update = False

            # every time the state changes, record that time
            if connected != was_connected:
                should_update = True
                flip_time = time.time()

            # if the state has been steady for debounce_time
            if (now - flip_time > debounce_time) and should_update:
                if connected:
                    self._publish_state_machine_event("fcc_connected_event")
                else:
                    self._publish_state_machine_event("fcc_disconnected_event")
                should_update = False

            was_connected = connected

    @decorators.async_try_except()
    async def battery_telemetry(self) -> None:
        """
        Runs the battery telemetry loop
        """
        Logging.normal(self, f"battery_telemetry loop started")
        async for battery in self.drone.telemetry.battery():
            
            update = {}
            update['voltage'] = battery.voltage_v
            #TODO see if mavsdk supports battery current
            #TODO see is mavsdk supports power draw
            update["soc"] = battery.remaining_percent * 100.0
            update["timestamp"] = datetime.now()

            # publish the proto
            mqtt.publish_to_topic(self.mqtt_client, "battery", update)

    @decorators.async_try_except()
    async def in_air_telemetry(self) -> None:
        """
        Runs the in_air telemetry loop
        """
        Logging.normal(self, f"in_air loop started")
        async for in_air in self.drone.telemetry.in_air():
            self.in_air = in_air

    @decorators.async_try_except()
    async def is_armed_telemetry(self) -> None:
        """
        Runs the is_armed telemetry loop
        """
        was_armed = False
        Logging.normal(self, f"is_armed loop started")
        async for armed in self.drone.telemetry.armed():

            # if the arming status is different than last time
            if armed != was_armed:
                if armed:
                    self._publish_state_machine_event("fcc_armed_event")
                else:
                    self._publish_state_machine_event("fcc_disarmed_event")
            was_armed = armed
            self.is_armed = armed

            update = {}

            update['armed'] = armed
            update['mode'] = self.fcc_mode
            update['timestamp'] = datetime.now()

            mqtt.publish_to_topic(self.mqtt_client, "status", update)

    @decorators.async_try_except()
    async def landed_state_telemetry(self) -> None:
        """
        Runs the landed state loop, returns one of:
        IN_AIR,LANDING,ON_GROUND,TAKING_OFF, or UNKNOWN
        """
        previous_state = "UNKNOWN"

        async for state in self.drone.telemetry.landed_state():
            mode = str(state)
            # if we have a state change
            if mode != previous_state:
                if mode == "IN_AIR":
                    self._publish_state_machine_event("landed_state_in_air_event")
                if mode == "LANDING":
                    self._publish_state_machine_event("landed_state_landing_event")
                if mode == "ON_GROUND":
                    self._publish_state_machine_event("landed_state_on_ground_event")
                if mode == "TAKING_OFF":
                    self._publish_state_machine_event("landed_state_taking_off_event")
                if mode == "UNKNOWN":
                    self._publish_state_machine_event("landed_state_unknown_event")
            previous_state = mode

    @decorators.async_try_except()
    async def flight_mode_telemetry(self) -> None:
        """
        Runs the flight_mode telemetry loop
        """
        fcc_mode_map = {
            "UNKNOWN": "fcc_unknown_mode_event",
            "READY": "fcc_ready_mode_event",
            "TAKEOFF": "fcc_takeoff_mode_event",
            "HOLD": "fcc_hold_mode_event",
            "MISSION": "fcc_mission_mode_event",
            "RETURN_TO_LAUNCH": "fcc_rtl_mode_event",
            "LAND": "fcc_land_mode_event",
            "OFFBOARD": "fcc_offboard_mode_event",
            "FOLLOW_ME": "fcc_follow_mode_event",
            "MANUAL": "fcc_manual_mode_event",
            "ALTCTL": "fcc_alt_mode_event",
            "POSCTL": "fcc_pos_mode_event",
            "ACRO": "fcc_acro_mode_event",
            "STABILIZED": "fcc_stabilized_mode_event",
            "RATTITUDE": "fcc_rattitude_mode_event",
        }

        fcc_mode = "UNKNOWN"

        Logging.normal(self, f"flight_mode_telemetry loop started")

        async for mode in self.drone.telemetry.flight_mode():

            update = {}

            update['mode'] = str(mode)
            update['armed'] = self.is_armed
            update['timestamp'] = datetime.now()

            mqtt.publish_to_topic(self.mqtt_client, "status", update)

            if status.mode != fcc_mode:  # type: ignore
                if status.mode in fcc_mode_map.keys():  # type: ignore
                    self._publish_state_machine_event(fcc_mode_map[str(mode)])  # type: ignore
                else:
                    self._publish_state_machine_event("fcc_mode_error_event")
            fcc_mode = status.mode  # type: ignore
            self.fcc_mode = status.mode  # type: ignore

    @decorators.async_try_except()
    async def position_ned_telemetry(self) -> None:
        """
        Runs the position_ned telemetry loop
        """
        Logging.normal(self, f"position_ned telemetry loop started")
        async for position in self.drone.telemetry.position_velocity_ned():

            n = position.position.north_m
            e = position.position.east_m
            d = position.position.down_m

            update = {}

            update['dX'] = n
            update['dY'] = e 
            update['dZ'] = d
            update['timestamp'] = datetime.now()

            mqtt.publish_to_topic(self.mqtt_client, "location/local", update)

    @decorators.async_try_except()
    async def position_lla_telemetry(self) -> None:
        """
        Runs the position_lla telemetry loop
        """
        Logging.normal(self, f"position_lla telemetry loop started")
        async for position in self.drone.telemetry.position():
            update = {}
            update['lat'] = position.latitude_deg # type: ignore
            update['lon'] = position.longitude_deg  # type: ignore
            update['alt'] = position.relative_altitude_m  # type: ignore
            update['hdg'] = self.heading # type: ignore
            update['timestamp'] = datetime.now()

            mqtt.publish_to_topic(self.mqtt_client, "location/global", update)

    @decorators.async_try_except()
    async def home_lla_telemetry(self) -> None:
        """
        Runs the home_lla telemetry loop
        """
        Logging.normal(self, f"home_lla telemetry loop started")
        async for home_position in self.drone.telemetry.home():
            update = {}
            update['lat'] = home_position.latitude_deg  # type: ignore
            update['lon'] = home_position.longitude_deg  # type: ignore
            update['alt'] = home_position.relative_altitude_m  # type: ignore # agl
            update['timestamp'] = datetime.now()
            
            mqtt.publish_to_topic(self.mqtt_client, "location/home", update)

    @decorators.async_try_except()
    async def attitude_euler_telemetry(self) -> None:
        """
        Runs the attitude_euler telemetry loop
        """

        Logging.normal(self, f"attitude_euler telemetry loop started")
        async for attitude in self.drone.telemetry.attitude_euler():
            # Logging.normal(self, str(attitude))

            psi = attitude.roll_deg
            theta = attitude.pitch_deg
            phi = attitude.yaw_deg

            # TODO data validation?

            # do any necessary wrapping here
            update = {}

            update['euler']['roll'] = psi  # type: ignore
            update['euler']['pitch'] = theta  # type: ignore
            update['euler']['yaw'] = phi  # type: ignore
            update['timestamp'] = datetime.now()

            if phi < 0:
                heading = (2 * math.pi) + phi
            else:
                heading = phi

            heading = math.degrees(heading)

            self.heading = heading

            # publish the attitude
            mqtt.publish_to_topic(self.mqtt_client, "attitude/euler", update)

    @decorators.async_try_except()
    async def velocity_ned_telemetry(self) -> None:
        """
        Runs the velocity_ned telemetry loop
        """

        Logging.normal(self, f"velocity_ned telemetry loop started")
        async for velocity in self.drone.telemetry.velocity_ned():
            update = {}

            update['vX'] = velocity.north_m_s  # type: ignore
            update['vY'] = velocity.east_m_s  # type: ignore
            update['vZ'] = velocity.down_m_s  # type: ignore
            update['timestamp'] = datetime.now()

            mqtt.publish_to_topic(self.mqtt_client, "velocity", update)

    # endregion ###############################################################

    # region ################## D I S P A T C H E R  ##########################

    @decorators.async_try_except()
    async def action_dispatcher(self) -> None:

        prefix = "FCC CMD Dispatcher"

        class DispatcherBusy(Exception):
            """
            Exception for when the action dispatcher is currently busy
            executing another action
            """

        class DispatcherManager(MAVMQTTBase):
            def __init__(self, client: MQTTClient) -> None:
                super().__init__(client)
                self.currently_running_task = None
                self.timeout = 10

            async def schedule_task(self, task: Callable, payload: Any, name: str):
                """
                Schedule a task (async func) to be run by the dispatcher with the
                given payload. Task name is also required for printing.
                """
                Logging.normal(prefix, f"Scheduling a task for '{name}'")
                # if the dispatcher is ok to take on a new task
                if self.currently_running_task is None:
                    await self.create_task(task, payload, name)
                # or if there is already a running task
                else:  # see if the task is done
                    if self.currently_running_task.done():
                        await self.create_task(task, payload, name)
                    # or tell the caller to go away
                    else:
                        raise DispatcherBusy

            async def create_task(self, task: Callable, payload: dict, name: str):
                """
                Create a task to be run.
                """
                self.currently_running_task = asyncio.create_task(
                    self.task_waiter(task, payload, name)
                )

            async def task_waiter(self, task: Callable, payload: dict, name: str):
                """
                Execute a task with a timeout.
                """
                try:
                    await asyncio.wait_for(task(**payload), timeout=self.timeout)
                    self._publish_state_machine_event("request_" + name + "_completed_event")
                    # Logging.normal(prefix, f"Task '{name}' returned")
                    self.currently_running_task = None

                except asyncio.TimeoutError:
                    try:
                        Logging.red(prefix, f"Task '{name}' timed out!")
                        self._publish_state_machine_event("action_timeout_event", name)
                        self.currently_running_task = None
                    except Exception as e:
                        Logging.exception(prefix, e, "ERROR IN TIMEOUT HANDLER")
                except Exception as e:
                    Logging.exception(prefix, e, "ERROR IN TASK WAITER")

        Logging.normal(self, f"action_dispatcher started")

        action_map = {
            "break": self.set_intentional_timeout,
            "connect": self.connect,
            "arm": self.set_arm,
            "disarm": self.set_disarm,
            "kill": self.set_kill,
            "land": self.set_land,
            "reboot": self.set_reboot,
            "takeoff": self.set_takeoff,
            "offboard_start": self.offboard_start,
            "offboard_stop": self.offboard_stop,
            "upload_mission": self.upload_mission,
            "begin_mission": self.begin_mission,
            "pause_mission": self.pause_mission,
            "resume_mission": self.resume_mission,
        }

        dispatch = DispatcherManager(self.mqtt_client)

        while True:
            try:
                #TODO - Casey, 6/27 start here and make action into a dict instead of proto 
                action = self.action_queue.get_nowait()

                if action.payload == "":  # type: ignore
                    # Logging.normal(prefix,"Creating empty JSON string because payload was empty")
                    action.payload = "{}"  # type: ignore

                if action.name in action_map:  # type: ignore
                    payload = json.loads(action.payload)  # type: ignore
                    await dispatch.schedule_task(
                        action_map[action.name], payload, action.name  # type: ignore
                    )
            except DispatcherBusy:
                Logging.info(prefix, "I'm busy running another task, try again later")
                self._publish_state_machine_event("fcc_busy_event",payload=action.name)
            except queue.Empty:
                await asyncio.sleep(0.1)
            except Exception as e:
                Logging.exception(prefix, e, "ERROR IN MAIN LOOP")

    async def simple_action_executor(
        self,
        action_fn: Callable,
        action_text: str,
    ) -> None:
        """
        Executes a given async action function, and publishes a success or failed
        state machine event given whether or not an `ActionError` was raised.
        """
        try:
            await action_fn()
            full_success_str = action_text + "_success_event"
            Logging.info(self, f"Sending {full_success_str}")
            self._publish_state_machine_event(full_success_str)
        except ActionError as e:
            full_fail_str = action_text + "_failed_event"
            Logging.info(self, f"Sending {full_fail_str}")
            self._publish_state_machine_event(full_fail_str)
            if e._result.result_str == "CONNECTION_ERROR":
                asyncio.create_task(self.connect())
            raise e

    # endregion ###############################################################

    # region #####################  A C T I O N S #############################

    @decorators.async_try_except()
    async def set_intentional_timeout(self, **kwargs) -> None:
        """
        Sets a 20 second timeout.
        """
        try:
            await asyncio.sleep(20)
        except asyncio.CancelledError:
            pass

    @decorators.async_try_except(reraise=True)
    async def set_arm(self, **kwargs) -> None:
        """
        Sets the drone to an armed state.
        """
        Logging.info(self, "Sending arm command")
        await self.simple_action_executor(self.drone.action.arm, "arm")

    @decorators.async_try_except(reraise=True)
    async def set_disarm(self, **kwargs) -> None:
        """
        Sets the drone to a disarmed state.
        """
        Logging.info(self, "Sending disarm command")
        await self.simple_action_executor(self.drone.action.disarm, "disarm")

    @decorators.async_try_except(reraise=True)
    async def set_kill(self, **kwargs) -> None:
        """
        Sets the drone to a kill state. This will forcefully shut off the drone
        regardless of being in the air or not.
        """
        Logging.warn(self, "Sending kill command")
        await self.simple_action_executor(self.drone.action.kill, "kill")

    @decorators.async_try_except(reraise=True)
    async def set_land(self, **kwargs) -> None:
        """
        Commands the drone to land at the current position.
        """
        Logging.info(self, "Sending land command")
        await self.simple_action_executor(self.drone.action.land, "land_cmd")

    @decorators.async_try_except(reraise=True)
    async def set_reboot(self, **kwargs) -> None:
        """
        Commands the drone computer to reboot.
        """
        Logging.warn(self, "Sending reboot command")
        await self.simple_action_executor(self.drone.action.reboot, "reboot")

    @decorators.async_try_except(reraise=True)
    async def set_takeoff(self, takeoff_alt: float, **kwargs) -> None:
        """
        Commands the drone to takeoff to the given altitude.
        Will arm the drone if it is not already.
        """
        Logging.info(self, f"Setting takeoff altitude to {takeoff_alt}")
        await self.drone.action.set_takeoff_altitude(takeoff_alt)
        await self.set_arm()
        Logging.info(self, "Sending takeoff command")
        await self.simple_action_executor(self.drone.action.takeoff, "takeoff")

    @decorators.async_try_except(reraise=True)
    async def upload_mission(self, waypoints: List[dict], **kwargs) -> None:
        """
        Calls the mission api to upload a mission to the fcc.
        """
        Logging.info(self, "Starting mission upload process")
        await self.mission_api.build_and_upload(waypoints)

    @decorators.async_try_except(reraise=True)
    async def begin_mission(self, **kwargs) -> None:
        """
        Arms the drone and calls the mission api to start a mission.
        """
        Logging.info(self, "Arming the drone")
        await self.set_arm()
        # we shouldn't have to check the armed status because
        # the arm fn should raise an exception if it is unsuccessful
        Logging.info(self, "Starting the mission")
        await self.mission_api.start()
        if self.in_air:
            self._publish_state_machine_event("mission_starting_from_air_event")

    @decorators.async_try_except(reraise=True)
    async def pause_mission(self, **kwargs) -> None:
        """
        Calls the mission api to pasue a mission to the fcc.
        """
        Logging.info(self, "Starting mission upload process")
        await self.mission_api.pause()

    @decorators.async_try_except(reraise=True)
    async def resume_mission(self, **kwargs) -> None:
        """
        Calls the mission api to pasue a mission to the fcc.
        """
        Logging.info(self, "Resuming Mission")
        await self.mission_api.resume()

    # endregion ###############################################################

    # region ##################### O F F B O A R D ############################

    async def offboard_tasks(self) -> asyncio.Future:
        """
        Gathers the offboard tasks
        """
        return asyncio.gather(self.offboard_ned(), self.offboard_body())

    async def offboard_start(self, **kwargs) -> None:
        """
        Starts offboard mode on the drone. Use with caution!
        """
        Logging.info(self, "Starting offboard mode")
        await self.drone.offboard.start()
        self.offboard_enabled = True

    async def offboard_stop(self, **kwargs) -> None:
        """
        Stops offboard mode on the drone.
        """
        Logging.info(self, "Stopping offboard mode")
        self.offboard_enabled = False
        await self.drone.offboard.stop()

    async def offboard_ned(self) -> None:
        """
        Feeds offboard NED data to the drone.
        """
        Logging.normal(self, f"offboard_ned loop started")

        @decorators.async_try_except()
        async def process_offboard_ned(
            proto_object: FlightControlModule_pb2.Offboard_NED,
        ) -> None:
            # if not currently in offboard mode, skip
            if not self.offboard_enabled:
                return

            north = proto_object.north  # type: ignore
            east = proto_object.east  # type: ignore
            down = proto_object.down  # type: ignore
            yaw = proto_object.yaw  # type: ignore
            await self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(north, east, down, yaw)
            )

        await self.async_queue_proto_action(
            self.offboard_ned_queue,
            FlightControlModule_pb2.Offboard_NED,
            process_offboard_ned,
            frequency=20,
        )

    async def offboard_body(self) -> None:
        """
        Feeds offboard body data to the drone.
        """
        Logging.normal(self, f"offboard_body loop started")

        @decorators.async_try_except()
        async def process_offboard_body(
            proto_object: FlightControlModule_pb2.Offboard_Body,
        ) -> None:
            # if not currently in offboard mode, skip
            if not self.offboard_enabled:
                return

            forward = proto_object.forward  # type: ignore
            right = proto_object.right  # type: ignore
            down = proto_object.down  # type: ignore
            yaw = proto_object.yaw  # type: ignore
            await self.drone.offboard.set_velocity_ned(
                VelocityBodyYawspeed(forward, right, down, yaw)
            )

        await self.async_queue_proto_action(
            self.offboard_body_queue,
            FlightControlModule_pb2.Offboard_Body,
            process_offboard_body,
            frequency=20,
        )

    # endregion ###############################################################


class MissionAPI(MAVMQTTBase):
    def __init__(self, drone: mavsdk.System, client: MQTTClient) -> None:
        super().__init__(client, drone)

    @decorators.async_try_except(reraise=True)
    async def set_geofence(
        self, min_lat: float, min_lon: float, max_lat: float, max_lon: float
    ) -> None:
        """
        Creates and uploads an inclusive geofence given min/max lat/lon.
        """
        Logging.info(
            self,
            f"Uploading geofence of ({min_lat}, {min_lon}), ({max_lat}, {max_lon})",
        )

        # need to create a rectangle, PX4 isn't quite smart enough
        # to recognize only two corners
        tl_point = Point(max_lat, min_lon)
        tr_point = Point(max_lat, max_lon)
        bl_point = Point(min_lat, min_lon)
        br_point = Point(min_lat, max_lon)

        fence = [
            Polygon(
                [tl_point, tr_point, bl_point, br_point], Polygon.FenceType.INCLUSION
            )
        ]
        await self.drone.geofence.upload_geofence(fence)

    @decorators.async_try_except(reraise=True)
    async def build(self, waypoints: List[dict]) -> List[MissionItem]:
        """
        Convert a list of waypoints (dict) to a list of MissionItems.
        """
        mission_items = []

        # if the first waypoint is not a takeoff waypoint, create one
        if waypoints[0]["type"] != "takeoff":
            # use the altitude of the first waypoint
            waypoints.insert(0, {"type": "takeoff", "alt": waypoints[0]["alt"]})

        # now, check if first waypoint has a lat/lon
        # and if not, add lat lon of current position
        waypoint_0 = waypoints[0]
        if "lat" not in waypoints[0] or "lon" not in waypoints[0]:
            # get the next update from the raw gps and use that
            # .position() only updates on new positions
            # TODO, verify this is correct
            position = await self.drone.telemetry.raw_gps().__anext__()
            waypoint_0["lat"] = position.latitude_deg
            waypoint_0["lon"] = position.longitude_deg

        # convert the dicts into mission_raw.MissionItems
        for seq, waypoint in enumerate(waypoints):
            waypoint_type = waypoint["type"]

            # https://mavlink.io/en/messages/common.html#MISSION_ITEM_INT
            command = None
            param1 = None
            param2 = None
            param3 = None
            param4 = None

            if waypoint_type == "takeoff":
                # https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_TAKEOFF
                command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
                param1 = 0  # pitch
                param2 = float("nan")  # empty
                param3 = float("nan")  # empty
                param4 = float("nan")  # yaw angle. NaN uses current yaw heading mode

            elif waypoint_type == "goto":
                # https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_WAYPOINT
                command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
                param1 = 0  # hold time
                param2 = 0  # accepteance radius
                param3 = 0  # pass radius, 0 goes straight through
                param4 = float("nan")  # yaw angle. NaN uses current yaw heading mode

            elif waypoint_type == "land":
                # https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LAND
                command = mavutil.mavlink.MAV_CMD_NAV_LAND
                param1 = 0  # abort altitude, 0 uses system default
                # https://mavlink.io/en/messages/common.html#PRECISION_LAND_MODE
                # precision landing mode
                param2 = mavutil.mavlink.PRECISION_LAND_MODE_DISABLED
                param3 = float("nan")  # empty
                param4 = float("nan")  # yaw angle. NaN uses current yaw heading mode

            # https://mavlink.io/en/messages/common.html#MAV_FRAME
            frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
            current = int(seq == 0)  # boolean
            autocontinue = int(True)
            x = int(float(waypoint["lat"]) * 10000000)
            y = int(float(waypoint["lon"]) * 10000000)
            z = float(waypoint["alt"])
            # https://mavlink.io/en/messages/common.html#MAV_MISSION_TYPE
            mission_type = mavutil.mavlink.MAV_MISSION_TYPE_MISSION

            mission_items.append(
                MissionItem(
                    seq=seq,
                    frame=frame,
                    command=command,
                    current=current,
                    autocontinue=autocontinue,
                    param1=param1,
                    param2=param2,
                    param3=param3,
                    param4=param4,
                    x=x,
                    y=y,
                    z=z,
                    mission_type=mission_type,
                )
            )

        return mission_items

    @decorators.async_try_except(reraise=True)
    async def upload(self, mission_items: List[MissionItem]) -> None:
        """
        Upload a given list of MissionItems to the drone.
        """
        try:
            Logging.info(self, "Clearing existing mission on the drone")
            await self.drone.mission_raw.clear_mission()
            Logging.info(self, "Uploading mission items to drone")
            await self.drone.mission_raw.upload_mission(mission_items)
            self._publish_state_machine_event("mission_upload_success_event")
        except MissionRawError as e:
            Logging.red(
                self, f"Mission upload failed because: {str(e._result.result_str)}"
            )
            self._publish_state_machine_event(
                "mission_upload_failed_event", str(e._result.result_str)
            )

    @decorators.async_try_except(reraise=True)
    async def build_and_upload(self, waypoints: List[dict]) -> None:
        """
        Upload a list of waypoints (dict) to the done.
        """
        mission_plan = await self.build(waypoints)
        await self.upload(mission_plan)

    @decorators.async_try_except(reraise=True)
    async def download(self) -> List[MissionItem]:
        """
        Download the current mission from the drone as a list of MissionItems.
        """
        Logging.info(self, "Downloading mission plan from drone")
        return await self.download()

    @decorators.async_try_except(reraise=True)
    async def wait_for_finish(self) -> None:
        """
        Async blocking function that waits for the current mission to be finished.
        """
        # self.drone.mission.is_missiion_finished unfortunately does not work,
        # with the mission_raw.MissionItems we've uploaded

        # if called immediately after a mission has been started, will immediately
        # exit as the drone hasn't even started moving to the first waypoint yet
        # give it 5 seconds to get moving first.
        mission_progress = await self.drone.mission_raw.mission_progress().__anext__()
        if mission_progress.current == 0:
            await asyncio.sleep(5)

        async for mission_progress in self.drone.mission_raw.mission_progress():
            if mission_progress.current == 0:
                return

    @decorators.async_try_except(reraise=True)
    async def start(self) -> None:
        """
        Commands the drone to start the current mission.
        Drone must already be armed.
        Will raise an exception if the active mission violates a geofence.
        """
        Logging.info(self, "Sending start mission command")
        await self.drone.mission_raw.start_mission()

    @decorators.async_try_except(reraise=True)
    async def hold(self) -> None:
        """
        Commands the drone to hold the current mission.
        """
        Logging.info(self, "Sending pause mission command")
        await self.drone.mission_raw.pause_mission()

    @decorators.async_try_except(reraise=True)
    async def pause(self) -> None:
        """
        Commands the drone to pause the current mission.
        """
        Logging.info(self, "Sending pause mission command")
        await self.hold()

    @decorators.async_try_except(reraise=True)
    async def resume(self) -> None:
        """
        Commands the drone to resume the paused mission.
        """
        Logging.info(self, "Sending resume mission command")
        await self.start()


class PyMAVLinkAgent(MAVMQTTBase):
    def __init__(self, client: MQTTClient, mocap_queue: queue.Queue) -> None:
        super().__init__(client)
        self.mocap_queue = mocap_queue

    @decorators.async_try_except()
    async def run(self) -> None:
        """
        Set up a mavlink connection and kick off any tasks
        """
        loop = asyncio.get_event_loop()

        # create a mavlink udp instance
        self.master = mavutil.mavlink_connection(
            "udpin:0.0.0.0:14542", source_system=254, dialect="bell"
        )

        await loop.run_in_executor(None, self.wait_for_heartbeat)
        asyncio.gather(self.set_hil_gps())

        await iot.async_spinner()

    def wait_for_heartbeat(self) -> Any:
        """
        Wait for a MAVLINK heartbeat message.
        """
        try:
            Logging.normal(self, "Waiting for mavlink heartbeat")
            m = self.master.recv_match(type="HEARTBEAT", blocking=True)
            Logging.normal(self, "C O N N E C T E D")
            return m
        except Exception as e:
            Logging.exception(self, e, "")

    @decorators.async_try_except()
    async def set_hil_gps(self) -> None:
        """
        Sends GPS position / velocity / heading to PX4,
        used to fake a GPS signal while we fly indoors.

        This is not the standard hil_gps message in the mavlink common message set,
        this is a custom mavlink message that includes heading,
        because the standard message doesn't.
        """

        def print_stats(last_print_time: float) -> float:
            """
            Takes the last print time, and determines
            whether or not to print statistics. Returns
            the last time statistics were printed.
            """
            if time.time() - last_print_time > 1:
                Logging.normal(self, f"Number of mocap messages {num_mocaps}")
                return time.time()
            return last_print_time

        def mocap_proto_to_offboard_proto(
            mocap_proto: mocap_pb2.Mocap,
        ) -> FlightControlModule_pb2.Offboard_GPS:
            """
            Function to convert the proto coming over the wire to the hil proto
            needed for the hil_gps message
            """
            hilgps_proto = FlightControlModule_pb2.Offboard_GPS()

            hilgps_proto.time_usec = int(mocap_proto.timestamp.ToMicroseconds())  # type: ignore
            hilgps_proto.fix_type = int(3)  # type: ignore  # this will always be "3" for a 3D fix
            hilgps_proto.lat = int(mocap_proto.current_position.latitude * 1e7)  # type: ignore
            hilgps_proto.lon = int(mocap_proto.current_position.longitude * 1e7)  # type: ignore
            hilgps_proto.alt = int(mocap_proto.current_position.altitude * 1000)  # type: ignore  # m to mm
            hilgps_proto.eph = int(10)  # type: ignore  # horizontal dilution of precision in ?
            hilgps_proto.epv = int(10)  # type: ignore  # vertical duilution of precision in ?
            hilgps_proto.vel = int(mocap_proto.groundspeed * 100)  # type: ignore  # m to cm
            hilgps_proto.v_north = int(mocap_proto.local_vel.x * 100)  # type: ignore  # m to cm
            hilgps_proto.v_east = int(mocap_proto.local_vel.y * 100)  # type: ignore  # m to cm
            hilgps_proto.v_down = int(mocap_proto.local_vel.z * 100)  # type: ignore  # m to cm
            hilgps_proto.cog = int(mocap_proto.course * 100)  # type: ignore  # deg to cdeg
            hilgps_proto.sats_visible = int(13)  # type: ignore
            hilgps_proto.heading = int(mocap_proto.heading * 100)  # type: ignore  # deg to cdeg

            return hilgps_proto

        def send_hil_gps(gps_data: FlightControlModule_pb2.Offboard_GPS) -> None:
            """
            Sends the HIL GPS message.
            """
            try:
                msg = self.master.mav.hil_gps_heading_encode(  # type: ignore
                    gps_data.time_usec,  # type: ignore
                    gps_data.fix_type,  # type: ignore
                    gps_data.lat,  # type: ignore
                    gps_data.lon,  # type: ignore
                    gps_data.alt,  # type: ignore
                    gps_data.eph,  # type: ignore
                    gps_data.epv,  # type: ignore
                    gps_data.vel,  # type: ignore
                    gps_data.v_north,  # type: ignore
                    gps_data.v_east,  # type: ignore
                    gps_data.v_down,  # type: ignore
                    gps_data.cog,  # type: ignore
                    gps_data.sats_visible,  # type: ignore
                    gps_data.heading,  # type: ignore
                )
                self.master.mav.send(msg)  # type: ignore
            except Exception as e:
                Logging.exception(self, e, "Issue send HIL GPS")

        HIL_FREQ = 15

        last_print_time = time.time()
        last_send_time = time.time()

        num_mocaps = 0

        while True:
            try:
                # print statistics
                last_print_time = print_stats(last_print_time)
                # get the next item
                data = self.mocap_queue.get_nowait()
                num_mocaps += 1
                now = time.time()

                # if time to send a new item, do so
                if now - last_send_time > (1 / HIL_FREQ):
                    # prepare hil data
                    hil_data = mocap_proto_to_offboard_proto(
                        protobuf.from_bytes(data, mocap_pb2.Mocap)
                    )
                    # send it
                    send_hil_gps(hil_data)
                    last_send_time = time.time()
            except queue.Empty:
                await asyncio.sleep(0.01)
            except Exception as e:
                Logging.exception(self, e, "Issue sending HIL GPS")
