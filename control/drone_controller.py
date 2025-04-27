import math

import pyproj
from pymavlink import mavutil

from config.path_utils import MISSIONS_DIR
from core.log_status_enum import LogStatusEnum
from core.flight_mode_enum import FlightModeEnum
from core.type_mask_helper import build_type_mask
from core.message_type_enum import MessageTypeEnum
from config.home_location import HOME_LAT, HOME_LON
from ros_nodes.altitude_monitor import AltitudeMonitor
from missions.missions_util import convert_mission_file_local_to_wgs

class DroneController:
    def __init__(self, connection_string: str = "tcp:127.0.0.1:5763"):
        self.connection = mavutil.mavlink_connection(connection_string)
        self.connection.wait_heartbeat()
        print(f"{LogStatusEnum.SUCCESS.value} Connected to drone on {connection_string}")

    def set_mode(self, mode: FlightModeEnum, timeout_per_attempt: int = 0.05, attempt_limit: int = 1000):
        def _wait_for_mode():
            attempt = 0

            while attempt < attempt_limit:
                msg = self.connection.recv_match(type='HEARTBEAT', blocking=True, timeout=timeout_per_attempt)

                if msg is None:
                    attempt += 1
                    continue
                
                if msg.get_srcSystem() != self.connection.target_system: # Heartbeat message came from a different system, 2 means drone
                    attempt += 1
                    continue

                if msg.custom_mode == mode.value:
                    return True
                else:
                    attempt += 1
                    continue
            
            raise Exception(f"{LogStatusEnum.ERROR.value} Failed to set mode {mode.value} after {attempt_limit} attempts.")
        
        print(f"{LogStatusEnum.WAITING.value} Setting mode to {mode.name}")

        self.connection.mav.set_mode_send(
            self.connection.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode.value
        )

        _wait_for_mode()

        print(f"{LogStatusEnum.SUCCESS.value} Mode successfully set to {mode.name}")

    def arm_drone(self, timeout_per_attempt: int = 0.1, attempt_limit: int = 20):
        def _wait_for_arm():
            attempt = 0

            while attempt < attempt_limit:
                msg = self.connection.recv_match(type='HEARTBEAT', blocking=True, timeout=timeout_per_attempt)

                if msg is None:
                    attempt += 1
                    continue
                
                if msg.get_srcSystem() != self.connection.target_system:
                    attempt += 1
                    continue

                if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                    return True
                else:
                    attempt += 1
                    continue
            
            raise Exception(f"{LogStatusEnum.ERROR.value} Failed to arm drone after {attempt_limit} attempts.")

        print(f"{LogStatusEnum.WAITING.value} Arming drone")

        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        
        _wait_for_arm()

        print(f"{LogStatusEnum.SUCCESS.value} Drone successfully armed")

    def takeoff(self, altitude: float, altitude_threshold: float = 0.95, attempt_limit: int = 50, altitude_monitor: AltitudeMonitor = None):
        altitude_monitor = altitude_monitor or AltitudeMonitor()

        def _wait_for_takeoff(tolerance: float = 0.05) -> bool:
            attempt = 0
            prev_alt = altitude_monitor.get_current_altitude()

            while prev_alt < altitude * altitude_threshold:
                curr_alt = altitude_monitor.get_current_altitude()

                if prev_alt < 0.2:
                    prev_alt = curr_alt
                    continue

                if abs(curr_alt - prev_alt) < tolerance:
                    attempt += 1
                    if attempt > attempt_limit:
                        return False
                else:
                    prev_alt = curr_alt
                    attempt = 0

            return True

        def _try_takeoff() -> bool:
            print(f"{LogStatusEnum.WAITING.value} Sending takeoff command to {altitude} meters")

            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 0, altitude, 0
            )

            return _wait_for_takeoff()

        if _try_takeoff():
            print(f"{LogStatusEnum.SUCCESS.value} Drone successfully took off to {altitude} meters")
            return True

        else:
            print(f"{LogStatusEnum.WARNING.value} Drone failed to take off, trying again")

            self.send_velocity_command(vz = -0.1)

            while not _wait_for_takeoff():
                print(f"{LogStatusEnum.WARNING.value} Drone failed to take off, trying again")

                self.send_velocity_command(vz = -0.1)

            print(f"{LogStatusEnum.SUCCESS.value} Drone successfully took off to {altitude} meters")
            
            self.stop_immediate()

    def send_velocity_command(
        self,
        frame: int = mavutil.mavlink.MAV_FRAME_BODY_NED,
        time_boot_ms: int = 0,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        vx: float = 0.0,
        vy: float = 0.0,
        vz: float = 0.0,
        ax: float = 0.0,
        ay: float = 0.0,
        az: float = 0.0,
        yaw: float = 0.0,
        yaw_rate: float = 0.0,
        type_mask: int = build_type_mask(ignore_position=True, ignore_acceleration=True)
    ):
        self.connection.mav.set_position_target_local_ned_send(
            time_boot_ms,
            self.connection.target_system,
            self.connection.target_component,
            frame,
            type_mask,
            x, y, z,
            vx, vy, vz,
            ax, ay, az,
            yaw,
            yaw_rate,
        )

    def start_auto_mission(self, mission_file_name: str, altitude: float = 10):
        print(f"{LogStatusEnum.WAITING.value} Starting auto mission from {mission_file_name}")

        def _get_waypoints_from_file(file_name: str):
            waypoints = []
            mission_file_path = MISSIONS_DIR / f"{file_name}.txt"
            with open(mission_file_path, "r") as mission_file:
                for line in mission_file:
                    parts = line.strip().split(',')
                    if len(parts) != 3:
                        continue
                    lat, lon, alt = map(float, parts)
                    waypoints.append((lat, lon, alt))
            return waypoints

        def _send_waypoints(waypoints):
            self.connection.mav.mission_clear_all_send(
                self.connection.target_system, self.connection.target_component
            )
            self.connection.recv_match(type='MISSION_ACK', blocking=True)
            
            count = len(waypoints)
            self.connection.mav.mission_count_send(
                self.connection.target_system,
                self.connection.target_component,
                count * 3
            )
            req = self.connection.recv_match(type='MISSION_REQUEST', blocking=True)

            for i, (lat, lon, alt) in enumerate(waypoints):
                while req.seq != i * 3:
                    req = self.connection.recv_match(type='MISSION_REQUEST', blocking=True)
                
                previous_lat, previous_lon = (waypoints[i - 1][:2] if i > 0 else (HOME_LAT, HOME_LON))
                target_lat, target_lon = (HOME_LAT, HOME_LON) if i == len(waypoints) - 1 else (lat, lon)

                geosedic = pyproj.Geod(ellps='WGS84')
                yaw_angle, _, _ = geosedic.inv(
                    previous_lon, previous_lat,
                    target_lon, target_lat
                )
                yaw_angle = round(yaw_angle % 360, 2)

                self.connection.mav.mission_item_int_send(
                    self.connection.target_system,
                    self.connection.target_component,
                    i * 3,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                    0,
                    1,
                    yaw_angle,
                    40,
                    0,
                    0,
                    0,
                    0,
                    0
                )
                req = self.connection.recv_match(type='MISSION_REQUEST', blocking=True)
                while req.seq != i * 3 + 1:
                    req = self.connection.recv_match(type='MISSION_REQUEST', blocking=True)

                self.connection.mav.mission_item_int_send(
                    self.connection.target_system,
                    self.connection.target_component,
                    i * 3 + 1,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_DELAY,
                    0,
                    1,
                    5,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                )

                req = self.connection.recv_match(type='MISSION_REQUEST', blocking=True)
                while req.seq != i * 3 + 2:
                    req = self.connection.recv_match(type='MISSION_REQUEST', blocking=True)
                
                command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT if i < count - 1 else mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
                self.connection.mav.mission_item_int_send(
                    self.connection.target_system,
                    self.connection.target_component,
                    i*3 + 2,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    command,
                    1 if i == 0 else 0,
                    1, 
                    0, 
                    0, 0, 0,
                    int(lat * 1e7),
                    int(lon * 1e7),
                    alt
                )
                if i < count - 1:
                    req = self.connection.recv_match(type='MISSION_REQUEST', blocking=True)
                else:
                    self.connection.recv_match(type='MISSION_ACK', blocking=True)

        convert_mission_file_local_to_wgs(mission_file_name, "mission_converted", HOME_LAT, HOME_LON)
        waypoints = _get_waypoints_from_file("mission_converted")
        _send_waypoints(waypoints)

        self.set_mode(FlightModeEnum.GUIDED)
        self.arm_drone()
        self.takeoff(altitude)
        self.set_mode(FlightModeEnum.AUTO)
        print(f"{LogStatusEnum.SUCCESS.value} Auto mission started")

    def stop_immediate(self, attempt_limit: int = 100):
        self.set_mode(FlightModeEnum.GUIDED)
        for _ in range(attempt_limit):
            self.send_velocity_command(0, 0, 0)

    def get_next_waypoint_yaw(self) -> float:
        current_msg = self._request_single_message(MessageTypeEnum.GLOBAL_POSITION)
        current_lon, current_lat = current_msg.lon / 1e7, current_msg.lat / 1e7
        heading = current_msg.hdg / 1e2

        mission_msg = self._request_single_message(MessageTypeEnum.MISSION_CURRENT)
        seq = mission_msg.seq

        self.connection.mav.mission_request_int_send(
            self.connection.target_system,
            self.connection.target_component,
            seq
        )

        waypoint_msg = self.connection.recv_match(type="MISSION_ITEM_INT", blocking=True)
        target_lon, target_lat = waypoint_msg.y / 1e7, waypoint_msg.x / 1e7

        geod = pyproj.Geod(ellps="WGS84")
        yaw, _, _ = geod.inv(current_lon, current_lat, target_lon, target_lat)
        yaw = round(yaw % 360, 2)

        relative_yaw = (yaw - heading + 360) % 360
        if relative_yaw > 180:
            relative_yaw -= 360
        relative_yaw_rad = math.radians(relative_yaw)

        return round(relative_yaw_rad, 2)

    def _request_single_message(self, message_type: MessageTypeEnum, interval_us: int = 1_000_000):
        self._set_message_interval(message_type, interval_us)

        msg = self.connection.recv_match(
            type=message_type.value.get("str_value"),
            blocking=True
        )

        self._set_message_interval(message_type, -1)

        return msg

    def _set_message_interval(self, message_type: MessageTypeEnum, interval: int):
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_type.value.get("int_value"),
            interval,
            0, 0, 0, 0, 0,
        )
