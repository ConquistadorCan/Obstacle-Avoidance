import math

from core.log_status_enum import LogStatusEnum
from core.flight_mode_enum import FlightModeEnum
from algorithms.base import BaseAvoidanceAlgorithm
from control.drone_controller import DroneController
from ros_nodes.lidar_2d_listener import Lidar2DListener

class ObstacleAvoidanceService:
    def __init__(self, detection_algorithm: BaseAvoidanceAlgorithm, drone_controller: DroneController, danger_threshold: int = 3):
        self.detection_algorithm = detection_algorithm
        self.drone_controller = drone_controller
        self.active = True

        self.lidar_listener = Lidar2DListener()

        self.danger_threshold = danger_threshold
        self.sector_width = math.ceil(math.sin(0.35 / self.danger_threshold) * 180 / math.pi)
        self.start_idx = self.lidar_listener.forward_angle - self.sector_width
        self.end_idx = self.lidar_listener.forward_angle + self.sector_width

        self.detection_algorithm.configure(sector_width=self.sector_width, forward_angle=self.lidar_listener.forward_angle)

    def start_detection_and_avoidance(self):
        self.active = True
        try:
            self._run()
        except KeyboardInterrupt as e:
            self.detection_algorithm.generate_report()
            raise e

    def _run(self):
        while self.active:
            lidar_data = self.lidar_listener.get_lidar_data()

            if self._is_path_clear(lidar_data):
                continue

            self.drone_controller.stop_immediate()

            while self.active and not self._is_path_clear(lidar_data):
                self._avoid_obstacle(lidar_data, True)
                lidar_data = self.lidar_listener.get_lidar_data()

            if not self.active:
                self.drone_controller.set_mode(FlightModeEnum.RTL)
                self.detection_algorithm.generate_report()
                return

            self.drone_controller.stop_immediate()

            print(f"{LogStatusEnum.INFO.value} Path is clear, resuming mission.")
            
            self.drone_controller.set_mode(FlightModeEnum.AUTO)

    def _avoid_obstacle(self, lidar_data, use_default_heading: bool = False):
        decision = self.detection_algorithm.make_decision(lidar_data, 0.5, self.danger_threshold)

        if not decision.valid:
            print(f"{LogStatusEnum.ERROR.value} Could not find valid path.")
            self.active = False
            return

        yaw = (
            self.drone_controller.get_next_waypoint_yaw()
            if decision.yaw == 0 and use_default_heading
            else decision.yaw
        )

        self.drone_controller.send_velocity_command(vx=decision.vx, vy=decision.vy, vz=decision.vz, yaw=yaw)

    def _is_path_clear(self, lidar_data):
        forward_distances = [
            d for d in lidar_data[self.start_idx:self.end_idx]
            if math.isfinite(d) and self.lidar_listener.RANGE_MIN <= d <= self.lidar_listener.RANGE_MAX
        ]

        if not forward_distances:
            return True

        if min(forward_distances) < self.danger_threshold:
            print(f"{LogStatusEnum.WARNING.value} Obstacle detected in front! Distance: {min(forward_distances):.2f} m")
            return False

        return True
