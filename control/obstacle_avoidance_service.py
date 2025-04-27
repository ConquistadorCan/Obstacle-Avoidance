import math

from core.log_status_enum import LogStatusEnum
from algorithms.base import BaseAvoidanceAlgorithm
from control.drone_controller import DroneController
from ros_nodes.lidar_2d_listener import Lidar2DListener

class ObstacleAvoidanceService:
    def __init__(self, detection_algorithm: BaseAvoidanceAlgorithm, drone_controller: DroneController, danger_threshold: int = 2):
        self.detection_algorithm = detection_algorithm
        self.drone_controller = drone_controller
        self.active = False

        self.lidar_listener = Lidar2DListener()

        self.danger_threshold = danger_threshold
        self.sector_width = math.ceil(math.sin(0.35 / self.danger_threshold) * 180 / math.pi)
        self.start_idx = self.lidar_listener.forward_angle - self.sector_width
        self.end_idx = self.lidar_listener.forward_angle + self.sector_width

    def start_detection_and_avoidance(self):
        self.active = True
        self._run()

    def _run(self):
        while self.active:
            lidar_data = self.lidar_listener.get_lidar_data()

            if self._is_path_clear(lidar_data):
                continue

            self.drone_controller.stop_immediate()

            while not self._is_path_clear(lidar_data):
                self._avoid_obstacle(lidar_data)
                lidar_data = self.lidar_listener.get_lidar_data()

    def _avoid_obstacle(self, lidar_data):
        decision = self.detection_algorithm.make_decision(lidar_data)

        self.drone_controller.send_velocity_command(decision.vx, decision.vy, decision.vz)     

    def _is_path_clear(self, lidar_data):
        forward_distances = [
            d for d in lidar_data[self.start_idx:self.end_idx]
            if math.isfinite(d) and self.lidar_listener.RANGE_MIN <= d <= self.lidar_listener.RANGE_MAX
        ]

        if not forward_distances:
            print(f"{LogStatusEnum.WARNING.value} No valid Lidar data in forward sector!")
            return True

        if min(forward_distances) < self.danger_threshold:
            print(f"{LogStatusEnum.WARNING.value} Obstacle detected in front! Distance: {min(forward_distances):.2f} m")
            return False

        return True
