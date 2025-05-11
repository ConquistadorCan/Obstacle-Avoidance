import math
import numpy as np
from enum import Enum
from datetime import datetime
import matplotlib.pyplot as plt

import rclpy # type: ignore
from ros_nodes.lidar_2d_listener import Lidar2DListener

from config.path_utils import ROOT_DIR
from algorithms.base import AvoidanceDecision
from core.log_status_enum import LogStatusEnum
from algorithms.base import BaseAvoidanceAlgorithm

class EscapeDirectionEnum(Enum):
    LEFT = "left"
    RIGHT = "right"

class RuleBasedAvoidanceAlgorithm(BaseAvoidanceAlgorithm):
    def __init__(self, visualize_progress: bool = True):
        super().__init__()
        self.escape_direction = EscapeDirectionEnum.RIGHT

        self.total_decisions = 0
        self.valid_decisions = 0
        self.escape_left_count = 0
        self.escape_right_count = 0

        self.visualize_progress = visualize_progress
        plt.ion()
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.fig.canvas.set_window_title("Rule-Based Avoidance Algorithm Visualization")
        plt.show(block=False)

    def make_decision(self, lidar_data, speed: float, threshold: int) -> AvoidanceDecision:
        front_indices, right_indices, left_indices = self.calculate_sector_indices()

        def _min_distance(indices):
            return min([lidar_data[i % 360] for i in indices])

        front_min = _min_distance(front_indices)
        right_min = _min_distance(right_indices)
        left_min = _min_distance(left_indices)

        if front_min > threshold:
            self.escape_direction = EscapeDirectionEnum.RIGHT
            return AvoidanceDecision()

        if self.escape_direction == EscapeDirectionEnum.RIGHT:
            if right_min > threshold:
                decision = AvoidanceDecision(vy=speed)
            elif left_min > threshold:
                self.escape_direction = EscapeDirectionEnum.LEFT
                decision = AvoidanceDecision(vy=-speed)
            else:
                decision = AvoidanceDecision(valid=False)
        elif self.escape_direction == EscapeDirectionEnum.LEFT:
            if left_min > threshold:
                decision = AvoidanceDecision(vy=-speed)
            else:
                decision = AvoidanceDecision(valid=False)
 
        self._record_decision(decision)

        if self.visualize_progress:
            def draw_sector(indices, color, label, radius=10):
                angles_deg = np.array(indices) - 180
                angles_rad = np.deg2rad(angles_deg)
                
                sector_angles = np.concatenate(([angles_rad[0]], angles_rad, [angles_rad[-1]]))
                sector_radii = np.concatenate(([0], [radius]*len(angles_rad), [0]))

                self.ax.fill(sector_angles, sector_radii, color=color, alpha=0.3, label=label)

            self.ax.clear()
            self.ax.set_theta_zero_location("N")
            self.ax.set_theta_direction(-1)
            self.ax.set_rlim(0, 10)

            angles_rad = np.deg2rad(np.arange(-180, 180))

            self.ax.plot(angles_rad, lidar_data[::-1], color='blue', label='Lidar Data')

            draw_sector(front_indices, 'blue', 'Front Sector')
            draw_sector(right_indices, 'green', 'Right Sector')
            draw_sector(left_indices, 'red', 'Left Sector')

            if decision.valid:
                yaw_rad = math.atan2(decision.vy, decision.vx)

                decision_radius = 10
                self.ax.plot([yaw_rad, yaw_rad], [0, decision_radius], color='black', linewidth=2, label='Decision Direction')
            else:
                self.ax.set_title("❌ No valid direction", color='red')

            self.ax.legend(loc='upper right')
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

        return decision

    def _record_decision(self, decision: AvoidanceDecision):
        self.total_decisions += 1
        if decision.valid:
            self.valid_decisions += 1
            if self.escape_direction == EscapeDirectionEnum.LEFT:
                self.escape_left_count += 1
            else:
                self.escape_right_count += 1

    def generate_report(self):
        report_content = (
            f"--- RuleBased Avoidance Algorithm Report ---\n"
            f"Total Decisions: {self.total_decisions}\n"
            f"Valid Decisions: {self.valid_decisions}\n"
            f"Escapes to RIGHT: {self.escape_right_count}\n"
            f"Escapes to LEFT: {self.escape_left_count}\n"
        )

        reports_dir = ROOT_DIR / "reports"
        reports_dir.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = reports_dir / f"avoidance_report_{timestamp}.txt"

        with open(filename, "w") as f:
            f.write(report_content)

        print(f"{LogStatusEnum.INFO.value} Report generated: {filename}")

    def calculate_sector_indices(self):
        front_angle = math.ceil(2 * self.sector_width * 1.1)
        half_angle = front_angle // 2

        def get_indices(center):
            start = (center - half_angle) % 360
            end = (center + half_angle) % 360
            if start < end:
                return list(range(start, end + 1))
            else:
                return list(range(start, 360)) + list(range(0, end + 1))

        front_indices = get_indices(self.forward_angle)
        right_indices = get_indices((self.forward_angle - 90) % 360)
        left_indices = get_indices((self.forward_angle + 90) % 360)

        return front_indices, right_indices, left_indices

if __name__ == "__main__":
    rclpy.init()

    speed = 1.0
    threshold = 3.0

    lidar_listener = Lidar2DListener()
    lidar_data = lidar_listener.get_lidar_data()
    sector_width = math.ceil(math.sin(0.35 / threshold) * 180 / math.pi)

    algorithm = RuleBasedAvoidanceAlgorithm()
    algorithm.configure(sector_width, lidar_listener.forward_angle)

    decision = algorithm.make_decision(lidar_data, speed, threshold)

    plt.ioff()
    plt.show()
    