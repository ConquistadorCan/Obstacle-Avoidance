import math
import numpy as np
from datetime import datetime
import matplotlib.pyplot as plt

import rclpy # type: ignore

from config.path_utils import ROOT_DIR
from ros_nodes.lidar_2d_listener import Lidar2DListener
from algorithms.base import BaseAvoidanceAlgorithm, AvoidanceDecision

class GapFollowingAvoidanceAlgorithm(BaseAvoidanceAlgorithm):
    def __init__(self, drone_width_m: float = 0.7, max_allowed_angle_deg: int = 120, visualize_progress: bool = True):
        super().__init__()
        self.drone_width_m = drone_width_m
        self.target_yaw = 0.0
        self.max_allowed_angle_deg = max_allowed_angle_deg

        self.total_decisions = 0
        self.angle_diffs = []
        self.decisions = []

        self.visualize_progress = visualize_progress
        plt.ion()
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.fig.canvas.set_window_title("Gap Following Avoidance Algorithm Visualization")
        self.fig.show()

    def make_decision(self, lidar_data: list, speed: float, threshhold: float) -> AvoidanceDecision:
        angles_rad = np.deg2rad(np.arange(-180, 180))
        lidar_data = lidar_data[::-1]

        total_len = len(lidar_data)
        inf_indices = [i for i, d in enumerate(lidar_data) if not np.isfinite(d)]

        inf_groups = []
        current = []

        for idx in inf_indices:
            if not current or idx == (current[-1] + 1) % total_len:
                current.append(idx)
            else:
                inf_groups.append(current)
                current = [idx]
        if current:
            inf_groups.append(current)

        if inf_groups and inf_groups[0][0] == 0 and inf_groups[-1][-1] == total_len - 1:
            merged = inf_groups[-1] + inf_groups[0]
            inf_groups = [merged] + inf_groups[1:-1]

        valid_inf_groups = []
        for group in inf_groups:
            left_idx = group[0] - 1 if group[0] > 0 else None
            right_idx = group[-1] + 1 if group[-1] < total_len - 1 else None

            d1 = lidar_data[left_idx] if left_idx is not None and np.isfinite(lidar_data[left_idx]) else np.inf
            d2 = lidar_data[right_idx] if right_idx is not None and np.isfinite(lidar_data[right_idx]) else np.inf
            min_dist = min(d1, d2)

            if not np.isfinite(min_dist):
                continue

            required_width = self._compute_min_gap_width_indices(threshhold=min_dist)

            if len(group) >= required_width:
                valid_inf_groups.append(group)

        best_angle = None
        min_angle_diff = float('inf')

        for group in valid_inf_groups:
            for i in range(0, len(group) - required_width + 1):
                subgap = group[i:i + required_width]
                if not self._is_contiguous(subgap, total_points=total_len):
                    continue

                subgap_angles = [angles_rad[j] for j in subgap]
                center_angle = np.arctan2(
                    np.mean(np.sin(subgap_angles)),
                    np.mean(np.cos(subgap_angles))
                )

                angle_diff = abs(np.arctan2(np.sin(center_angle - self.target_yaw), np.cos(center_angle - self.target_yaw)))
                angle_diff_deg = np.rad2deg(angle_diff)
                if angle_diff_deg > self.max_allowed_angle_deg:
                    continue

                if angle_diff < min_angle_diff:
                    min_angle_diff = angle_diff
                    best_angle = center_angle

        if best_angle is None:
            decision = AvoidanceDecision(valid=False)
            self._record_decision(decision)
            return decision

        vx = speed * math.cos(best_angle)
        vy = speed * math.sin(best_angle)

        if self.visualize_progress:
            cmap = plt.cm.get_cmap('tab20', len(inf_groups))
            self.ax.clear()
            self.ax.set_theta_zero_location("N")
            self.ax.set_theta_direction(-1)
            self.ax.set_rlim(0, 10)

            angles_rad = np.deg2rad(np.arange(-180, 180))

            for idx, group in enumerate(inf_groups):
                color = cmap(idx)
                for i in group:
                    angle = angles_rad[i]
                    self.ax.plot(angle, 10.0, 'o', color=color, markersize=5)

            self.ax.plot([best_angle, best_angle], [0, 10.0], color='black', linewidth=2, label='Decision Direction')

            self.ax.plot([self.target_yaw, self.target_yaw], [0, 10.0], color='red', linestyle='--', label='Target Yaw')

            self.ax.set_title("Gap Following – Real-time", va='bottom')
            self.ax.legend(loc='upper right')
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

        angle_diff_deg = np.rad2deg(min_angle_diff)
        self.angle_diffs.append(angle_diff_deg)

        decision = AvoidanceDecision(vx=vx, vy=vy, valid=True)
        self._record_decision(decision)
        return decision

    def _compute_min_gap_width_indices(self, threshhold: float = 5):
        diagonal = self.drone_width_m * math.sqrt(2)
        radius = diagonal / 2
        angle_rad = 2 * np.arcsin(radius / threshhold)
        angle_deg = np.rad2deg(angle_rad)
        return int(np.ceil(angle_deg))

    def _is_contiguous(self, subgap, total_points=360):
        for a, b in zip(subgap, subgap[1:]):
            if (b - a) % total_points != 1:
                return False
        return True

    def _record_decision(self, decision):
        self.total_decisions += 1
        self.decisions.append(decision)

    def generate_report(self):
        report_content = (
            f"--- GapFollowing Avoidance Algorithm Report ---\n"
            f"Total Decisions: {self.total_decisions}\n"
        )

        if self.angle_diffs:
            avg_diff = np.mean(self.angle_diffs)
            max_diff = np.max(self.angle_diffs)
            report_content += f"Average Yaw Deviation: {avg_diff:.2f} degrees\n"
            report_content += f"Max Yaw Deviation: {max_diff:.2f} degrees\n"

        reports_dir = ROOT_DIR / "reports"
        reports_dir.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = reports_dir / f"gap_following_report_{timestamp}.txt"

        with open(filename, "w") as f:
            f.write(report_content)

if __name__ == "__main__":
    rclpy.init()

    gap_following_algorithm = GapFollowingAvoidanceAlgorithm()

    lidar_listener = Lidar2DListener()

    lidar_data = lidar_listener.get_lidar_data()
    decision = gap_following_algorithm.make_decision(lidar_data, speed=1.0, threshhold=1)

    plt.ioff()
    plt.show()
