import math
import numpy as np

import rclpy # type: ignore

from ros_nodes.lidar_2d_listener import Lidar2DListener
from algorithms.base import BaseAvoidanceAlgorithm, AvoidanceDecision

class GapFollowingAvoidanceAlgorithm(BaseAvoidanceAlgorithm):
    def __init__(self, drone_width_m: float = 0.7, max_allowed_angle_deg: int = 120):
        super().__init__()
        self.drone_width_m = drone_width_m
        self.target_yaw = 0.0
        self.max_allowed_angle_deg = max_allowed_angle_deg

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
            return AvoidanceDecision(valid=False)

        vx = speed * math.cos(best_angle)
        vy = speed * math.sin(best_angle)

        return AvoidanceDecision(vx=vx, vy=vy, valid=True)

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
        print(f"Decision: {decision.valid}, vx: {decision.vx}, vy: {decision.vy}")

    def _visualize_gap_selection(self, lidar_data, angles_rad, inf_groups, best_angle):
        import matplotlib.cm as cm
        import matplotlib.pyplot as plt

        cmap = cm.get_cmap('tab20', len(inf_groups))
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, polar=True)

        ax.plot(angles_rad, lidar_data, label='Lidar Data', color='blue')

        for idx, group in enumerate(inf_groups):
            color = cmap(idx)
            for i in group:
                ax.plot(angles_rad[i], 10.0, 'o', color=color, markersize=4)

        if best_angle is not None:
            ax.plot(best_angle, 10.0, 'o', color='green', markersize=10, label='Best Direction')

        ax.plot([self.target_yaw, self.target_yaw], [0, 10.0], color='red', linewidth=2, label='Target Yaw')

        ax.set_theta_zero_location("N")
        ax.set_theta_direction(-1)
        ax.set_title("Gap Following – Seçilen Yön", va='bottom')
        ax.legend(loc='upper right')
        plt.show()

    def generate_report(self):
        print("No report generation implemented for GapFollowingAvoidanceAlgorithm.")

if __name__ == "__main__":
    rclpy.init()

    gap_following_algorithm = GapFollowingAvoidanceAlgorithm()

    lidar_listener = Lidar2DListener()

    lidar_data = lidar_listener.get_lidar_data()
    decision = gap_following_algorithm.make_decision(lidar_data, speed=1.0, threshhold=1)
    print(f"Decision: {decision.valid}, vx: {decision.vx}, vy: {decision.vy}")
