import math
import numpy as np
from datetime import datetime
import matplotlib.pyplot as plt

import rclpy  # type: ignore

from config.path_utils import ROOT_DIR
from ros_nodes.lidar_2d_listener import Lidar2DListener
from algorithms.base import BaseAvoidanceAlgorithm, AvoidanceDecision

class VectorFieldAvoidanceAlgorithm(BaseAvoidanceAlgorithm):
    def __init__(self,
                 repulsion_gain: float = 10.0,
                 attraction_gain: float = 1.0,
                 visualize_progress: bool = True):
        super().__init__()
        self.repulsion_gain = repulsion_gain
        self.attraction_gain = attraction_gain

        self.total_decisions = 0
        self.vx_list = []
        self.vy_list = []

        self.visualize_progress = visualize_progress
        plt.ion()
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.fig.canvas.set_window_title("Vector Field Avoidance")
        self.fig.show()

    def make_decision(self, lidar_data: list, speed: float, threshold: float) -> AvoidanceDecision:
        lidar_data = lidar_data[::-1]
        angles_deg = np.arange(-180, 180)
        angles_rad = np.deg2rad(angles_deg)

        fx, fy = 0.0, 0.0

        if self.visualize_progress:
            self.ax.clear()
            self.ax.set_theta_zero_location("N")
            self.ax.set_theta_direction(-1)
            self.ax.set_rlim(0, 10)
            self.ax.plot(angles_rad, lidar_data, label="Lidar Data", color="blue")

        for i, d in enumerate(lidar_data):
            theta = angles_rad[i] + np.pi

            if -90 <= angles_deg[i] <= 90:
                if np.isfinite(d) and d <= 10:
                    strength = self.repulsion_gain / (d ** 2)
                    fx += strength * math.cos(theta)
                    fy += strength * math.sin(theta)
                    if self.visualize_progress:
                        self.ax.plot([theta, theta], [0, strength], color="red", alpha=0.5)
            else:
                if np.isfinite(d) and d <= 10:
                    strength = self.repulsion_gain * (1 + (1 / d))
                else:
                    strength = self.attraction_gain

                fx += strength * math.cos(theta)
                fy += strength * math.sin(theta)

                if self.visualize_progress:
                    self.ax.plot([theta, theta], [0, strength], color="green", alpha=0.5)

        magnitude = math.hypot(fx, fy)
        if magnitude < 1e-3:
            return AvoidanceDecision(valid=False)

        vx = speed * (fx / magnitude)
        vy = speed * (fy / magnitude)

        if self.visualize_progress:
            result_theta = math.atan2(fy, fx)
            result_mag = math.hypot(fx, fy)
            self.ax.plot([result_theta, result_theta], [0, result_mag], color="black", linewidth=3, label="Resultant")
            self.ax.set_title("DEBUG: Vector Field Composition")
            self.ax.legend(loc="upper right")
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

        self._record_decision(AvoidanceDecision(vx=vx, vy=vy, valid=True))

        return AvoidanceDecision(vx=vx, vy=vy, valid=True)

    def _record_decision(self, decision: AvoidanceDecision):
        self.vx_list.append(decision.vx)
        self.vy_list.append(decision.vy)
        self.total_decisions += 1
    
    def generate_report(self):
        report_lines = [
            "--- VectorField Avoidance Algorithm Report ---",
            f"Total Decisions: {self.total_decisions}",
        ]

        if self.vx_list:
            vx_array = np.array(self.vx_list)
            vy_array = np.array(self.vy_list)
            angles_rad = np.arctan2(vy_array, vx_array)
            mean_angle = np.arctan2(np.mean(np.sin(angles_rad)), np.mean(np.cos(angles_rad)))
            mean_angle_deg = np.rad2deg(mean_angle)
            report_lines += [
                f"Average vx: {np.mean(vx_array):.2f}",
                f"Average vy: {np.mean(vy_array):.2f}",
                f"Average Direction: {mean_angle:.2f} rad ({mean_angle_deg:.2f} deg)",
                f"Direction Std Dev: {np.std(angles_rad):.2f} rad",
            ]

        report_text = "\n".join(report_lines)

        reports_dir = ROOT_DIR / "reports"
        reports_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_path = reports_dir / f"vectorfield_report_{timestamp}.txt"
        with open(report_path, "w") as f:
            f.write(report_text)

if __name__ == "__main__":
    rclpy.init()

    lidar_listener = Lidar2DListener()
    algorithm = VectorFieldAvoidanceAlgorithm()

    lidar_data = lidar_listener.get_lidar_data()
    decision = algorithm.make_decision(lidar_data, speed=1.0, threshold=3.0)

    print(f"Decision: valid={decision.valid}, vx={decision.vx:.2f}, vy={decision.vy:.2f}")

    plt.ioff()
    plt.show()
