import math
from enum import Enum
from datetime import datetime

from config.path_utils import ROOT_DIR
from algorithms.base import AvoidanceDecision
from core.log_status_enum import LogStatusEnum
from algorithms.base import BaseAvoidanceAlgorithm

class EscapeDirectionEnum(Enum):
    LEFT = "left"
    RIGHT = "right"

class RuleBasedAvoidanceAlgorithm(BaseAvoidanceAlgorithm):
    def __init__(self):
        super().__init__()
        self.escape_direction = EscapeDirectionEnum.RIGHT

        self.total_decisions = 0
        self.valid_decisions = 0
        self.escape_left_count = 0
        self.escape_right_count = 0

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
