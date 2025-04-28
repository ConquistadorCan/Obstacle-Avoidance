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
        front_indices = list(range(170, 191))
        right_indices = list(range(80, 101))
        left_indices = list(range(260, 281))

        def _min_distance(indices):
            return min([lidar_data[i % 360] for i in indices])

        front_min = _min_distance(front_indices)
        left_min = _min_distance(left_indices)
        right_min = _min_distance(right_indices)

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
                print("Moving left")
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
