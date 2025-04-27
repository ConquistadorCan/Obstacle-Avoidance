from enum import Enum

from algorithms.base import AvoidanceDecision
from algorithms.base import BaseAvoidanceAlgorithm

class EscapeDirectionEnum(Enum):
    LEFT = "left"
    RIGHT = "right"

class RuleBasedAvoidanceAlgorithm(BaseAvoidanceAlgorithm):
    def __init__(self):
        super().__init__()
        self.escape_direction = EscapeDirectionEnum.RIGHT

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
            return AvoidanceDecision()

        if self.escape_direction == EscapeDirectionEnum.RIGHT and right_min > threshold:
            return AvoidanceDecision(vy=speed)
        if self.escape_direction == EscapeDirectionEnum.LEFT and left_min > threshold:
            return AvoidanceDecision(vy=-speed)

        if right_min > threshold:
            return AvoidanceDecision(vy=speed)
        if left_min > threshold:
            self.escape_direction = EscapeDirectionEnum.LEFT
            return AvoidanceDecision(vy=-speed)

        return AvoidanceDecision(valid=False)
