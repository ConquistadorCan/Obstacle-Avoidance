from abc import ABC, abstractmethod

class AvoidanceDecision:
    def __init__(self, vx=0, vy=0, vz=0, yaw=0, valid=True):
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.yaw = yaw
        self.valid = True


class BaseAvoidanceAlgorithm(ABC):
    @abstractmethod
    def make_decision(self, lidar_data, speed: float, threshold: int) -> AvoidanceDecision:
        pass
