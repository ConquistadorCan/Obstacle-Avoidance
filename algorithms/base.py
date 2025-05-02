from abc import ABC, abstractmethod

class AvoidanceDecision:
    def __init__(self, vx=0, vy=0, vz=0, yaw=0, valid=True):
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.yaw = yaw
        self.valid = valid


class BaseAvoidanceAlgorithm(ABC):
    def __init__(self):
        self.sector_width = None
        self.forward_angle = None

    @abstractmethod
    def make_decision(self, lidar_data, speed: float, threshold: int) -> AvoidanceDecision:
        raise NotImplementedError("This method should be overridden in subclasses.")

    @abstractmethod
    def generate_report(self):
        raise NotImplementedError("This method should be overridden in subclasses.")

    @abstractmethod
    def _record_decision(self, decision: AvoidanceDecision):
        raise NotImplementedError("This method should be overridden in subclasses.")
    
    def configure(self, sector_width: int, forward_angle: int):
        self.sector_width = sector_width
        self.forward_angle = forward_angle
