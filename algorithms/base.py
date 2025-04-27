from abc import ABC, abstractmethod

class AvoidanceDecision:
    def __init__(self, vx=0, vy=0, vz=0):
        self.vx = vx
        self.vy = vy
        self.vz = vz

class BaseAvoidanceAlgorithm(ABC):
    @abstractmethod
    def make_decision(self, lidar_data) -> AvoidanceDecision:
        pass
