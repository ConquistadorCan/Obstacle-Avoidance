import rclpy # type: ignore

from core.flight_mode_enum import FlightModeEnum
from control.drone_controller import DroneController
from algorithms.rule_base import RuleBasedAvoidanceAlgorithm
from algorithms.vector_field import VectorFieldAvoidanceAlgorithm
from algorithms.gap_following import GapFollowingAvoidanceAlgorithm
from control.obstacle_avoidance_service import ObstacleAvoidanceService

rclpy.init()

drone_controller = DroneController()
#rule_based_avoidance_algorithm = RuleBasedAvoidanceAlgorithm()
vector_field_avoidance_algorithm = VectorFieldAvoidanceAlgorithm()
#gap_following_avoidance_algorithm = GapFollowingAvoidanceAlgorithm()
obstacle_avoidance_service = ObstacleAvoidanceService(vector_field_avoidance_algorithm, drone_controller, 3)

try:
    drone_controller.start_auto_mission("mission", 3)
    obstacle_avoidance_service.start_detection_and_avoidance()
except KeyboardInterrupt:
    drone_controller.set_mode(FlightModeEnum.RTL)
except Exception as e:
    print(f"An error occurred: {e}")
    drone_controller.set_mode(FlightModeEnum.RTL)
