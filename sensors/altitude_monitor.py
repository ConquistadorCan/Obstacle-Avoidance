import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from geometry_msgs.msg import PoseStamped # type: ignore
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy # type: ignore

class AltitudeMonitor(Node):
    def __init__(self):
        super().__init__("altitude_monitor")

        self.current_altitude = None

        qos_policy = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            PoseStamped,
            "/ap/pose/filtered",
            self.altitude_callback,
            qos_policy
        )

    def altitude_callback(self, msg):
        self.current_altitude = round(msg.pose.position.z, 2)

    def get_current_altitude(self):
        rclpy.spin_once(self)

        if self.current_altitude is None:
            raise Exception("No altitude data received yet.")
        
        return self.current_altitude
        

if __name__ == "__main__":
    rclpy.init()
    altitude_monitor = AltitudeMonitor()
    print(f"Current Altitude: {altitude_monitor.get_current_altitude()}")
    rclpy.shutdown()