import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from sensor_msgs.msg import LaserScan # type: ignore

class Lidar2DListener(Node):
    def __init__(self):
        super().__init__("lidar_2d_listener")

        self.lidar_data = None
        self.RANGE_MIN = None
        self.RANGE_MAX = None

        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.subscription = self.create_subscription(
            LaserScan, # type: ignore
            "/scan",
            self.listener_callback,
            qos_profile,
        )

        print("Lidar2DListener initialized and waiting for data...")

    def listener_callback(self, msg: LaserScan):
        self.lidar_data = msg.ranges
        self.RANGE_MIN = msg.range_min
        self.RANGE_MAX = msg.range_max

    def get_lidar_data(self):
        rclpy.spin_once(self)

        if self.lidar_data is None:
            raise Exception("No Lidar data received yet.")
        
        return self.lidar_data

if __name__ == "__main__":
    rclpy.init()
    lidar_2d_listener = Lidar2DListener()
    print(f"Lidar Data: {lidar_2d_listener.get_lidar_data()}")
    rclpy.shutdown()