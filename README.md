# Obstacle Avoidance System

This project is a modular software solution for ROS 2-based Unmanned Aerial Vehicles (UAVs/Drones), featuring modern obstacle avoidance algorithms. The system utilizes Lidar sensor data to detect surroundings and perform safe avoidance maneuvers in real-time.

## 🚀 Features

- **Multi-Algorithm Support:** Provides three core algorithms for different operation scenarios:
  - **Vector Field Histograms (VFH):** Advanced path planning that models obstacles as repulsive forces and the target as an attractive force.
  - **Gap Following:** Analyzes sensor data to identify the safest "gaps" or clear paths for passage.
  - **Rule-Based:** Predefined avoidance maneuvers triggered by specific distance thresholds.
- **ROS 2 Based Architecture:** Built using `rclpy` for high-performance, asynchronous communication.
- **Drone Control Center:** Capabilities for autonomous mission initiation, altitude maintenance, and emergency maneuvers (RTL - Return to Launch).
- **Real-Time Monitoring:** Continuous environmental awareness via Lidar feedback and altitude monitoring nodes.

## 📂 Project Structure

```text
├── algorithms/       # Core avoidance logic modules
│   ├── vector_field.py
│   ├── gap_following.py
│   └── rule_base.py
├── control/          # Drone movement management and services
│   ├── drone_controller.py
│   └── obstacle_avoidance_service.py
├── core/             # Data types, Enums, and helper utilities
├── ros_nodes/        # ROS 2 Nodes (Lidar listener, monitors)
├── missions/         # Mission definitions and waypoint plans
└── main.py           # Main entry point to start the system
```

## 🛠️ Installation

Ensure ROS 2 (Humble or later) is installed on your system before running.

1. Install essential dependencies:

   ```bash
   pip install rclpy
   # Ensure MAVROS and relevant ROS 2 packages are installed
   ```

2. Clone the repository:
   ```bash
   git clone https://github.com/ConquistadorCan/Obstacle-Avoidance.git
   cd Obstacle-Avoidance
   ```

## 💻 Usage

To launch the system with the default algorithm (Vector Field):

```bash
python main.py
```

You can switch between algorithms by modifying `main.py`:

```python
# To use a different algorithm:
avoidance_algorithm = GapFollowingAvoidanceAlgorithm()
service = ObstacleAvoidanceService(avoidance_algorithm, drone_controller, distance_threshold=3)
```

## ⚠️ Safety Note

This software enables autonomous flight capabilities. It is **not recommended** to use on real hardware without prior testing in simulation environments (e.g., Gazebo, AirSim). In case of a critical error, the system is designed to automatically switch to **RTL (Return to Launch)** mode.

## 📄 License

This project is for personal development and research purposes.
