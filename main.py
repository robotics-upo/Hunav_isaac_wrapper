#!/usr/bin/env python3
"""
main.py

Entry point for running the TeleopHuNavSim node.
"""

from teleop_hunav_sim import TeleopHuNavSim
import rclpy
import os


def main():
    rclpy.init()

    ######## Set scenario, agents configuration and robot  ########

    # Available scenarios: "warehouse" | "hospital" | "office" | "empty_world"
    scenario = "warehouse"

    # Available agents configurations:  "agents_warehouse" | "agents_hospital" | "agents_office"
    hunav_config = "agents_warehouse.yaml"

    # Available robots: "jetbot" | "create3" | "carter" | "carter_ROS" (make sure to unzip 'nova_carter_ros2_sensors' located in config/robots)
    robot = "carter_ROS"

    node = TeleopHuNavSim(
        map_name=scenario,
        hunav_config=os.path.join(os.path.dirname(__file__), "config/", hunav_config),
        robot_name=robot,
    )
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
