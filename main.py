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
    
    # Adjust the configuration path and scenario as needed.
    hunav_config_path = os.path.join(os.path.dirname(__file__), "config/agents_2.yaml")
    
    map_name = "empty_world"
    
    node = TeleopHuNavSim(map_name=map_name, hunav_config=hunav_config_path)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


