#!/usr/bin/env python3
"""
teleop_hunav_sim.py

Contains the TeleopHuNavSim class which combines:
- ROS 2 teleoperation for a differential robot.
- World loading via WorldBuilder.
- Agent management via HuNavManager.
"""
from isaacsim import SimulationApp

# Start Isaac Sim
simulation_app = SimulationApp({"headless": False})

import sys
import os
import signal
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from omni.isaac.core import World
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.wheeled_robots.controllers.differential_controller import (
    DifferentialController,
)
# Import the WorldBuilder and HuNavManager modules.
from world_builder import WorldBuilder
from hunav_manager import HuNavManager

import omni


class TeleopHuNavSim(Node):
    """
    Combines:
    - Differential robot teleop (subscribing to /cmd_vel)
    - USD map loading (via WorldBuilder)
    - Agent management and update (via HuNavManager)
    """

    def __init__(self, map_name, hunav_config):
        super().__init__("hunav_sim")
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        # Assets root
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            print("Could not find Nucleus root.")

        self.first_run_flag = True
        self.builder = WorldBuilder(base_path=os.path.dirname(__file__))
        if map_name:
            self.builder.load_map(map_name)

        # Create a World
        timestep = 1.0 / 60.0
        self.world = World(
            stage_units_in_meters=1.0, physics_dt=timestep, rendering_dt=timestep
        )
        self.world.scene.add_default_ground_plane()

        # Robot (Jetbot) set up
        robot_path = os.path.join(
            assets_root_path, "Isaac", "Robots", "Jetbot", "jetbot.usd"
        )
        self.robot = self.world.scene.add(
            WheeledRobot(
                prim_path="/World/Robot",
                name="Robot",
                wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
                create_robot=True,
                usd_path=robot_path,
                position=[0.0, 0.0, 0.0],
            )
        )

        # Reset World to propagate physics
        self.world.reset()

        # Create a differential controller (wheel_radius and wheel_base are specific to the robot used)
        self.diff_controller = DifferentialController(
            name="diff_drive_controller", wheel_radius=0.03, wheel_base=0.1125
        )

        # ROS2 cmd_vel subscriber
        self.cmd_lin = 0.00
        self.cmd_ang = 0.00
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self._cmd_vel_callback, 10
        )

        # Setup HuNavManager
        self.hunav = HuNavManager(
            node=self,
            world=self.world,
            config_file_path=hunav_config,
            robot_prim_path="/World/Robot",
            robot=self.robot,
        )
        self.hunav.initialize_agents()
        self.hunav.initialize_hunav_nodes()

        self.robot_init_pose = ([0.0, 0.0, 0.0], [0, 0, 0, 1])
        self.physx_interface = omni.physx.acquire_physx_interface()
        self.physx_sub = self.physx_interface.subscribe_physics_step_events(
            self._on_physics_step
        )

    def _signal_handler(self, signum, frame):
        print("\n\nCaught shutdown signal, closing app and stopping hunav nodes...\n\n")
        self.hunav.close_hunav_nodes()
        simulation_app.close()
        sys.exit(0)

    def _cmd_vel_callback(self, msg):
        self.cmd_lin = msg.linear.x
        self.cmd_ang = msg.angular.z

    def _on_physics_step(self, dt: float):
        """
        Called automatically by PhysX each physics frame.
        """
        if self.world.is_playing():
            # Update hunav agents
            self.hunav.send_agents_msg()
            # Apply differential controller command to the robot
            wheel_action = self.diff_controller.forward([self.cmd_lin, self.cmd_ang])
            self.robot.apply_action(wheel_action)

    def run(self):
        while simulation_app.is_running():
            simulation_app.update()
        # Cleanup
        self.hunav.close_hunav_nodes()
        simulation_app.close()
