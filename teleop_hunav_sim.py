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
CONFIG = {
    "width": 1280,
    "height": 720,
    "sync_loads": True,
    "headless": False,
    "renderer": "RaytracedLighting",
}
simulation_app = SimulationApp(CONFIG)

import os
import signal
from rclpy.node import Node
from geometry_msgs.msg import Twist
from isaacsim.core.api import World
from isaacsim.storage.native import get_assets_root_path
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.robot.wheeled_robots.controllers.differential_controller import (
    DifferentialController,
)
import omni
import omni.graph.core as og
 
# Import the WorldBuilder and HuNavManager modules.
from world_builder import WorldBuilder
from hunav_manager import HuNavManager

class TeleopHuNavSim(Node):
    """
    Combines:
    - Differential robot teleop (subscribing to /cmd_vel)
    - USD map loading (via WorldBuilder)
    - Agent management and update (via HuNavManager)
    """

    def __init__(self, map_name, hunav_config, robot_name):
        super().__init__("hunav_sim")
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        # Assets root
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            print("Could not find Nucleus root.")

        # Load USD stage
        self.builder = WorldBuilder(base_path=os.path.dirname(__file__))
        if map_name:
            self.builder.load_map(map_name)

        # Create World object
        timestep = 1.0 / 24.0
        self.world = World(
            stage_units_in_meters=1, physics_dt=timestep, rendering_dt=timestep
        )

        if map_name == "empty_world":
            self.world.scene.add_default_ground_plane()

        # Define configuration for each wheeled robot available
        robot_configs = {
            "jetbot": {
                "name": "Jetbot",
                "usd_relative_path": os.path.join(
                    "Isaac", "Robots", "Jetbot", "jetbot.usd"
                ),
                "wheel_dof_names": ["left_wheel_joint", "right_wheel_joint"],
                "wheel_radius": 0.0325,
                "wheel_base": 0.118,
            },
            "create3": {
                "name": "Create3",
                "usd_relative_path": os.path.join(
                    "Isaac", "Robots", "iRobot", "create_3.usd"
                ),
                "wheel_dof_names": ["left_wheel_joint", "right_wheel_joint"],
                "wheel_radius": 0.03575,
                "wheel_base": 0.233,
            },
            "carter": {
                "name": "Nova_Carter",
                "usd_relative_path": os.path.join(
                    "Isaac", "Robots", "Carter", "nova_carter_sensors.usd"
                ),
                "wheel_dof_names": ["joint_wheel_left", "joint_wheel_right"],
                "wheel_radius": 0.14,
                "wheel_base": 0.413,
            },
            "carter_ROS": {
                "name": "Nova_Carter",
                "usd_relative_path": os.path.join(
                    os.path.dirname(__file__),
                    "config/robots",
                    "nova_carter_ros2_sensors.usd",
                ),
                "wheel_dof_names": ["joint_wheel_left", "joint_wheel_right"],
                "wheel_radius": 0.14,
                "wheel_base": 0.413,
            },
        }

        if robot_name not in robot_configs:
            raise ValueError(f"Unsupported robot_name: {robot_name}")

        robot_config = robot_configs[robot_name]
        robot_path = os.path.join(assets_root_path, robot_config["usd_relative_path"])

        # Add robot to world
        robot_prim_path = f"/World/{robot_config['name']}"
        self.robot = self.world.scene.add(
            WheeledRobot(
                prim_path=robot_prim_path,
                name="Robot",
                wheel_dof_names=robot_config["wheel_dof_names"],
                create_robot=True,
                usd_path=robot_path,
                position=[0.0, 0.0, 0.0],
                orientation=[0, 0, 0, 1],
            )
        )

        # Create differential drive controller
        self.diff_controller = DifferentialController(
            name="diff_drive_controller",
            wheel_radius=robot_config["wheel_radius"],
            wheel_base=robot_config["wheel_base"],
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
            robot_prim_path=robot_prim_path,
            robot=self.robot,
        )

        self.create_ros_clock_action_graph()

        self.hunav.initialize_agents()
        self.hunav.initialize_hunav_nodes()

    def _signal_handler(self, signum, frame):
        print("\n\nCaught shutdown signal, closing app and stopping hunav nodes...\n\n")
        self.hunav.close_hunav_nodes()
        simulation_app.close()

    def _cmd_vel_callback(self, msg):
        self.cmd_lin = msg.linear.x
        self.cmd_ang = msg.angular.z

    def _on_physics_step(self, dt: float):
        """
        Called automatically by PhysX each physics frame.
        """
        self.hunav.send_agents_msg()

    def create_ros_clock_action_graph(self, graph_path="/World/ROS2"):
        try:
            keys = og.Controller.Keys
            graph_params = {
                # Create the necessary nodes.
                keys.CREATE_NODES: [
                    # Node for generating a tick on playback.
                    ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                    # Node for reading the simulation time.
                    (
                        "isaac_read_simulation_time",
                        "isaacsim.core.nodes.IsaacReadSimulationTime",
                    ),
                    # Node to create a ROS2 context.
                    ("ros2_context", "isaacsim.ros2.bridge.ROS2Context"),
                    # Node to publish the clock over ROS2.
                    ("ros2_publish_clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ],
                # Connect outputs to inputs:
                keys.CONNECT: [
                    # Connect context output to the publish clock's context input.
                    (
                        "ros2_context.outputs:context",
                        "ros2_publish_clock.inputs:context",
                    ),
                    # Connect tick output to publish clock's execIn.
                    (
                        "on_playback_tick.outputs:tick",
                        "ros2_publish_clock.inputs:execIn",
                    ),
                    # Connect simulation time output to publish clock's timeStamp.
                    (
                        "isaac_read_simulation_time.outputs:simulationTime",
                        "ros2_publish_clock.inputs:timeStamp",
                    ),
                ],
                keys.SET_VALUES: [
                    # For the simulation time node.
                    ("isaac_read_simulation_time.inputs:resetOnStop", True),
                    ("isaac_read_simulation_time.inputs:swhFrameNumber", 0),
                    # For the ROS2PublishClock node.
                    ("ros2_publish_clock.inputs:nodeNamespace", ""),
                    ("ros2_publish_clock.inputs:qosProfile", ""),
                    ("ros2_publish_clock.inputs:queueSize", 10),
                    ("ros2_publish_clock.inputs:timeStamp", 0.0),
                    ("ros2_publish_clock.inputs:topicName", "clock"),
                    # For the ROS2Context node.
                    ("ros2_context.inputs:useDomainIDEnvVar", True),
                    ("ros2_context.inputs:domain_id", 0),
                ],
            }
            og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                graph_params,
            )
            print(f"Successfully created ROS_Clock action graph at {graph_path}")
        except Exception as e:
            print(f"Error creating ROS_Clock action graph: {e}")

    def run(self):
        self.world.reset()
        self.physx_interface = omni.physx.acquire_physx_interface()
        self.physx_sub = self.physx_interface.subscribe_physics_step_events(
            self._on_physics_step
        )
        self.hunav.send_agents_msg()
        while simulation_app.is_running():
            self.world.step(render=True)
            wheel_action = self.diff_controller.forward([self.cmd_lin, self.cmd_ang])
            self.robot.apply_wheel_actions(wheel_action)
