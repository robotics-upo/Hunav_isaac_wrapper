#!/usr/bin/env python3
"""
hunav_manager.py

Contains the HuNavManager class for spawning and managing Hunav agents,
setting up animations (including retargeting), and handling physics.
"""

import os
import math
import random
import yaml
import numpy as np
import subprocess, signal
from typing import Tuple, List, Optional

# ROS messages
import rclpy
from geometry_msgs.msg import Quaternion, Pose, Point
from hunav_msgs.srv import ComputeAgents
from hunav_msgs.msg import Agent, Agents, AgentBehavior
from std_msgs.msg import Header

# Isaac Sim imports
import omni
import omni.kit.commands
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.utils.extensions import enable_extension

from pxr import Sdf, Gf, UsdGeom, UsdPhysics, PhysxSchema
import carb

# Import auxiliary animation functions
from .animation_utils import *

enable_extension("omni.anim.retarget.core")

import omni.anim.graph.core as ag


class HuNavManager:
    """
    Manages HunavSim agents by reading configuration from a YAML file,
    spawning agents as SkelRoot prims, setting up animations (and retargeting),
    and handling ROS 2 communications.
    """

    def __init__(self, node, world, config_file_path, robot_prim_path, robot):
        self.node = node
        self.stage = world.stage
        self.robot_prim_path = robot_prim_path
        self.robot_obj = robot
        self.world = world
        self.config_file_path = config_file_path

        self.assets_root = get_assets_root_path()
        self._usd_context = omni.usd.get_context()

        # HuNavSim's ROS 2 service client
        self.compute_agents_client = self.node.create_client(
            ComputeAgents, "/compute_agents"
        )

        # List of target model assets
        character_root_path = os.path.join(self.assets_root, "Isaac/People/Characters/")

        character_models = [
            "F_Business_02/F_Business_02.usd",
            "F_Medical_01/F_Medical_01.usd",
            "M_Medical_01/M_Medical_01.usd",
            "male_adult_construction_01_new/male_adult_construction_01_new.usd",
            "male_adult_construction_05_new/male_adult_construction_05_new.usd",
            "female_adult_police_01_new/female_adult_police_01_new.usd",
            "female_adult_police_02/female_adult_police_02.usd",
            "female_adult_police_03_new/female_adult_police_03_new.usd",
            "male_adult_police_04/male_adult_police_04.usd",
            "original_female_adult_business_02/female_adult_business_02.usd",
            "original_female_adult_medical_01/female_adult_medical_01.usd",
        ]

        self.target_model_paths = [
            f"{character_root_path}{model}" for model in character_models
        ]
        
        # Mapping from skin ID to character model index
        # This allows agents to specify which character model to use via the 'skin' field
        # in their configuration. Valid options:
        #   0-10: Specific character models (see mapping below)
        #   "random": Random character model selection
        self.skin_to_model_mapping = {
            1: 0,   # F_Business_02
            2: 1,   # F_Medical_01
            3: 2,   # M_Medical_01
            4: 3,   # male_adult_construction_01_new
            5: 4,   # male_adult_construction_05_new
            6: 5,   # female_adult_police_01_new
            7: 6,   # female_adult_police_02
            8: 7,   # female_adult_police_03_new
            9: 8,   # male_adult_police_04
            10: 9,  # original_female_adult_business_02
            11: 10, # original_female_adult_medical_01
        }

        # Data holders
        self.agents = []
        self.agent_initial_states = []
        self.animationDict = {}
        self._hunav_processes = []
        self.retarget_flag = False
        self.bound_animations = {}
        self.flag_anim = {}
        
        # Orientation smoothing
        self.agent_previous_orientations = {}  # Store previous orientations for smoothing
        self.orientation_smoothing_factor = 0.15  # Lower = smoother but more lag (0.05-0.3 range)

        self.robot_prim = None

        if config_file_path is not None:
            self.config = self._load_yaml(config_file_path)
        else:
            self.config = None

        # Define the default character source asset (for animation retargeting)
        self.default_biped_usd = os.path.join(
            self.assets_root, "Isaac/People/Characters/Biped_Setup.usd"
        )

    def _load_yaml(self, relative_path):
        full_path = os.path.join(os.path.dirname(__file__), relative_path)
        with open(full_path, "r") as file:
            return yaml.safe_load(file)

    def slerp_quaternions(self, q1: Gf.Quatf, q2: Gf.Quatf, t: float) -> Gf.Quatf:
        """
        Spherical linear interpolation between two quaternions for smooth rotation.
        
        Args:
            q1: Starting quaternion
            q2: Target quaternion  
            t: Interpolation factor (0.0 = q1, 1.0 = q2)
            
        Returns:
            Interpolated quaternion
        """
        # Ensure we take the shortest path by checking dot product
        dot = q1.GetReal() * q2.GetReal() + sum(a * b for a, b in zip(q1.GetImaginary(), q2.GetImaginary()))
        
        # If dot product is negative, negate one quaternion to take shorter path
        if dot < 0.0:
            q2 = Gf.Quatf(-q2.GetReal(), -q2.GetImaginary()[0], -q2.GetImaginary()[1], -q2.GetImaginary()[2])
            dot = -dot
        
        # If quaternions are very close, use linear interpolation to avoid division by zero
        if dot > 0.9995:
            result_real = q1.GetReal() + t * (q2.GetReal() - q1.GetReal())
            result_imag = [
                q1.GetImaginary()[i] + t * (q2.GetImaginary()[i] - q1.GetImaginary()[i])
                for i in range(3)
            ]
            result = Gf.Quatf(result_real, result_imag[0], result_imag[1], result_imag[2])
            return result.GetNormalized()
        
        # Calculate spherical interpolation
        theta_0 = math.acos(abs(dot))
        sin_theta_0 = math.sin(theta_0)
        theta = theta_0 * t
        sin_theta = math.sin(theta)
        
        s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0
        
        result_real = s0 * q1.GetReal() + s1 * q2.GetReal()
        result_imag = [
            s0 * q1.GetImaginary()[i] + s1 * q2.GetImaginary()[i]
            for i in range(3)
        ]
        
        return Gf.Quatf(result_real, result_imag[0], result_imag[1], result_imag[2]).GetNormalized()

    def normalize_angle(self, a: float) -> float:
        value = a
        while value <= -math.pi:
            value += 2 * math.pi
        while value > math.pi:
            value -= 2 * math.pi
        return value

    def initialize_hunav_nodes(self):
        process_1 = subprocess.Popen(
            [
                "ros2",
                "run",
                "hunav_agent_manager",
                "hunav_loader",
                "--ros-args",
                "--params-file",
                self.config_file_path,
            ],
            preexec_fn=os.setsid,
        )
        process_2 = subprocess.Popen(
            [
                "ros2",
                "run",
                "hunav_agent_manager",
                "hunav_agent_manager",
                "--ros-args",
                "--params-file",
                self.config_file_path,
            ],
            preexec_fn=os.setsid,
        )
        process_3 = subprocess.Popen(
            ["ros2", "run", "hunav_evaluator", "hunav_evaluator_node"],
            preexec_fn=os.setsid,
        )

        self._hunav_processes = [process_1, process_2, process_3]

    def close_hunav_nodes(self):
        for process in self._hunav_processes:
            print(process)
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        self._hunav_processes = []

    def initialize_agents(self):
        """
        Reads the YAML configuration for agents, spawns them, and sets up their animations
        using a two-level hierarchy. For each agent, the outer container prim (type "Xform") is updated by
        compute_agents (global transform), while the inner child prim (type "SkelRoot") is
        animated via an AnimationGraph created beforehand.
        """
        if self.config is None:
            print("[HuNavManager] No config loaded, skipping agent creation.")
            return

        agent_configs = self.config["hunav_loader"]["ros__parameters"]["agents"]

        # Create animations using the auxiliary module functions
        animations_path = os.path.join(
            self.assets_root, "Isaac", "People", "Animations"
        )
        walk_anim = create_animation(
            self.stage,
            "/World/Animations/WalkLoop",
            os.path.join(animations_path, "stand_walk_loop_in_place.skelanim.usd"),
        )
        idle_anim = create_animation(
            self.stage,
            "/World/Animations/IdleLoop",
            os.path.join(animations_path, "stand_idle_loop.skelanim.usd"),
        )
        self.source_animation_dict = {0: idle_anim.GetPath(), 1: walk_anim.GetPath()}
        self.target_animation_parent_path = "/World/Characters"
        self.retarget_anims_path = [
            os.path.join(self.target_animation_parent_path, "IdleLoop"),
            os.path.join(self.target_animation_parent_path, "WalkLoop"),
        ]
        self.animationDict = {
            0: self.retarget_anims_path[0],
            1: self.retarget_anims_path[1],
        }

        # Set up a rotation for upright orientation
        rotX = Gf.Rotation(Gf.Vec3d(1, 0, 0), 90).GetQuat()
        rotXQ = Gf.Quatf(rotX)

        # Spawn the default biped for skeletal binding
        init_pos_src = Gf.Vec3d(0, 0, 0)
        init_rot_src = Gf.Quatf(1, 0, 0, 0) * rotXQ
        source_prim_path = "/World/Biped_Setup"
        source_agent_prim = self.stage.DefinePrim(source_prim_path, "SkelRoot")
        source_agent_prim.GetReferences().AddReference(self.default_biped_usd)
        xform_src = UsdGeom.Xformable(source_agent_prim)
        found_translate = False
        for op in xform_src.GetOrderedXformOps():
            if op.GetOpName() == "xformOp:translate":
                op.Set(init_pos_src)
                found_translate = True
                break
        if not found_translate:
            xform_src.AddTranslateOp().Set(init_pos_src)
        xform_src.AddOrientOp().Set(init_rot_src)
        source_agent_prim.GetAttribute("visibility").Set("invisible")
        source_agent_prim.CreateAttribute(
            "physxRigidBody:disableGravity", Sdf.ValueTypeNames.Bool
        ).Set(True)
        source_agent_prim.CreateAttribute(
            "physxContact:collisionEnabled", Sdf.ValueTypeNames.Bool
        ).Set(False)

        asset_cycle = self.target_model_paths.copy()
        random.shuffle(asset_cycle)

        # For each agent defined in the agents_x.yaml:
        for agent_name in agent_configs:
            agent_cfg = self.config["hunav_loader"]["ros__parameters"][agent_name]

            # Use skin value to select specific character model
            skin_value = agent_cfg["skin"]
            asset_path = self.get_character_model_from_skin(skin_value)
            
            if asset_path is None:
                # Invalid skin value, fall back to round-robin selection
                self.node.get_logger().warn(
                    f"Invalid skin value '{skin_value}' for agent {agent_name}, "
                    f"falling back to random selection. Valid skin values are 0 (random) or 1-{len(self.target_model_paths)}"
                )
                if len(asset_cycle) == 0:
                    asset_cycle = self.target_model_paths.copy()
                    random.shuffle(asset_cycle)
                asset_path = asset_cycle.pop()
            else:
                if skin_value == 0:
                    self.node.get_logger().info(
                        f"Agent {agent_name} using random skin: {asset_path.split('/')[-2]}"
                    )
                else:
                    self.node.get_logger().info(
                        f"Agent {agent_name} using skin {skin_value}: {asset_path.split('/')[-2]}"
                    )
            
            init_pose = agent_cfg["init_pose"]
            # translation
            global_pos = Gf.Vec3d(init_pose["x"], init_pose["y"], init_pose["z"])

            h_rad = init_pose.get("h", 0.0)
            h_deg = h_rad * 180.0 / math.pi

            # X-tilt rotation (90°)
            rotX = Gf.Rotation(Gf.Vec3d(1, 0, 0), 90.0)
            qdX = rotX.GetQuat()  # this is a Gf.Quatd
            # convert to single‐precision quaternion:
            rotXQ = Gf.Quatf(qdX.GetReal(), Gf.Vec3f(qdX.GetImaginary()))

            # Z-heading rotation
            rotZ = Gf.Rotation(Gf.Vec3d(0, 0, 1), h_deg)
            qdZ = rotZ.GetQuat()
            rotZQ = Gf.Quatf(qdZ.GetReal(), Gf.Vec3f(qdZ.GetImaginary()))

            # combine: yaw then tilt
            global_rot = rotZQ * rotXQ

            # now apply to container:
            container_path = f"/World/Characters/{agent_name}"
            container = self.stage.DefinePrim(container_path, "Xform")
            xf = UsdGeom.Xformable(container)
            xf.AddTranslateOp().Set(global_pos)
            xf.AddOrientOp().Set(global_rot)
            
            PhysxSchema.PhysxRigidBodyAPI.Apply(container)
            UsdPhysics.RigidBodyAPI.Apply(container)

            # Create the inner animated SkelRoot as a child of the container
            anim_path = container_path + "/Animation"
            agent_skelroot = self.stage.DefinePrim(anim_path, "SkelRoot")
            agent_skelroot.GetReferences().AddReference(asset_path)

            # Apply retargeting and create the AnimationGraph on the inner SkelRoot
            if not self.retarget_flag:
                setup_anim_retargeting(
                    self.stage,
                    agent_skelroot,
                    self.source_animation_dict,
                    self.target_animation_parent_path,
                )
                self.retarget_flag = True

            # Create and apply AnimationGraph
            anim_graph_path = create_agent_animation_graph(
                self.stage,
                agent_skelroot,
                idle_anim_path=self.animationDict.get(0),
                walk_anim_path=self.animationDict.get(1),
            )
            apply_animation_graph(
                self.stage.GetPrimAtPath(find_skelroot_path(agent_skelroot)),
                anim_graph_path,
            )
            self.bound_animations[agent_skelroot.GetPath()] = anim_graph_path

            self.flag_anim[agent_skelroot.GetPath()] = False

            # Add the container (global transform) to the agents list
            self.agents.append(container)
            self.agent_initial_states.append(
                {"position": global_pos, "orientation": global_rot}
            )

        # Set up the robot prim
        if self.robot_prim_path:
            rp = self.stage.GetPrimAtPath(self.robot_prim_path)
            if rp.IsValid():
                self.robot_prim = rp
            else:
                print(
                    f"[HuNavManager] Warning: no valid robot prim at {self.robot_prim_path}"
                )
        else:
            print("[HuNavManager] no robot_prim_path provided")

    def reset_agent_states(self):
        for agent, init_state in zip(self.agents, self.agent_initial_states):
            agent.GetAttribute("xformOp:translate").Set(init_state["position"])
            agent.GetAttribute("xformOp:orient").Set(init_state["orientation"])
        
        self.agent_previous_orientations.clear()
        print("[HuNavManager] agent states reset.")

    def clear_simulation(self):
        self.close_hunav_nodes()
        stage = self._usd_context.get_stage()
        world_prim = stage.GetPrimAtPath("/World")
        for prim in world_prim.GetChildren():
            stage.RemovePrim(prim.GetPath())
        self.agents.clear()
        self.robot = None
        self.agent_initial_states.clear()
        self.animationDict.clear()
        self.agent_previous_orientations.clear()  # Clean up orientation tracking

    # Obstacle detection functions
    def generate_lasers(self, num_lasers: int) -> List[Gf.Vec3f]:
        """
        Generate ray directions evenly distributed over 360° in the global frame.
        """
        directions = []
        angle_increment = 360.0 / num_lasers
        for i in range(num_lasers):
            angle_rad = math.radians(i * angle_increment)
            x = math.cos(angle_rad)
            y = math.sin(angle_rad)
            directions.append(Gf.Vec3f(x, y, 0.0))
        return directions

    def get_closest_obstacles(
        self,
        agent_position: Gf.Vec3d,
        max_distance: float,
        sensor_offsets: List[float],
        num_lasers: int = 90,
    ) -> List[Tuple[float, Optional[Gf.Vec3f]]]:
        """
        Cast rays from the agent's sensor origins at multiple heights in fixed directions.
        For each ray direction, iterate over the provided sensor_offsets and select the hit
        with the smallest distance (if any).
        """
        directions = self.generate_lasers(num_lasers)
        scene_query = omni.physx.get_physx_scene_query_interface()
        closest_hits = []

        # Iterate over each ray direction
        for direction in directions:
            best_distance = max_distance
            best_hit = None
            hit_found = False
            # Test each sensor offset
            for offset in sensor_offsets:
                sensor_origin = Gf.Vec3f(
                    float(agent_position[0]),
                    float(agent_position[1]),
                    float(agent_position[2]) + offset,
                )
                hit = scene_query.raycast_closest(
                    sensor_origin, direction, max_distance
                )
                if hit.get("hit", False):
                    hit_found = True
                    distance = hit.get("distance", max_distance)
                    # Keep the closest hit
                    if distance < best_distance:
                        best_distance = distance
                        best_hit = hit.get("position")
            # If at least one hit was found, append the best hit; else, use defaults
            if hit_found:
                closest_hits.append((best_distance, best_hit))
            else:
                closest_hits.append((max_distance, None))

        return closest_hits

    def euler_from_quaternion(
        self, x: float, y: float, z: float, w: float
    ) -> Tuple[float, float, float]:
        """
        Converts a quaternion (with w as the scalar part) to Euler roll, pitch, yaw.
        quaternion = [x, y, z, w]
        """
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def send_agents_msg(self):
        """
        Called every physics step to call /compute_agents and update agent transforms.
        """
        if self.robot_prim is None:
            print("[HuNavManager] No robot assigned.")
            return

        # Build Agents message
        agents_msg = Agents()
        agents_msg.header = Header()
        now = self.node.get_clock().now().to_msg()
        agents_msg.header.stamp.sec = now.sec
        agents_msg.header.stamp.nanosec = now.nanosec
        agents_msg.header.frame_id = "world"

        # Build robot message
        robot_msg = self._create_robot_msg()

        # For each agent, create and add an Agent message
        for idx, agent_prim in enumerate(self.agents):
            agent_msg = self._create_agent_msg(agent_prim, idx)
            agents_msg.agents.append(agent_msg)

        # Wait for the compute_agents service and call it
        if not self.compute_agents_client.wait_for_service(timeout_sec=2.0):
            print("[HuNavManager] /compute_agents not available.")
            return
        self._call_compute(agents_msg, robot_msg)

    def _create_robot_msg(self):
        # Retrieve robot pose and velocities from the WheeledRobot object
        pos, quat = self.robot_obj.get_world_pose()
        lin_vel = self.robot_obj.get_linear_velocity()
        ang_vel = self.robot_obj.get_angular_velocity()

        robot = Agent()
        robot.id = 0
        robot.type = Agent.ROBOT
        robot.skin = 1
        robot.name = "Robot"
        robot.group_id = 0
        robot.radius = 0.5
        robot.desired_velocity = 1.0
        robot.linear_vel = np.sqrt(lin_vel[0] ** 2 + lin_vel[1] ** 2 + lin_vel[2] ** 2)
        robot.angular_vel = np.sqrt(ang_vel[0] ** 2 + ang_vel[1] ** 2 + ang_vel[2] ** 2)

        # Pose
        robot.position.position.x = float(pos[0])
        robot.position.position.y = float(pos[1])
        robot.position.position.z = float(pos[2])
        robot.position.orientation.w = float(quat[0])
        robot.position.orientation.x = float(quat[1])
        robot.position.orientation.y = float(quat[2])
        robot.position.orientation.z = float(quat[3])
        _, _, yaw = self.euler_from_quaternion(quat[1], quat[2], quat[3], quat[0])
        robot.yaw = yaw

        # Velocities
        robot.velocity.linear.x = float(lin_vel[0])
        robot.velocity.linear.y = float(lin_vel[1])
        robot.velocity.linear.z = float(lin_vel[2])
        robot.velocity.angular.x = float(ang_vel[0])
        robot.velocity.angular.y = float(ang_vel[1])
        robot.velocity.angular.z = float(ang_vel[2])
        robot.cyclic_goals = True
        robot.goal_radius = 0.5
        robot.closest_obs = []
        return robot

    def _create_agent_msg(self, agent_prim, index):
        agent_ref = self.config["hunav_loader"]["ros__parameters"]["agents"][index]
        agent_cfg = self.config["hunav_loader"]["ros__parameters"][agent_ref]

        agent = Agent()
        agent.id = int(agent_cfg["id"])
        agent.type = Agent.PERSON
        if self.config["hunav_loader"]["ros__parameters"]["simulator"] == "Gazebo":
            agent.skin = agent_cfg["skin"]
        else:
            agent.skin = 0  
        agent.name = f"Agent{index + 1}"
        agent.group_id = int(agent_cfg["group_id"])
        agent.radius = float(agent_cfg["radius"])
        agent.desired_velocity = float(agent_cfg["max_vel"])

        # Read transforms
        pos = agent_prim.GetAttribute("xformOp:translate").Get()
        rot = agent_prim.GetAttribute("xformOp:orient").Get()
        rw = rot.GetReal()
        rx, ry, rz = rot.GetImaginary()

        # Position
        agent.position.position.x = float(pos[0])
        agent.position.position.y = float(pos[1])
        agent.position.position.z = float(pos[2])
        agent.position.orientation.x = float(rx)
        agent.position.orientation.y = float(ry)
        agent.position.orientation.z = float(rz)
        agent.position.orientation.w = float(rw)
        _, _, yaw = self.euler_from_quaternion(
            float(rx), float(ry), float(rz), float(rw)
        )
        agent.yaw = self.normalize_angle(yaw - math.pi / 2.0)

        # Velocities
        lin = agent_prim.GetAttribute("physics:velocity").Get()
        ang = agent_prim.GetAttribute("physics:angularVelocity").Get()
        agent.linear_vel = float(np.linalg.norm(lin))
        agent.angular_vel = float(np.linalg.norm(ang))
        agent.velocity.linear.x = float(lin[0])
        agent.velocity.linear.y = float(lin[1])
        agent.velocity.linear.z = float(lin[2])
        agent.velocity.angular.x = float(ang[0])
        agent.velocity.angular.y = float(ang[1])
        agent.velocity.angular.z = float(ang[2])

        # Goals
        agent.cyclic_goals = agent_cfg["cyclic_goals"]
        agent.goal_radius = float(agent_cfg["goal_radius"])
        
        # Behavior
        beh = agent_cfg["behavior"]
        configuration = int(beh["configuration"])
        
        # SFM parameter configuration constants
        DEFAULT_SFM_PARAMS = {
            "goal_force_factor": 10.0,
            "obstacle_force_factor": 2.0,
            "social_force_factor": 5.0
        }
        
        SFM_CONSTRAINTS = {
            "goal_force_factor": (5.0, 10.0),
            "obstacle_force_factor": (0.5, 5.0),
            "social_force_factor": (5.0, 20.0)
        }
        
        def clamp_value(value: float, min_val: float, max_val: float) -> float:
            """Constrain value within specified range."""
            return max(min_val, min(value, max_val))
        
        # Set SFM parameters based on configuration type
        if configuration == 0:  # Default Isaac Sim configuration
            sfm_params = DEFAULT_SFM_PARAMS.copy()
            sfm_params["other_force_factor"] = float(beh["other_force_factor"])
        elif configuration == 1:  # Custom unconstrained configuration
            sfm_params = {
                "social_force_factor": float(beh["social_force_factor"]),
                "goal_force_factor": float(beh["goal_force_factor"]),
                "obstacle_force_factor": float(beh["obstacle_force_factor"]),
                "other_force_factor": float(beh["other_force_factor"])
            }
        else:  # Other configurations with constraints
            sfm_params = {
                "social_force_factor": float(beh["social_force_factor"]),
                "goal_force_factor": float(beh["goal_force_factor"]),
                "obstacle_force_factor": float(beh["obstacle_force_factor"]),
                "other_force_factor": float(beh["other_force_factor"])
            }
            
            # Apply constraints for non-custom configurations
            for param, (min_val, max_val) in SFM_CONSTRAINTS.items():
                if param in sfm_params:
                    sfm_params[param] = clamp_value(sfm_params[param], min_val, max_val)
        
        agent.behavior = AgentBehavior(
            # type=int(beh["type"]),
            state=1,
            configuration=configuration,
            # duration=float(beh["duration"]),
            # once=beh["once"],
            # vel=float(beh["vel"]),
            # dist=float(beh["dist"]),
            social_force_factor=sfm_params["social_force_factor"],
            goal_force_factor=sfm_params["goal_force_factor"],
            obstacle_force_factor=sfm_params["obstacle_force_factor"],
            other_force_factor=sfm_params["other_force_factor"],
        )
        
        # Obstacle detection
        max_distance = 4.0
        agent.closest_obs = []
        sensor_offsets = [0.05, 0.1, 0.25, 0.5, 1.0]
        hits = self.get_closest_obstacles(pos, max_distance, sensor_offsets)
        for hit in hits:
            if hit[1] is not None:
                pt = Point(
                    x=float(hit[1][0]),
                    y=float(hit[1][1]),
                    z=float(hit[1][2]),
                )
            else:
                pt = Point(x=10000.0, y=10000.0, z=10000.0)
            agent.closest_obs.append(pt)
        return agent

    def _call_compute(self, agents_msg, robot_msg):
        try:
            req = ComputeAgents.Request()
            req.current_agents = agents_msg
            req.robot = robot_msg
            future = self.compute_agents_client.call_async(req)

            rclpy.spin_until_future_complete(self.node, future)
            if future.done():
                resp = future.result()
                if resp is None:
                    print("[HuNavManager] No response from service.")
                else:
                    self._update_agents(resp.updated_agents)
                return resp
            else:
                print("[HuNavManager] Service response not completed.")
                return None
        except Exception as e:
            print(f"[HuNavManager] Error calling service: {e}")
            return None

    def _update_agents(self, updated_agents):
        for upd in updated_agents.agents:
            idx = upd.id - 1
            agent_prim = self.agents[idx]
            agent_skelroot_prim = self.stage.GetPrimAtPath(
                agent_prim.GetPath().AppendChild("Animation")
            )

            anim_graph_path = self.bound_animations.get(agent_skelroot_prim.GetPath())

            if not self.flag_anim[agent_skelroot_prim.GetPath()]:
                self.stage.GetPrimAtPath(
                    find_skelroot_path(agent_skelroot_prim)
                ).SetMetadata("kind", "component")
                self.flag_anim[agent_skelroot_prim.GetPath()] = True

            char = ag.get_character(str(find_skelroot_path(agent_skelroot_prim)))

            # Set position
            new_pos = Gf.Vec3d(
                upd.position.position.x,
                upd.position.position.y,
                upd.position.position.z,
            )
            agent_prim.GetAttribute("xformOp:translate").Set(new_pos)

            # Set orientation with smoothing
            new_quat = Gf.Quatf(
                upd.position.orientation.w,
                upd.position.orientation.x,
                upd.position.orientation.y,
                upd.position.orientation.z,
            )
            
            # Pre-calculate orientation corrections
            rotX = Gf.Rotation(Gf.Vec3d(0, 0, 1), 90).GetQuat()
            rotXQ = Gf.Quatf(rotX)
            rotZ = Gf.Rotation(Gf.Vec3d(1, 0, 0), 90).GetQuat()
            rotZQ = Gf.Quatf(rotZ)
            
            # Apply corrections to get the target final orientations
            target_prim_quat = new_quat * rotXQ * rotZQ  # Final orientation for prim
            rotZQ_anim = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1, 0, 0), 0).GetQuat())
            target_anim_quat = new_quat * rotXQ * rotZQ_anim  # Final orientation for animation
            
            # Get current orientations for smoothing
            agent_path = agent_prim.GetPath()
            
            # Apply orientation smoothing to the corrected orientations
            if agent_path in self.agent_previous_orientations:
                # Get previous animation quaternion (stored separately)
                prev_prim_quat, prev_anim_quat = self.agent_previous_orientations[agent_path]
                
                # Interpolate both prim and animation orientations for smooth turning
                smoothed_prim_quat = self.slerp_quaternions(
                    prev_prim_quat, 
                    target_prim_quat, 
                    self.orientation_smoothing_factor
                )
                smoothed_anim_quat = self.slerp_quaternions(
                    prev_anim_quat,
                    target_anim_quat,
                    self.orientation_smoothing_factor
                )
            else:
                # First frame - use target orientations directly
                smoothed_prim_quat = target_prim_quat
                smoothed_anim_quat = target_anim_quat
            
            # Store current orientations for next frame (both prim and animation)
            self.agent_previous_orientations[agent_path] = (smoothed_prim_quat, smoothed_anim_quat)
            
            # Apply the smoothed orientations
            agent_prim.GetAttribute("xformOp:orient").Set(smoothed_prim_quat)

            lin = Gf.Vec3d(
                upd.velocity.linear.x,
                upd.velocity.linear.y,
                upd.velocity.linear.z,
            )

            # Animation orientation correction (use smoothed animation orientation)
            if char:
                pos_carb = carb.Float3(new_pos[0], new_pos[1], new_pos[2])
                real = smoothed_anim_quat.GetReal()
                imag = smoothed_anim_quat.GetImaginary()
                rot_carb = carb.Float4(imag[0], imag[1], imag[2], real)
                char.set_world_transform(pos_carb, rot_carb)

            # Set velocities
            agent_prim.GetAttribute("physics:velocity").Set(lin)
            ang = Gf.Vec3d(
                upd.velocity.angular.x,
                upd.velocity.angular.y,
                upd.velocity.angular.z,
            )
            agent_prim.GetAttribute("physics:angularVelocity").Set(ang)

            # Set animation based on agent's speed
            speed = np.linalg.norm(lin)
            max_expected_speed = 1.5
            normalized_speed = np.clip(speed / max_expected_speed, 0.0, 1.0)
            if anim_graph_path:
                set_anim_graph_speed(
                    self.stage, char, anim_graph_path, normalized_speed
                )
            else:
                print(f"No AnimationGraph bound for {agent_prim.GetPath()}")

    def get_character_model_from_skin(self, skin_value):
        """
        Get character model path based on skin value.
        
        Args:
            skin_value (int): The skin ID from agent configuration
                            - 0: Random character model selection
                            - 1-11: Specific character models
            
        Returns:
            str: Path to the character model, or None if skin_value is invalid
        """
        # Handle random skin option (skin value 0)
        if skin_value == 0:
            random_index = random.randint(0, len(self.target_model_paths) - 1)
            return self.target_model_paths[random_index]
        
        # Handle specific skin values (1-11 mapped to model indices 0-10)
        if isinstance(skin_value, (int, float)) and skin_value in self.skin_to_model_mapping:
            model_index = self.skin_to_model_mapping[int(skin_value)]
            if 0 <= model_index < len(self.target_model_paths):
                return self.target_model_paths[model_index]
        
        return None
