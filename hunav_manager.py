#!/usr/bin/env python3
"""
hunav_manager.py

Contains the HuNavManager class for spawning and managing Hunav agents,
setting up animations (including retargeting), and handling physics.
"""

import os
import math
import yaml
import numpy as np
import subprocess, signal
from typing import Tuple, List, Optional
from random import choice
import time

# ROS messages
import rclpy
from geometry_msgs.msg import Twist, Quaternion, Pose, Point
from hunav_msgs.srv import ComputeAgents
from hunav_msgs.msg import Agent, Agents, AgentBehavior
from std_msgs.msg import Header

# Isaac Sim imports
import omni
import omni.usd
import omni.physx
import omni.kit.commands
import omni.isaac.dynamic_control
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.core.utils.extensions import disable_extension, enable_extension

from pxr import Sdf, Gf, UsdGeom, UsdPhysics, PhysxSchema, UsdSkel, Vt

enable_extension("omni.anim.retarget.core")
disable_extension("omni.anim.graph.core")
disable_extension("omni.anim.skelJoint")


class HuNavManager:
    """
    Manages HunavSim agents by reading configuration from a YAML file,
    spawning agents as SkelRoot prims, setting up animations (and retargeting),
    and handling ROS 2 communications.
    """

    def __init__(
        self, node, world, config_file_path=None, robot_prim_path=None, robot=None
    ):
        self.node = node
        self.stage = world.stage
        self.robot_prim_path = robot_prim_path
        self.robot_obj = robot

        self.assets_root = get_assets_root_path()
        self._usd_context = omni.usd.get_context()

        self.dc_interface = (
            omni.isaac.dynamic_control._dynamic_control.acquire_dynamic_control_interface()
        )

        # HuNavSim's ROS 2 service client
        self.compute_agents_client = self.node.create_client(
            ComputeAgents, "/compute_agents"
        )

        # List of target model assets
        self.target_model_paths = [
            "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/People/Characters/F_Business_02/F_Business_02.usd",
            "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/People/Characters/F_Medical_01/F_Medical_01.usd",
            "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/People/Characters/M_Medical_01/M_Medical_01.usd",
            "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/People/Characters/male_adult_construction_01_new/male_adult_construction_01_new.usd",
            "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/People/Characters/male_adult_construction_05_new/male_adult_construction_05_new.usd",
            "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/People/Characters/female_adult_police_01_new/female_adult_police_01_new.usd",
        ]
        # Data holders
        self.agents = []
        self.agent_initial_states = []
        self.animationDict = {}
        self._hunav_processes = []

        self.robot_prim = None

        # Sensor offset above the agent's base to avoid getting undesired ground plane detections
        self.sensor_height = 0.1

        if config_file_path is not None:
            self.config = self._load_yaml(config_file_path)
        else:
            self.config = None

        # Define the default source (binding) asset.
        self.default_biped_usd = "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/People/Characters/Biped_Setup.usd"

    def _load_yaml(self, relative_path):
        full_path = os.path.join(os.path.dirname(__file__), relative_path)
        with open(full_path, "r") as file:
            return yaml.safe_load(file)

    def initialize_hunav_nodes(self):
        process_1 = subprocess.Popen(
            ["ros2", "run", "hunav_evaluator", "hunav_evaluator_node"],
            preexec_fn=os.setsid,
        )
        process_2 = subprocess.Popen(
            ["ros2", "run", "hunav_agent_manager", "hunav_agent_manager"],
            preexec_fn=os.setsid,
        )
        self._hunav_processes = [process_1, process_2]

    def close_hunav_nodes(self):
        for process in self._hunav_processes:
            print(process)
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        self._hunav_processes = []

    def findSkeletonPath(self, agentPrim):
        """
        Recursively searches for a prim of type "Skeleton" within agentPrim.
        Skips prims with "Looks" or "CharacterAnimation" in their path.
        Returns agentPrim.GetPath() + "/Root" if no Skeleton is found.
        """
        if agentPrim.GetTypeName() == "Skeleton":
            return agentPrim.GetPath()
        for child in agentPrim.GetChildren():
            if not child.IsValid():
                continue
            child_path_str = child.GetPath().pathString
            if ("Looks" in child_path_str) or ("CharacterAnimation" in child_path_str):
                continue
            if child.GetTypeName() == "Skeleton":
                return child.GetPath()
            result = self.findSkeletonPath(child)
            if result:
                return result
        print(
            f"Warning: No Skeleton found for {agentPrim.GetPath()}, using /Root by default."
        )
        return Sdf.Path(f"{agentPrim.GetPath()}/Root")

    def create_animation(self, animation_path, source_path):
        animation = self.stage.DefinePrim(animation_path, "SkelAnimation")
        animation.GetReferences().AddReference(source_path)
        return animation

    def set_animation(self, agent_prim, anim_list):
        skel_path = self.findSkeletonPath(agent_prim)
        skeleton_prim = self.stage.GetPrimAtPath(skel_path)
        binding_api = UsdSkel.BindingAPI.Apply(skeleton_prim)
        binding_api.GetAnimationSourceRel().SetTargets(anim_list)

    def setup_anim_retargeting(self, agent_prim):
        """
        Uses the CreateRetargetAnimationsCommand to set up retargeting.
        The command uses the default source skeleton (from /World/biped_demo) and
        the target agent's skeleton to create new retargeted animation clips.
        """
        source_agent_prim = self.stage.GetPrimAtPath("/World/biped_demo")
        if not source_agent_prim or not source_agent_prim.IsValid():
            print("Default biped prim not found.")
            return None

        source_skel_path = self.findSkeletonPath(source_agent_prim)
        if not source_skel_path:
            print("Default biped skeleton not found.")
            return None
        source_skel_str = str(source_skel_path)

        target_skel_path = self.findSkeletonPath(agent_prim)
        if not target_skel_path:
            print("Target skeleton not found.")
            return None
        target_skel_str = str(target_skel_path)

        source_anim_path = str(self.source_animation_dict[1])

        omni.kit.commands.execute(
            "CreateRetargetAnimationsCommand",
            source_skeleton_path=source_skel_str,
            target_skeleton_path=target_skel_str,
            source_animation_paths=[source_anim_path],
            target_animation_parent_path=self.target_animation_parent_path,
            set_root_identity=False,
        )
        omni.kit.commands.execute(
            "CreateRetargetAnimationsCommand",
            source_skeleton_path=source_skel_str,
            target_skeleton_path=target_skel_str,
            source_animation_paths=[str(self.source_animation_dict[0])],
            target_animation_parent_path=self.target_animation_parent_path,
            set_root_identity=False,
        )

    def initialize_agents(self):
        """
        Reads the YAML configuration for agents, spawns them, sets up animations,
        and applies retargeting for target models.
        """
        if self.config is None:
            print("[HuNavManager] No config loaded, skipping agent creation.")
            return

        agent_configs = self.config["hunav_loader"]["ros__parameters"]["agents"]

        # Create animations (source clips).
        animations_path = os.path.join(
            self.assets_root, "Isaac", "People", "Animations"
        )
        self.target_animation_parent_path = "/World"
        walk_anim = self.create_animation(
            "/World/Animations/WalkLoop",
            os.path.join(animations_path, "stand_walk_loop_in_place.skelanim.usd"),
        )
        idle_anim = self.create_animation(
            "/World/Animations/IdleLoop",
            os.path.join(animations_path, "stand_idle_loop.skelanim.usd"),
        )
        self.source_animation_dict = {0: idle_anim.GetPath(), 1: walk_anim.GetPath()}
        retarget_anims_path = [
            os.path.join(self.target_animation_parent_path, "IdleLoop"),
            os.path.join(self.target_animation_parent_path, "WalkLoop"),
        ]
        self.animationDict = {0: retarget_anims_path[0], 1: retarget_anims_path[1]}

        # Rotation for upright orientation.
        rotX = Gf.Rotation(Gf.Vec3d(1, 0, 0), 90).GetQuat()
        rotXQ = Gf.Quatf(rotX)

        # Spawn the default biped for skeletal binding.
        init_pos_src = Gf.Vec3d(0, 0, 0)
        init_rot_src = Gf.Quatf(1, 0, 0, 0) * rotXQ
        source_prim_path = "/World/biped_demo"
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

        # Spawn each agent from the config.
        for agent_name in agent_configs:
            agent_cfg = self.config["hunav_loader"]["ros__parameters"][agent_name]
            asset_path = choice(self.target_model_paths) # Random model selection
            init_pose = agent_cfg["init_pose"]
            init_pos = Gf.Vec3d(init_pose["x"], init_pose["y"], init_pose["z"])
            init_rot = Gf.Quatf(1, 0, 0, 0) * rotXQ

            prim_path = f"/World/{agent_name}"
            agent_prim = self.stage.DefinePrim(prim_path, "SkelRoot")
            agent_prim.GetReferences().AddReference(asset_path)
            UsdPhysics.RigidBodyAPI.Apply(agent_prim)
            PhysxSchema.PhysxRigidBodyAPI.Apply(agent_prim)
            xform = UsdGeom.Xformable(agent_prim)
            xform.AddOrientOp()
            agent_prim.GetAttribute("xformOp:orient").Set(init_rot_src)
            agent_prim.GetAttribute("xformOp:translate").Set(init_pos)
            agent_prim.CreateAttribute(
                "physxRigidBody:disableGravity", Sdf.ValueTypeNames.Bool
            ).Set(True)

            self.agents.append(agent_prim)
            self.agent_initial_states.append(
                {"position": init_pos, "orientation": init_rot}
            )

            # Apply retargeting using the extension command.
            self.setup_anim_retargeting(agent_prim)
            # Assign the retargeted animation to the agent.
            self.set_animation(agent_prim, [self.animationDict[0]])

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
        self, agent_position: Gf.Vec3d, max_distance: float, num_lasers: int = 90
    ) -> List[Tuple[float, Optional[Gf.Vec3f]]]:
        """
        Cast rays from the agent's sensor origin (with an offset) in fixed directions.

        Returns a list of tuples (hit_distance, hit_position) for each ray.
        If no obstacle is hit, hit_position is set to None.
        """
        sensor_origin = Gf.Vec3f(
            float(agent_position[0]),
            float(agent_position[1]),
            float(agent_position[2]) + self.sensor_height,
        )
        directions = self.generate_lasers(num_lasers)
        closest_hits = []
        scene_query = omni.physx.get_physx_scene_query_interface()
        for direction in directions:
            hit = scene_query.raycast_closest(sensor_origin, direction, max_distance)
            if hit.get("hit", False):
                distance = hit.get("distance", max_distance)
                hit_position = hit.get("position")
                closest_hits.append((distance, hit_position))
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

        # Build robot message from self.robot_prim
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
        agent.skin = agent_cfg["skin"]
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
        agent.yaw = float(rz)

        # Velocities
        lin = agent_prim.GetAttribute("physics:velocity").Get()
        ang = agent_prim.GetAttribute("physics:angularVelocity").Get()
        agent.linear_vel = np.sqrt(lin[0] ** 2 + lin[1] ** 2 + lin[2] ** 2)
        agent.angular_vel = np.sqrt(ang[0] ** 2 + ang[1] ** 2 + ang[2] ** 2)
        agent.velocity.linear.x = float(lin[0])
        agent.velocity.linear.y = float(lin[1])
        agent.velocity.linear.z = float(lin[2])
        agent.velocity.angular.x = float(ang[0])
        agent.velocity.angular.y = float(ang[1])
        agent.velocity.angular.z = float(ang[2])

        # Goals
        agent.cyclic_goals = agent_cfg["cyclic_goals"]
        agent.goal_radius = float(agent_cfg["goal_radius"])
        goals = []
        for goal in agent_cfg["goals"]:
            goal_config = agent_cfg[goal]
            goal_position = Pose(
                position=Point(x=goal_config["x"], y=goal_config["y"], z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            )
            goals.append(goal_position)
        agent.goals = goals

        # Behavior
        beh = agent_cfg["behavior"]
        agent.behavior = AgentBehavior(
            type=int(beh["type"]),
            state=1,
            configuration=int(beh["configuration"]),
            duration=float(beh["duration"]),
            once=beh["once"],
            vel=float(beh["vel"]),
            dist=float(beh["dist"]),
            social_force_factor=float(beh["social_force_factor"]),
            goal_force_factor=float(beh["goal_force_factor"]),
            obstacle_force_factor=float(beh["obstacle_force_factor"]),
            other_force_factor=float(beh["other_force_factor"]),
        )

        # Obstacle detection
        max_distance = 4.0
        agent.closest_obs = []
        hits = self.get_closest_obstacles(pos, max_distance)
        for hit in hits:
            if hit[1] is not None:
                pt = Point(
                    x=float(hit[1][0]),
                    y=float(hit[1][1]),
                    z=float(hit[1][2]),
                )
            else:
                # No hit detected; append a default point
                pt = Point(x=0.0, y=0.0, z=0.0)
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
        rotX = Gf.Rotation(Gf.Vec3d(0, 0, 1), 90).GetQuat()
        rotXQ = Gf.Quatf(rotX)
        rotZ = Gf.Rotation(Gf.Vec3d(1, 0, 0), 0).GetQuat()
        rotZQ = Gf.Quatf(rotZ)
        for upd in updated_agents.agents:
            idx = upd.id - 1
            agent_prim = self.agents[idx]

            # Set position
            new_pos = Gf.Vec3d(
                upd.position.position.x,
                upd.position.position.y,
                upd.position.position.z,
            )
            agent_prim.GetAttribute("xformOp:translate").Set(new_pos)

            # Set orientation (with corrections applied)
            new_quat = Gf.Quatf(
                upd.position.orientation.w,
                upd.position.orientation.x,
                upd.position.orientation.y,
                upd.position.orientation.z,
            )
            agent_prim.GetAttribute("xformOp:orient").Set(new_quat * rotXQ * rotZQ)
            lin = Gf.Vec3d(
                upd.velocity.linear.x,
                upd.velocity.linear.y,
                upd.velocity.linear.z,
            )

            # Set velocities
            agent_prim.GetAttribute("physics:velocity").Set(lin)
            ang = Gf.Vec3d(
                upd.velocity.angular.x,
                upd.velocity.angular.y,
                upd.velocity.angular.z,
            )
            agent_prim.GetAttribute("physics:angularVelocity").Set(ang)

            # Set animation based on speed
            speed = max(abs(lin[0]), abs(lin[1]), abs(lin[2]))
            if speed < 0.02:
                self.set_animation(agent_prim, [self.animationDict.get(0)])
            else:
                self.set_animation(
                    agent_prim,
                    [self.animationDict.get(upd.behavior.type, self.animationDict[1])],
                )
