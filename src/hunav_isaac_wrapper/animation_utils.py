#!/usr/bin/env python3
"""
animation_utils.py

This module contains helper functions for handling animations and animation graphs,
including functions to create/apply animations, set up retargeting, and to find
Skeleton and SkelRoot prims.
"""

import omni.kit.commands
from pxr import Sdf


def find_skeleton_path(agentPrim):
    """
    Recursively searches for a prim of type "Skeleton" within agentPrim.

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
        result = find_skeleton_path(child)
        if result:
            return result
    print(
        f"Warning: No Skeleton found for {agentPrim.GetPath()}, using /Root by default."
    )
    return Sdf.Path(f"{agentPrim.GetPath()}/Root")


def find_skelroot_path(agentPrim):
    """
    Recursively searches for a SkelRoot prim within the children of agentPrim.

    """
    for child in agentPrim.GetChildren():
        if not child.IsValid():
            continue
        child_path_str = child.GetPath().pathString

        if ("Looks" in child_path_str) or ("CharacterAnimation" in child_path_str):
            continue

        if "ManRoot" in child_path_str:
            for grandchild in child.GetChildren():
                if grandchild.IsValid() and grandchild.GetTypeName() == "SkelRoot":
                    return grandchild.GetPath()

        if child.GetTypeName() == "SkelRoot":
            return child.GetPath()

        result = find_skelroot_path(child)
        if result:
            return result

    print(
        f"Warning: No SkelRoot found within {agentPrim.GetPath()}, using fallback /Root."
    )
    return Sdf.Path(f"{agentPrim.GetPath()}/Root")


def create_animation(stage, animation_path, source_path):
    """
    Creates an animation prim (of type "SkelAnimation") at the specified path,
    referencing the source animation USD file.
    """
    animation = stage.DefinePrim(animation_path, "SkelAnimation")
    animation.GetReferences().AddReference(source_path)
    return animation


def create_agent_animation_graph(stage, agent_prim, idle_anim_path, walk_anim_path):
    """
    Creates an AnimationGraph for the given agent prim that blends between idle
    and walk animations.

    Returns:
        The Sdf.Path to the newly created AnimationGraph prim.
    """
    # Define a unique path for the AnimationGraph prim under the agent's prim
    graph_path = Sdf.Path(f"{str(agent_prim.GetPath())}/AnimationGraph")
    anim_graph_prim = stage.DefinePrim(graph_path, "AnimationGraph")

    # Create and configure child nodes:
    # Blend node
    blend_path = graph_path.AppendChild("Blend")
    blend_prim = stage.DefinePrim(blend_path, "Blend")
    # Idle AnimationClip node
    idle_clip_path = graph_path.AppendChild("IdleLoop")
    idle_clip_prim = stage.DefinePrim(idle_clip_path, "AnimationClip")
    idle_clip_prim.CreateRelationship("inputs:animationSource").SetTargets(
        [idle_anim_path]
    )
    # Walk AnimationClip node
    walk_clip_path = graph_path.AppendChild("WalkLoop")
    walk_clip_prim = stage.DefinePrim(walk_clip_path, "AnimationClip")
    walk_clip_prim.CreateRelationship("inputs:animationSource").SetTargets(
        [walk_anim_path]
    )
    # ReadVariable node for speed
    speed_node_path = graph_path.AppendChild("speed")
    speed_node_prim = stage.DefinePrim(speed_node_path, "ReadVariable")
    speed_node_prim.CreateAttribute(
        "inputs:variableName", Sdf.ValueTypeNames.Token
    ).Set("speed")

    # Set up connections:
    blend_prim.CreateRelationship("inputs:blendWeight").SetTargets([speed_node_path])
    blend_prim.CreateRelationship("inputs:pose0").SetTargets([idle_clip_path])
    blend_prim.CreateRelationship("inputs:pose1").SetTargets([walk_clip_path])
    anim_graph_prim.CreateRelationship("inputs:pose").SetTargets([blend_path])

    # Bind the AnimationGraph to the agent's skeleton
    skel_path = find_skeleton_path(agent_prim)
    anim_graph_prim.CreateRelationship("skel:skeleton").SetTargets([skel_path])

    print(f"Created AnimationGraph for {agent_prim.GetPath()} at {graph_path}")
    return graph_path


def apply_animation_graph(agent_prim, graph_path):
    """
    Applies an AnimationGraph to the given agent's prim by executing the built-in command.
    """
    omni.kit.commands.execute(
        "ApplyAnimationGraphAPICommand",
        paths=[Sdf.Path(agent_prim.GetPath())],
        animation_graph_path=graph_path,
    )
    print(f"Applied AnimationGraph ({graph_path}) to {agent_prim.GetPath()}")


def setup_anim_retargeting(
    stage, agent_prim, source_animation_dict, target_animation_parent_path
):
    """
    Sets up retargeting for the target agent's animation by using the default source biped.

    Args:
        stage: The USD stage.
        agent_prim: The target agent prim for which retargeting will be applied.
        source_animation_dict: A dictionary with source animation paths.
        target_animation_parent_path: The USD path under which retargeted animations will be created.
    """
    source_agent_prim = stage.GetPrimAtPath("/World/Biped_Setup/biped_demo_meters")
    if not source_agent_prim or not source_agent_prim.IsValid():
        print("Default biped prim not found.")
        return None

    source_skel_path = find_skeleton_path(source_agent_prim)
    if not source_skel_path:
        print("Default biped skeleton not found.")
        return None
    source_skel_str = str(source_skel_path)

    target_skel_path = find_skeleton_path(agent_prim)
    if not target_skel_path:
        print("Target skeleton not found.")
        return None
    target_skel_str = str(target_skel_path)

    source_anim_path = str(source_animation_dict[1])

    omni.kit.commands.execute(
        "CreateRetargetAnimationsCommand",
        source_skeleton_path=source_skel_str,
        target_skeleton_path=target_skel_str,
        source_animation_paths=[source_anim_path],
        target_animation_parent_path=target_animation_parent_path,
        set_root_identity=False,
    )
    omni.kit.commands.execute(
        "CreateRetargetAnimationsCommand",
        source_skeleton_path=source_skel_str,
        target_skeleton_path=target_skel_str,
        source_animation_paths=[str(source_animation_dict[0])],
        target_animation_parent_path=target_animation_parent_path,
        set_root_identity=False,
    )


def set_anim_graph_speed(stage, anim_graph_character, graph_path, speed_value):
    """
    Sets the 'anim:graph:variable:speed' attribute on the AnimationGraph prim.
    If the attribute does not exist, it is created.

    Args:
        stage: The USD stage.
        anim_graph_character: The anim graph character instance.
        graph_path: The Sdf.Path to the AnimationGraph prim.
        speed_value: The float value to set for the 'speed' variable.
    """
    graph_prim = stage.GetPrimAtPath(graph_path)
    if not graph_prim or not graph_prim.IsValid():
        print(f"set_anim_graph_speed: AnimationGraph prim not found at {graph_path}")
        return

    speed_attr = graph_prim.GetAttribute("anim:graph:variable:speed")
    if not speed_attr or not speed_attr.IsValid():
        speed_attr = graph_prim.CreateAttribute(
            "anim:graph:variable:speed", Sdf.ValueTypeNames.Float, custom=True
        )
    anim_graph_character.set_variable("speed", speed_value)
