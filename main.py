#!/usr/bin/env python3
"""
main.py

Interactive launcher for the Hunav Isaac Wrapper simulation environment.

This script provides a command-line interface that allows users to:
- Select between existing agent configuration files or create new ones via RViz
- Choose from built-in presets (warehouse, hospital, office) or custom YAML files
- Automatically infer the simulation world based on the configuration file
- Select the robot type (jetbot, create3, carter, carter_ROS)
- Launch the TeleopHuNavSim node with the selected parameters

"""
import os
import sys
import argparse
import subprocess
import rclpy
from teleop_hunav_sim import TeleopHuNavSim

# Paths
HOME_WRAPPER = os.path.expanduser("~/Hunav_isaac_wrapper")
DOCKER_WRAPPER = "/workspace/hunav_isaac_ws/src/Hunav_isaac_wrapper"
BASE_WRAPPER = DOCKER_WRAPPER if os.path.isdir(DOCKER_WRAPPER) else HOME_WRAPPER
CONFIG_DIR = os.path.join(BASE_WRAPPER, "scenarios")

# built-in presets
PRESETS = {"agents_warehouse", "agents_hospital", "agents_office"}
KNOWN_SCENARIOS = {"warehouse", "hospital", "office", "empty_world"}
ROBOTS = ["jetbot", "create3", "carter", "carter_ROS"]


def print_header():
    print("\n" * 2)
    print("=" * 60)
    print("     Hunav Isaac Wrapper Interactive Launcher")
    print("=" * 60)
    print()


def choose(prompt, choices, default=None):
    """
    Prompt the user to pick one of choices. Returns the chosen item.
    If default is provided, pressing Enter selects it.
    """
    while True:
        print("\n" + prompt)
        for i, opt in enumerate(choices, 1):
            mark = " (default)" if default and opt == default else ""
            print(f"  {i}) {opt}{mark}")
        suffix = f" [1-{len(choices)}]" + (f" (default {default})" if default else "")
        ans = input(f"Choose{suffix}: ").strip()
        if not ans and default:
            return default
        if ans.isdigit():
            idx = int(ans)
            if 1 <= idx <= len(choices):
                return choices[idx - 1]
        print("  → invalid choice, try again.")


def find_config(basename):
    candidate = os.path.join(CONFIG_DIR, basename)
    if os.path.isfile(candidate):
        return candidate

    print(f"ERROR: cannot find {basename} in {CONFIG_DIR}", file=sys.stderr)
    sys.exit(1)


def infer_scenario_from_config(config_path: str) -> str:
    """
    Look for one of KNOWN_SCENARIOS in the filename; fallback to 'warehouse'.
    """
    base = os.path.basename(config_path).lower()
    for scen in KNOWN_SCENARIOS:
        if scen in base:
            return scen
    return "warehouse"


def main():
    # handle -h/--help ourselves
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        "-h", "--help", action="store_true", help="Show this help and exit"
    )
    args, _ = parser.parse_known_args()
    if args.help:
        print_header()
        print("Interactive launcher for TeleopHuNavSim:\n")
        print("  1) Use an existing agents_XYZ.yaml")
        print("  2) Create a new agents configuration via RViz panel")
        print("  3) Quit\n")
        sys.exit(0)

    print_header()
    # rclpy.init()

    # --- top‐level mode selection ---
    while True:
        mode = choose(
            "Select mode:",
            ["Use existing agents yaml", "Create new agents yaml (RViz panel)", "Quit"],
            default="Use existing agents yaml",
        )
        if mode == "Quit":
            sys.exit(0)
        if mode == "Create new agents yaml (RViz panel)":
            print("\n→ Launching RViz2 panel (in background)...\n")
            subprocess.Popen(
                ["ros2", "launch", "hunav_rviz2_panel", "hunav_rviz2_launch.py"]
            )
            # go back to mode selection
            continue
        # else: use existing config
        break

    # --- pick agents config ---
    cfg_choices = sorted(PRESETS) + ["List all agent yamls in package"]
    config_choice = choose(
        "Select agents configuration file:",
        cfg_choices,
        default=sorted(PRESETS)[0],
    )

    if config_choice == "List all agent yamls in package":
        config_dir = CONFIG_DIR
        # only personalised YAMLs
        all_yaml = sorted(
            f
            for f in os.listdir(config_dir)
            if os.path.splitext(f)[1].lower() in (".yaml")
            and os.path.splitext(f)[0] not in PRESETS
        )
        if not all_yaml:
            print(
                "No personalised YAMLs found in package scenarios directory!",
                file=sys.stderr,
            )
            sys.exit(1)
        selected = choose(
            "Available extra configurations:", all_yaml, default=all_yaml[0]
        )
        config_path = os.path.join(config_dir, selected)
    else:
        # user picked one of the built-in presets
        filename = config_choice + ".yaml"
        config_path = find_config(filename)

    print(f"\n→ Using agents yaml: {config_path}")

    # --- infer scenario ---
    world = infer_scenario_from_config(config_path)
    print(f"→ World inferred as: {world}")

    # --- pick robot ---
    robot = choose("Select your robot:", ROBOTS, default=ROBOTS[3])
    print(f"→ Robot chosen: {robot}")

    # --- final summary ---
    print("\n" + "-" * 60)
    print("Launching with:")
    print(f"  scenario = {world}")
    print(f"  agents   = {config_path}")
    print(f"  robot    = {robot}")
    print("-" * 60 + "\n")

    # pass the chosen agents.yaml to the hunav_loader node as a parameter
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            f"hunav_loader.yaml_base_name:={config_path}",
        ]
    )

    # instantiate and run the TeleopHuNavSim node
    node = TeleopHuNavSim(
        map_name=world,
        hunav_config=config_path,
        robot_name=robot,
    )
    try:
        node.run()
    except KeyboardInterrupt:
        print("\nInterrupted by user, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
