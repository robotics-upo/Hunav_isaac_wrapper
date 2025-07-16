#!/usr/bin/env python3
"""
ROS2 launcher for HuNav Isaac Wrapper.

This script provides the same functionality as launch_hunav_isaac.sh
but can be called via 'ros2 run hunav_isaac_wrapper hunav_isaac_launcher'
"""

import os
import sys
import subprocess
import argparse
from pathlib import Path


def find_isaac_python():
    """Find the Isaac Sim python executable."""
    isaac_paths = [
        # Container-specific path
        Path("/isaac-sim/python.sh"),
        # Standard user installation paths
        Path.home() / "isaacsim" / "python.sh",
        Path.home() / ".local" / "share" / "ov" / "pkg"
    ]
    
    # Check container path first
    if isaac_paths[0].exists():
        return f"bash {isaac_paths[0]}"
    
    # Check direct user path
    if isaac_paths[1].exists():
        return f"bash {isaac_paths[1]}"
    
    # Check AppImage installation
    if isaac_paths[2].exists():
        isaac_dirs = list(isaac_paths[2].glob("isaac_sim-*"))
        if isaac_dirs:
            python_sh = isaac_dirs[0] / "python.sh"
            if python_sh.exists():
                return f"bash {python_sh}"
    
    # Check if isaacsim command is available
    try:
        subprocess.run(["which", "isaacsim"], check=True, capture_output=True)
        return "isaacsim"
    except subprocess.CalledProcessError:
        pass
    
    return None


def find_main_script():
    """Find the main.py script."""
    # Try to find via ROS2 package
    try:
        result = subprocess.run(
            ["ros2", "pkg", "prefix", "hunav_isaac_wrapper"],
            capture_output=True, text=True, check=True
        )
        pkg_path = Path(result.stdout.strip())
        main_script = pkg_path / "lib" / "hunav_isaac_wrapper" / "main.py"
        if main_script.exists():
            return str(main_script)
    except (subprocess.CalledProcessError, FileNotFoundError):
        pass
    
    # Try relative to this script (development mode)
    script_dir = Path(__file__).parent.parent
    main_script = script_dir / "scripts" / "main.py"
    if main_script.exists():
        return str(main_script)
    
    # Try workspace relative path
    workspace_paths = [
        Path.cwd() / "src" / "scripts" / "main.py",
        Path.cwd() / "scripts" / "main.py"
    ]
    
    for path in workspace_paths:
        if path.exists():
            return str(path)
    
    return None


def get_package_share_directory():
    """Get the package share directory for resource files."""
    try:
        result = subprocess.run(
            ["ros2", "pkg", "prefix", "hunav_isaac_wrapper"],
            capture_output=True, text=True, check=True
        )
        pkg_path = Path(result.stdout.strip())
        share_dir = pkg_path / "share" / "hunav_isaac_wrapper"
        if share_dir.exists():
            return str(share_dir)
    except (subprocess.CalledProcessError, FileNotFoundError):
        pass
    
    # Fallback to development workspace
    workspace_paths = [
        Path.cwd() / "src",
        Path.cwd()
    ]
    
    for path in workspace_paths:
        if path.exists():
            return str(path)
    
    return str(Path.cwd())


def print_help():
    """Print help information."""
    print("HuNav Isaac Wrapper ROS2 Launcher")
    print("")
    print("Usage:")
    print("  ros2 run hunav_isaac_wrapper hunav_isaac_launcher              # Interactive mode")
    print("  ros2 run hunav_isaac_wrapper hunav_isaac_launcher [scenario]   # Launch with scenario")
    print("  ros2 run hunav_isaac_wrapper hunav_isaac_launcher [args...]    # Pass arguments")
    print("")
    print("Examples:")
    print("  ros2 run hunav_isaac_wrapper hunav_isaac_launcher")
    print("  ros2 run hunav_isaac_wrapper hunav_isaac_launcher warehouse_agents.yaml")
    print("  ros2 run hunav_isaac_wrapper hunav_isaac_launcher --config myfile.yaml --batch")


def main():
    """Main launcher function."""
    # Parse arguments
    if len(sys.argv) > 1 and sys.argv[1] in ["--help", "-h"]:
        print_help()
        return 0
    
    # Find Isaac Sim python
    isaac_python = find_isaac_python()
    if isaac_python is None:
        print("⚠️  Warning: Isaac Sim python not found.")
        print("⚠️  Falling back to system python3. Some Isaac Sim features may not work.")
        print("⚠️  For best results, please install Isaac Sim and source its environment.")
        print("")
        isaac_python = "python3"
        print(f"Using system python: {isaac_python}")
    else:
        print(f"Using Isaac Sim python: {isaac_python}")
    
    # Find main script
    main_script = find_main_script()
    if main_script is None:
        print("❌ Error: Cannot find main.py script")
        print("Make sure the package is properly installed or you're in the correct workspace.")
        return 1
    
    # Get package share directory for resources
    share_dir = get_package_share_directory()
    print(f"Using package share directory: {share_dir}")
    print(f"Using script: {main_script}")
    print("HuNav Isaac Wrapper Launcher")
    
    # Change to share directory so main.py can find resources
    original_cwd = os.getcwd()
    os.chdir(share_dir)
    
    # Build command
    cmd_parts = isaac_python.split() + [main_script]
    
    # Handle arguments
    if len(sys.argv) == 1:
        # Interactive mode
        print("Launching in interactive mode...")
    elif len(sys.argv) == 2 and not sys.argv[1].startswith('--'):
        # Single scenario file specified (doesn't start with --)
        scenario = sys.argv[1]
        print(f"Launching with scenario: {scenario}")
        cmd_parts.extend(["--config", scenario, "--batch"])
    else:
        # Multiple arguments or arguments that start with --
        args = sys.argv[1:]
        
        # Check if the first argument is a scenario file (doesn't start with --)
        # and there are additional arguments
        if len(args) >= 2 and not args[0].startswith('--'):
            # First argument is likely a scenario file, convert it to --config format
            scenario = args[0]
            remaining_args = args[1:]
            print(f"Launching with scenario: {scenario} and additional arguments: {' '.join(remaining_args)}")
            cmd_parts.extend(["--config", scenario] + remaining_args)
        else:
            # All arguments start with -- or it's a single -- argument
            print(f"Launching with arguments: {' '.join(args)}")
            cmd_parts.extend(args)
    
    # Execute the command
    try:
        result = subprocess.run(cmd_parts)
        return result.returncode
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user")
        return 0
    except Exception as e:
        print(f"❌ Error executing command: {e}")
        return 1
    finally:
        # Restore original working directory
        os.chdir(original_cwd)


if __name__ == "__main__":
    sys.exit(main())
