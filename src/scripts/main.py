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
import signal
import time
import json
from pathlib import Path

# Add the hunav_isaac_wrapper package to the Python path
script_dir = os.path.dirname(os.path.abspath(__file__))
package_dir = os.path.join(os.path.dirname(script_dir), "hunav_isaac_wrapper")
if os.path.exists(package_dir):
    sys.path.insert(0, os.path.dirname(script_dir))

import rclpy

TeleopHuNavSim = None  # Import Isaac Sim components only when needed to avoid startup issues

# Paths - Updated for new ROS2 package structure
script_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.dirname(script_dir)
workspace_root = os.path.dirname(src_dir)

# Try to detect if we're in development or installed mode
if os.path.basename(src_dir) == "src":
    # Development mode - running from src directory
    BASE_WRAPPER = workspace_root
    CONFIG_DIR = os.path.join(src_dir, "scenarios")
    WORLDS_DIR = os.path.join(src_dir, "worlds") 
    CONFIG_CONFIG_DIR = os.path.join(src_dir, "config")
else:
    # Fallback to old paths for backward compatibility
    HOME_WRAPPER = os.path.expanduser("~/Hunav_isaac_wrapper")
    DOCKER_WRAPPER = "/workspace/hunav_isaac_ws/src/Hunav_isaac_wrapper"
    BASE_WRAPPER = DOCKER_WRAPPER if os.path.isdir(DOCKER_WRAPPER) else HOME_WRAPPER
    CONFIG_DIR = os.path.join(BASE_WRAPPER, "scenarios")
    WORLDS_DIR = os.path.join(BASE_WRAPPER, "worlds")
    CONFIG_CONFIG_DIR = os.path.join(BASE_WRAPPER, "config")

LAST_CONFIG_FILE = os.path.join(CONFIG_CONFIG_DIR, "last_launch_config.json")

# built-in presets
PRESETS = {"warehouse_agents", "hospital_agents", "office_agents"}
KNOWN_WORLDS = {"warehouse", "hospital", "office", "empty_world"}
ROBOTS = ["jetbot", "create3", "carter", "carter_ROS"]

# Colors for terminal output
class Colors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def print_header():
    print("\n" * 2)
    print(f"{Colors.HEADER}{'=' * 70}{Colors.ENDC}")
    print(f"{Colors.HEADER}{Colors.BOLD}       HuNav Isaac Wrapper - Interactive Launcher{Colors.ENDC}")
    print(f"{Colors.HEADER}{'=' * 70}{Colors.ENDC}")
    print(f"{Colors.OKCYAN}  A simulation framework for human-robot interaction research{Colors.ENDC}")
    print()


def print_success(message):
    print(f"{Colors.OKGREEN}✓ {message}{Colors.ENDC}")


def print_warning(message):
    print(f"{Colors.WARNING}⚠ {message}{Colors.ENDC}")


def print_error(message):
    print(f"{Colors.FAIL}✗ {message}{Colors.ENDC}")


def print_info(message):
    print(f"{Colors.OKBLUE}ℹ {message}{Colors.ENDC}")


def choose(prompt, choices, default=None, show_descriptions=None):
    """
    Prompt the user to pick one of choices. Returns the chosen item.
    If default is provided, pressing Enter selects it.
    show_descriptions: dict mapping choice to description
    """
    while True:
        print(f"\n{Colors.BOLD}{prompt}{Colors.ENDC}")
        for i, opt in enumerate(choices, 1):
            mark = f" {Colors.OKGREEN}(default){Colors.ENDC}" if default and opt == default else ""
            desc = f" - {Colors.OKCYAN}{show_descriptions[opt]}{Colors.ENDC}" if show_descriptions and opt in show_descriptions else ""
            print(f"  {Colors.BOLD}{i}){Colors.ENDC} {opt}{mark}{desc}")
        
        suffix = f" [{Colors.BOLD}1-{len(choices)}{Colors.ENDC}]"
        if default:
            suffix += f" (press Enter for {Colors.OKGREEN}{default}{Colors.ENDC})"
        
        try:
            ans = input(f"{Colors.BOLD}Choose{suffix}: {Colors.ENDC}").strip()
        except (KeyboardInterrupt, EOFError):
            print(f"\n{Colors.WARNING}Operation cancelled by user.{Colors.ENDC}")
            sys.exit(0)
            
        if not ans and default:
            return default
        if ans.isdigit():
            idx = int(ans)
            if 1 <= idx <= len(choices):
                return choices[idx - 1]
        print(f"  {Colors.FAIL}→ Invalid choice. Please enter a number between 1 and {len(choices)}.{Colors.ENDC}")


def save_last_config(config_path, world, robot):
    """Save the last used configuration for quick reuse."""
    last_config = {
        "config_path": config_path,
        "world": world,
        "robot": robot,
        "timestamp": time.time(),
        "config_name": os.path.basename(config_path)
    }
    
    try:
        with open(LAST_CONFIG_FILE, 'w') as f:
            json.dump(last_config, f, indent=2)
    except Exception as e:
        print_warning(f"Could not save last configuration: {e}")


def load_last_config():
    """Load the last used configuration if available."""
    try:
        if os.path.isfile(LAST_CONFIG_FILE):
            with open(LAST_CONFIG_FILE, 'r') as f:
                last_config = json.load(f)
            
            # Validate that the config file still exists
            if os.path.isfile(last_config.get("config_path", "")):
                return last_config
            else:
                print_warning("Last configuration file no longer exists, ignoring saved settings.")
                return None
        return None
    except Exception as e:
        print_warning(f"Could not load last configuration: {e}")
        return None


def format_last_config_info(last_config):
    """Format last configuration info for display."""
    timestamp = last_config.get("timestamp", 0)
    time_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(timestamp))
    
    return (f"Config: {last_config.get('config_name', 'Unknown')}, "
            f"World: {last_config.get('world', 'Unknown')}, "
            f"Robot: {last_config.get('robot', 'Unknown')}, "
            f"Used: {time_str}")


def validate_environment():
    """Validate that required directories and dependencies exist."""
    print_info("Validating environment...")
    
    # Check if scenarios directory exists
    if not os.path.isdir(CONFIG_DIR):
        print_error(f"Scenarios directory not found: {CONFIG_DIR}")
        print("Please ensure you're running from the correct directory.")
        sys.exit(1)
    
    # Check if worlds directory exists  
    if not os.path.isdir(WORLDS_DIR):
        print_warning(f"Worlds directory not found: {WORLDS_DIR}")
        print("Some functionality may be limited.")
    
    # Check for at least one preset configuration
    preset_found = False
    for preset in PRESETS:
        if os.path.isfile(os.path.join(CONFIG_DIR, f"{preset}.yaml")):
            preset_found = True
            break
    
    if not preset_found:
        print_warning("No preset configurations found in scenarios directory.")
    
    print_success("Environment validation completed.")


def list_available_configs():
    """List all available configuration files with details."""
    print(f"\n{Colors.BOLD}Available Configurations:{Colors.ENDC}")
    
    # Built-in presets
    print(f"\n{Colors.OKCYAN}Built-in Presets:{Colors.ENDC}")
    for preset in sorted(PRESETS):
        config_file = os.path.join(CONFIG_DIR, f"{preset}.yaml")
        if os.path.isfile(config_file):
            print(f"  ✓ {preset}")
        else:
            print(f"  ✗ {preset} (missing)")
    
    # Custom configurations
    custom_configs = []
    if os.path.isdir(CONFIG_DIR):
        custom_configs = [
            f[:-5]  # remove .yaml extension
            for f in os.listdir(CONFIG_DIR)
            if f.endswith('.yaml') and f[:-5] not in PRESETS
        ]
    
    if custom_configs:
        print(f"\n{Colors.OKCYAN}Custom Configurations:{Colors.ENDC}")
        for config in sorted(custom_configs):
            print(f"  ✓ {config}")
    else:
        print(f"\n{Colors.WARNING}No custom configurations found.{Colors.ENDC}")


def find_config(basename):
    """Find configuration file and provide helpful error messages."""
    candidate = os.path.join(CONFIG_DIR, basename)
    if os.path.isfile(candidate):
        return candidate

    print_error(f"Configuration file not found: {basename}")
    print(f"Searched in: {CONFIG_DIR}")
    
    # Suggest similar files
    if os.path.isdir(CONFIG_DIR):
        similar_files = [f for f in os.listdir(CONFIG_DIR) if f.endswith('.yaml')]
        if similar_files:
            print(f"\n{Colors.OKCYAN}Available configuration files:{Colors.ENDC}")
            for f in sorted(similar_files):
                print(f"  • {f}")
    
    sys.exit(1)


def infer_scenario_from_config(config_path: str) -> str:
    """
    Look for one of KNOWN_WORLDS in the filename; fallback to 'warehouse'.
    """
    base = os.path.basename(config_path).lower()
    for scen in KNOWN_WORLDS:
        if scen in base:
            return scen
    return "warehouse"


def parse_arguments():
    """Parse command line arguments with comprehensive options."""
    parser = argparse.ArgumentParser(
        description="Interactive launcher for HuNav Isaac Wrapper simulation environment",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=f"""
Examples:
  {sys.argv[0]}                           # Interactive mode (default)
  {sys.argv[0]} --list-configs           # List available configurations
  {sys.argv[0]} --config agents_warehouse --robot carter
  {sys.argv[0]} --config custom_config.yaml --world office --robot jetbot
  {sys.argv[0]} --batch                  # Non-interactive mode with defaults

Configuration files are searched in: {CONFIG_DIR}
        """
    )
    
    # Mode selection
    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument(
        "--interactive", "-i", 
        action="store_true", 
        default=True,
        help="Run in interactive mode (default)"
    )
    mode_group.add_argument(
        "--batch", "-b", 
        action="store_true", 
        help="Run in batch mode with default/specified parameters"
    )
    mode_group.add_argument(
        "--list-configs", "-l", 
        action="store_true", 
        help="List available configuration files and exit"
    )
    
    # Configuration options
    parser.add_argument(
        "--config", "-c", 
        type=str, 
        help="Agent configuration file (with or without .yaml extension)"
    )
    parser.add_argument(
        "--world", "-w", 
        choices=KNOWN_WORLDS, 
        help="Override world/scenario selection"
    )
    parser.add_argument(
        "--robot", "-r", 
        choices=ROBOTS, 
        help="Robot model to use"
    )
    
    # Advanced options
    parser.add_argument(
        "--validate-only", 
        action="store_true", 
        help="Only validate environment and configuration, don't launch"
    )
    parser.add_argument(
        "--no-color", 
        action="store_true", 
        help="Disable colored output"
    )
    parser.add_argument(
        "--verbose", "-v", 
        action="store_true", 
        help="Enable verbose output"
    )
    
    return parser.parse_args()


def main():
    args = parse_arguments()
    
    # Disable colors if requested
    if args.no_color:
        for attr in dir(Colors):
            if not attr.startswith('_'):
                setattr(Colors, attr, '')
    
    # Handle list-configs mode
    if args.list_configs:
        validate_environment()
        list_available_configs()
        sys.exit(0)
    
    # Print header unless in batch mode
    if not args.batch:
        print_header()
    
    # Validate environment
    validate_environment()
    
    # Handle validate-only mode
    if args.validate_only:
        print_success("Validation completed successfully.")
        sys.exit(0)
    
    config_path = None
    world = None
    robot = None
    
    # Determine configuration source
    if args.config:
        # Use specified configuration
        config_filename = args.config if args.config.endswith('.yaml') else f"{args.config}.yaml"
        config_path = find_config(config_filename)
        if args.verbose:
            print_info(f"Using specified configuration: {config_path}")
    elif args.batch:
        # Use default configuration in batch mode
        default_config = sorted(PRESETS)[0] + ".yaml"
        config_path = find_config(default_config)
        print_info(f"Batch mode: using default configuration {default_config}")
    else:
        # Interactive mode for configuration selection
        result = interactive_config_selection()
        if isinstance(result, tuple):
            # Last configuration was used, returns (config_path, world, robot)
            config_path, world, robot = result
            last_config_used = True
        else:
            # New configuration was selected, returns only config_path
            config_path = result
            last_config_used = False
    
    # Determine world
    if args.world:
        world = args.world
        if args.verbose:
            print_info(f"Using specified world: {world}")
    elif 'last_config_used' in locals() and last_config_used:
        # World already set from last configuration
        if args.verbose:
            print_info(f"Using world from last configuration: {world}")
    else:
        world = infer_scenario_from_config(config_path)
        if not args.batch:
            print_info(f"Inferred world from configuration: {world}")
    
    # Determine robot
    if args.robot:
        robot = args.robot
        if args.verbose:
            print_info(f"Using specified robot: {robot}")
    elif 'last_config_used' in locals() and last_config_used:
        # Robot already set from last configuration
        if args.verbose:
            print_info(f"Using robot from last configuration: {robot}")
    elif args.batch:
        robot = ROBOTS[3]  # Default to carter_ROS
        print_info(f"Batch mode: using default robot {robot}")
    else:
        # Interactive robot selection
        robot_descriptions = {
            "jetbot": "Small differential drive robot for basic navigation",
            "create3": "iRobot Create3 educational robot platform", 
            "carter": "NVIDIA Carter robot for advanced navigation",
            "carter_ROS": "Carter with full ROS2 Nav2 stack support"
        }
        robot = choose(
            "Select robot:",
            ROBOTS,
            default=ROBOTS[3],
            show_descriptions=robot_descriptions
        )
    
    # Final summary
    print(f"\n{Colors.BOLD}{'─' * 70}{Colors.ENDC}")
    print(f"{Colors.BOLD}Launch Configuration:{Colors.ENDC}")
    if 'last_config_used' in locals() and last_config_used:
        print(f"  {Colors.OKGREEN}Source:{Colors.ENDC} Last configuration")
    print(f"  {Colors.OKCYAN}World:{Colors.ENDC} {world}")
    print(f"  {Colors.OKCYAN}Agents:{Colors.ENDC} {os.path.basename(config_path)}")
    print(f"  {Colors.OKCYAN}Robot:{Colors.ENDC} {robot}")
    print(f"  {Colors.OKCYAN}Config Path:{Colors.ENDC} {config_path}")
    print(f"{Colors.BOLD}{'─' * 70}{Colors.ENDC}\n")
    
    if not args.batch:
        try:
            input(f"{Colors.BOLD}Press Enter to launch simulation (Ctrl+C to cancel)...{Colors.ENDC}")
        except (KeyboardInterrupt, EOFError):
            print(f"\n{Colors.WARNING}Launch cancelled by user.{Colors.ENDC}")
            sys.exit(0)
            
    # Save configuration for future quick reuse
    save_last_config(config_path, world, robot)
    
    # Launch simulation
    launch_simulation(world, config_path, robot, args.verbose)


def interactive_config_selection():
    """Handle interactive configuration file selection."""
    # Check if we have a last configuration available
    last_config = load_last_config()
    
    # --- top‐level mode selection ---
    while True:
        # Build mode choices dynamically based on whether we have a last config
        mode_choices = []
        mode_descriptions = {}
        
        if last_config:
            mode_choices.append("Use last launch configuration")
            last_info = format_last_config_info(last_config)
            mode_descriptions["Use last launch configuration"] = f"Quick reuse: {last_info}"
        
        mode_choices.extend([
            "Use existing agents yaml",
            "Create new agents yaml (RViz panel)",
            "Show available configurations", 
            "Quit"
        ])
        
        mode_descriptions.update({
            "Use existing agents yaml": "Select from available configuration files",
            "Create new agents yaml (RViz panel)": "Launch RViz2 panel to create new configuration",
            "Show available configurations": "Display all available config files",
            "Quit": "Exit the launcher"
        })
        
        # Set default based on whether we have a last config
        default_mode = "Use last launch configuration" if last_config else "Use existing agents yaml"
        
        mode = choose(
            "Select mode:",
            mode_choices,
            default=default_mode,
            show_descriptions=mode_descriptions
        )
        
        if mode == "Quit":
            print_info("Goodbye!")
            sys.exit(0)
        elif mode == "Use last launch configuration":
            print_success(f"Using last configuration: {last_config['config_name']}")
            print_info(f"World: {last_config['world']}, Robot: {last_config['robot']}")
            return last_config['config_path'], last_config['world'], last_config['robot']
        elif mode == "Show available configurations":
            list_available_configs()
            continue
        elif mode == "Create new agents yaml (RViz panel)":
            print_info("Launching RViz2 panel for agent configuration...")
            
            # Check if RViz panel is already running
            try:
                result = subprocess.run(
                    ["pgrep", "-f", "hunav_rviz2_launch"],
                    capture_output=True,
                    text=True
                )
                if result.returncode == 0:
                    pids = result.stdout.strip().split('\n')
                    print_warning(f"RViz2 panel appears to be already running (PID(s): {', '.join(pids)}).")
                    restart_choice = input(f"{Colors.BOLD}Would you like to restart it? (y/N): {Colors.ENDC}").strip().lower()
                    if restart_choice in ['y', 'yes']:
                        print_info("Stopping existing RViz2 panel instances...")
                        
                        # Kill existing processes more thoroughly
                        try:
                            # First try graceful termination
                            os.killpg(os.getpgid(rviz_process.pid), signal.SIGTERM)
                            print_info("Sent termination signal, waiting for processes to stop...")
                            time.sleep(1)
                            
                            # Check if any processes are still running
                            check_result = subprocess.run(
                                ["pgrep", "-f", "hunav_rviz2_launch"],
                                capture_output=True,
                                text=True
                            )
                            
                            if check_result.returncode == 0:
                                print_warning("Some processes still running, forcing termination...")
                                os.killpg(os.getpgid(rviz_process.pid), signal.SIGKILL)
                                time.sleep(2)
                                
                                # Final check
                                final_check = subprocess.run(
                                    ["pgrep", "-f", "hunav_rviz2_launch"],
                                    capture_output=True,
                                    text=True
                                )
                                
                                if final_check.returncode == 0:
                                    print_error("Failed to stop all RViz2 processes. Please stop them manually:")
                                    print(f"  {Colors.OKCYAN}pkill -f hunav_rviz2_launch{Colors.ENDC}")
                                    continue
                                else:
                                    print_success("All RViz2 processes stopped successfully.")
                            else:
                                print_success("RViz2 processes stopped gracefully.")
                                
                        except Exception as e:
                            print_error(f"Error stopping RViz2 processes: {e}")
                            continue
                    else:
                        print_info("Using existing RViz2 panel instance.")
                        print("Please check your RViz2 window and configure agents there.")
                        input(f"{Colors.BOLD}Press Enter when you've finished configuring agents...{Colors.ENDC}")
                        continue
            except FileNotFoundError:
                pass  # pgrep not available, continue normally
            
            # Try to launch RViz panel
            launch_success = False
            rviz_process = None
            
            try:
                print_info("Starting RViz2 panel in background...")
                rviz_process = subprocess.Popen(
                    ["ros2", "launch", "hunav_rviz2_panel", "hunav_rviz2_launch.py"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    preexec_fn=os.setsid,
                    text=True
                )
                
                # Wait a bit and check if it started successfully
                time.sleep(3)
                if rviz_process.poll() is None:
                    # Process is still running
                    print_success("RViz2 panel launched successfully!")
                    launch_success = True
                else:
                    # Process exited early, check for errors
                    _, stderr = rviz_process.communicate(timeout=1)
                    print_error("RViz2 panel failed to start.")
                    if stderr:
                        print(f"Error: {stderr.strip()}")
                
            except FileNotFoundError:
                print_error("RViz2 panel launch failed: 'ros2' command not found.")
                print("Make sure ROS2 is installed and sourced:")
                print(f"  {Colors.OKCYAN}source /opt/ros/humble/setup.bash{Colors.ENDC}")
            except subprocess.TimeoutExpired:
                print_warning("RViz2 panel is taking longer than expected to start...")
                if rviz_process and rviz_process.poll() is None:
                    print_info("Process appears to be running. Continuing...")
                    launch_success = True
            except Exception as e:
                print_error(f"Failed to launch RViz2 panel: {e}")
                
                # Check for common issues and provide solutions
                error_str = str(e).lower()
                if "lark" in error_str or "no module named 'lark'" in error_str:
                    print(f"\n{Colors.BOLD}Missing Lark Parser Dependency Detected!{Colors.ENDC}")
                    print("The lark parser is required for ROS2 launch files.")
                    
                    install_choice = choose(
                        "Would you like to install the lark parser now?",
                        ["Yes, install automatically", "No, I'll install it manually"],
                        default="Yes, install automatically"
                    )
                    
                    if install_choice == "Yes, install automatically":
                        print_info("Installing lark parser...")
                        
                        # Try the working installation method first, then fallbacks
                        install_commands = [
                            ["python3", "-m", "pip", "install", "--user", "lark"],
                            ["python3", "-m", "pip", "install", "--user", "lark-parser"],
                            ["sudo", "apt", "install", "-y", "python3-lark"]
                        ]
                        
                        install_success = False
                        for i, cmd in enumerate(install_commands):
                            try:
                                print_info(f"Trying installation method {i+1}: {' '.join(cmd)}")
                                result = subprocess.run(
                                    cmd, 
                                    capture_output=True, 
                                    text=True, 
                                    timeout=60
                                )
                                if result.returncode == 0:
                                    print_success(f"Successfully installed lark using: {' '.join(cmd)}")
                                    install_success = True
                                    break
                                else:
                                    print_warning(f"Method {i+1} failed: {result.stderr.strip()}")
                            except subprocess.TimeoutExpired:
                                print_warning(f"Method {i+1} timed out.")
                            except FileNotFoundError:
                                print_warning(f"Method {i+1} failed: Command not found.")
                            except Exception as install_error:
                                print_warning(f"Method {i+1} failed: {install_error}")
                        
                        if install_success:
                            print_success("Lark parser installed successfully!")
                            retry_choice = input(f"{Colors.BOLD}Would you like to retry launching RViz2 panel? (Y/n): {Colors.ENDC}").strip().lower()
                            if retry_choice not in ['n', 'no']:
                                print_info("Retrying RViz2 panel launch...")
                                continue
                        else:
                            print_error("Automatic installation failed. Please install manually:")
                            print(f"  {Colors.OKCYAN}python3 -m pip install --user lark{Colors.ENDC}")
                    
                    else:  # Manual installation
                        print(f"\n{Colors.BOLD}Manual Installation Required:{Colors.ENDC}")
                        print(f"Install lark parser: {Colors.OKCYAN}python3 -m pip install --user lark{Colors.ENDC}")
                        print("Then restart this launcher to try again.")
                
                elif "hunav_rviz2_panel" in error_str:
                    print(f"\n{Colors.BOLD}HuNav RViz2 Panel package not found.{Colors.ENDC}")
                    print("Please install or build the hunav_rviz2_panel package.")
                    print(f"You can also create configurations manually using the example format.")
                
                else:
                    print(f"\n{Colors.BOLD}RViz2 launch error:{Colors.ENDC}")
                    print("Please check your ROS2 installation and environment.")
                    print(f"Make sure ROS2 is sourced: {Colors.OKCYAN}source /opt/ros/humble/setup.bash{Colors.ENDC}")
                    print(f"Error details: {str(e)}")
            
            if launch_success:
                print(f"\n{Colors.BOLD}RViz2 Panel Instructions:{Colors.ENDC}")
                print("1. Use the RViz2 interface to configure your agents")
                print("2. Set agent positions, goals, and behaviors")
                print("3. Save your configuration to a YAML file")
                print(f"4. Save the file to: {Colors.OKCYAN}{CONFIG_DIR}/{Colors.ENDC}")
                print("5. Return here to select your saved configuration")

                print(f"\n{Colors.WARNING}Note:{Colors.ENDC} The RViz2 panel is running in the background.")
                print("You can switch to it, configure agents, and return here.")
                
                # Offer options for what to do next
                while True:
                    next_action = choose(
                        "What would you like to do?",
                        [
                            "Continue with existing configuration",
                            "Wait for RViz configuration (I'll configure now)",
                            "Check if new configuration was saved",
                            "Stop RViz panel and return to menu"
                        ],
                        default="Continue with existing configuration"
                    )
                    
                    if next_action == "Continue with existing configuration":
                        print_info("Continuing with existing configuration selection...")
                        break
                        
                    elif next_action == "Wait for RViz configuration (I'll configure now)":
                        print_info("Take your time to configure agents in RViz2...")
                        print("Switch to the RViz2 window to configure agents")
                        print("Don't forget to save your configuration!")
                        
                        try:
                            input(f"\n{Colors.BOLD}Press Enter when you've finished and saved your configuration...{Colors.ENDC}")
                            print_success("Configuration completed! You can now select your saved file.")
                            break
                        except (KeyboardInterrupt, EOFError):
                            print_info("Configuration interrupted.")
                            break
                            
                    elif next_action == "Check if new configuration was saved":
                        print_info("Checking for new configuration files...")
                        
                        # Look for recently modified files
                        recent_configs = []
                        current_time = time.time()
                        
                        try:
                            for f in os.listdir(CONFIG_DIR):
                                if f.endswith('.yaml'):
                                    filepath = os.path.join(CONFIG_DIR, f)
                                    mtime = os.path.getmtime(filepath)
                                    # Files modified in the last 5 minutes
                                    if current_time - mtime < 300:
                                        recent_configs.append((f, mtime))
                        except OSError:
                            print_warning("Could not check configuration directory.")
                        
                        if recent_configs:
                            recent_configs.sort(key=lambda x: x[1], reverse=True)
                            print_success(f"Found {len(recent_configs)} recently modified configuration(s):")
                            for config, mtime in recent_configs:
                                time_str = time.strftime("%H:%M:%S", time.localtime(mtime))
                                print(f"  • {config} (modified at {time_str})")
                        else:
                            print_info("No recently modified configurations found.")
                            print("Make sure to save your RViz configuration to the scenarios folder.")
                    
                    elif next_action == "Stop RViz panel and return to menu":
                        if rviz_process and rviz_process.poll() is None:
                            print_info("Stopping RViz2 panel...")
                            try:
                                # First try graceful termination of the process group
                                os.killpg(os.getpgid(rviz_process.pid), signal.SIGTERM)
                                
                                # Wait for the process to terminate
                                try:
                                    rviz_process.wait(timeout=5)
                                    print_success("RViz2 panel stopped gracefully.")
                                except subprocess.TimeoutExpired:
                                    print_warning("Process didn't stop gracefully, forcing termination...")
                                    os.killpg(os.getpgid(rviz_process.pid), signal.SIGKILL)
                                    rviz_process.wait(timeout=2)
                                    print_success("RViz2 panel force-stopped.")
                                    
                            except (ProcessLookupError, OSError):
                                print_info("RViz2 panel process already terminated.")
                            except Exception as e:
                                print_warning(f"Error stopping RViz2 panel: {e}")
                                print("You may need to stop it manually if it's still running.")
                        else:
                            print_info("No RViz2 panel process to stop.")
                        break           
            
            continue
        else:
            # Use existing config
            break

    # --- pick agents config ---
    config_type_descriptions = {
        "Pre-built configurations": "Choose from warehouse, hospital, or office presets",
        "Custom configurations": "Browse user-created configuration files"
    }
    
    config_type = choose(
        "Select configuration type:",
        ["Pre-built configurations", "Custom configurations"],
        default="Pre-built configurations",
        show_descriptions=config_type_descriptions
    )
    
    if config_type == "Pre-built configurations":
        # Show only preset configurations
        preset_list = sorted(list(PRESETS))
        config_choice = choose(
            "Select pre-built configuration:",
            preset_list,
            default=preset_list[0],
        )
        # User picked one of the built-in presets
        filename = config_choice + ".yaml"
        config_path = find_config(filename)
        
    else:  # Custom configurations
        if not os.path.isdir(CONFIG_DIR):
            print_error(f"Configuration directory not found: {CONFIG_DIR}")
            sys.exit(1)
            
        # Get all YAML files with their modification times, excluding presets
        all_yaml_with_times = []
        try:
            for f in os.listdir(CONFIG_DIR):
                if f.endswith('.yaml') and os.path.isfile(os.path.join(CONFIG_DIR, f)):
                    # Exclude preset files (they were already offered as direct choices)
                    config_name = f[:-5]  # Remove .yaml extension
                    if config_name not in PRESETS:
                        filepath = os.path.join(CONFIG_DIR, f)
                        try:
                            mtime = os.path.getmtime(filepath)
                            all_yaml_with_times.append((f, mtime))
                        except OSError:
                            # If we can't get modification time, add with timestamp 0
                            all_yaml_with_times.append((f, 0))
        except OSError as e:
            print_error(f"Error reading configuration directory: {e}")
            sys.exit(1)

        if not all_yaml_with_times:
            print_warning("No custom configuration files found!")
            print_info("You can create custom configurations using the 'Create new agents yaml (RViz panel)' option.")
            print_info("Falling back to pre-built configurations...")
            
            # Fall back to preset selection
            config_choice = choose(
                "Select pre-built configuration:",
                sorted(PRESETS),
                default=sorted(PRESETS)[0],
            )
            filename = config_choice + ".yaml"
            config_path = find_config(filename)
        else:
            # Sort by modification time (newest first)
            all_yaml_with_times.sort(key=lambda x: x[1], reverse=True)
            all_yaml = [f for f, _ in all_yaml_with_times]

            # Show file details
            print(f"\n{Colors.OKCYAN}Custom configuration files (newest first):{Colors.ENDC}")
            file_details = {}
            for f, mtime in all_yaml_with_times:
                filepath = os.path.join(CONFIG_DIR, f)
                try:
                    size = os.path.getsize(filepath)
                    if mtime > 0:
                        mtime_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(mtime))
                        file_details[f] = f"({size} bytes, modified: {mtime_str})"
                    else:
                        file_details[f] = f"({size} bytes, modification time unavailable)"
                except OSError:
                    file_details[f] = "(details unavailable)"

            selected = choose(
                "Select configuration file:",
                all_yaml,
                default=all_yaml[0],
                show_descriptions=file_details
            )
            config_path = os.path.join(CONFIG_DIR, selected)

    print_success(f"Selected configuration: {os.path.basename(config_path)}")
    return config_path


def launch_simulation(world, config_path, robot, verbose=False):
    """Launch the simulation with the specified parameters."""
    print_info("Initializing simulation...")
    
    try:
        # Setup signal handler for graceful shutdown
        def signal_handler(sig, frame):
            print(f"\n{Colors.WARNING}Received interrupt signal. Shutting down gracefully...{Colors.ENDC}")
            if 'node' in locals():
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            sys.exit(0)
        
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        # Initialize ROS2
        if verbose:
            print_info("Initializing ROS2...")
        
        rclpy.init(
            args=[
                "--ros-args",
                "-p",
                f"hunav_loader.yaml_base_name:={config_path}",
            ]
        )
        
        if verbose:
            print_info("Creating TeleopHuNavSim node...")
        
        # Import Isaac Sim components only when needed to avoid startup issues
        global TeleopHuNavSim
        if TeleopHuNavSim is None:
            from hunav_isaac_wrapper.teleop_hunav_sim import TeleopHuNavSim
        
        # Create and run the TeleopHuNavSim node
        node = TeleopHuNavSim(
            map_name=world,
            hunav_config=config_path,
            robot_name=robot,
        )
        
        print_success("Simulation launched successfully!")
        print_info("Use Ctrl+C to stop the simulation.")
        
        if verbose:
            print_info("Starting simulation loop...")
        
        node.run()
        
    except KeyboardInterrupt:
        print(f"\n{Colors.WARNING}Simulation interrupted by user.{Colors.ENDC}")
    except Exception as e:
        print_error(f"Simulation failed: {e}")
        if verbose:
            import traceback
            traceback.print_exc()
        sys.exit(1)
    finally:
        if verbose:
            print_info("Cleaning up...")
        try:
            if 'node' in locals():
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            if verbose:
                print_warning(f"Cleanup warning: {e}")
        print_success("Simulation shutdown complete.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(f"\n{Colors.WARNING}Program interrupted by user.{Colors.ENDC}")
        sys.exit(0)
    except Exception as e:
        print_error(f"Unexpected error: {e}")
        sys.exit(1)
