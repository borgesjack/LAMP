# -*- coding: utf-8 -*-
from __future__ import print_function
import os
import sys
import time
import numpy as np

# Ensure root repository directory and simulators directory are in sys.path
ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

SIM_DIR = os.path.join(ROOT_DIR, "simulators", "openrave")
if SIM_DIR not in sys.path:
    sys.path.insert(0, SIM_DIR)

# Set default display and Qt environment variables if not already set
if "DISPLAY" not in os.environ:
    os.environ["DISPLAY"] = ":0.0"
os.environ["QT_X11_NO_MITSHM"] = "1"

# 1. Initialize configuration for Fetch
from Config import Config
print("Declaring configuration for domain 'CafeWorld' with robot 'Fetch'...")
Config.declare_config("CafeWorld", "Fetch")

# Ensure sys.modules['SimConfig'] maps to simulators.openrave.SimConfig so Robots.py gets URDF_DIR
import simulators.openrave.SimConfig as sim_cfg_mod
sys.modules['SimConfig'] = sim_cfg_mod

sim_config_object = sim_cfg_mod.SimConfig
for a in getattr(sim_config_object, "REQ_ATTR"):
    if hasattr(Config, a):
        setattr(sim_config_object, a, getattr(Config, a))
sim_config_object.update_dependant_attributes()

import openravepy
from Robots import Fetch


def interpolate_motion(robot, env, target_dofs, steps=30, delay=0.03):
    """Smoothly moves the robot arm to target DOF values."""
    current_dofs = robot.obj.GetActiveDOFValues()
    for alpha in np.linspace(0, 1, steps):
        dofs = (1.0 - alpha) * np.array(current_dofs) + alpha * np.array(target_dofs)
        robot.obj.SetActiveDOFValues(dofs)
        env.UpdatePublishedBodies()
        time.sleep(delay)


def main():
    print("Creating OpenRAVE environment...")
    env = openravepy.Environment()
    env.SetDebugLevel(openravepy.DebugLevel.Info)

    # 2. Start the OpenRAVE viewer for visualization
    print("Setting up OpenRAVE viewer ('qtcoin')...")
    viewer = env.SetViewer('qtcoin')
    if not viewer:
        print("Note: 'qtcoin' viewer returned False/None. Trying 'qtosg'...")
        viewer = env.SetViewer('qtosg')

    if env.GetViewer() is not None:
        print("Viewer successfully initialized: {}".format(env.GetViewer()))
    else:
        print("Warning: Could not initialize graphical viewer. Ensure DISPLAY ({}) is reachable.".format(os.environ.get("DISPLAY")))

    # 3. Initialize Fetch robot using class Fetch from Robots.py
    print("Initializing Fetch robot (id=1)...")
    fetch = Fetch(env=env, id=1)
    print("Fetch robot initialized successfully!")

    # 4. Print robot attributes and gripper details
    print("=" * 60)
    print("Robot Name: {}".format(fetch.obj_name))
    active_manip = fetch.obj.GetActiveManipulator()
    if active_manip:
        print("Active Manipulator: {}".format(active_manip.GetName()))
        gripper_indices = active_manip.GetGripperIndices()
        print("Gripper Indices: {}".format(gripper_indices))
        gripper_joint_names = [fetch.obj.GetJointFromDOFIndex(idx).GetName() for idx in gripper_indices]
        print("Gripper Joints: {}".format(gripper_joint_names))

    print("Geometry Limits: {}".format(fetch.geometry_limits))
    print("Robot-Type Mappings: {}".format(fetch.robot_type_object_mappings))
    print("Grasping Offsets: {}".format(fetch.grasping_offset))
    print("Grab Ranges: {}".format(fetch.grab_range))

    # 5. Draw 3D coordinate axes at gripper link and fingertip frame
    handles = []
    try:
        wrist_link = fetch.obj.GetLink("wrist_roll_link")
        finger_link = fetch.obj.GetLink("r_gripper_finger_link")

        if wrist_link:
            h_wrist = openravepy.misc.DrawAxes(env, wrist_link.GetTransform(), dist=0.15)
            handles.append(h_wrist)
            print("Drew coordinate axes at 'wrist_roll_link' (15cm).")

        if finger_link:
            h_finger = openravepy.misc.DrawAxes(env, finger_link.GetTransform(), dist=0.10)
            handles.append(h_finger)
            print("Drew coordinate axes at 'r_gripper_finger_link' (10cm).")
    except Exception as e:
        print("Could not draw debug axes: {}".format(e))

    env.UpdatePublishedBodies()

    # 6. Movement Demonstration: Unfold arm forward and tuck back
    print("=" * 60)
    print("Demonstrating Fetch arm movement...")
    fetch.activate_manip_joints()
    tucked_dofs = list(fetch.grabbed_armTuckDOFs)
    forward_ready_dofs = [0.15, 0.0, 0.0, 0.0, 0.5, 0.0, 0.5, 0.0]

    # Smoothly move arm to forward position
    print("Moving arm forward...")
    interpolate_motion(fetch, env, forward_ready_dofs, steps=35, delay=0.03)
    time.sleep(0.5)

    # Open gripper
    print("Opening gripper...")
    fetch.openGripper()
    env.UpdatePublishedBodies()
    time.sleep(0.5)

    # Smoothly return to tucked pose
    print("Tucking arm back...")
    interpolate_motion(fetch, env, tucked_dofs, steps=35, delay=0.03)

    print("=" * 60)
    print("Visualization is active in OpenRAVE (or noVNC at http://localhost:8080/vnc.html).")
    print("Press Enter in the terminal to exit (or Ctrl+C)...")

    # Keep published bodies updated and viewer responsive until exit
    try:
        try:
            input_func = raw_input
        except NameError:
            input_func = input

        env.UpdatePublishedBodies()
        input_func()
    except (KeyboardInterrupt, EOFError):
        pass

    print("Cleaning up and exiting...")
    env.Destroy()
    print("Done!")


if __name__ == "__main__":
    main()
