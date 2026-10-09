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

# 1. Initialize configuration for YuMi (domain Keva)
from Config import Config
print("Declaring configuration for domain 'Keva' with robot 'yumi'...")
Config.declare_config("Keva", "yumi")

# Ensure sys.modules['SimConfig'] maps to simulators.openrave.SimConfig so Robots.py gets URDF_DIR
import simulators.openrave.SimConfig as sim_cfg_mod
sys.modules['SimConfig'] = sim_cfg_mod

sim_config_object = sim_cfg_mod.SimConfig
for a in getattr(sim_config_object, "REQ_ATTR"):
    if hasattr(Config, a):
        setattr(sim_config_object, a, getattr(Config, a))
sim_config_object.update_dependant_attributes()

import openravepy
from Robots import yumi


def interpolate_motion(robot, env, target_dofs, steps=30, delay=0.03):
    """Smoothly moves the currently active arm to target DOF values."""
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

    # 3. Initialize YuMi robot using class yumi from Robots.py
    print("Initializing YuMi robot (id=1)...")
    yumi_bot = yumi(env=env, id=1)
    print("YuMi robot initialized successfully!")

    # 4. Print robot attributes and manipulator details
    print("=" * 60)
    print("Robot Name: {}".format(yumi_bot.obj_name))
    print("Manipulator Groups: {}".format(yumi_bot.manipulator_groups))
    print("Left Arm Joints: {}".format(yumi_bot.left_arm_joints))
    print("Right Arm Joints: {}".format(yumi_bot.right_arm_joints))
    print("Geometry Limits: {}".format(yumi_bot.geometry_limits))
    print("Robot-Type Mappings: {}".format(yumi_bot.robot_type_object_mappings))
    print("Grasping Offsets: {}".format(yumi_bot.grasping_offset))
    print("Grab Ranges: {}".format(yumi_bot.grab_range))

    # 5. Draw 3D coordinate axes at left and right gripper links
    handles = []
    try:
        left_link = yumi_bot.obj.GetLink("gripper_l_base")
        right_link = yumi_bot.obj.GetLink("gripper_r_base")

        if left_link:
            h_left = openravepy.misc.DrawAxes(env, left_link.GetTransform(), dist=0.15)
            handles.append(h_left)
            print("Drew coordinate axes at 'gripper_l_base' (15cm).")

        if right_link:
            h_right = openravepy.misc.DrawAxes(env, right_link.GetTransform(), dist=0.15)
            handles.append(h_right)
            print("Drew coordinate axes at 'gripper_r_base' (15cm).")
    except Exception as e:
        print("Could not draw debug axes: {}".format(e))

    env.UpdatePublishedBodies()

    # 6. Movement Demonstration: Move left and right arms and cycle grippers
    print("=" * 60)
    print("Demonstrating YuMi dual-arm movement...")

    # Left arm forward ready pose
    left_forward_dofs = [0.0, -1.0, 1.0, 0.5, 0.0, 0.5, 0.0]
    # Right arm forward ready pose
    right_forward_dofs = [0.0, -1.0, -1.0, 0.5, 0.0, 0.5, 0.0]

    # Move Left Arm
    print("Moving left arm forward...")
    yumi_bot.activate_manip_joints("left")
    interpolate_motion(yumi_bot, env, left_forward_dofs, steps=35, delay=0.03)
    time.sleep(0.3)

    # Move Right Arm
    print("Moving right arm forward...")
    yumi_bot.activate_manip_joints("right")
    interpolate_motion(yumi_bot, env, right_forward_dofs, steps=35, delay=0.03)
    time.sleep(0.5)

    # Open grippers
    print("Opening left and right grippers...")
    yumi_bot.openGripper("left")
    yumi_bot.openGripper("right")
    env.UpdatePublishedBodies()
    time.sleep(0.5)

    # Tuck both arms back
    print("Tucking both arms back...")
    yumi_bot.activate_manip_joints("left")
    interpolate_motion(yumi_bot, env, list(yumi_bot.left_arm_tuck_DOFs), steps=35, delay=0.03)

    yumi_bot.activate_manip_joints("right")
    interpolate_motion(yumi_bot, env, list(yumi_bot.right_arm_tuck_DOFs), steps=35, delay=0.03)

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
