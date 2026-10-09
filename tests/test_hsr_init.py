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

# 1. Initialize configuration for HSR
from Config import Config
print("Declaring configuration for domain 'CafeWorld' with robot 'HSR'...")
Config.declare_config("CafeWorld", "HSR")

# Ensure sys.modules['SimConfig'] maps to simulators.openrave.SimConfig so Robots.py gets URDF_DIR
import simulators.openrave.SimConfig as sim_cfg_mod
sys.modules['SimConfig'] = sim_cfg_mod

sim_config_object = sim_cfg_mod.SimConfig
for a in getattr(sim_config_object, "REQ_ATTR"):
    if hasattr(Config, a):
        setattr(sim_config_object, a, getattr(Config, a))
sim_config_object.update_dependant_attributes()

import openravepy
from Robots import HSR

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

    # 3. Initialize HSR robot using class HSR from Robots.py
    print("Initializing HSR robot (id=1)...")
    hsr = HSR(env=env, id=1)
    print("HSR robot initialized successfully!")
    

    # 4. Print robot attributes and gripper details
    print("=" * 60)
    print("Robot Name: {}".format(hsr.obj_name))
    active_manip = hsr.obj.GetActiveManipulator()
    if active_manip:
        print("Active Manipulator: {}".format(active_manip.GetName()))
        gripper_indices = active_manip.GetGripperIndices()
        print("Gripper Indices: {}".format(gripper_indices))
        gripper_joint_names = [hsr.obj.GetJointFromDOFIndex(idx).GetName() for idx in gripper_indices]
        print("Gripper Joints: {}".format(gripper_joint_names))

    print("Geometry Limits: {}".format(hsr.geometry_limits))
    print("Robot-Type Mappings: {}".format(hsr.robot_type_object_mappings))

    # 5. Draw 3D coordinate axes at palm and fingertip frame for clear visual verification
    try:
        palm_link = hsr.obj.GetLink("hand_palm_link")
        tip_link = hsr.obj.GetLink("hand_r_finger_tip_frame")

        handles = []
        if palm_link:
            h_palm = openravepy.misc.DrawAxes(env, palm_link.GetTransform(), dist=0.15)
            handles.append(h_palm)
            print("Drew coordinate axes at 'hand_palm_link' (15cm).")

        if tip_link:
            h_tip = openravepy.misc.DrawAxes(env, tip_link.GetTransform(), dist=0.10)
            handles.append(h_tip)
            print("Drew coordinate axes at 'hand_r_finger_tip_frame' (10cm).")
    except Exception as e:
        print("Could not draw debug axes: {}".format(e))

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
