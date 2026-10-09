# -*- coding: utf-8 -*-
from __future__ import print_function
import os
import sys
import time
import numpy as np

# Ensure root repository directory and simulators directory are in sys.path
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
if ROOT_DIR not in sys.path:
    sys.path.append(ROOT_DIR)

SIM_DIR = os.path.join(ROOT_DIR, "simulators", "openrave")
if SIM_DIR not in sys.path:
    sys.path.append(SIM_DIR)

# Set default display and Qt environment variables if not already set
if "DISPLAY" not in os.environ:
    os.environ["DISPLAY"] = ":0.0"
os.environ["QT_X11_NO_MITSHM"] = "1"

# 1. Initialize configuration for HSR
from Config import Config
print("Declaring configuration for domain 'CafeWorld' with robot 'HSR'...")
Config.declare_config("CafeWorld", "HSR")
Config.MAX_IK_ATTEMPTS = 50

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
    print("=" * 60)
    print("HSR IK Solving and Motion Example")
    print("=" * 60)

    # Initialize OpenRAVE environment
    print("Creating OpenRAVE environment...")
    env = openravepy.Environment()
    env.SetDebugLevel(openravepy.DebugLevel.Info)

    # Setup viewer for visualization
    print("Setting up OpenRAVE viewer ('qtcoin')...")
    viewer = env.SetViewer('qtcoin')
    if not viewer:
        print("Note: 'qtcoin' viewer returned False/None. Trying 'qtosg'...")
        viewer = env.SetViewer('qtosg')

    if env.GetViewer() is not None:
        print("Viewer successfully initialized: {}".format(env.GetViewer()))
    else:
        print("Warning: Could not initialize graphical viewer. Ensure DISPLAY ({}) is reachable.".format(os.environ.get("DISPLAY")))

    # Initialize HSR robot using the HSR class from Robots.py
    print("Initializing HSR robot (id=1)...")
    hsr = HSR(env=env, id=1)
    print("HSR robot initialized successfully!")

    # 1. Start robot in tucked arm pose using hsr.tuck_arm()
    print("\n[Step 1] Tucking HSR arm...")
    hsr.tuck_arm()
    env.UpdatePublishedBodies()
    time.sleep(1.0)

    tucked_dofs = hsr.get_active_dof_values()
    print("Current DOF values (tucked): {}".format(tucked_dofs))

    # 2. Define target point in 3D space
    # Obtain a guaranteed reachable target point and orientation for hand_palm_link
    # using an outstretched reaching posture:
    sample_reach_angles = [0.20, -0.60, 0.0, -0.60, 0.0]
    hsr.set_active_dof_values(sample_reach_angles)
    target_transform = np.array(hsr.get_link_transform("hand_palm_link"))
    target_point = target_transform[:3, 3]

    print("\n[Step 2] Target in 3D Space:")
    print("  Target Point (x, y, z): [{:.3f}, {:.3f}, {:.3f}]".format(
        target_point[0], target_point[1], target_point[2]))

    # Reset arm back to tucked pose before solving IK
    hsr.tuck_arm()
    env.UpdatePublishedBodies()
    time.sleep(1.0)

    # Draw coordinate axes marker at target point in the viewer
    marker = None
    try:
        marker = openravepy.misc.DrawAxes(env, target_transform, dist=0.15)
        print("  Target coordinate marker drawn in 3D viewer.")
    except Exception as e:
        print("  Could not draw target marker: {}".format(e))

    # 3. Solve IK using hsr.get_ik_solutions()
    print("\n[Step 3] Calling hsr.get_ik_solutions()...")
    ik_solution = hsr.get_ik_solutions(target_transform)

    if ik_solution is None or len(ik_solution) == 0:
        print("[ERROR] hsr.get_ik_solutions() could not find a solution for target.")
        return

    print("IK Solution found successfully!")
    print("  Joint names: {}".format(hsr.jointnames))
    print("  Solved DOF values: {}".format(ik_solution))

    # 4. Move robot to target point using hsr.set_active_dof_values()
    print("\n[Step 4] Moving HSR to target point...")
    start_dofs = list(hsr.get_active_dof_values())
    target_dofs = list(ik_solution)

    num_steps = 50
    for step in range(1, num_steps + 1):
        alpha = float(step) / float(num_steps)
        interp_dofs = [
            (1.0 - alpha) * start_dofs[j] + alpha * target_dofs[j]
            for j in range(len(start_dofs))
        ]
        hsr.set_active_dof_values(interp_dofs)
        env.UpdatePublishedBodies()
        time.sleep(0.04)

    print("Movement complete!")

    # 5. Verify reaching accuracy
    final_transform = hsr.get_link_transform("hand_palm_link")
    final_point = final_transform[:3, 3]
    position_error = np.linalg.norm(final_point - target_point)

    print("\n[Step 5] Verification:")
    print("  Actual End-Effector Point: [{:.3f}, {:.3f}, {:.3f}]".format(
        final_point[0], final_point[1], final_point[2]))
    print("  Target Point:             [{:.3f}, {:.3f}, {:.3f}]".format(
        target_point[0], target_point[1], target_point[2]))
    print("  Position Error:           {:.6f} m".format(position_error))

    # 6. Gripper interaction using hsr.openGripper()
    print("\n[Step 6] Opening gripper at target...")
    hsr.openGripper()
    env.UpdatePublishedBodies()

    print("\n" + "=" * 60)
    print("Example finished successfully.")
    print("Check noVNC (http://localhost:8080/vnc.html) to view the result.")
    print("Press Enter in this terminal to exit...")
    print("=" * 60)

    try:
        try:
            input_func = raw_input
        except NameError:
            input_func = input

        env.UpdatePublishedBodies()
        input_func()
    except (KeyboardInterrupt, EOFError):
        pass

    print("Cleaning up environment...")
    env.Destroy()
    print("Done!")

if __name__ == "__main__":
    main()
