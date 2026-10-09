import os
import sys
import time
import numpy as np

# Ensure repository root and simulator directories are in sys.path
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

OPENRAVE_DIR = os.path.join(REPO_ROOT, "simulators", "openrave")
if OPENRAVE_DIR not in sys.path:
    sys.path.insert(0, OPENRAVE_DIR)

from Config import Config
import useful_functions
from simulators.openrave.sim_utils import plot_transform


def main():
    Config.declare_config("CafeWorld","HSR")
    SimClass = Config.get_simulator_module("SimClass").SimClass
    Object = Config.get_simulator_module("Object").Object
    model_gen_utils = Config.get_simulator_module("model_gen_utils")

    # Initialize OpenRAVE simulator with visualization enabled for noVNC viewer
    sim_object = SimClass(robot_name="HSR", visualize=True)

    # Spawn table near the origin (parameters matching CafeWorld.spawn_table)
    thickness = 0.1
    legheight = 0.55
    table_color = [1, 0.8, 0]
    table_pose = [0.0, 0.0, 0.63]
    table_name = "surface_1"

    table = Object(model_gen_utils.create_cafe_table(
        sim_object.env,
        table_name,
        0.90,       # dim1
        0.90,       # dim2
        thickness,  # thickness
        0.1,        # legdim1
        0.1,        # legdim2
        legheight,  # legheight
        table_pose, # pose
        table_color # color
    ))
    sim_object.add_obj(table)

    # Spawn can on top of the table near the origin (parameters matching CafeWorld.spawn_object)
    can_radius = 0.045
    can_height = 0.2
    can_color = [0, 0.8, 1]
    can_name = "can_1"
    clearance = 0.005

    # Position can resting on top of the table surface
    z_on_table = thickness / 2.0 + clearance
    rel_transform = useful_functions.transform_from_sevend_pose([1, 0, 0, 0, 0.4, 0.0, z_on_table])
    can_transform = table.get_transform().dot(rel_transform)

    can = Object(model_gen_utils.create_cylinder(
        sim_object.env,
        can_name,
        can_transform,
        [can_radius, can_height],
        can_color
    ))
    sim_object.add_obj(can)

    # spawning in the HSR bot
    # setting the pose to tucked
    sim_object.robot.tuck_arm()
    # edit the robot's transformation to spawn at a far away location
    robot_t = useful_functions.transform_from_sevend_pose([1,0,0,0,5,5,0])
    sim_object.robot.set_transform(robot_t)

    sim_object.env.UpdatePublishedBodies()

    # walking over to the table
    # this pose is sitting near the +X side of the table and rotate 180 degrees to face the table
    target_base_pose = [0.85,0.08,np.pi]
    
    # computing the plan
    sim_object.robot.activate_base_joints()
    base_traj = sim_object.compute_motion_plan(target_base_pose)

    # animation for moving over to the table
    if base_traj is not None:
        for i in range(base_traj.GetNumWaypoints()):
            wp = base_traj.GetWaypoint(i)
            sim_object.robot.set_active_dof_values(wp)
            
            # Update the visual axes with the robot as it moves
            gripper_t = sim_object.robot.gripper_link.GetTransform()
            sim_object.env.UpdatePublishedBodies()
            time.sleep(0.02)

    # Plot coordinate axes for the HSR robot's gripper (Red=X, Green=Y, Blue=Z)
    gripper_t = sim_object.robot.gripper_link.GetTransform()

    # now we need to find the pose for the grasp (this is based on the sample_grasp method)
    # first get the pose of the can wrt to the world
    world_T_object = sim_object.get_obj("can_1").get_transform()
    world_T_robot = sim_object.robot.get_link_transform("hand_palm_link")

    gripper_offset = sim_object.robot.grasping_offset["can"]
    object_T_gripper = useful_functions.transform_from_sevend_pose([1, 0, 0, 0, -0.06, 0, can_height/2.0])
    object_T_gripper = np.matmul(useful_functions.rotmat_from_rotvec([0,0,np.pi]),object_T_gripper)

    robot_T_object = np.matmul(np.linalg.inv(world_T_robot), world_T_object)

    t1 = useful_functions.rotmat_from_rotvec([0, np.pi/2, 0])
    t2 = useful_functions.rotmat_from_rotvec([0, 0, np.pi])

    # get the grasp pose which is the combination of the can wrt world and the gripper wrt can
    grasp_T = np.matmul(np.matmul(world_T_object, object_T_gripper),np.matmul(t1,t2))

    # gen the ik solutions and see if its possible
    ik_sols = sim_object.robot.get_ik_solutions(grasp_T)

    if len(ik_sols) > 0:
        # generate the trajectory
        arm_traj = sim_object.compute_motion_plan(ik_sols)

        # animating the motion
        if arm_traj is not None:
            for i in range(arm_traj.GetNumWaypoints()):
                sim_object.robot.set_active_dof_values(arm_traj.GetWaypoint(i))
                sim_object.env.UpdatePublishedBodies()
                time.sleep(0.02)

    # grabbing the can
    taskmanip = sim_object.robot.interfaces.TaskManipulation(sim_object.robot.obj)
    with sim_object.robot.obj:
        taskmanip.CloseFingers()
    sim_object.robot.obj.WaitForController(0)
    sim_object.env.UpdatePublishedBodies()

    sim_object.robot.grab("can_1")
    sim_object.env.UpdatePublishedBodies()
    time.sleep(0.5)

    # raise the arm up a little so that it does collide with the table
    current_dofs = list(sim_object.robot.get_active_dof_values())
    current_dofs[0] = min(0.65, current_dofs[0] + 0.10)
    sim_object.robot.set_active_dof_values(current_dofs)
    sim_object.env.UpdatePublishedBodies()
    time.sleep(0.5)


    sim_object.robot.activate_base_joints()

    # move the robot again away
    new_base_pose = [2.0, 1.5, -np.pi]
    base_traj = sim_object.compute_motion_plan(new_base_pose)
    if base_traj is not None:
        for i in range(base_traj.GetNumWaypoints()):
            sim_object.robot.set_active_dof_values(base_traj.GetWaypoint(i))
            sim_object.env.UpdatePublishedBodies()
            time.sleep(0.02)


    print("Spawned {} and {} around origin.".format(table_name, can_name))
    print("Viewer active. Press Ctrl+C or Enter to exit...")

    try:
        try:
            raw_input()
        except NameError:
            input()
    except (KeyboardInterrupt, EOFError):
        pass


if __name__ == "__main__":
    main()
