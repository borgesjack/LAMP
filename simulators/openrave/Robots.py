import numpy as np
import time
import os
import sys
from trac_ik_python.trac_ik import IK
from useful_functions import blockPrinting, sixd_pose_from_transform, transform_from_sixd_pose, get_relative_transform

def get_parent_with_file(file_name):
    current_dir = os.getcwd()
    while True:
        if file_name in os.listdir(current_dir):
            return current_dir
        else:
            current_dir = os.path.abspath(os.path.join(current_dir, os.pardir))
    
    raise Exception("File Not Found in any Parent")

ROOT_DIR = get_parent_with_file("__init__.py")
if ROOT_DIR is not None and ROOT_DIR not in sys.path:
    sys.path.append(ROOT_DIR)

from SimConfig import SimConfig
from Simulator import Simulator
from Object import Object

sim = Simulator()

class Robot(Object): 
    # env: openravepy.Environment() object representing the whole 3D space
    # id: id value for the robot (hard set to 1 in SimClass.py)
    # real_world_exp: flag that determines if 
    def __init__(self,env,id,real_world_exp=False):
        # call to parent class object.py setting the current openrave obj to None
        # self.obj: the openrave object representing the robot
        super(Robot,self).__init__(obj=None)

        # attribute that determines if the robot has real world 
        # experience (likely dont need to worry abt)
        self.real_world_exp = real_world_exp
        # attr for the environment for the domain
        self.env = env
        # set to 1 from SimClass 
        self.id = id
        # base object has no initial pose?
        self.init_pose = []

        # these use setattr bc they have dynamic names
        setattr(self,"grabbed_flag_{}".format(self.id),False)
        setattr(self,"grabbed_object_{}".format(self.id),None)

    # method to receive the initial DOF (Degree of Freedom Values)
    # DOF Values: [x,y,z,]
    def get_init_pose(self):
        self.init_pose = self.obj.GetActiveDOFValues()
    
    # method to set the initial pose by entering the given initial DOF values
    # calls to function in robot class that then calls to obj
    def set_init_pose(self):
        self.set_active_dof_values(self.init_pose)


    def grab(self,obj=None,arm=None,delta_mp=False):
        # check to make sure that an openrave object is linked else it wont work
        if obj is not None:
            # get the object from environment with the name of object
            o = self.env.GetKinBody(obj)

            # gets the transform or pose of the gripper link
            robot_t = self.gripper_link.GetTransform()

            # gets the pose for whatever the object's base link is
            ot = o.GetTransform()

            # finds the straight line distance between the gripper and obj
            euclidean_distance = np.linalg.norm(robot_t[:3,3]-ot[:3,3])

            # finds the object type
            # ex: obj = can_1 -> [can,1] -> obj_type_ind = 0 -> can
            obj_type = obj.split("_")[SimConfig.OBJ_TYPE_IND]

            # queries the grab range for the given type of object
            # this is defined in each kind of robot's __init__ method
            # this is defined for every kind of object it could pick up
            grab_range = self.grab_range[obj_type]
            
            # if flag is set to grab everything regardless
            # or grab if it is in the distance range that is defined
            if delta_mp or \
               (euclidean_distance<grab_range[1] and euclidean_distance>grab_range[0]):
                # openRAVE call to grab object
                self.obj.Grab(o)
                # set the grabbed object flags as true and set the object that is
                setattr(self,"grabbed_flag_{}".format(self.id),True) 
                setattr(self,"grabbed_object_{}".format(self.id),obj)
            else:
                # make sure that the item is not grabbed
                # show that it is not in range
                setattr(self,"grabbed_flag_{}".format(self.id),False) 
                print("object out of grasp range")

    # 
    def release(self,arm=None):
        # call to openRAVE to release all obj (assumes only one can be grabbed)
        # reset flags
        self.obj.ReleaseAllGrabbed()
        setattr(self,"grabbed_flag_{}".format(self.id),False) 
        setattr(self,"grabbed_object_{}".format(self.id),None)

    # does nothing?
    def activate_base_joints(self):
        pass
    
    # defined in the child class
    def activate_manip_joints(self):
        pass
    
    def activate_body(self):
        pass

    # wrapper method to call openRAVE API
    def set_active_dof_values(self,values):
        self.obj.SetActiveDOFValues(values)

    # wrapper method to call openRAVE API
    def get_active_dof_values(self):
        return self.obj.GetActiveDOFValues()
    
    # wrapper method to call openRAVE API
    def get_robot_transform(self):
        return self.obj.GetTransform()

    # return whatever is set as the grabbed object for the object
    def get_grabbed_object(self):
        return getattr(self,"grabbed_object_{}".format(self.id))

    # converts the string name of a link to the link object in the openRAVE obj
    def get_link(self,link_name):
        return self.obj.GetLink(link_name)

    # get the link (checks to make sure that the type of link is an openRAVE type)
    # accepts str or openRAVE link obj
    def get_link_transform(self,link):
        if type(link) == str:
            link = self.get_link(link)
        
        return link.GetTransform()
    
    # returns all of the links for an obj
    def get_links(self):
        return self.obj.GetLinks()

    #
    def random_config_robot(self,current_dof=None,collision_fn=None,limits=None,use_collision_set=False,**kwargs):
        # checks to make sure no pose is passed
        # if so take the number of dof that are current else take the amount passed
        if current_dof is None:
            current_active_dof = self.obj.GetActiveDOF()
            current_dof = self.get_active_dof_values()
        else:
            current_active_dof = len(current_dof)

        # if it is 3 that means it is the base, in which case get that pose
        # otherwise it must be something else so get the pose for the gripper
        if current_active_dof == 3:
            self.activate_base_joints()
            t = self.get_link("base_link").GetTransform()
        # this might need to be edited later for the HSR to work but a good start for accommodating the activate_body method
        else:
            self.activate_manip_joints()
            t = self.gripper_link.GetTransform()

        # if there are limits for the poses of the robot set them
        # otherwise just take what is currently set for the obj
        if limits is None:
            lower_limits,upper_limits = self.obj.GetActiveDOFLimits()
        else:
            lower_limits,upper_limits = limits
        

        while True:
            random_dof_values = []
            for i in range(current_active_dof):
                # a range offset is applied so the robot does not have too extreme of a pose
                range_offset = abs(upper_limits[i]-lower_limits[i])/6.0
                # generate a random value for the pose from a uniform distr
                r = np.random.uniform(lower_limits[i]+range_offset,upper_limits[i]-range_offset)
                random_dof_values.append(r)
            
            # if the obj is not current grabbing anything
            if getattr(self,"grabbed_object_{}".format(self.id)) is None:
                # if the robot is colliding with anything break the loop
                if not Robot.check_collision(self,random_dof_values,collision_fn,use_collision_set):
                    break
            else:
                grabbed_object = self.env.GetKinBody(getattr(self,"grabbed_object_{}".format(self.id)))
                if not Robot.check_collision(self,random_dof_values,collision_fn,use_collision_set):
                    break
        
        # set the active dof values to either what was passed or pulled
        self.set_active_dof_values(current_dof)

        # return the list of the random_dof_values alongside the current pose matrix
        return random_dof_values, t

    # get the attribute from this object if it exists, otherwise pass to parent class
    # in this case, parent class is the openRAVE obj class
    def __getattr__(self, name):
        if hasattr(self.obj,name) and callable(getattr(self.obj,name)):
            return getattr(self.obj,name)
        else:
            return super(Robot,self).__getattr__(name)


    @staticmethod
    def check_collision(robot, solution, collision_fn,use_collision_set=False):  
        # get the current DOF values count
        cdof = robot.GetActiveDOFValues() 

        # try to set the passed values (solution) as the DOF values
        try:
            robot.SetActiveDOFValues(solution)
        # if there is an exception, return true
        except:
            return True
        
        # blank list for all objects in environment
        env_objects = []

        # if you are not using a defined set, get all of the objects in the environment
        # for testing
        if not use_collision_set:
            env_objects = [Object(o) for o in robot.GetEnv().GetBodies()]
        
        # try all of the objects in the collision function if there is a collision
        # return true
        collision = collision_fn(env_objects,use_collision_set) 
        try:
            robot.SetActiveDOFValues(cdof)
        except:
            return True
        
        # true if there was collision
        return collision

    # end_effector_solution is the matrix representing where the robot wants to do
    # this method converts it to the 6d matrix and makes sure that there are 
    # no collisions
    def get_ik_solutions(self,end_effector_solution,check_collisions=True,robot_param=None,collision_fn = None):
        
        # turns the 4x4 matrix from openRAVE into a 6D vector [x,y,z,rx,ry,rz]
        solution = sixd_pose_from_transform(end_effector_solution)

        # if there is a collision function that exists and flag
        # check if there are any collisions going to that location
        if collision_fn is not None and check_collisions: 
            if Robot.check_collision(self.obj, solution, collision_fn):
                return [ ]
        return solution

    # for child method def
    def openGripper(self):
        pass
    
    # for child method def
    def closeGripper(self):
        pass

class MagicGripper(Robot):
    def __init__(self,env,id,collection=False,real_world_exp=False):
        super(MagicGripper,self).__init__(env,id,real_world_exp)

        self.obj_name = "gripper_{}".format(id)
        self.obj = self.load_robot()

        self.obj_offset = 0.07
        self.grab_range = {
            "plank": [0,0.15]
        }
        self.collection_flag = False

        finger_length = 0.02708
        finger_name = "rfinger"
        finger_wrist_T = get_relative_transform(self.obj.GetLink("base").GetTransform(),
                                                self.obj.GetLink(finger_name).GetTransform())

        self.geometry_limits = {
            "gripper": (2,[finger_wrist_T[2,3]-finger_length/2.0,finger_wrist_T[2,3]+finger_length/2.0]), 
            "base": (0,[0])
        }

        self.robot_type_object_mappings = {"base":"gripper_{}".format(self.id)}

        self.activate_manip_joints(collection_flag=collection)
        self.grasping_offset = {"plank": -0.135}
        self.gripper_link = self.obj
        self.ik_link = self.gripper_link

    def load_robot(self):
        self.env.Load(SimConfig.ROB_DIR+'MagicGripper/robot.xml')
        robot = self.env.GetRobot("gripper")
        robot.SetName(self.obj_name)
        return robot

    def activate_manip_joints(self,collection_flag=False):
        self.obj.SetActiveDOFs([],sim.DOFAffine.X|sim.DOFAffine.Y|sim.DOFAffine.Z|sim.DOFAffine.Rotation3D)
        self.obj.SetAffineRotation3DLimits(np.array([-np.pi]*3),np.array([np.pi]*3))

        if not self.collection_flag and collection_flag:
            self.collection_flag = collection_flag

        if self.collection_flag:
            self.obj.SetAffineTranslationLimits(np.array([-1,-1,-1]),np.array([1,1,2]))
        else:
            self.obj.SetAffineTranslationLimits(np.array([-100,-100,-5]),np.array([100,100,5]))

class ImageGripper(Robot):
    def __init__(self,env,id,collection=False,real_world_exp=False):
        super(ImageGripper,self).__init__(env,id,real_world_exp)

        self.obj_name = "gripper_{}".format(id)
        self.obj = self.load_robot()

        self.obj_offset = 0.07
        self.grab_range = {
            "plank": [0,0.15]
        }
        
        finger_length = 0.018
        finger_name = "rfinger"
        finger_wrist_T = get_relative_transform(self.obj.GetLink("base").GetTransform(),
                                                self.obj.GetLink(finger_name).GetTransform())

        self.geometry_limits = {
            "gripper": (2,[finger_wrist_T[2,3]-finger_length/2.0,finger_wrist_T[2,3]+finger_length/2.0]), 
            "base": (0,[0])
        }

        self.robot_type_object_mappings = {"base":"gripper_{}".format(self.id)}

        self.activate_manip_joints()
        self.grasping_offset = {"plank": -0.065}
        self.gripper_link = self.obj
        self.ik_link = self.gripper_link

    def load_robot(self):
        self.env.Load(SimConfig.ROB_DIR+'MagicGripper/robot_images.xml')
        robot = self.env.GetRobot("gripper")
        robot.SetName(self.obj_name)
        return robot
    
    def activate_manip_joints(self):
        self.obj.SetActiveDOFs([],sim.DOFAffine.X|sim.DOFAffine.Y|sim.DOFAffine.Z|sim.DOFAffine.Rotation3D)
        self.obj.SetAffineRotation3DLimits(np.array([-np.pi]*3),np.array([np.pi]*3))
        self.obj.SetAffineTranslationLimits(np.array([-1.5,-1.5,-2]),np.array([1.5,1.5,2]))

class MagneticGripper(Robot):
    def __init__(self,env,id,collection=False,real_world_exp=False):
        super(MagneticGripper,self).__init__(env,id,real_world_exp)

        self.name = "gripper_{}".format(id)
        self.obj = self.load_robot()

        self.obj_offset = 0.07
        self.grab_range = {
            "can": [0,0.24]
        }

        self.robot_type_object_mappings = {"base":"gripper_{}".format(self.id)}
        self.gripper_link = self.obj
        self.ik_link = self.gripper_link

        self.geometry_limits = {
            "gripper": (2,[-0.20,-0.22]), #TODO: needse to be the actual grasp offset between pose of object and gripper frame
            "base": (0,[0])
        }

        self.activate_manip_joints()
        self.grasping_offset = {"plank": -0.07,
                                "can": 0.21}

    def load_robot(self):
        self.env.Load(SimConfig.ROB_DIR+'MagneticGripper/robot.xml')
        robot = self.env.GetRobot("gripper")
        robot.SetName(self.name)
        return robot

    def activate_manip_joints(self):
        self.obj.SetActiveDOFs([],sim.DOFAffine.X|sim.DOFAffine.Y|sim.DOFAffine.Z|sim.DOFAffine.Rotation3D)
        self.obj.SetAffineRotation3DLimits(np.array([-np.pi]*3),np.array([np.pi]*3))
        self.obj.SetAffineTranslationLimits(np.array([-1.5,-1.5,-2]),np.array([1.5,1.5,2]))

class HSR(Robot):
    def __init__(self,env,id,collection=False,real_world_exp=False):
        # call to the parent class of Robot
        super(HSR,self).__init__(env,id,real_world_exp)
        
        # this is a list of names that comes from SRDF file
        # these should only be the joints that impact the arm movements
        self.jointnames = (["arm_lift_joint", "arm_flex_joint", "arm_roll_joint",
                            "wrist_flex_joint", "wrist_roll_joint"])
        
        # currently ignoring the case of DinnerTable
        # these values are taked from the go group state in the srdf file
        self.grabbed_armTuckDOFs = [0,0,-1.57,-1.57,0]

        # this automatically fetches the urdf and srdf files for HSR
        # would work the same for any robot class
        self.hsr_urdf = SimConfig.URDF_DIR + "hsrb4s.urdf"
        self.hsr_srdf = SimConfig.URDF_DIR + "hsrb.srdf"

        # this method gets the openRAVE object for the Fetch bot
        self.obj = self.load_robot()

        # this sets which joint is going to be manipulated currently
        # this is only for IK
        self.obj.SetActiveManipulator("eef")

        # this sets a maximum velocity for the base to move in the simulation in [x,y,z] directions
        # done in m/s
        self.obj.SetAffineTranslationMaxVels([10.5, 10.5, 10.5])

        # this sets the max angular velocity for the base, rad/s
        # the rotation is described as [max_mag, av_x, av_y, av_z]
        self.obj.SetAffineRotationAxisMaxVels(np.ones(4))

        # this is the spawn transformation matrix (spawn location and settings)
        # creates a matrix that represents obj at [0,0,0,1,1,1]
        robot_spawn_t = np.eye(4)
        # sets the z value to -.025
        robot_spawn_t[0,3] -= 0.025
        # this sets the base transform to the above matrix
        self.obj.SetTransform(robot_spawn_t)

        # get the number of DOF for every join
        self.obj.SetActiveDOFs([self.obj.GetJoint(name).GetDOFIndex() for name in self.jointnames])
        
        # self.obj.SetActiveDOFValues(np.asarray(self.armTuckDOFs))
        # set all of the DOF values to what was defined above in gra
        self.obj.SetActiveDOFValues(np.asarray(self.grabbed_armTuckDOFs))

        # manip_joints: list of active joint index numbers [0,1,2,3,4,5,6,6,7]
        self.manip_joints = self.obj.GetActiveDOFIndices()

        # init attr for gripper
        # gripper is defined in the SRDF file as a group
        self.initGripper()

        # opens the fingers of the robot
        # would want to do this now so that the robot has open hand default
        self.openGripper()

        # this takes all of the changes that are not on the GUI and pushes them to the GUI
        # GUI does not automatically update alongside changes internally, they need to be specifically updated
        # right now this means that the robot is now on screen with their fingers open
        self.env.UpdatePublishedBodies()

        # get the name attr of the object
        # format decided earlier as hsrb_id
        self.obj_name = self.obj.GetName()

        # gripper: the end of the robot that allows grabbing
        # this maps the URDF links to the high level meaning of the links
        # you need to see manually which links correspond to these different attributes
        self.robot_type_object_mappings = {"base_link":"freight_{}".format(self.id),
                                           "hand_palm_link":"gripper_{}".format(self.id)}

        # defining these links inside of the robot obj itself
        self.gripper_link = self.obj.GetLink("hand_palm_link")
        self.base_link = self.obj.GetLink("base_link")

        # calculate max extension by getting the distance 
        # between the base of the hand and the tip of the finger
        finger_wrist_T = get_relative_transform(self.obj.GetLink("hand_palm_link").GetTransform(), 
                               self.obj.GetLink("hand_r_finger_tip_frame").GetTransform())

        # limits of how far the gripper and the freight can move
        # (axis, [min,max])
        #(axis, [radius])
        # this might be wrong
        self.geometry_limits = {
            # finger_wrist_T is the Z coord distance between wrist and fingertip
            # distance can be from 0 to wrist to fingertip distance + buffer
            # 0.07 in tis case is a buffer value that takes into account the physical base and a safety buffer (have to figure out manually)
            "gripper": (2,[0,finger_wrist_T[2,3]+ 0.07]), #TODO: make it for each object 
            "freight": (0,[0.560])
        }

        # these are manually calced for each obj
        # supposedly calced by doing extension length+obj diameter+grasp margin
        self.grab_range = {
                            "can":[0,0.26],
                            "glass": [0,0.22],
                            "bowl": [0,0.28],
                            "jenga": [0,0.22] 
                        }
        # this is how far away from the object before doing final straight line motion
        self.obj_offset = 0.15
        
        # this is an offset from the depth of an object so that the gripper has the most control over the obj
        # this is best calculated by taking the center of mass vs the origin point of the object
        # you want to grab objects at their center of mass
        self.grasping_offset = {"can": -0.04,
                                "glass": -0.015,
                                "bowl": 0.24,
                                "jenga": -0.04
                                }

        # set an attribute for the gripper_link for doing IK
        self.ik_link = self.gripper_link

        # open the URDF file for fetch and read it in as a string
        self.urdf_string = None
        with open(self.hsr_urdf,"r") as f:
            self.urdf_string = f.read()

        # this gives us virtual joint values for the base so that the IK calculation can use them
        virtual_base_xml = """
            <link name="world_base"/>
            <link name="virtual_base_x_link"/>
            <joint name="virtual_base_x_joint" type="prismatic">
                <origin xyz="0 0 0" rpy="0 0 0"/>
                <parent link="world_base"/>
                <child link="virtual_base_x_link"/>
                <axis xyz="1 0 0"/>
                <limit lower="-40.0" upper="40.0" effort="100.0" velocity="1.0"/>
            </joint>
            <link name="virtual_base_y_link"/>
            <joint name="virtual_base_y_joint" type="prismatic">
                <origin xyz="0 0 0" rpy="0 0 0"/>
                <parent link="virtual_base_x_link"/>
                <child link="virtual_base_y_link"/>
                <axis xyz="0 1 0"/>
                <limit lower="-40.0" upper="40.0" effort="100.0" velocity="1.0"/>
            </joint>
            <joint name="virtual_base_yaw_joint" type="revolute">
                <origin xyz="0 0 0" rpy="0 0 0"/>
                <parent link="virtual_base_y_link"/>
                <child link="base_footprint"/>
                <axis xyz="0 0 1"/>
                <limit lower="-3.14159" upper="3.14159" effort="100.0" velocity="1.0"/>
            </joint>
            </robot>"""
        augmented_urdf = self.urdf_string.replace("</robot>", virtual_base_xml)

        # the IK solvers for the given object
        # IK(base,tip,URDF)
        # it traces out all of the joints intermediate using the tree of joints
        # URDF gives the solver all of the mathematical detail for solving
        # ik_solver: an ik solver specifically for solving this
        self.ik_solver = IK("world_base","hand_palm_link",urdf_string=augmented_urdf)
    
    # this sets the limits for movement to whatever limits dictates
    def set_translation_limits(self,limits):
        self.activate_base_joints()
        # limits are [minx,miny,0] and [maxx, maxy, 0]
        self.obj.SetAffineTranslationLimits(np.array(limits[0] + [0]),np.array(limits[1] + [0]))

    # this method returns an openRAVE robot object
    # this is what is loaded into the robot parent class of the specific robot
    def load_robot(self):
        # sed -i: this means edit the file in place
        # "s|project_directory": s means substitute (find and replace)
        # | is the delimiter
        # project_directory: the thing you are searcing for inside of the urdf file
        # SimConfig.ROOT_DIR[:-1]: the root of the repo without the trailing \, this is what is replacing project_directory
        # g: global so it is done on all occurences
        # SimConfig.SIM_ROOT_DIR: this is the directory of the simulator folder, exists so you can append correct dir next
        sedstr = "sed -i \"s|project_directory|"+SimConfig.ROOT_DIR[:-1]+"|g\" " + SimConfig.SIM_ROOT_DIR
        # this line executes the above string with the suffix of the relative URDF dir
        # that suffix determines which file you are editing
        # the whole goal of this is to replace all of the project_directory instances in the URDF config
        # with the correct file path
        os.system(sedstr + SimConfig.REL_URDF_DIR +"hsrb4s.urdf")

        # create a urdf cpp openRAVE API module
        module = self.RaveCreateModule(self.env, 'urdf')

        # using with self.env in order to lock the env while changes are being made
        # with openRAVE modules, in order to use the functions that are required, you need to do sendcommand
        # loadURI: reads the URDF and SRDF files and created an object with the associated attributes
        # loadURI returns teh name of the robot
        # this robot that is created is placed within self.env
        with self.env:
            name = module.SendCommand('loadURI ' + self.hsr_urdf+" "+self.hsr_srdf)
        sedstr = "sed -i \"s|"+SimConfig.ROOT_DIR[:-1]+"|project_directory|g\" " + SimConfig.SIM_ROOT_DIR
        os.system(sedstr + SimConfig.REL_URDF_DIR +"hsrb4s.urdf")
        
        # this method fetches the robot object that was just created
        # the odom_x and other joints that were defined in the srdf was not in urdf
        robot = self.env.GetRobot(name)
        # name is set to Fetch_id
        robot.SetName("{}_{}".format(name,self.id))

        return robot

    def initGripper(self):
        """Setup gripper closing direction and tool direction """
        # gripperManip: joint that is set as the manipulator
        gripperManip = self.obj.GetActiveManipulator()

        # gripperIndices: list of indicies for gripper finger joints
        gripperIndices = gripperManip.GetGripperIndices()

        # closingDirection: list of values size of gripperIndicies that has closing dir
        # init to zero as std practice
        closingDirection = np.zeros(len(gripperIndices))

        # sets the closing direction for each joint to -1
        for i, gi in enumerate(gripperIndices):
            closingDirection[i] = -1.

        # chucking: robotics term for clamping
        # set the clamp direction for both joints to the closing direction
        gripperManip.SetChuckingDirection(closingDirection)

        # this is the approach vector of the hand
        # in this case, the hand approaches directly on the x axis
        gripperManip.SetLocalToolDirection([1, 0, 0])

    def openGripper(self):
        # taskmanip is an object that allows for high level function calls for movement
        taskmanip = self.interfaces.TaskManipulation(self.obj)
        # lock the object while it is moving
        # ReleaseFingers: takes finger joints in the oppo chuck dir until they hit max
        with self.obj:
            taskmanip.ReleaseFingers()
        # tells openrave to wait until the movement is 100% completed and the robot is stationary
        self.obj.WaitForController(0)

    # method that: releases the object, activates the arm, sets the arm pose to the tucked pose
    def tuck_arm(self):
        self.release()
        self.activate_manip_joints()
        # self.obj.SetActiveDOFValues(np.asarray(self.armTuckDOFs))
        self.obj.SetActiveDOFValues(np.asarray(self.grabbed_armTuckDOFs))

    # method to make the robot move across the floor (using the base)
    def activate_base_joints(self):
        self.obj.SetActiveDOFs([], sim.DOFAffine.X | sim.DOFAffine.Y | sim.DOFAffine.RotationAxis)

    # method for activating the arm
    def activate_manip_joints(self):
        self.obj.SetActiveDOFs([self.obj.GetJoint(name).GetDOFIndex() for name in self.jointnames])

    def activate_body(self):
        self.obj.SetActiveDOFs([self.obj.GetJoint(name).GetDOFIndex() for name in self.jointnames], sim.DOFAffine.X | sim.DOFAffine.Y | sim.DOFAffine.RotationAxis)
    
    # blockPrinting: turns off all of the cpp terminal output from openRAVE

    def get_ik_solutions(self,end_effector_solution,check_collisions=True,robot_param="gripper",collision_fn = None):
        solutions = []

        # in the case you are moving that base, the DOF you have to set are the
        # physical position so x,y,rotation so just pull those from the end solution
        # only caviat is that you have to check for collision
        if robot_param == SimConfig.BASE_NAME:
            _x = end_effector_solution[0,3]
            _y = end_effector_solution[1,3]
            _yaw = self.axisAngleFromRotationMatrix(end_effector_solution[:3,:3])[-1]
            solutions = [_x,_y,_yaw]
            self.activate_base_joints()
            if collision_fn is not None and check_collisions: 
                if Robot.check_collision(self.obj, solutions, collision_fn):
                    return [ ]
        
        else:
            self.activate_body()
            current_state = self.obj.GetActiveDOFValues()
            collision = check_collisions
            
            # this is doing (baselink global pos)^-1 * end solution
            # this calculates the robot end position wrt to the base of the robot
            required_T = np.linalg.pinv(self.base_link.GetTransform()).dot(end_effector_solution)
            # calls the function to get T to be in form [x,y,z,rx,ry,rz]
            pose = sixd_pose_from_transform(end_effector_solution)
            # position value for the pose [x,y,z]
            pos = pose[:3]
            # this gets you the quaternion for the pose
            # [theta, qx,qy,qz]
            orn = sim.quatFromAxisAngle(pose[3:])

            ik_count = 0
            collision = True
            while collision:
                # this needs to be the same size as the dof
                lower_limits, upper_limits = self.obj.GetActiveDOFLimits()
                seed_state = np.random.uniform(lower_limits,upper_limits).tolist()
                seed_state = seed_state[5:] + seed_state[:5]
                # use the IK solver to compute what DOF values each joint should take to get the solution
                solutions = self.ik_solver.get_ik(seed_state,
                                                 pos[0], pos[1], pos[2],  # X, Y, Z
                                                 orn[1], orn[2], orn[3], orn[0],  # QX, QY, QZ, QW
                                                 0.001, 0.001, 0.001, # x y z tols
                                                 0.01, 0.01, 0.01 # angle tolerances
                                                 )
                # convert the solutions to be arm, body
                if solutions is not None:
                    solutions = solutions[3:] + solutions[:3]
                ik_count += 1
                # check for collision and if the IK has ran too many times
                if ik_count<=SimConfig.MAX_IK_ATTEMPTS:
                    if solutions is not None:
                        collision = False
                        if collision_fn is not None and check_collisions: 
                            if Robot.check_collision(self.obj, solutions, collision_fn):
                                collision=True
                else:
                    print("max ik attempts exceeded")
                    solutions = []
                    break
            
            # set back to the original DOF values
            self.obj.SetActiveDOFValues(current_state)
            
        # returns list of DOF values that get to desired state
        return solutions
                
    def get_arm_tuck_dofs(self,object_name=None,pose = [],grasp_num=None,surface_id=None):
        # grabbed_flag = getattr(self,"grabbed_flag_{}".format(self.id))
        # if grabbed_flag:
        #     return self.grabbed_armTuckDOFs
        
        # return self.armTuckDOFs
        return self.grabbed_armTuckDOFs

class Fetch(Robot):
    def __init__(self,env,id,collection=False,real_world_exp=False):
        # call to the parent class of Robot
        super(Fetch,self).__init__(env,id,real_world_exp)
        
        # this is a list of names that comes from SRDF file
        self.jointnames = (["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                            "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"])


        # grabbed_armTuckDOFs: a list that contains joint values for each of the joins in jointnames
        # in the case of DinnerTable, the environment requires slightly different parameters, so these are set here
        if SimConfig.DOMAIN_NAME != "DinnerTable":
            self.grabbed_armTuckDOFs = [0, 1.32, 1.4, -0.2, 1.72, 0, 1.3599999999999999, 0.0]
        else:
            self.grabbed_armTuckDOFs = [0.15, 1.32, 1.4, -0.2, 1.72, -0.35, 1.3599999999999999, 0.0] 

        # this automatically fetches the urdf and srdf files for Fetch
        # would work the same for any robot class
        self.fetch_urdf = SimConfig.URDF_DIR + "Fetch.urdf"
        self.fetch_srdf = SimConfig.URDF_DIR + "Fetch.srdf"
        
        # this method gets the openRAVE object for the Fetch bot
        self.obj = self.load_robot()

        # this sets which joint is going to be manipulated currently
        # this is only for IK
        self.obj.SetActiveManipulator("arm_torso")

        # this sets a maximum velocity for the base to move in the simulation in [x,y,z] directions
        # done in m/s
        self.obj.SetAffineTranslationMaxVels([10.5, 10.5, 10.5])

        # this sets the max angular velocity for the base, rad/s
        # the rotation is described as [max_mag, av_x, av_y, av_z]
        self.obj.SetAffineRotationAxisMaxVels(np.ones(4))

        # this is the spawn transformation matrix (spawn location and settings)
        # creates a matrix that represents obj at [0,0,0,1,1,1]
        robot_spawn_t = np.eye(4)
        # sets the z value to -.025
        robot_spawn_t[0,3] -= 0.025
        # this sets the base transform to the above matrix
        self.obj.SetTransform(robot_spawn_t)

        # get the number of DOF for every join
        self.obj.SetActiveDOFs([self.obj.GetJoint(name).GetDOFIndex() for name in self.jointnames])
        
        # self.obj.SetActiveDOFValues(np.asarray(self.armTuckDOFs))
        # set all of the DOF values to what was defined above in gra
        self.obj.SetActiveDOFValues(np.asarray(self.grabbed_armTuckDOFs))

        # manip_joints: list of active joint index numbers [0,1,2,3,4,5,6,6,7]
        self.manip_joints = self.obj.GetActiveDOFIndices()

        # init attr for gripper
        # gripper is defined in the SRDF file as a group
        self.initGripper()

        # opens the fingers of the robot
        # would want to do this now so that the robot has open hand default
        self.openGripper()

        # this takes all of the changes that are not on the GUI and pushes them to the GUI
        # GUI does not automatically update alongside changes internally, they need to be specifically updated
        # right now this means that the robot is now on screen with their fingers open
        self.env.UpdatePublishedBodies()

        # get the name attr of the object
        # format decided earlier as Fetch_id
        self.obj_name = self.obj.GetName()

        # freight: the base of the robot
        # gripper: the end of the robot that allows grabbing
        # this maps the URDF links to the high level meaning of the links
        # you need to see manually which links correspond to these different attributes
        self.robot_type_object_mappings = {"base_link":"freight_{}".format(self.id),
                                           "wrist_roll_link":"gripper_{}".format(self.id)}

        # defining these links inside of the robot obj itself
        self.gripper_link = self.obj.GetLink("wrist_roll_link")
        self.base_link = self.obj.GetLink("base_link")

        # finger_lenght: constant that comes from the physical length, can be found through URDF
        finger_length = 0.06
        # hard-coded name of finger, needs to do through URDF
        finger_name = "r_gripper_finger_link"

        # this gets the pose of the finger wrt to the wrist
        # this is useful since the distance between the finger and wrist are fixed
        # that means that this needs to be used if the wrist moves the fingers also have the correct info
        finger_wrist_T = get_relative_transform(self.obj.GetLink("wrist_roll_link").GetTransform(),
                                                self.obj.GetLink(finger_name).GetTransform())

        # limits of how far the gripper and the freight can move
        # (axis, [min,max])
        #(axis, [radius])
        self.geometry_limits = {
            # finger_wrist_T is the X coord in the matrix, this represents the distance to the center of the finger link
            # finger_length/2 then gives you the distance to the base or the tip
            # 0.07 in tis case is a buffer value that takes into account the physical base and a safety buffer (have to figure out manually)
            "gripper": (0,[finger_wrist_T[0,3]-finger_length/2.0 + 0.07,finger_wrist_T[0,3]+finger_length/2.0 + 0.07]), #TODO: make it for each object 
            "freight": (0,[0.560])
        }

        # these are manually calced for each obj
        # supposedly calced by doing extension length+obj diameter+grasp margin
        self.grab_range = {
                            "can":[0,0.26],
                            "glass": [0,0.22],
                            "bowl": [0,0.28],
                            "jenga": [0,0.22] 
                        }
        # this is how far away from the object before doing final straight line motion
        self.obj_offset = 0.15
        
        # this is an offset from the depth of an object so that the gripper has the most control over the obj
        # this is best calculated by taking the center of mass vs the origin point of the object
        # you want to grab objects at their center of mass
        self.grasping_offset = {"can": -0.04,
                                "glass": -0.015,
                                "bowl": 0.24,
                                "jenga": -0.04
                                }

        # set an attribute for the gripper_link for doing IK
        self.ik_link = self.gripper_link

        # open the URDF file for fetch and read it in as a string
        self.urdf_string = None
        with open(self.fetch_urdf,"r") as f:
            self.urdf_string = f.read()

        # the IK solvers for the given object
        # IK(base,tip,URDF)
        # it traces out all of the joints intermediate using the tree of joints
        # URDF gives the solver all of the mathematical detail for solving
        # ik_solver: an ik solver specifically for solving this
        self.ik_solver = IK("base_link","wrist_roll_link",urdf_string=self.urdf_string)
    
    # this sets the limits for movement to whatever limits dictates
    def set_translation_limits(self,limits):
        self.activate_base_joints()
        # limits are [minx,miny,0] and [maxx, maxy, 0]
        self.obj.SetAffineTranslationLimits(np.array(limits[0] + [0]),np.array(limits[1] + [0]))

    # this method returns an openRAVE robot object
    # this is what is loaded into the robot parent class of the specific robot
    def load_robot(self):
        # sed -i: this means edit the file in place
        # "s|project_directory": s means substitute (find and replace)
        # | is the delimiter
        # project_directory: the thing you are searcing for inside of the urdf file
        # SimConfig.ROOT_DIR[:-1]: the root of the repo without the trailing \, this is what is replacing project_directory
        # g: global so it is done on all occurences
        # SimConfig.SIM_ROOT_DIR: this is the directory of the simulator folder, exists so you can append correct dir next
        sedstr = "sed -i \"s|project_directory|"+SimConfig.ROOT_DIR[:-1]+"|g\" " + SimConfig.SIM_ROOT_DIR
        # this line executes the above string with the suffix of the relative URDF dir
        # that suffix determines which file you are editing
        # the whole goal of this is to replace all of the project_directory instances in the URDF config
        # with the correct file path
        os.system(sedstr + SimConfig.REL_URDF_DIR +"Fetch.urdf")

        # create a urdf cpp openRAVE API module
        module = self.RaveCreateModule(self.env, 'urdf')

        # using with self.env in order to lock the env while changes are being made
        # with openRAVE modules, in order to use the functions that are required, you need to do sendcommand
        # loadURI: reads the URDF and SRDF files and created an object with the associated attributes
        # loadURI returns teh name of the robot
        # this robot that is created is placed within self.env
        with self.env:
            name = module.SendCommand('loadURI ' + self.fetch_urdf+" "+self.fetch_srdf)
        sedstr = "sed -i \"s|"+SimConfig.ROOT_DIR[:-1]+"|project_directory|g\" " + SimConfig.SIM_ROOT_DIR
        os.system(sedstr + SimConfig.REL_URDF_DIR +"Fetch.urdf")
        
        # this method fetches the robot object that was just created
        robot = self.env.GetRobot(name)
        # name is set to Fetch_id
        robot.SetName("{}_{}".format(name,self.id))

        return robot
    
    # if you have realworldexp then set to the specific values that exist for irl, otherwise call to parent class method for this
    def random_config_robot(self,current_dof=None,collision_fn=None,limits=None,**kwargs):
        if self.real_world_exp:
            random_dof_values = [0.15, 0.72466344, -0.05064385, -1.73952133, 2.25099986, -1.50486781, -0.02543545, 2.76926565]
            t = self.gripper_link.GetTransform()

            return random_dof_values, t
                    
        else:
            return super(Fetch,self).random_config_robot(current_dof,collision_fn,limits,**kwargs)

    def initGripper(self):
        """Setup gripper closing direction and tool direction """
        # gripperManip: joint that is set as the manipulator
        gripperManip = self.obj.GetActiveManipulator()

        # gripperIndices: list of indicies for gripper finger joints
        gripperIndices = gripperManip.GetGripperIndices()

        # closingDirection: list of values size of gripperIndicies that has closing dir
        # init to zero as std practice
        closingDirection = np.zeros(len(gripperIndices))

        # sets the closing direction for each joint to -1
        for i, gi in enumerate(gripperIndices):
            closingDirection[i] = -1.

        # chucking: robotics term for clamping
        # set the clamp direction for both joints to the closing direction
        gripperManip.SetChuckingDirection(closingDirection)

        # this is the approach vector of the hand
        # in this case, the hand approaches directly on the x axis
        gripperManip.SetLocalToolDirection([1, 0, 0])

    def openGripper(self):
        # taskmanip is an object that allows for high level function calls for movement
        taskmanip = self.interfaces.TaskManipulation(self.obj)
        # lock the object while it is moving
        # ReleaseFingers: takes finger joints in the oppo chuck dir until they hit max
        with self.obj:
            taskmanip.ReleaseFingers()
        # tells openrave to wait until the movement is 100% completed and the robot is stationary
        self.obj.WaitForController(0)
    
    # method that: releases the object, activates the arm, sets the arm pose to the tucked pose
    def tuck_arm(self):
        self.release()
        self.activate_manip_joints()
        # self.obj.SetActiveDOFValues(np.asarray(self.armTuckDOFs))
        self.obj.SetActiveDOFValues(np.asarray(self.grabbed_armTuckDOFs))

    # method to make the robot move across the floor (using the base)
    def activate_base_joints(self):
        self.obj.SetActiveDOFs([], sim.DOFAffine.X | sim.DOFAffine.Y | sim.DOFAffine.RotationAxis)

    # method for activating the arm
    def activate_manip_joints(self):
        self.obj.SetActiveDOFs([self.obj.GetJoint(name).GetDOFIndex() for name in self.jointnames])
    
    # blockPrinting: turns off all of the cpp terminal output from openRAVE
    @blockPrinting
    def get_ik_solutions(self,end_effector_solution,check_collisions=True,robot_param="gripper",collision_fn = None):
        solutions = []

        # in the case you are moving that base, the DOF you have to set are the
        # physical position so x,y,rotation so just pull those from the end solution
        # only caviat is that you have to check for collision
        if robot_param == SimConfig.BASE_NAME:
            _x = end_effector_solution[0,3]
            _y = end_effector_solution[1,3]
            _yaw = self.axisAngleFromRotationMatrix(end_effector_solution[:3,:3])[-1]
            solutions = [_x,_y,_yaw]
            self.activate_base_joints()
            if collision_fn is not None and check_collisions: 
                if Robot.check_collision(self.obj, solutions, collision_fn):
                    return [ ]
        
        else:
            self.activate_manip_joints()
            current_state = self.obj.GetActiveDOFValues()
            collision = check_collisions
            
            # this is doing (baselink global pos)^-1 * end solution
            # this calculates the robot end position wrt to the base of the robot
            required_T = np.linalg.pinv(self.base_link.GetTransform()).dot(end_effector_solution)
            # calls the function to get T to be in form [x,y,z,rx,ry,rz]
            pose = sixd_pose_from_transform(required_T)
            # position value for the pose [x,y,z]
            pos = pose[:3]
            # this gets you the quaternion for the pose
            # [theta, qx,qy,qz]
            orn = sim.quatFromAxisAngle(pose[3:])

            ik_count = 0
            collision = True
            while collision:
                # inital guess, where for each dof you make a guess between -pi and pi
                # this needs to be the same size as the dof
                seed_state = [np.random.uniform(-3.14, 3.14)] * self.ik_solver.number_of_joints
                # use the IK solver to compute what DOF values each joint should take to get the solution
                solutions = self.ik_solver.get_ik(seed_state,
                                                 pos[0], pos[1], pos[2],  # X, Y, Z
                                                 orn[1], orn[2], orn[3], orn[0]  # QX, QY, QZ, QW
                                                 )
                ik_count += 1
                # check for collision and if the IK has ran too many times
                if ik_count<=SimConfig.MAX_IK_ATTEMPTS:
                    if solutions is not None:
                        collision = False
                        if collision_fn is not None and check_collisions: 
                            if Robot.check_collision(self.obj, solutions, collision_fn):
                                collision=True
                else:
                    print("max ik attempts exceeded")
                    solutions = []
                    break
            
            # set back to the original DOF values
            self.obj.SetActiveDOFValues(current_state)
            
        # returns list of DOF values that get to desired state
        return solutions
                
    def get_arm_tuck_dofs(self,object_name=None,pose = [],grasp_num=None,surface_id=None):
        # grabbed_flag = getattr(self,"grabbed_flag_{}".format(self.id))
        # if grabbed_flag:
        #     return self.grabbed_armTuckDOFs
        
        # return self.armTuckDOFs
        return self.grabbed_armTuckDOFs

class yumi(Robot):
    def __init__(self,env,id,collection=False,real_world_exp=False):
        super(yumi,self).__init__(env,id,real_world_exp)

        self.right_arm_joints = ["yumi_joint_1_r", "yumi_joint_2_r", "yumi_joint_7_r", "yumi_joint_3_r",
                                 "yumi_joint_4_r", "yumi_joint_5_r", "yumi_joint_6_r"]
        self.left_arm_joints = ["yumi_joint_1_l", "yumi_joint_2_l", "yumi_joint_7_l", "yumi_joint_3_l",
                                "yumi_joint_4_l", "yumi_joint_5_l", "yumi_joint_6_l"]

        self.left_arm_tuck_DOFs = [ 0., -2.26892803, 2.35619449, 0.52359878,  0. ,  0.6981317 , -0. ]
        self.right_arm_tuck_DOFs = [  0.00000000e+00,  -2.26892803e+00, -2.35619449e+00,  5.23947841e-01, 5.23598776e-04,   6.76489618e-01,  -1.74532925e-04 ]
        self.grabbed_armTuckDOFs = self.left_arm_tuck_DOFs

        self.yumi_urdf = SimConfig.URDF_DIR + "yumi.urdf"
        self.yumi_srdf = SimConfig.URDF_DIR + "yumi.srdf"
        with open(self.yumi_urdf,"r") as f:
            self.urdf_string = f.read()

        self.obj = self.load_robot()
        self.manipulator_groups = ["right","left"] 
        
        r_T = np.eye(4)
        r_T[:3,3] = [-0.46,0,-0.08]
        self.obj_init_transform = r_T
        self.obj.SetTransform(r_T)

        self.initGripper()
        self.openGripper()
        self.env.UpdatePublishedBodies()
        
        self.obj.SetName(SimConfig.ROBOT_NAME)
        self.obj_name = str(self.obj.GetName())
        self.robot_type_object_mappings = {"gripper_l_base":"gripper_1",}
                                        #    "gripper_r_base":"gripper_2"}

        for manip in self.manipulator_groups:
            self.tuck_arm(manip)

        self.obj_offset = 0.055
        self.grab_range = {
            "plank": [0,0.15]
        }

        self.left_gripper_link = self.obj.GetLink("gripper_l_base")
        self.right_gripper_link = self.obj.GetLink("gripper_r_base")
        self.gripper_link = self.left_gripper_link
        # self.ik_link = self.obj.GetLink("gripper_l_finger_r")
        self.ik_link = self.left_gripper_link

        self.left_ik_solver = IK("world","gripper_l_base",urdf_string=self.urdf_string)
        self.right_ik_solver = IK("world","gripper_r_base",urdf_string=self.urdf_string)
         
        finger_length = 0.047
        finger_name = "gripper_l_finger_r"
        finger_wrist_T = get_relative_transform(self.obj.GetLink("gripper_l_base").GetTransform(),
                                                self.obj.GetLink(finger_name).GetTransform())

        self.geometry_limits = {
            "gripper": (2,[finger_wrist_T[2,3]-finger_length/2.0,finger_wrist_T[2,3]+finger_length/2.0]), 
            "base": (0,[0])
        }
        
        self.grasping_offset = {"plank": -0.135}
    
    def random_config_robot(self,current_dof=None,collision_fn=None,limits=None,**kwargs):
        if self.real_world_exp:
            random_dof_values = [ 0., -2.26892803, 2.35619449, 0.52359878,  0. ,  0.6981317 , -0. ]
            t = self.gripper_link.GetTransform()

            return random_dof_values, t
        
        else:
            return super(yumi,self).random_config_robot(current_dof,collision_fn,limits)
        
    def load_robot(self):
        sedstr = "sed -i \"s|project_directory|"+SimConfig.ROOT_DIR[:-1]+"|g\" " + SimConfig.SIM_ROOT_DIR
        os.system(sedstr + SimConfig.REL_URDF_DIR+"yumi.urdf")
        module = self.RaveCreateModule(self.env, 'urdf')
        with self.env:
            name = module.SendCommand('loadURI ' + self.yumi_urdf+" "+self.yumi_srdf)
        sedstr = "sed -i \"s|"+SimConfig.ROOT_DIR[:-1]+"|project_directory|g\" " + SimConfig.SIM_ROOT_DIR
        os.system(sedstr + SimConfig.REL_URDF_DIR+"yumi.urdf")
        
        robot = self.env.GetRobot(name)
        robot.SetName("{}_1".format(name))

        return robot
    
    def activate_manip_joints(self,arm="left",**kwargs):
        if arm not in self.manipulator_groups:
            arm = "left"

        joints = getattr(self,"{}_arm_joints".format(arm))
        self.obj.SetActiveDOFs([self.obj.GetJoint(name).GetDOFIndex() for name in joints])
        self.obj.SetActiveManipulator("{}_arm_effector".format(arm))

    def openGripper(self,arm="left"):
        self.activate_manip_joints(arm)
        self.initGripper()
        taskmanip = self.interfaces.TaskManipulation(self.obj)
        attempt_count = 0
        # time.sleep(1)
        while attempt_count < 10:
            try:
                with self.obj:
                    taskmanip.ReleaseFingers(movingdir=[1])
            except:
                attempt_count += 1
            else:
                break

        self.release()
        self.obj.WaitForController(0)

    def closeGripper(self,obj,arm="left"):
        self.activate_manip_joints(arm)
        self.initGripper()
        taskmanip = self.interfaces.TaskManipulation(self.obj)
        with self.obj:
            taskmanip.CloseFingers(movingdir=[-1])
        self.grab(obj,arm)
        self.obj.WaitForController(0)

    def tuck_arm(self,arm):
        self.activate_manip_joints(arm)
        self.release(arm)
        solution = getattr(self,"{}_arm_tuck_DOFs".format(arm))
        try:
            self.obj.SetActiveDOFValues(solution)
        except:
            pass
        
        self.openGripper()        
        return True
    
    def initGripper(self):
        """Setup gripper closing direction and tool direction """
        gripperManip = self.obj.GetActiveManipulator()
        gripperIndices = gripperManip.GetGripperIndices()
        closingDirection = np.zeros(len(gripperIndices))

    def grab(self,obj=None,arm="left",delta_mp=False):
        if obj is not None:
            o = self.env.GetKinBody(obj)
            gripper_link = getattr(self,"{}_gripper_link".format(arm))
            robot_t = gripper_link.GetTransform()
            ot = o.GetTransform()
            euclidean_distance = np.linalg.norm(robot_t[:3,3]-ot[:3,3])
            obj_type = obj.split("_")[SimConfig.OBJ_TYPE_IND]
            grab_range = self.grab_range[obj_type]
            if delta_mp or \
               (euclidean_distance<grab_range[1] and euclidean_distance>grab_range[0]):
                self.obj.Grab(o)
                setattr(self,"grabbed_flag_{}".format(self.id),True) 
                setattr(self,"grabbed_object_{}".format(self.id),obj)
            else:
                setattr(self,"grabbed_flag_{}".format(self.id),False) 
                print("object out of grasp range")

    def release(self,arm="left"):
        self.activate_manip_joints(arm)
        # if getattr(self,"grabbed_flag_{}".format(self.id)):
        #     self.openGripper()
        self.obj.ReleaseAllGrabbed()
        setattr(self,"grabbed_flag_{}".format(self.id),False) 
        setattr(self,"grabbed_object_{}".format(self.id),None)

    @blockPrinting
    def get_ik_solutions(self,end_effector_solution,robot_param="left",collision_fn = None,check_collisions=True):
        self.activate_manip_joints(arm=robot_param)
        current_state = self.obj.GetActiveDOFValues()
        collision = True
        if robot_param not in self.manipulator_groups:
            robot_param = "left"
        ik_solver = getattr(self,"{}_ik_solver".format(robot_param))
        ik_count = 0
        
        required_T = np.linalg.pinv(self.obj_init_transform).dot(end_effector_solution)
        pose = sixd_pose_from_transform(required_T)
        pos = pose[:3]
        orn = sim.quatFromAxisAngle(pose[3:])

        while collision:
            seed_state = [np.random.uniform(-3.14, 3.14)] * ik_solver.number_of_joints
            joint_values = ik_solver.get_ik(seed_state,
                                                 pos[0], pos[1], pos[2],  # X, Y, Z
                                                 orn[1], orn[2], orn[3], orn[0]  # QX, QY, QZ, QW
                                                 )
            ik_count += 1
            if ik_count<=SimConfig.MAX_IK_ATTEMPTS:
                if joint_values is not None:
                    self.obj.SetActiveDOFValues(joint_values)
                    print(joint_values)        
                    if collision_fn is not None and check_collisions: 
                        if Robot.check_collision(self.obj, joint_values, collision_fn):
                            collision=True
                        else:
                            collision = False
                # else:
                #     print("no joint_values")
            else:
                print("max ik attempts exceeded")
                joint_values = []
                break
        
        self.obj.SetActiveDOFValues(current_state)
        return joint_values
