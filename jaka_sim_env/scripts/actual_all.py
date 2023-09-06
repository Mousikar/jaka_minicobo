#!/usr/bin/python3.8
# encoding: utf-8
from __future__ import print_function
import open3d as o3d
import copy
import math
from geometry_msgs.msg import PoseStamped
from pyquaternion import Quaternion
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Path
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

## END_SUB_TUTORIAL

# 考虑x/y方向的路径
def XdirectionZigPath(xmin,xmax,ymin,ymax,zmax,Nx):
    paths = []
    dx = float(xmax-xmin)/(Nx-1)  # the y step-over
    flag=1      #奇偶分别
    path=[]
    for n in range(0,Nx):
        x = xmin+n*dx              # current y-coordinate 
        if flag==1:
            p1 = [x,ymin,zmax]   # start-point of line
            p2 = [x,ymax,zmax]   # end-point of line
            # print(flag)
        if flag==-2:
            p1 = [x,ymax,zmax]   # start-point of line
            p2 = [x,ymin,zmax]   # end-point of line
            # print(flag)
        path.append(p1)       # add the line to the path
        path.append(p2)
        flag=~flag
    return path

# 得到步进x/y方向的路径
def feedPath(path,step,step_num):
    flag=1      #奇偶分别
    fpath=[]
    n=len(path)
    for i in range(n//2):
        for j in range(step_num):
            if flag==1:
                p=[path[2*i][0],path[2*i][1]+j*step,path[2*i][2]]
            if flag==-2:
                p=[path[2*i][0],path[2*i][1]-j*step,path[2*i][2]]
            fpath.append(p)
        flag=~flag
    return fpath

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self,group_name):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[5] += pi/15

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = -0.23986644734241247  #-2**0.5/2
        pose_goal.orientation.y = -0.17554538580973483  #-0.01
        pose_goal.orientation.z = 0.2800833569960983    #2**0.5/2
        pose_goal.orientation.w = 0.04733517921144183   #0
        pose_goal.position.x = -0.9700065690794372  #-0.4
        pose_goal.position.y = -0.21430690613561565 #0.0
        pose_goal.position.z = 0.10449491248065935  #0.4

        move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        print(current_pose)
        return all_close(pose_goal, current_pose, 0.01)


    def add__pose(self,dx=0,dy=0,dz=0,dq0=0,dq1=0,dq2=0,dq3=0):
        current_pose = self.move_group.get_current_pose().pose

        move_group = self.move_group
        pose_goal = current_pose
        pose_goal.orientation.x = pose_goal.orientation.x + dq0
        pose_goal.orientation.y = pose_goal.orientation.y + dq1
        pose_goal.orientation.z = pose_goal.orientation.z + dq2
        pose_goal.orientation.w = pose_goal.orientation.w + dq3
        pose_goal.position.x = pose_goal.position.x + dx
        pose_goal.position.y = pose_goal.position.y + dy
        pose_goal.position.z = pose_goal.position.z + dz

        move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        print(current_pose)
        return all_close(pose_goal, current_pose, 0.01)


    def plan_cartesian_path(self, scale=1):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ##
        waypoints = []

        wpose = move_group.get_current_pose().pose
        origin_x = wpose.position.x
        origin_y = wpose.position.y
        R_path = 0.1
        for t in np.linspace(0,2*pi,20):
            x = origin_x + R_path*np.cos(t)*scale
            y = origin_y + R_path*np.sin(t)*scale
            wpose.position.x = x
            wpose.position.y = y
            waypoints.append(copy.deepcopy(wpose))
            

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

        ## END_SUB_TUTORIAL

    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        ## END_SUB_TUTORIAL

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL

class TeachRobot(object):
    def __init__(self):
        self.depth_pc = rospy.Publisher('/teach_pointcloud', PointCloud2, queue_size=1)

        self.robot_name = "jaka_minicobo"
        tutorial = MoveGroupPythonInterfaceTutorial(self.robot_name)
        # -----------------------------改参数-----------------------------
        Nx=rospy.get_param("Nx",30)  # number of lines in the x-direction
        step_num=rospy.get_param("step_num",20) # number of lines in the y-direction
        high=rospy.get_param("high",0.05) # 高度
        # ----------------------------------------------------------
        num_point = 3
        zpath=[]
        for i in range(num_point):
            input("Press `Enter` to begin the tutorial by setting up the moveit_commander ...")
            pose = tutorial.move_group.get_current_pose().pose
            print(pose)
            zpath.append([pose.position.x, pose.position.y, pose.position.z])
        zpath = np.array(zpath)
        xmin = np.min(zpath[:,0])
        xmax = np.max(zpath[:,0])
        ymin = np.min(zpath[:,1])
        ymax = np.max(zpath[:,1])
        zmax = np.max(zpath[:,2]) + high
        path = XdirectionZigPath(xmin,xmax,ymin,ymax,zmax,Nx)
        # 设置每条线走几步
        step = (ymax-ymin)/(step_num-1)
        fpath = feedPath(path,step,step_num)
        # rospy.loginfo(path,xmin,xmax,ymin,ymax,zmax)

        # 将路径画在点云上,可视化
        #添加顶点，点云
        points_pcd = o3d.geometry.PointCloud()# 传入3d点云
        points_pcd.points = o3d.utility.Vector3dVector(fpath)  # point_points 二维 numpy 矩阵,将其转换为 open3d 点云格式

        # publish
        header = Header()
        header.frame_id = "Link_0" #pcd_msg.header.frame_id #"d435_depth_frame"
        header.stamp = rospy.Time.now()
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)
        ]
        pc = point_cloud2.create_cloud(header, fields, points_pcd.points)        
        self.depth_pc.publish(pc)

        # move
        try:
            # '''
            # cartesian_plan, fraction = tutorial.plan_cartesian_path()
            # 设置速度缩放因子
            velocity_scaling_factor = 1  # 设置速度为原来的50%
            tutorial.move_group.set_max_velocity_scaling_factor(velocity_scaling_factor)
            waypoints = []
            wpose = tutorial.move_group.get_current_pose().pose
            cpose = tutorial.move_group.get_current_pose().pose

            for i in range(len(fpath)):
                wpose.position.x = fpath[i][0]
                wpose.position.y = fpath[i][1]
                wpose.position.z = fpath[i][2]

                wpose.orientation.x = -1
                wpose.orientation.y = 0
                wpose.orientation.z = 0
                wpose.orientation.w = 0
                waypoints.append(copy.deepcopy(wpose))

            # goback to origin position
            wpose.position.x = cpose.position.x
            wpose.position.y = cpose.position.y
            wpose.position.z = cpose.position.z + high + 0.02
            wpose.orientation.x = cpose.orientation.x
            wpose.orientation.y = cpose.orientation.y
            wpose.orientation.z = cpose.orientation.z
            wpose.orientation.w = cpose.orientation.w
            waypoints.append(copy.deepcopy(wpose))

            (plan, fraction) = tutorial.move_group.compute_cartesian_path(
                waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
            )  # jump_threshold
            tutorial.display_trajectory(plan)
            tutorial.execute_plan(plan)
            waypoints = []
            wpose = tutorial.move_group.get_current_pose().pose
        except rospy.ROSInterruptException:
            return
        except KeyboardInterrupt:
            return
        
# 主函数
if __name__ == "__main__": 
    rospy.init_node("robteach")
    TeachRobot()
    rospy.spin()