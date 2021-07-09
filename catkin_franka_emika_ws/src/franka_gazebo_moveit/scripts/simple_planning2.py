#!/usr/bin/env python

import sys
import copy
import rospy
import time
import math
import numpy as np
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String, Bool
from moveit_commander.conversions import pose_to_list
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene, AttachedCollisionObject
import pickle
import re
import threading
import json
from std_msgs.msg import String

from transforms import quaternion_matrix, quaternion_from_matrix, euler_to_quaternion, quaternion_to_euler

id = 1;
flag = True;

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


def talker(pose_start):
  pub = rospy.Publisher('startingPose', geometry_msgs.msg.Pose, queue_size=10)
 # rospy.init_node('talker', anonymous=True)
  rate = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    #  rospy.loginfo(pose_start)
    pub.publish(pose_start)
    rate.sleep()


def publishTrajectoryResult(result):
  pub = rospy.Publisher('trajectoryResult', String, queue_size=10)
 # rospy.init_node('talker', anonymous=True)
  rate = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    #  rospy.loginfo(pose_start)
    pub.publish(result)
    rate.sleep()


def callbackCheckTrajectory(data):

    info = data.data.split("id")
    trajectoryId = info[0]

    if int(trajectoryId) == id:
        #experiment = planTrajectory()
        #experiment.group.set_max_velocity_scaling_factor(0.5)
        pose_start = experiment.group.get_current_pose().pose


        trajectory = info[1]
        trajectory = trajectory.replace('}}', ',')
        trajectory = trajectory.replace('{', '')
        trajectory = trajectory.replace('"', '')
        trajectory = trajectory.replace('x', '')
        trajectory = trajectory.replace('y', '')
        trajectory = trajectory.replace('z', '')
        trajectory = trajectory.replace('w', '')
        trajectory = trajectory.replace(':', '')
        trajectory = trajectory.replace('}', '')
        #trajectory = trajectory.translate({ord(i): None for i in '{"xyzw:}'})
        trajectory = trajectory.replace('position', '')
        trajectory = trajectory.replace('orientation', '')
        trajectory = trajectory.split(",")

        pose = geometry_msgs.msg.Pose()
        waypoints = []


        for i in range(0,len(trajectory)-1, 7):
             pose.position.x = float(trajectory[i])
             pose.position.y = float(trajectory[i+1])
             pose.position.z = float(trajectory[i+2])


             pose.orientation.x = float(trajectory[i+3])
             pose.orientation.y = float(trajectory[i+4])
             pose.orientation.z = float(trajectory[i+5])
             pose.orientation.w = float(trajectory[i+6])


             waypoints.append(copy.deepcopy(pose))

        print("Waypoints ", waypoints)


        plan, fraction = experiment.group.compute_cartesian_path(waypoints,
                                             0.01,
                                             0.0)


        print("fraction", fraction*100, "%")

        if fraction == 1:
            valid = "true"
            result = '{ "id":' + str(id) +', "valid":' + valid + ', "fraction":' + str(fraction*100) + ' }'
            print(result)
            x = threading.Thread(target=publishTrajectoryResult, args=(result,))
            x.start()

        else:
            valid = "false"
            result = '{ "id":' + str(id) +', "valid":' + valid + ', "fraction":' + str(fraction*100) + ' }'
            print(result)
            x = threading.Thread(target=publishTrajectoryResult, args=(result,))
            x.start()

        global id

        id += 1;
        print("id ", id)
        rospy.sleep(1)






def checkTrajectory():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/robot/checkTrajectory", String, callbackCheckTrajectory)#, queue_size=1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


def callbackBestTrajectory(data):


    if data.data and flag:


        bestTrajectory = data.data

        print(bestTrajectory)

        bestTrajectory = bestTrajectory.replace('[', '')
        bestTrajectory = bestTrajectory.replace(']', '')
        bestTrajectory = bestTrajectory.replace('}}', '')
        bestTrajectory = bestTrajectory.replace(',,', ',')
        bestTrajectory = bestTrajectory.replace('{', '')
        bestTrajectory = bestTrajectory.replace('"', '')
        bestTrajectory = bestTrajectory.replace('x', '')
        bestTrajectory = bestTrajectory.replace('y', '')
        bestTrajectory = bestTrajectory.replace('z', '')
        bestTrajectory = bestTrajectory.replace('w', '')
        bestTrajectory = bestTrajectory.replace(':', '')
        bestTrajectory = bestTrajectory.replace('}', '')
        #trajectory = trajectory.translate({ord(i): None for i in '{"xyzw:}'})
        bestTrajectory = bestTrajectory.replace('position', '')
        bestTrajectory = bestTrajectory.replace('orientation', '')
        bestTrajectory = bestTrajectory.replace(' ', '')
        bestTrajectory = bestTrajectory.split(",")



        print("Trajectory: ", bestTrajectory)

        global flag
        flag = False

        pose = geometry_msgs.msg.Pose()
        waypoints = []

        #waypoints.append(copy.deepcopy(pose_start))
        """
        roll = 180
        pitch = 0
        yaw = -45
        q = euler_to_quaternion(yaw, pitch, roll)
        """

        for i in range(0,len(bestTrajectory)-1, 7):
             pose.position.x = float(bestTrajectory[i])
             pose.position.y = float(bestTrajectory[i+1])
             pose.position.z = float(bestTrajectory[i+2])


             pose.orientation.x = float(bestTrajectory[i+3])
             pose.orientation.y = float(bestTrajectory[i+4])
             pose.orientation.z = float(bestTrajectory[i+5])
             pose.orientation.w = float(bestTrajectory[i+6])


             waypoints.append(copy.deepcopy(pose))

        print("Waypoints ", waypoints)


        #experiment = planTrajectory()
        #experiment.group.set_max_velocity_scaling_factor(0.5)
        rospy.sleep(3)


        plan, fraction = experiment.group.compute_cartesian_path(waypoints,
                                             0.01,
                                             0.0)

        print("Which percentage of the trajectory was successfully planned?")
        print(fraction*100,"%")
        rospy.sleep(3)

        experiment.execute_plan(plan)

        rospy.sleep(3)




def bestTrajectory():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("robot/trajectory", String, callbackBestTrajectory)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



class planTrajectory():

  def __init__(self):


    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('plan_trajectory',
                    anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "panda_arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    scene.remove_attached_object('world')
    rospy.sleep(2)
    scene.remove_world_object()
    rospy.sleep(2)

    print "ROBOT PLANNING FRAME: ", robot.get_planning_frame()

    self.scene = scene

    po = geometry_msgs.msg.PoseStamped()
    po.header.frame_id = planning_frame#robot.get_planning_frame()

    po.pose.orientation.x = 0
    po.pose.orientation.y = 0
    po.pose.orientation.z = 0
    po.pose.orientation.w = 1


    po.pose.position.x = 0.7
    po.pose.position.y = 0.3
    po.pose.position.z = 0.5
    box_name = "box"
    #scene.add_box(box_name, po, (0.75, 0.75, 1.8))

    """
    po.pose.position.x = -0.4
    po.pose.position.y = -0.1
    po.pose.position.z = 0.5

    self.box_name = box_name


    po.pose.position.x = -0.5
    po.pose.position.y = -0.1
    po.pose.position.z = 0.5

    box_name = "box1"
    scene.add_box(box_name, po, (0.35, 0.45, 0.8))"""
    #scene.add_sphere("sphere", po, 0.4)
    sphere_name = "sphere"

    #rospy.sleep(3)

    #self.sphere_name = sphere_name

    #self.scene.world.collision_objects.push_back()
    #scene.remove_attached_object('world')






    box_pose_st = geometry_msgs.msg.PoseStamped()
    box_pose_st.header.frame_id = robot.get_planning_frame()#planning_frame#robot.get_planning_frame()
    box_pose_st.pose.orientation.w = 1.0
    box_pose_st.pose.position.x = 0.7#1#0.7
    box_pose_st.pose.position.y = 0.3#0.4#0.3
    box_pose_st.pose.position.z = 0.5

    """
    collision_object = moveit_msgs.msg.CollisionObject()
    collision_object.header.frame_id = robot.get_planning_frame()#planning_frame#robot.get_planning_frame()#planning_frame#robot.get_planning_frame
    collision_object.id = "box1"
    primitive = SolidPrimitive()
    primitive.type = primitive.BOX
    primitive.dimensions = [0.75, 0.75, 1.8]"""

    box_pose = geometry_msgs.msg.Pose()
    box_pose.orientation.w = 1.0
    box_pose.position.x = 0.7
    box_pose.position.y = 0.3
    box_pose.position.z = 0.5

    """
    collision_object.primitives.append(primitive)
    collision_object.primitive_poses.append(box_pose)
    collision_object.operation = CollisionObject.ADD"""

    scene.attach_box("world", "box" , box_pose_st, [0.75, 0.75, 1.8])
    self.wait_for_state_update("box", object_is_attached=True, timeout=2)


    planning_scene_msg = PlanningScene()



    #planning_scene_msg.world.collision_objects.append(collision_object)
    #planning_scene_msg.world.collision_objects[-1].header.frame_id = planning_frame#robot.get_planning_frame()#planning_frame#"world"

    #planning_scene_pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=1)

    #while planning_scene_pub.get_num_connections() < 1:
    #    rospy.sleep(0.1)

    #planning_scene_msg.is_diff = True
    #planning_scene_pub.publish(planning_scene_msg)


    """
    attached_collision_object = AttachedCollisionObject()
    attached_collision_object.link_name = "world"
    attached_collision_object.object.header.frame_id = planning_frame
    attached_collision_object.object.id = "box1"
    attached_collision_object.object.primitives.append(primitive)
    attached_collision_object.object.primitive_poses.append(box_pose)
    attached_collision_object.object.operation = attached_collision_object.object.ADD

    planning_scene_pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=1)

    #planning_scene_msg = PlanningScene()
    #scene.attach_object(collision_object)

    planning_scene_msg.world.collision_objects.append(attached_collision_object.object)

    while planning_scene_pub.get_num_connections() < 1:
        rospy.sleep(0.1)

    planning_scene_msg.is_diff = True
    planning_scene_pub.publish(planning_scene_msg)


    self.wait_for_state_update("box1", object_is_known=True, timeout=2)
    """
    #scene.attach_box('world', "box" , box_pose, [0.75, 0.75, 1.8])


    #planning_scene_pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=10)
#    scene.attach_box("world", "box", box_pose)


    #scene.attach_box("world", "box" , box_pose_st, [0.75, 0.75, 1.8])
#    self.wait_for_state_update("box", object_is_attached=True, timeout=2)


    box_pose_st.header.frame_id = robot.get_planning_frame()#planning_frame#robot.get_planning_frame()
    box_pose_st.pose.orientation.w = 1.0
    box_pose_st.pose.position.x = -0.4#-0.6#-0.4
    box_pose_st.pose.position.y = -0.1#-0.2#-0.1
    box_pose_st.pose.position.z = 0.5


    collision_object = moveit_msgs.msg.CollisionObject()
    collision_object.header.frame_id = robot.get_planning_frame()#planning_frame#robot.get_planning_frame()#planning_frame#robot.get_planning_frame
    collision_object.id = "box2"
    primitive = SolidPrimitive()
    primitive.type = primitive.BOX
    primitive.dimensions = [0.35, 0.45, 0.8]

    box_pose = geometry_msgs.msg.Pose()
    box_pose.orientation.w = 1.0
    box_pose.position.x = -0.4
    box_pose.position.y = -0.1
    box_pose.position.z = 0.5
    """
    collision_object.primitives.append(primitive)
    collision_object.primitive_poses.append(box_pose)
    collision_object.operation = CollisionObject.ADD"""

    scene.attach_box("world", "box2" , box_pose_st, [0.35, 0.45, 0.8])
    self.wait_for_state_update("box2", object_is_attached=True, timeout=2)

    """
    planning_scene_msg = PlanningScene()



    planning_scene_msg.world.collision_objects.append(collision_object)
    planning_scene_msg.world.collision_objects[-1].header.frame_id = planning_frame#robot.get_planning_frame()#planning_frame#"world"

    #scene.apply_planning_scene(planning_scene_msg)
    #self.wait_for_state_update("box2", object_is_known=True, timeout=2)


    planning_scene_pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=1)

    while planning_scene_pub.get_num_connections() < 1:
        rospy.sleep(0.1)

    planning_scene_msg.is_diff = True
    planning_scene_pub.publish(planning_scene_msg)

    self.wait_for_state_update("box2", object_is_known=True, timeout=2)"""
    """
    while planning_scene_pub.get_num_connections() < 1:
        rospy.sleep(0.1)

    planning_scene_msg.is_diff = True
    planning_scene_pub.publish(planning_scene_msg)$$$$
    """




    #print robot.get_planning_frame()
    """
    collision_object = moveit_msgs.msg.CollisionObject()
    collision_object.header.frame_id = planning_frame#robot.get_planning_frame()#planning_frame
    collision_object.id = "sphere"
    primitive = SolidPrimitive()
    primitive.type = primitive.SPHERE
    #primitive.dimensions.resize(3);
    #primitive.dimensions[0] = 0.75;
    #primitive.dimensions[1] = 0.75;
    #primitive.dimensions[2] = 1.8;
    primitive.dimensions = [0.4]

    box_pose = geometry_msgs.msg.Pose()
    #box_pose.header.frame_id = robot.get_planning_frame()
    box_pose.orientation.w = 1.0
    box_pose.position.x = -0.5
    box_pose.position.y = -0.1
    box_pose.position.z = 0.5"""
    """$$$$
    collision_object.primitives.append(primitive)
    collision_object.primitive_poses.append(box_pose)
    collision_object.operation = CollisionObject.ADD
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = planning_frame
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = -0.5
    box_pose.pose.position.y = -0.1
    box_pose.pose.position.z = 0.5  $$$$$$"""

    """
    attached_collision_object = AttachedCollisionObject()
    attached_collision_object.link_name = "world"
    attached_collision_object.object.header.frame_id = planning_frame
    attached_collision_object.object.id = "sphere"
    attached_collision_object.object.primitives.append(primitive)
    attached_collision_object.object.primitive_poses.append(box_pose)
    attached_collision_object.object.operation = attached_collision_object.object.ADD

    planning_scene_pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=1)

    #planning_scene_msg = PlanningScene()
    #scene.attach_object(collision_object)

    planning_scene_msg.world.collision_objects.append(attached_collision_object.object)"""
    #planning_scene_msg.world.collision_objects[-1].header.frame_id = planning_frame#robot.get_planning_frame()#planning_frame
    """
    while planning_scene_pub.get_num_connections() < 1:
        rospy.sleep(0.1)

    planning_scene_msg.is_diff = True
    planning_scene_pub.publish(planning_scene_msg)

    self.wait_for_state_update("sphere", object_is_known=True, timeout=2)
    """

    #time.sleep(10)

    #print "Attached objects********** ", scene.get_attached_objects([box_name])
    #print "***************************************"

    #print "Known objects: ", scene.get_known_object_names()
#    print " collision objects" , planning_scene_msg.world.collision_objects
    #print "***************************************"


    #scene.world.collision_objects.append(collision_object.object)
    """


    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = group.getPlanningFrame();

    /* The id of the object is used to identify it. */
    collision_object.id = "box1";

    /* Define a box to add to the world. */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.4;
    primitive.dimensions[1] = 0.1;
    primitive.dimensions[2] = 0.4;

    /* A pose for the box (specified relative to frame_id) */
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x =  0.6;
    box_pose.position.y = -0.4;
    box_pose.position.z =  1.2;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    """

    #print self.scene.get_objects()


#    self.addBox("box", 0.75, 0.75, 1.8, 0.5, 0.5, 0.5, True)




    # Misc variables
    self.robot = robot
    self.scene = scene
    self.group = group
    self.planning_frame = planning_frame

  def wait_for_state_update(self, name, object_is_known=False, object_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## Ensuring Collision Updates Are Receieved
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_known_object_names()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (object_is_attached == is_attached) and (object_is_known == is_known):
        print "True: ", seconds-start
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out

    return False
    ## END_SUB_TUTORIAL

  def add_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene
    robot = self.robot

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = robot.get_planning_frame()
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 0.7
    box_pose.pose.position.y = 0.3
    box_pose.pose.position.z = 0.5
    scene.add_box(box_name, box_pose, size=(0.75, 0.75, 1.8))

    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    self.box_name=box_name
    return self.wait_for_state_update(box_name, object_is_known=True, timeout=timeout)

  def add_sphere(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.sphere_name
    scene = self.scene
    robot = self.robot

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = robot.get_planning_frame()
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = -0.5
    box_pose.pose.position.y = -0.1
    box_pose.pose.position.z = 0.5
    scene.add_sphere(box_name, box_pose, 0.4)

    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    #self.sphere=sphere_name
    return self.wait_for_state_update(box_name, object_is_known=True, timeout=timeout)


  def execute_plan(self, plan):
    group = self.group
    return group.execute(plan, wait=False)

  """
  def addBox(self, name, size_x, size_y, size_z, x, y, z, wait=True):
    s = SolidPrimitive()
    s.dimensions = [size_x, size_y, size_z]
    s.type = s.BOX

    ps = PoseStamped()
    ps.header.frame_id = self._fixed_frame
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = z
    ps.pose.orientation.w = 1.0

    self.addSolidPrimitive(name, s, ps.pose, wait)


  def addCylinder(self, name, height, radius, x, y, z, wait=True):
    s = SolidPrimitive()
    s.dimensions = [height, radius]
    s.type = s.CYLINDER

    ps = PoseStamped()
    ps.header.frame_id = self._fixed_frame
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = z
    ps.pose.orientation.w = 1.0

    self.addSolidPrimitive(name, s, ps.pose, wait)

  """
  def plan_to_pose(self, p, q):

    group = self.group

    wpose = geometry_msgs.msg.Pose()
    wpose.position.x = p.x
    wpose.position.y = p.y
    wpose.position.z = p.z

    wpose.orientation.x = q.x
    wpose.orientation.y = q.y
    wpose.orientation.z = q.z
    wpose.orientation.w = q.w

    (plan, fraction) = group.compute_cartesian_path(
                                       [wpose],   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    time.sleep(0.1)
    if fraction > 0.:
        return self.execute_plan(plan)
    else:
        return False





experiment = planTrajectory()
experiment.group.set_max_velocity_scaling_factor(0.5)


def main():

  try:


    #experiment = planTrajectory()
    #experiment.group.set_max_velocity_scaling_factor(0.5)

    pose_start = experiment.group.get_current_pose().pose

    #print pose_start
    #experiment.add_box()
    #experiment.add_sphere()

#    print "Novo: ", experiment.group.get_end_effector_link

    #success = experiment.plan_to_pose(new_pose.position, new_pose.orientation) # ovaj dio izvrsi plan & execute, planira se gibanje u jednu tocku zadanu sa new_pose



    #talker(pose_start)

    # kako se planira gibanje u jednu tocku
    # zadaje se pozicija i orijentacija vrha robotske ruke
    """
    x = threading.Thread(target=talker, args=(pose_start,))
    x.start()

    y = threading.Thread(target=checkTrajectory)
    y.start()

    z = threading.Thread(target=bestTrajectory)
    z.start()
    """
    #checkTrajectory()


    waypoints = []


    new_pose = geometry_msgs.msg.Pose()
    """


    new_pose.orientation.x = pose_start.orientation.x
    new_pose.orientation.y = pose_start.orientation.y
    new_pose.orientation.z = pose_start.orientation.z
    new_pose.orientation.w = pose_start.orientation.w

    new_pose.position.x = pose_start.position.x + 0.15
    new_pose.position.y = pose_start.position.y - 0.1
    new_pose.position.z = pose_start.position.z - 0.25

    waypoints.append(copy.deepcopy(new_pose))


    new_pose.orientation.x = pose_start.orientation.x
    new_pose.orientation.y = pose_start.orientation.y
    new_pose.orientation.z = pose_start.orientation.z
    new_pose.orientation.w = pose_start.orientation.w

    new_pose.position.x = pose_start.position.x + 0.3
    new_pose.position.y = pose_start.position.y - 0.4
    new_pose.position.z = pose_start.position.z - 0.5

    print("Zavrsna poza: ", new_pose)
    waypoints.append(copy.deepcopy(new_pose))

    #print(waypoints)
    plan, fraction = experiment.group.compute_cartesian_path(waypoints,
                                    0.01,
                                    0.0)

    print("Which percentage of the trajectory was successfully planned?")
    print(fraction*100,"%")
    """
    #success = experiment.plan_to_pose(new_pose.position, new_pose.orientation)

    """
    new_pose.orientation.x = -0.999
    new_pose.orientation.y = 2.31798
    new_pose.orientation.z = 0.0001
    new_pose.orientation.w = 7e-07

    new_pose.position.x = 1.0
    new_pose.position.y = -2.0
    new_pose.position.z = 1.0

    print("new pose")
    print(new_pose)

    pose3 = geometry_msgs.msg.Pose()


    #success = experiment.plan_to_pose(pose2.position, pose2.orientation)
    success = experiment.plan_to_pose(new_pose.position, new_pose.orientation) # ovaj dio izvrsi plan & execute, planira se gibanje u jednu tocku zadanu sa new_pose

    print("move successful? ", success)

    # robotu treba neko vrijeme da stigne u zadanu tocku, pricekamo da smo sigurni da je tamo
    time.sleep(2.0)

    pose_start = experiment.group.get_current_pose().pose
    print(pose_start)

    print("\n\n\n\n\n\n\n\n\n\n\n\n\n\n")
    """
    # kako se planira kroz vise tocaka - tocke se appendaju u listu tocaka waypoints
    """
    waypoints = []
    pose3.position.x = pose_start.position.x - 0.05
    waypoints.append(copy.deepcopy(pose3))

    pose3.position.y = pose3.position.y - 0.1
    waypoints.append(copy.deepcopy(pose3))

    pose3.position.z += 0.05
    waypoints.append(copy.deepcopy(pose3))

    # ovo je kao plan dio, predajemo cijelu listu waypoints u kojoj su sve medutocke
    plan, fraction = experiment.group.compute_cartesian_path(waypoints,
                                       0.01,
                                       0.0)
    print("Which percentage of the trajectory was successfully planned?")
    print(fraction*100,"%")

    # ovo je execute dio
    experiment.execute_plan(plan)

    time.sleep(0.2)



    *********************************************


    waypoints = []

    new_pose = geometry_msgs.msg.Pose()

    """
    """
    new_pose.orientation.x = pose_start.orientation.x - 0.1
    new_pose.orientation.y = pose_start.orientation.y - 0.1
    new_pose.orientation.z = pose_start.orientation.z - 0.1
    new_pose.orientation.w = pose_start.orientation.w - 0.1

    new_pose.position.x = pose_start.position.x - 0.1
    new_pose.position.y = pose_start.position.y - 0.1
    new_pose.position.z = pose_start.position.z - 0.1

    {"position":{"x":0.08097301851114759,"y":-0.044709861127933494,"z":0.8316939222634463},"orientation":{"x":0.9999999940154279,"y":9.493752474521609E-8,"z":-0.00010778333272227095,"w":2.7252432101990304E-9}}
{"position":{"x":0.07348603306245183,"y":-0.10872098209323072,"z":0.6772401268196487},"orientation":{"x":0.9999999941924482,"y":-0.000005139317550972195,"z":-0.00011363516034107933,"w":2.690605650401787E-9}}
{"position":{"x":-0.01250784314252041,"y":-0.3886185298486774,"z":0.6043521627744746},"orientation":{"x":0.9999999944028589,"y":-0.000004626892991816148,"z":-0.00010580257985837844,"w":3.781887547114857E-9}}
{"position":{"x":-0.02836904240003349,"y":-0.4237137379131917,"z":0.5538263671933711},"orientation":{"x":0.9999999940762615,"y":-6.490497200828932E-7,"z":-0.00010867354041439406,"w":3.5193946993974606E-9}}
-0.2
-0.5
0.5
0.999999993501
6.00076083517e-06
-0.000113848705833
2.20874366934e-09
    """
    """

    #*********************************************************+ NE RADI DOBRO
    new_pose.orientation.x = 0.9999999940154279
    new_pose.orientation.y = 9.493752474521609E-8
    new_pose.orientation.z = -0.00010778333272227095
    new_pose.orientation.w = 2.7252432101990304E-9

    new_pose.position.x = 0.08097301851114759
    new_pose.position.y = -0.044709861127933494
    new_pose.position.z = 0.8316939222634463

    waypoints.append(copy.deepcopy(new_pose))

    new_pose.orientation.x = 0.9999999941924482
    new_pose.orientation.y = -0.000005139317550972195
    new_pose.orientation.z = -0.00011363516034107933
    new_pose.orientation.w = 2.690605650401787E-9

    new_pose.position.x = 0.07348603306245183
    new_pose.position.y = -0.10872098209323072
    new_pose.position.z = 0.6772401268196487
    waypoints.append(copy.deepcopy(new_pose))


    new_pose.orientation.x = 0.9999999944028589
    new_pose.orientation.y = -0.000004626892991816148
    new_pose.orientation.z = -0.00010580257985837844
    new_pose.orientation.w = 3.781887547114857E-9

    new_pose.position.x = -0.01250784314252041
    new_pose.position.y = -0.3886185298486774
    new_pose.position.z = 0.6043521627744746
    waypoints.append(copy.deepcopy(new_pose))

    new_pose.orientation.x = 0.9999999940762615
    new_pose.orientation.y = -6.490497200828932E-7
    new_pose.orientation.z = -0.00010867354041439406
    new_pose.orientation.w = 3.5193946993974606E-9

    new_pose.position.x = -0.02836904240003349
    new_pose.position.y = -0.4237137379131917
    new_pose.position.z = 0.5538263671933711
    waypoints.append(copy.deepcopy(new_pose))

    new_pose.orientation.x = 0.999999993501
    new_pose.orientation.y = 6.00076083517e-6
    new_pose.orientation.z = -0.000113848705833
    new_pose.orientation.w = 2.20874366934e-9

    new_pose.position.x = -0.2
    new_pose.position.y = -0.5
    new_pose.position.z = 0.5
    waypoints.append(copy.deepcopy(new_pose))

    """
    """
    new_pose.orientation.x = 0.999999993501
    new_pose.orientation.y = 6.00076083517e-6
    new_pose.orientation.z = -0.000113848705833
    new_pose.orientation.w = 2.20874366934e-9

    new_pose.position.x = -0.3
    new_pose.position.y = -0.3
    new_pose.position.z = 0.5
    """
    """
        collision_detection::CollisionRequest c_req;
    collision_detection::CollisionResult c_res;
    c_req.group_name = robot.getGroupName();
    c_req.contacts = true;
    c_req.max_contacts = 100;
    c_req.max_contacts_per_pair = 5;
    c_req.verbose = false;

    Checking for Collisions

    We check for collisions between robot and itself or the world.

    g_planning_scene->checkCollision(c_req, c_res, *robot.robotState())
    """

    new_pose.orientation.x = 1
    new_pose.orientation.y = 0
    new_pose.orientation.z = 0
    new_pose.orientation.w = 0

    new_pose.position.x = 0.05
    new_pose.position.y = 0.05
    new_pose.position.z = 0.9

    waypoints.append(copy.deepcopy(new_pose))

    new_pose.orientation.x = 1
    new_pose.orientation.y = 0
    new_pose.orientation.z = 0
    new_pose.orientation.w = 0

    new_pose.position.x = 0.1
    new_pose.position.y = 0.15
    new_pose.position.z = 0.85

    waypoints.append(copy.deepcopy(new_pose))

    new_pose.orientation.x = 1
    new_pose.orientation.y = 0
    new_pose.orientation.z = 0
    new_pose.orientation.w = 0

    new_pose.position.x = 0.1
    new_pose.position.y = 0.2
    new_pose.position.z = 0.8

    waypoints.append(copy.deepcopy(new_pose))

    print(waypoints)
    plan, fraction = experiment.group.compute_cartesian_path(waypoints,
                                    0.01,
                                    0.0,
                                    True)

    print("Which percentage of the trajectory was successfully planned?")
    print(fraction*100,"%")

    """
    new_pose.orientation.x = 0.999999993501
    new_pose.orientation.y = 6.00076083517e-6
    new_pose.orientation.z = -0.000113848705833
    new_pose.orientation.w = 2.20874366934e-9

    new_pose.position.x = 0.0
    new_pose.position.y = 0.05
    new_pose.position.z = 0.75

    waypoints.append(copy.deepcopy(new_pose))

    new_pose.orientation.x = 0.999999993501
    new_pose.orientation.y = 6.00076083517e-6
    new_pose.orientation.z = -0.000113848705833
    new_pose.orientation.w = 2.20874366934e-9

    new_pose.position.x = 0.0
    new_pose.position.y = -0.1
    new_pose.position.z = 0.65

    waypoints.append(copy.deepcopy(new_pose))

    new_pose.orientation.x = 0.999999993501
    new_pose.orientation.y = 6.00076083517e-6
    new_pose.orientation.z = -0.000113848705833
    new_pose.orientation.w = 2.20874366934e-9

    new_pose.position.x = -0.2
    new_pose.position.y = -0.5
    new_pose.position.z = 0.5

    waypoints.append(copy.deepcopy(new_pose))


    print(waypoints)
    plan, fraction = experiment.group.compute_cartesian_path(waypoints,
                                    0.01,
                                    0.0)

    print("Which percentage of the trajectory was successfully planned?")
    print(fraction*100,"%")

    """
    #success = experiment.plan_to_pose(new_pose.position, new_pose.orientation)

    # ovo je execute dio
    #experiment.execute_plan(plan)
#    *********************************

    time.sleep(0.2)


  except rospy.ROSInterruptException:
      return
  except KeyboardInterrupt:
      print "this is a KeyboardInterrupt that we never catch"
  print("============ Python experiment demo complete!")
  return

if __name__ == '__main__':
    main()
