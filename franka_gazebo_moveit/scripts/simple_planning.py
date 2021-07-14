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
import signal
from math import pi
from std_msgs.msg import String, Bool
from moveit_commander.conversions import pose_to_list
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene, AttachedCollisionObject
import threading
import json
from std_msgs.msg import String
from moveit_msgs.srv import GetStateValidityRequest, GetStateValidity
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

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


def publishStartPose(pose_start):
    pub = rospy.Publisher('plan_trajectory/start_pose', geometry_msgs.msg.Pose, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        pub.publish(pose_start)
        rate.sleep()



def publishTrajectoryResult(result):
    pub = rospy.Publisher('plan_trajectory/trajectory_result', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        pub.publish(result)
        rate.sleep()


def callbackCheckTrajectory(data):

    info = data.data.split("id")
    trajectoryId = info[0]

    if int(trajectoryId) == id:
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


        #time.sleep(2)


        print("fraction", fraction*100, "%")

        if(fraction == 1.0) :
            collision_checker_node = StateValidity()

            for i in plan.joint_trajectory.points:

                if collision_checker_node.start_collision_checker(list(i.positions)) == False:
                    print "Neuspjesna trajektorija"
                    fraction = 0.0
                    break



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



def checkTrajectory():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/plan_trajectory/check_trajectory", String, callbackCheckTrajectory)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


def callbackBestTrajectory(data):

    if data.data and flag:

        bestTrajectory = data.data

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
        bestTrajectory = bestTrajectory.replace('position', '')
        bestTrajectory = bestTrajectory.replace('orientation', '')
        bestTrajectory = bestTrajectory.replace(' ', '')
        bestTrajectory = bestTrajectory.split(",")

        print "Best trajectory!!!!!! ", bestTrajectory

        global flag
        flag = False

        pose = geometry_msgs.msg.Pose()
        waypoints = []


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


        plan, fraction = experiment.group.compute_cartesian_path(waypoints,
                                             0.01,
                                             0.0)

        time.sleep(5)
        print("Which percentage of the trajectory was successfully planned?")
        print(fraction*100,"%")

        experiment.execute_plan(plan)

        time.sleep(5)



def bestTrajectory():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("plan_trajectory/final_trajectory", String, callbackBestTrajectory)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

  
class StateValidity():
    def __init__(self):
        # subscribe to joint joint states
        #rospy.Subscriber("joint_states", JointState, self.jointStatesCB, queue_size=1)
        # prepare service for collision check
        self.sv_srv = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
        # wait for service to become available
        self.sv_srv.wait_for_service()
        rospy.loginfo('service is avaiable')
        # prepare msg to interface with moveit
        self.rs = RobotState()
        #self.rs.joint_state_header = Header()
        self.rs.joint_state.header.stamp = rospy.Time.now()
        self.rs.joint_state.name = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
        self.rs.joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  


    def checkCollision(self):
        '''
        check if robot is in collision
        '''


        res = self.getStateValidity()

        if res.valid:
            return True
        else:
            return False



    def jointStatesCB(self, msg):
        '''
        update robot state
        '''

    def getStateValidity(self, group_name="panda_arm", constraints=None):
        '''
        Given a RobotState and a group name and an optional Constraints
        return the validity of the State
        '''

        gsvr = GetStateValidityRequest()
        gsvr.robot_state = self.rs
        gsvr.group_name = group_name

        if constraints != None:
            gsvr.constraints = constraints
        result = self.sv_srv.call(gsvr)
        return result


    def start_collision_checker(self, positions):
        self.rs.joint_state.position = positions
        return self.checkCollision()



class planTrajectory():

  def __init__(self):


    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('plan_trajectory',
                    anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface(synchronous=True)

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



    collision_object = moveit_msgs.msg.CollisionObject()
    collision_object.header.frame_id = planning_frame
    collision_object.id = "box"

    primitive = SolidPrimitive()
    primitive.type = primitive.BOX
    primitive.dimensions = [0.75, 0.75, 1]

    box_pose = geometry_msgs.msg.Pose()
    box_pose.orientation.w = 1.0
    box_pose.position.x = 0.7
    box_pose.position.y = 0.3
    box_pose.position.z = 0.5

    collision_object.primitives.append(primitive)
    collision_object.primitive_poses.append(box_pose)
    collision_object.operation = CollisionObject.ADD


    planning_scene_msg = PlanningScene()

    planning_scene_msg.world.collision_objects.append(collision_object)
    planning_scene_msg.world.collision_objects[-1].header.frame_id = planning_frame


    collision_object = moveit_msgs.msg.CollisionObject()
    collision_object.header.frame_id = planning_frame
    collision_object.id = "box2"

    primitive = SolidPrimitive()
    primitive.type = primitive.BOX
    primitive.dimensions = [0.45, 1, 0.5]

    box_pose = geometry_msgs.msg.Pose()
    box_pose.orientation.w = 1.0
    box_pose.position.x = 0.0
    box_pose.position.y = 0.8
    box_pose.position.z = 0.25

    collision_object.primitives.append(primitive)
    collision_object.primitive_poses.append(box_pose)
    collision_object.operation = CollisionObject.ADD

    planning_scene_msg.world.collision_objects.append(collision_object)
    planning_scene_msg.world.collision_objects[-1].header.frame_id = planning_frame


    collision_object2 = moveit_msgs.msg.CollisionObject()
    collision_object2.header.frame_id = planning_frame
    collision_object2.id = "sphere"

    primitive2 = SolidPrimitive()
    primitive2.type = primitive2.SPHERE
    primitive2.dimensions = [0.3]

    sphere_pose = geometry_msgs.msg.Pose()
    sphere_pose.orientation.w = 1.0
    sphere_pose.position.x = -0.6
    sphere_pose.position.y = -0.1
    sphere_pose.position.z = 0.3


    collision_object2.primitives.append(primitive2)
    collision_object2.primitive_poses.append(sphere_pose)
    collision_object2.operation = CollisionObject.ADD

    planning_scene_msg.world.collision_objects.append(collision_object2)
    planning_scene_msg.world.collision_objects[-1].header.frame_id = planning_frame


    planning_scene_pub_s = rospy.Publisher('/planning_scene', PlanningScene, queue_size=10)

    while planning_scene_pub_s.get_num_connections() < 1:
        rospy.sleep(0.1)

    planning_scene_msg.is_diff = True
    planning_scene_pub_s.publish(planning_scene_msg)


    rospy.sleep(1)


    print "Known objects: ", scene.get_known_object_names()

    # Misc variables
    self.robot = robot
    self.scene = scene
    self.group = group
    self.planning_frame = planning_frame


  def execute_plan(self, plan):
    group = self.group
    return group.execute(plan, wait=False)


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

    pose_start = experiment.group.get_current_pose().pose

    print pose_start

    x = threading.Thread(target=publishStartPose, args=(pose_start,))
    x.start()

    y = threading.Thread(target=checkTrajectory)
    y.start()

    z = threading.Thread(target=bestTrajectory)
    z.start()

    time.sleep(0.2)


  except rospy.ROSInterruptException:
      return
  except KeyboardInterrupt:
    print "KeyboardInterrupt"

  print("============ Python experiment complete! ============")


if __name__ == '__main__':
    main()
