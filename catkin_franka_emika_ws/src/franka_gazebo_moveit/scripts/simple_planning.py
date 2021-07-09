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
#import pickle
#import re
import threading
import json
from std_msgs.msg import String
from moveit_msgs.srv import GetStateValidityRequest, GetStateValidity
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

id = 1;
flag = True;

exit_event = threading.Event()


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
    #  rospy.loginfo(pose_start)
        pub.publish(pose_start)
        rate.sleep()

        #if exit_event.is_set():
            #sys.exit(0)



def publishTrajectoryResult(result):
    pub = rospy.Publisher('plan_trajectory/trajectory_result', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        pub.publish(result)
        rate.sleep()

    #if exit_event.is_set():
        #sys.exit(0)


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

                #print "###########: ", list(i.positions)
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


    """if exit_event.is_set():
        sys.exit(0)
        """

def checkTrajectory():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/plan_trajectory/check_trajectory", String, callbackCheckTrajectory)#, queue_size=1)

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

        #experiment = planTrajectory()
        #experiment.group.set_max_velocity_scaling_factor(0.5)

        plan, fraction = experiment.group.compute_cartesian_path(waypoints,
                                             0.01,
                                             0.0)

        time.sleep(5)
        print("Which percentage of the trajectory was successfully planned?")
        print(fraction*100,"%")

        experiment.execute_plan(plan)

        time.sleep(5)

        """
    if exit_event.is_set():
            sys.exit(0)"""


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

    """
class StateValidity():
    def __init__(self):
        # subscribe to joint joint states
        rospy.Subscriber("joint_states", JointState, self.jointStatesCB, queue_size=1)
        # prepare service for collision check
        self.sv_srv = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
        # wait for service to become available
        self.sv_srv.wait_for_service()
        rospy.loginfo('service is avaiable')
        # prepare msg to interface with moveit
        self.rs = RobotState()
        self.rs.joint_state.name = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
        self.rs.joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_states_received = False


    def checkCollision(self):
        '''
        check if robotis in collision
        '''
        if self.getStateValidity().valid:
            rospy.loginfo('robot not in collision, all ok!')
            return True
        else:
            rospy.logwarn('robot in collision')
            return False


    def jointStatesCB(self, msg):
        '''
        update robot state
        '''
        self.rs.joint_state.position = [msg.position[0], msg.position[1], msg.position[2], msg.position[3], msg.position[4], msg.position[5], msg.position[6]]
        self.joint_states_received = True


    def getStateValidity(self, group_name='panda_arm', constraints=None):
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


    def start_collision_checker(self):
        while not self.joint_states_received:
            rospy.sleep(0.1)
        rospy.loginfo('joint states received! continue')
        return self.checkCollision()
        #rospy.spin()


    """
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
    #    print "Joint state names: ", self.rs.joint_state.name

    #    print "Joint state positions: ", self.rs.joint_state.position
    #    print "$$$$$$$$$$$$$: ", str(experiment.group.get_joint_value_target)

        #self.joint_states_received = False

        #self.list = []


    def checkCollision(self):
        '''
        check if robot is in collision
        '''
        """
        if self.getStateValidity().valid:
            print "TRUE"
            return True
        else:
            print "FALSE"
            return False
            """
        #experiment.group.set_joint_value_target(self.list)
        #print "Plan: ", experiment.group.plan()




        res = self.getStateValidity()
        #print "+++++++++++++++++++++++++:", res.contacts

        if res.valid:
            #print "TRUE"
            return True
        else:
            #print "FALSE"
            return False



    def jointStatesCB(self, msg):
        '''
        update robot state
        '''
        #self.rs.joint_state.position = [msg.position[0], msg.position[1], msg.position[2], msg.position[3], msg.position[4], msg.position[5], msg.position[6]]
        #self.joint_states_received = True


    def getStateValidity(self, group_name="panda_arm", constraints=None):
        '''
        Given a RobotState and a group name and an optional Constraints
        return the validity of the State
        '''

        gsvr = GetStateValidityRequest()
        gsvr.robot_state = self.rs
        #print "===============: ", self.rs.joint_state.position
        gsvr.group_name = group_name
    #    gsvr.check_collisions = True

        if constraints != None:
            gsvr.constraints = constraints
        result = self.sv_srv.call(gsvr)
        return result


    def start_collision_checker(self, positions):
        #while not self.joint_states_received:
        #    rospy.sleep(0.1)
        #rospy.loginfo('joint states received! continue')
        self.rs.joint_state.position = positions
    #    self.list = positions
        #print "Joint state names: ", self.rs.joint_state.name
        #print "Joint state positions: ", self.rs.joint_state.position
        return self.checkCollision()
        #rospy.spin()



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


    attached_collision_object = moveit_msgs.msg.AttachedCollisionObject()
    attached_collision_object.link_name = 'world'
    attached_collision_object.object = collision_object

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

    """
    planning_scene_pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=10)

    while planning_scene_pub.get_num_connections() < 1:
        rospy.sleep(0.1)

    planning_scene_msg.is_diff = True
    planning_scene_pub.publish(planning_scene_msg)

    rospy.sleep(1)
    """
    """


    planning_scene_pub = rospy.Publisher('/collision_object', CollisionObject, queue_size=10)
    #time.sleep(2)
    ""

    #planning_scene_pub = rospy.Publisher('/attached_collision_object', AttachedCollisionObject, queue_size=10)

    while planning_scene_pub.get_num_connections() < 1:
        rospy.sleep(0.1)
    #planning_scene_pub.publish(attached_collision_object)


    planning_scene_pub.publish(collision_object)

    #time.sleep(2)
    """

    """
    planning_scene_msg = PlanningScene()

    planning_scene_msg.world.collision_objects.append(collision_object)
    planning_scene_msg.world.collision_objects[-1].header.frame_id = planning_frame

    planning_scene_pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=10)

    while planning_scene_pub.get_num_connections() < 1:
        rospy.sleep(0.1)

    planning_scene_msg.is_diff = True
    planning_scene_pub.publish(planning_scene_msg)
    """


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
    """
    attached_collision_object2 = moveit_msgs.msg.AttachedCollisionObject()
    attached_collision_object2.link_name = 'world'
    attached_collision_object2.object = collision_object2
    """

    #planning_scene_msg = PlanningScene()

    planning_scene_msg.world.collision_objects.append(collision_object2)
    planning_scene_msg.world.collision_objects[-1].header.frame_id = planning_frame

    #group.apply_planning_scene(planning_scene_msg)

    planning_scene_pub_s = rospy.Publisher('/planning_scene', PlanningScene, queue_size=10)#'/move_group/monitored_planning_scene', PlanningScene, queue_size=10)

    while planning_scene_pub_s.get_num_connections() < 1:
        rospy.sleep(0.1)

    planning_scene_msg.is_diff = True
    planning_scene_pub_s.publish(planning_scene_msg)


    rospy.sleep(1)





    #planning_scene_msg = PlanningScene()
    #planning_scene_msg.world.collision_objects.append(collision_object)
    #planning_scene_msg.world.collision_objects[-1].header.frame_id = planning_frame

    #planning_scene_pub_att = rospy.Publisher('/attached_collision_object', AttachedCollisionObject, queue_size=10)
    """
    planning_scene_pub = rospy.Publisher('/collision_object', CollisionObject, queue_size=10)

    while planning_scene_pub.get_num_connections() < 1:
        rospy.sleep(0.1)

    planning_scene_pub.publish(collision_object2)
    """

    #planning_scene_pub_att.publish(attached_collision_object2)

    #planning_scene_pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=10)

    #planning_scene_msg.is_diff = True
    #planning_scene_pub.publish(planning_scene_msg)
    #time.sleep(2)

    #group.attach_object("sphere", "world")
    """
    planning_scene_msg = PlanningScene()

    planning_scene_msg.world.collision_objects.append(collision_object)
    planning_scene_msg.world.collision_objects[-1].header.frame_id = planning_frame


    planning_scene_pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=10)

    while planning_scene_pub.get_num_connections() < 1:
        rospy.sleep(0.1)

    planning_scene_msg.is_diff = True
    planning_scene_pub.publish(planning_scene_msg)"""

    print "Attached objects********** ", scene.get_attached_objects()
    print "***************************************"

    print "Known objects: ", scene.get_known_object_names()
    print "Collision objects: ", scene.get_objects
    #print "Collision objects" , planning_scene_msg.world.collision_objects
    #print "***************************************"

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


"""
def signal_handler(signum, frame):
    exit_event.set()
"""

experiment = planTrajectory()
experiment.group.set_max_velocity_scaling_factor(0.5)

def main():


  try:

    #experiment = planTrajectory()
    #experiment.group.set_max_velocity_scaling_factor(0.5)
    pose_start = experiment.group.get_current_pose().pose

    print pose_start

    x = threading.Thread(target=publishStartPose, args=(pose_start,))
    x.start()

    y = threading.Thread(target=checkTrajectory)
    y.start()

    z = threading.Thread(target=bestTrajectory)
    z.start()

    time.sleep(0.2)

    """
    print "_____________________________________________"
    print "Attached objects********** ", experiment.scene.get_attached_objects()
    print "***************************************"

    print "Known objects: ", experiment.scene.get_known_object_names()
    print "Collision objects: ", experiment.scene.get_objects
    print "Pose start: ", pose_start
    experiment.group.set_start_state_to_current_state()
    """
    waypoints = []

    new_pose = geometry_msgs.msg.Pose()

    """

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

    new_pose.orientation.x = 1.0
    new_pose.orientation.y = -0
    new_pose.orientation.z = -0.0
    new_pose.orientation.w = 0

    new_pose.position.x = -0.02836904240003349
    new_pose.position.y = -0.4237137379131917
    new_pose.position.z = 0.5538263671933711
    waypoints.append(copy.deepcopy(new_pose))



    new_pose.position.x = -0.2
    new_pose.position.y = -0.5
    new_pose.position.z = 0.5
    waypoints.append(copy.deepcopy(new_pose))

    new_pose.orientation.x = 1.0
    new_pose.orientation.y = -0
    new_pose.orientation.z = -0.0
    new_pose.orientation.w = 0

    new_pose.position.x = -0.02836904240003349
    new_pose.position.y = 0.3
    new_pose.position.z = 0.5538263671933711
    waypoints.append(copy.deepcopy(new_pose))



    new_pose.position.x = -0.3
    new_pose.position.y = 0.4
    new_pose.position.z = 0.5
    waypoints.append(copy.deepcopy(new_pose))
    """
    """
    new_pose.orientation.x = 0
    new_pose.orientation.y = 0
    new_pose.orientation.z = 0.0
    new_pose.orientation.w = 1


    new_pose.position.x = 0.3
    new_pose.position.y = -0.2
    new_pose.position.z = 0.60
    waypoints.append(copy.deepcopy(new_pose))



    new_pose.position.x = 0.3
    new_pose.position.y = -0.2
    new_pose.position.z = 0.55


    waypoints.append(copy.deepcopy(new_pose))

    #experiment.group.set_pose_targets(waypoints)
    #plan = experiment.group.plan()
    #print(plan)

#    if plan.joint_trajectory.points:  # True if trajectory contains points
        #move_success = move_group.execute(plan)
#        print("Planning was successful")
#    else:
#        print("Trajectory is empty. Planning was unsuccessful.")

    plan, fraction = experiment.group.compute_cartesian_path(waypoints,
                                    0.0075,#0.005,#0.01,
                                    0.0,
                                    True)


    print("Which percentage of the trajectory was successfully planned?")
    print(fraction*100,"%")
    rospy.sleep(5)



    if(fraction == 1.0) :
        collision_checker_node = StateValidity()
        isSuccessful = True

        for i in plan.joint_trajectory.points:

            #print "###########: ", list(i.positions)
            if collision_checker_node.start_collision_checker(list(i.positions)) == False:
                print "Neuspjesna trajektorija"
                isSuccessful = False
                break



        print "Uspjesno? ", isSuccessful


        if isSuccessful:
            experiment.execute_plan(plan)
        rospy.sleep(10.2)
        """

  except rospy.ROSInterruptException:
      return
  except KeyboardInterrupt:
    print "KeyboardInterrupt"


  """while not rospy.is_shutdown():
      if rospy.is_shutdown():
          signal.signal(signal.SIGINT, signal_handler)
          """
  print("============ Python experiment complete! ============")


if __name__ == '__main__':
    main()
