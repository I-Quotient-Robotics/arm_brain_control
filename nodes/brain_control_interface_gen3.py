#!/usr/bin/env python
import time
import math
import rospy
import socket
import threading
import actionlib
import select
import tf
import PyKDL
from tf_conversions import posemath

from brain_control.msg import brain_control_msg
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from kortex_driver.msg import *
from kortex_driver.srv import *
from std_msgs.msg import UInt32MultiArray, Empty
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

from std_msgs.msg import UInt32MultiArray
from aruco_msgs.msg import MarkerArray
from sensor_msgs.msg import JointState

class BrainControlInterface:
  _moving = False
  _vel_stop = False
  _drink_pose_saved = False
  _cartesian_vel_msg = TwistCommand()
  _arm_base_msg = BaseCyclic_Feedback()
  _empty_msg = Empty()
  # _pose_drink_marker = Pose()
  _pose_stamped_drink_marker = PoseStamped()
  _pose_stamped_drink_marker.pose.position.x = -0.0236711222678
  _pose_stamped_drink_marker.pose.position.y = 0.586763203144
  _pose_stamped_drink_marker.pose.position.z = 0.256260246038
  _pose_stamped_drink_marker.pose.orientation.x = 90.1911087036
  _pose_stamped_drink_marker.pose.orientation.y = -1.5087274313
  _pose_stamped_drink_marker.pose.orientation.z = 176.417007446
  _pose_stamped_drink_marker.pose.orientation.w = 1.0
  # _joints_person = [-1.7930190102665966, -0.813059885007883, -0.34033973677111184, -1.9760117329847748, 1.618402461808935, -1.2835999695763158, -1.1947441324380401]
  
  # _joints_person = [-0.6559743933739952, 0.9842364948645166, 2.8178790315040456, -1.9159012577044878, 2.192828332601669, 1.3148982376552536, 1.1212547347895212] #  a little to the right
  _joints_person = [-1.0321245920589668, 1.1234470489988744, 2.753361557251895, -1.4093784046996678, 2.114625406139819, 1.3174574023047654, 0.7564660473680183]
  _joints_person1 = [-0.796869463476856, 1.1657176739269937, 2.843103960717058, -1.1297922966120977, 2.275459696482157, 1.6978092702415553, 0.6397798439432781]  #  a little to the forward
  # _joints_table = [0.08517289341015634, 0.7100715174919112, -3.0923816977028733, -2.311939144461844, 0.45694075233048337, 1.4553051503303023, 1.560596419819893]
  _joints_table = [0.04076120347891113, 0.48361224406859976, -3.0863427136150094, -2.34279239832263, 0.45706708603469404, 1.0405597553376298, 1.4736889513351192] # wrist downward
  # _pose_drink = [0.0619054846466, -0.441089421511, 0.323041915894, -64.6799697876, 176.703430176, 121.39691925]
  _joints_drink = [-1.4591108075280257, 0.8634645362744043, 2.823540911981554, -2.323301254936558, -0.15237382703331903, 1.6257926006758503, 1.808233649574219]
  _pose_ready = [-0.0317343100905, -0.523988306522, 0.306685239077, -91.2066879272, 179.187179565, 170.936584473]
  _scale_vel = 0.01           # vel = 1 or -1 * scale
  _scale_step = 0.05          # vel = 1 or -1 * scale
  _translation_speed = 0.1   # speed limit, m/s
  _orientation_speed = 30.0  # speed limit, degree/s
  # _joint_angle_speed = 100.0   # speed limit, degree/s

  _action_finish = False
  HOME_ACTION_IDENTIFIER = 2

  def __init__(self):

    self._kinova_pose_init = Pose()
    self._sub = rospy.Subscriber("aruco_marker_publisher/markers_list", UInt32MultiArray, self.arucoDetectCallback)
    # self._sub_aruco = rospy.Subscriber("aruco_marker_publisher/markers", MarkerArray, self.arucoPoseCallback)
    # self._sub_arm_base = rospy.Subscriber("/arm/base_feedback", BaseCyclic_Feedback, self.armBaseCallback)
    self._pub_cartesian_vel = rospy.Publisher('/arm/in/cartesian_velocity', TwistCommand, queue_size=1)
    self._pub_stop = rospy.Publisher('/arm/in/stop', Empty, queue_size=1)
    self._pub_joint_vel = rospy.Publisher('/arm/in/joint_velocity', Base_JointSpeeds, queue_size=1)
    # self._pub = rospy.Publisher('brain_control_msg', brain_control_msg, queue_size=1)
    self.client = actionlib.SimpleActionClient("/arm/robotiq_2f_85_gripper_controller/gripper_cmd", GripperCommandAction)

    rospy.loginfo('Init the action topic subscriber')
    self.__action_topic_sub = rospy.Subscriber('arm/action_topic', ActionNotification, self.__cb_action_topic)
    self.__last_action_notif_type = None

    rospy.loginfo('Setup cartesian trajectory service')
    send_cartesian_command_full_name = 'arm/base/play_cartesian_trajectory'
    rospy.wait_for_service(send_cartesian_command_full_name)
    self.__send_cartesian_command_srv = rospy.ServiceProxy(send_cartesian_command_full_name, PlayCartesianTrajectory)

    rospy.loginfo('Setup joint angle trajectory service')
    send_joint_command_full_name = 'arm/base/play_joint_trajectory'
    rospy.wait_for_service(send_joint_command_full_name)
    self.__send_joint_command_srv = rospy.ServiceProxy(send_joint_command_full_name, PlayJointTrajectory)

    # Init the services
    clear_faults_full_name = '/arm/base/clear_faults'
    rospy.wait_for_service(clear_faults_full_name)
    self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

    read_action_full_name = '/arm/base/read_action'
    rospy.wait_for_service(read_action_full_name)
    self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

    execute_action_full_name = '/arm/base/execute_action'
    rospy.wait_for_service(execute_action_full_name)
    self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

    send_gripper_command_full_name = '/arm/base/send_gripper_command'
    rospy.wait_for_service(send_gripper_command_full_name)
    self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

    rospy.loginfo('Activate arm action state notification')
    self.__activate_notifications()

    try:
      self._udpSocket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
      self._udpSocket_send = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

      # computer_liuhang
      # self._udpSocket.bind(("127.0.0.1", 8848)) #receive
      # self._udpSocket.bind(("192.168.3.82", 8848)) #receive
      self._udpSocket.bind(("192.168.43.55", 8848)) #receive
      self.server_address = ("127.0.0.1", 8848) #send
      # self.server_address = ("192.168.1.61", 8848) #send

      # computer_zhouyajun
      # self._udpSocket.bind(("192.168.1.145", 8848)) #receive
      # self.server_address = ("192.168.43.103", 8080) #send

      # self._udpSocket.bind(("", 8848))
      print 'Bind UDP on 8848...'
    except:
      print 'Build UDP socket error!! exit~'
      exit(-1)

  def UDPLinstenLoop(self):
    rospy.loginfo("start udp linster..")
    linear_X = 0
    linear_Y = 0
    linear_Z = 0
    angular_X = 0
    angular_Y = 0
    angular_Z = 0
    joint_4 = 0
    joint_5 = 0
    joint_6 = 0
    joint_7 = 0
    mode = 1
    movePlane = "PRESENT"

    while not rospy.is_shutdown():

      rospy.loginfo("linste..")
      timeout=60*60*5
      self._udpSocket.settimeout(timeout)
      data, addr = self._udpSocket.recvfrom(1024)

      data_combin = self.dateCombin(data)
      rospy.loginfo("%s", data_combin)

      if data_combin == "ModeSwitch":
        if mode == 0:
          self._moving = True
          rospy.loginfo("changed to mode 1 / step")
          mode = 1
        elif mode == 1:
          rospy.loginfo("changed to mode 0 / continuous")
          mode = 0
      elif data_combin == "FigOpen":
        self.fingerControl(0.02)
      elif data_combin == "FigClose":
        self.fingerControl(0.14)
      elif data_combin == "Object1":
        self.objectPick(101)
      elif data_combin == "Object2":
        self.objectPick(102)
      elif data_combin == "Object3":
        self.objectPick(103)
      elif data_combin == "Home":
        if self._moving:
          self._pub_stop.publish(self._empty_msg)
        rospy.sleep(1.0)
        self.home_the_robot()
      elif data_combin == "DrinkPose":
        self.getDrinkPose()
      elif data_combin == "WatchPose":
        self.__move_by_joint_angle(self._joints_drink)
      elif data_combin == "Stop":
        self._moving = False
        self._pub_stop.publish(self._empty_msg)
      else:
        linear_X, linear_Y, linear_Z, angular_X, angular_Y, angular_Z, joint_4, joint_5, joint_6, joint_7, movePlane \
        = self.getBaseMoveMsgGen3(data_combin, linear_X, linear_Y, linear_Z, angular_X, angular_Y, angular_Z, joint_4, joint_5, joint_6, joint_7, movePlane)
        joints = [0.0, 0.0, 0.0, joint_4, joint_5, joint_6, joint_7]
        if mode == 0:
          self._moving = True
          self.pubMsg(linear_X, linear_Y, linear_Z, angular_X, angular_Y, angular_Z, joints, self._scale_vel)
        elif mode == 1:
          self.pubStepMsg(linear_X, linear_Y, linear_Z, angular_X, angular_Y, angular_Z, joints, self._scale_step)
      data_combin = "/"

  def __cb_action_topic(self, notif):
      self.last_action_notif_type = notif.action_event

  def __wait_for_action_end_or_abort(self):
      while not rospy.is_shutdown():
          if (self.last_action_notif_type == ActionEvent.ACTION_END):
              rospy.loginfo("Received ACTION_END notification")
              return True
          elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
              rospy.loginfo("Received ACTION_ABORT notification")
              return False
          else:
              time.sleep(0.01)

  def __activate_notifications(self):
      activate_publishing_of_action_notification_full_name = 'arm/base/activate_publishing_of_action_topic'
      rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
      activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)

      req = OnNotificationActionTopicRequest()
      rospy.loginfo('Activating the action notifications...')
      try:
          activate_publishing_of_action_notification(req)
      except rospy.ServiceException:
          rospy.logerr('Failed to call OnNotificationActionTopic')
          return False
      else:
          rospy.loginfo('Successfully activated the Action Notifications!')

      rospy.sleep(1.0)
      return True

  # def arucoDetectCallback(self, msg):
  #   marker = msg.data
  #   if len(marker) >= 1:
  #     self._pub_stop.publish(self._empty_msg)

  def arucoDetectCallback(self, msg):
    marker = msg.data
    # rospy.loginfo("len_marker, %d", len(marker))
    if len(marker) >= 1:
      for x in xrange(0,len(marker)):
        # if marker[x] == 101:
        if marker[x] == 101 and self._moving:
          rospy.loginfo("robot stopped")
          self._pub_stop.publish(self._empty_msg)
          self._moving=False
          continue

  # def arucoDetectCallback(self, msg):
  #   marker = msg.data
  #   # rospy.loginfo("len_marker, %d", len(marker))
  #   for x in xrange(1,len(marker)):
  #     if marker[x-1] == 101 & self._moving:
  #       self._pub_stop.publish(self._empty_msg)
  #       self._moving=False
  #       continue

  # def arucoPoseCallback(self, msg):
  #   br = tf.TransformBroadcaster()
  #   # rate = rospy.Rate(10.0)
  #   while not rospy.is_shutdown():
  #   # print("aaa")

  #     for i in xrange(0,len(msg.markers)):    
  #       pose = Pose()
  #       frame_id = "object_link" + str(msg.markers[i].id)
  #       # print("%s", frame_id)

  #       # f = PyKDL.Frame(PyKDL.Rotation.RPY(0, math.pi/2.0, 0), PyKDL.Vector(0, 0, 0))
  #       f = PyKDL.Frame(PyKDL.Rotation.RPY(math.pi, 0, math.pi/2.0), PyKDL.Vector(0, 0, 0))
  #       pose = posemath.toMsg(posemath.fromMsg(msg.markers[0].pose.pose)*f)

  #       br.sendTransform((pose.position.x, pose.position.y, pose.position.z),
  #                        (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
  #                        rospy.Time.now(),
  #                        frame_id,
  #                        "camera_color_frame")    
  #       # f = PyKDL.Frame(PyKDL.Rotation.RPY(math.pi, 0, math.pi/2.0), PyKDL.Vector(0.1, 0, 0))
  #       f = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, -math.pi/2.0), PyKDL.Vector(0.1, 0, 0))
  #       pose = posemath.toMsg(posemath.fromMsg(msg.markers[0].pose.pose)*f)

  #       br.sendTransform((pose.position.x, pose.position.y, pose.position.z),
  #                        (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
  #                        rospy.Time.now(),
  #                        "test",
  #                        "camera_color_frame")    

  # def armBaseCallback(self, msg):
    # self._arm_base_msg = msg
    # print("%f %f %f", msg.base.tool_pose_theta_x, msg.base.tool_pose_theta_y, msg.base.tool_pose_theta_z)

  def pubMsg(self, linear_X, linear_Y, linear_Z, angular_X, angular_Y, angular_Z, joints, scale_vel):
    if linear_X != 0 or linear_Y != 0 or linear_Z != 0 \
      or angular_X != 0 or angular_Y != 0 or angular_Z != 0:
      cartesian_vel_msg = TwistCommand()
      cartesian_vel_msg.twist.linear_x = linear_X * scale_vel
      cartesian_vel_msg.twist.linear_y = linear_Y * scale_vel
      cartesian_vel_msg.twist.linear_z = linear_Z * scale_vel
      cartesian_vel_msg.twist.angular_x = angular_X * scale_vel
      cartesian_vel_msg.twist.angular_y = angular_Y * scale_vel
      cartesian_vel_msg.twist.angular_z = angular_Z * scale_vel
      self._pub_cartesian_vel.publish(cartesian_vel_msg)
      rospy.loginfo("publish..")

    if joints[0] != 0 or joints[1] != 0 or joints[2] != 0 or joints[3] != 0 \
      or joints[4] !=0 or joints[5] != 0 or joints[6] != 0:

      joint_vel_msg = Base_JointSpeeds()
      joint_vel = JointSpeed()
      joint_vel.value = 0.05
      for i in xrange(0, len(joints)):
        if joints[i] != 0:
          joint_vel.joint_identifier = i
          joint_vel.value = joints[i] * scale_vel
      joint_vel_msg.joint_speeds.append(joint_vel)
      self._pub_joint_vel.publish(joint_vel_msg)

  def pubStepMsg(self, linear_X, linear_Y, linear_Z, angular_X, angular_Y, angular_Z, joints, scale_step):
    if linear_X != 0 or linear_Y != 0 or linear_Z != 0 \
      or angular_X != 0 or angular_Y != 0 or angular_Z != 0:
      # scale_step = 0.05
      feedback = rospy.wait_for_message("/arm/base_feedback", BaseCyclic_Feedback)
      linear_x = feedback.base.tool_pose_x + linear_X * scale_step
      linear_y = feedback.base.tool_pose_y + linear_Y * scale_step
      linear_z = feedback.base.tool_pose_z + linear_Z * scale_step
      angular_x = feedback.base.tool_pose_theta_x + angular_X * scale_step * 100
      angular_y = feedback.base.tool_pose_theta_y + angular_Y * scale_step * 100
      angular_z = feedback.base.tool_pose_theta_z + angular_Z * scale_step * 100
      self.__move_by_pose(linear_x, linear_y, linear_z, angular_x, angular_y, angular_z)

      rospy.loginfo("linearX, %f", linear_X)
      rospy.loginfo("linearY, %f", linear_Y)
      rospy.loginfo("linearZ, %f", linear_Z)

    if joints[0] != 0 or joints[1] != 0 or joints[2] != 0 or joints[3] != 0 \
      or joints[4] !=0 or joints[5] != 0 or joints[6] != 0:
      # scale_step = 0.1
      feedback = rospy.wait_for_message("/arm/base_feedback/joint_state", JointState)
      joint_angles = [feedback.position[0]+joints[0]*scale_step,feedback.position[1]+joints[1]*scale_step,feedback.position[2]+joints[2]*scale_step,\
      feedback.position[3]+joints[3]*scale_step,feedback.position[4]+joints[4]*scale_step,feedback.position[5]+joints[5]*scale_step,feedback.position[6]+joints[6]*scale_step]
      self.__move_by_joint_angle(joint_angles)

  def fingerControl(self, position):
    finger_goal = GripperCommandGoal()
    finger_goal.command.position = position
    self.client.send_goal(finger_goal)
    rospy.loginfo("finger action..")

  def dateCombin(self, data):
    data_combin = "" 
    for index in range(len(data)):
      data_combin = data_combin + data[index]
    return data_combin

  # def arucoDetectCallback(self, msg):
  #   marker_id = "/"
  #   marker = msg.data
  #   if len(marker) == 1:
  #     self._vel_stop = True
  #   for index in range(len(marker)):
  #     marker_id = str(marker[index]) + marker_id
  #   marker_id = marker_id + str(len(marker))
    # self.client_socket.sendto(marker_id, self.server_address)

  # def __wait_for_action_end_or_abort(self):
  #     # Reset notification flag
  #     self._action_finish = False

  #     while not rospy.is_shutdown() and not self._action_finish:
  #         print("wait")
  #         rospy.sleep(0.1)

  def home_the_robot(self):
      # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
      self.last_action_notif_type = None
      req = ReadActionRequest()
      req.input.identifier = self.HOME_ACTION_IDENTIFIER
      try:
          res = self.read_action(req)
      except rospy.ServiceException:
          rospy.logerr("Failed to call ReadAction")
          return False
      # Execute the HOME action if we could read it
      else:
          # What we just read is the input of the ExecuteAction service
          req = ExecuteActionRequest()
          req.input = res.output
          rospy.loginfo("Sending the robot home...")
          try:
              self.execute_action(req)
          except rospy.ServiceException:
              rospy.logerr("Failed to call ExecuteAction")
              return False
          else:
              return self.__wait_for_action_end_or_abort()
              
  def send_gripper_command(self, value):
      # Initialize the request
      # Close the gripper
      req = SendGripperCommandRequest()
      finger = Finger()
      finger.finger_identifier = 0
      finger.value = value
      req.input.gripper.finger.append(finger)
      req.input.mode = GripperMode.GRIPPER_POSITION

      rospy.loginfo("Sending the gripper command...")

      # Call the service 
      try:
          self.send_gripper_command(req)
      except rospy.ServiceException:
          rospy.logerr("Failed to call SendGripperCommand")
          return False
      else:
          time.sleep(0.5)
          return True

  def __move_by_pose(self, linear_X, linear_Y, linear_Z, angular_X, angular_Y, angular_Z):
      self.last_action_notif_type = None
      print("move by pose")
      result = False
      scale_step = 0.05
      req = PlayCartesianTrajectoryRequest()

      req.input.target_pose.x = linear_X
      req.input.target_pose.y = linear_Y
      req.input.target_pose.z = linear_Z
      req.input.target_pose.theta_x = angular_X
      req.input.target_pose.theta_y = angular_Y
      req.input.target_pose.theta_z = angular_Z

      pose_speed = CartesianSpeed()
      pose_speed.translation = self._translation_speed
      pose_speed.orientation = self._orientation_speed

      req.input.constraint.oneof_type.speed.append(pose_speed)
      rospy.loginfo("Sending the robot to the cartesian pose...")
      try:
          self.__send_cartesian_command_srv(req)
      except rospy.ServiceException:
          rospy.logerr("Failed to call PlayCartesianTrajectory")
          return False
      else:
          return self.__wait_for_action_end_or_abort()

      return result

  def __move_by_joint_angle(self, joints):
      self.last_action_notif_type = None
      result = False      
      req = PlayJointTrajectoryRequest()
      for i in range(7):
          if joints[i] < 0.0:
              j = math.degrees(math.pi*2.0+joints[i])
          else:
              j = math.degrees(joints[i])

          temp_angle = JointAngle()
          temp_angle.joint_identifier = i
          temp_angle.value = j
          req.input.joint_angles.joint_angles.append(temp_angle)
          # joint_constraint = JointTrajectoryConstraint()
          # joint_constraint.type = 2
          # joint_constraint.value = self._joint_angle_speed
          # req.input.constraint = joint_constraint
      rospy.loginfo("Sending the robot joint angles...")
      try:
          self.__send_joint_command_srv(req)
      except rospy.ServiceException:
          rospy.logerr("Failed to call PlayJointTrajectory")
          return False
      else:
          return self.__wait_for_action_end_or_abort()

      return result

  def get_tf_pose(self, pose):
    pose_stamped = PoseStamped()
    pose_stamped.pose = pose
    pose_stamped.header.frame_id = "/camera_color_frame"
    # pose_stamped.header.stamp = rospy.Time.now()
    # pose_stamped.header.stamp = rospy.get_rostime()
    rate = rospy.Rate(10.0)
    listener = tf.TransformListener()
    get_pose = False
    while not rospy.is_shutdown() and not get_pose:
      try:
        # now = rospy.Time.now() - rospy.Duration(5.0)
        now = rospy.Time.now()
        listener.waitForTransform("/base_link", "camera_color_frame", now, rospy.Duration(0.3))
        pose_stamped_return = listener.transformPose("/base_link", pose_stamped)
        # print(pose_stamped_return.pose.position.x, pose_stamped_return.pose.position.y, pose_stamped_return.pose.position.z, pose_stamped_return.pose.orientation.x, pose_stamped_return.pose.orientation.y, pose_stamped_return.pose.orientation.z, pose_stamped_return.pose.orientation.w)      
        # print(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3])
        get_pose = True
      except (tf.Exception, tf.LookupException, tf.ConnectivityException):
        print("tf echo error")
        continue
        # return
      rate.sleep()
    return pose_stamped_return

  def getTfTransform(self, base_link, target_link):
    rate = rospy.Rate(10.0)
    listener = tf.TransformListener()
    # listener.waitForTransform("/object_link0", "/base_link", rospy.Time(), rospy.Duration(4.0))
    get_pose = False
    while not rospy.is_shutdown() and not get_pose:
      try:
        # now = rospy.Time.now() - rospy.Duration(5.0)
        now = rospy.Time.now()
        listener.waitForTransform(base_link, target_link, now, rospy.Duration(0.3))
        (trans, rot) = listener.lookupTransform(base_link, target_link, now)
        # print(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3])
        get_pose = True
      except (tf.Exception, tf.LookupException, tf.ConnectivityException):
        print("tf echo error")
        continue
        # return
      rate.sleep()
    return (trans, rot)

  def getDrinkPose(self):
    # self.__move_by_pose(self._pose_ready[0], self._pose_ready[1], self._pose_ready[2], self._pose_ready[3], self._pose_ready[4], self._pose_ready[5])
    # rospy.sleep(1.0)
    # self.__move_by_pose(self._pose_drink[0], self._pose_drink[1], self._pose_drink[2], self._pose_drink[3], self._pose_drink[4], self._pose_drink[5])
    self.__move_by_joint_angle(self._joints_person)
    rospy.sleep(1.0)
    markers = rospy.wait_for_message("/aruco_marker_publisher/markers", MarkerArray)
    length = len(markers.markers)
    pose_drink_marker = Pose()
    if length > 0:    
      if markers.markers[0].id == 102:
        pose_drink_marker = markers.markers[0].pose.pose
        f = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, -math.pi/2.0), PyKDL.Vector(0, 0, 0))
        # f = PyKDL.Frame(PyKDL.Rotation.RPY(math.pi, 0, -math.pi/2.0), PyKDL.Vector(0, 0, 0))
        pose = posemath.toMsg(posemath.fromMsg(pose_drink_marker)*f)
        # print(pose.position.x, pose.position.y, pose.position.z)
        self._pose_stamped_drink_marker = self.get_tf_pose(pose) 
        # print("%f \n %f \n %f", self._pose_stamped_drink_marker.pose.position.x, self._pose_stamped_drink_marker.pose.position.y, self._pose_stamped_drink_marker.pose.position.z )   
        rospy.loginfo("x, %f , y, %f , z, %f", self._pose_stamped_drink_marker.pose.position.x, self._pose_stamped_drink_marker.pose.position.y, self._pose_stamped_drink_marker.pose.position.z )   
        self._drink_pose_saved = True
    else: 
      rospy.loginfo("dont find marker")   
    rospy.sleep(1.0)
    self.__move_by_joint_angle(self._joints_table)
    rospy.loginfo("saved marker")   

  def objectPick(self, id):

    markers = rospy.wait_for_message("/aruco_marker_publisher/markers", MarkerArray)
    length = len(markers.markers)
    if length > 0:
      object_pose = Pose()
      find_object = False
      for i in xrange(0, length):
        if markers.markers[i].id == id:
          object_pose = markers.markers[i].pose.pose
          find_object = True
          rospy.loginfo("find object id %d", id)
      if not find_object:
        rospy.loginfo("find object id %d", id)
        return
      # print(object_pose.position.x, object_pose.position.y, object_pose.position.z)
      f = PyKDL.Frame(PyKDL.Rotation.RPY(math.pi/2.0, math.pi/2.0, 0), PyKDL.Vector(0, 0, 0))
      pose = posemath.toMsg(posemath.fromMsg(object_pose)*f)
      print(pose.position.x, pose.position.y, pose.position.z)
      pose_stamped = self.get_tf_pose(pose)    

      f = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(0, 0, -0.08))
      (pose, theta_x, theta_y, theta_z) = self.Quaternion2Theta(pose_stamped, f)
      self.__move_by_pose(pose.position.x, pose.position.y, pose.position.z, theta_x, theta_y, theta_z)

      rospy.sleep(1.0)
      self.fingerControl(0.0) # finger open
      rospy.sleep(1.0)
      f = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(0, 0, 0))
      (pose, theta_x, theta_y, theta_z) = self.Quaternion2Theta(pose_stamped, f)
      self.__move_by_pose(pose.position.x, pose.position.y, pose.position.z, theta_x, theta_y, theta_z)
      rospy.sleep(1.0)
      self.fingerControl(0.14) # finger close 
      rospy.sleep(1.0)
      f = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(0.0, 0.05,0))
      (pose, theta_x, theta_y, theta_z) = self.Quaternion2Theta(pose_stamped, f)
      self.__move_by_pose(pose.position.x, pose.position.y, pose.position.z, theta_x, theta_y, theta_z)
      rospy.sleep(1.0)
      f = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(0, 0.05, -0.08))
      (pose, theta_x, theta_y, theta_z) = self.Quaternion2Theta(pose_stamped, f)
      self.__move_by_pose(pose.position.x, pose.position.y, pose.position.z, theta_x, theta_y, theta_z)
      rospy.sleep(1.0)

      self.__move_by_joint_angle(self._joints_drink)
      rospy.sleep(1.0)

      # f = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(-0.15, -0.06, 0.27)) # frame : aruco
      f = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(-0.27, 0.10, -0.06)) # frame : nobody konws
      # f = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(0, 0, 0))
      (pose, theta_x, theta_y, theta_z) = self.Quaternion2Theta(self._pose_stamped_drink_marker, f)
      self.__move_by_pose(pose.position.x, pose.position.y, pose.position.z, theta_x, theta_y, theta_z)      
      rospy.sleep(1.0)

      # f = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(0, 0.05,0))
      # (pose, theta_x, theta_y, theta_z) = self.Quaternion2Theta(self._pose_stamped_drink_marker, f)
      # self.__move_by_pose(pose.position.x, pose.position.y, pose.position.z, theta_x, theta_y, theta_z)      
      # rospy.sleep(1.0)

      # f = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(0, 0.1, 0))
      # (pose, theta_x, theta_y, theta_z) = self.Quaternion2Theta(self._pose_stamped_drink_marker, f)
      # self.__move_by_pose(pose.position.x, pose.position.y, pose.position.z, theta_x, theta_y, theta_z)      
      # rospy.sleep(1.0)

      # f = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(0.05, 0.1, 0))
      # (pose, theta_x, theta_y, theta_z) = self.Quaternion2Theta(self._pose_stamped_drink_marker, f)
      # self.__move_by_pose(pose.position.x, pose.position.y, pose.position.z, theta_x, theta_y, theta_z)
      # rospy.sleep(5.0)
      # print("test")
      # feedback = rospy.wait_for_message("/arm/base_feedback/joint_state", JointState)
      # joint_angles = [feedback.position[0],feedback.position[1],feedback.position[2],\
      # feedback.position[3],feedback.position[4],feedback.position[5],feedback.position[6]-0.8]
      # print(feedback.position[0],feedback.position[1],feedback.position[2],\
      # feedback.position[3],feedback.position[4],feedback.position[5],feedback.position[6]-0.8)
      # self.__move_by_joint_angle(joint_angles)
      # rospy.sleep(1.0)
      # f = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(0.10, 0.1, 0))
      # (pose, theta_x, theta_y, theta_z) = self.Quaternion2Theta(self._pose_stamped_drink_marker, f)
      # self.__move_by_pose(pose.position.x, pose.position.y, pose.position.z, theta_x, theta_y, theta_z)
      # rospy.sleep(1.0)
      feedback = rospy.wait_for_message("/arm/base_feedback/joint_state", JointState)
      joint_angles = [feedback.position[0],feedback.position[1],feedback.position[2],\
      feedback.position[3],feedback.position[4],feedback.position[5],feedback.position[6]+0.8]
      self.__move_by_joint_angle(joint_angles)

      self.__move_by_joint_angle(self._joints_table) 

  def Quaternion2Theta(self, pose_stamped, f):
    pose = posemath.toMsg(posemath.fromMsg(pose_stamped.pose)*f)
    rotation = PyKDL.Rotation.Quaternion(pose.orientation.x,
                                         pose.orientation.y,
                                         pose.orientation.z,
                                         pose.orientation.w)
    theta_x = math.degrees(rotation.GetRPY()[0])
    theta_y = math.degrees(rotation.GetRPY()[1])
    theta_z = math.degrees(rotation.GetRPY()[2])
    return (pose, theta_x, theta_y, theta_z)
                
  def Quaternion2EulerXYZ(self, Q_raw):
    qx_ = Q_raw.orientation.x
    qy_ = Q_raw.orientation.y
    qz_ = Q_raw.orientation.z
    qw_ = Q_raw.orientation.w

    tx_ = math.atan2((2 * qw_ * qx_ - 2 * qy_ * qz_), (qw_ * qw_ - qx_ * qx_ - qy_ * qy_ + qz_ * qz_))
    ty_ = math.asin(2 * qw_ * qy_ + 2 * qx_ * qz_)
    tz_ = math.atan2((2 * qw_ * qz_ - 2 * qx_ * qy_), (qw_ * qw_ + qx_ * qx_ - qy_ * qy_ - qz_ * qz_))
    EulerXYZ_ = [tx_,ty_,tz_]
    return EulerXYZ_

  def getBaseMoveMsg(self, data_combin, linear_X, linear_Y, linear_Z, angular_X, angular_Y, angular_Z, joint_4, joint_5, joint_6, joint_7, movePlane):
    if data_combin == "UP":
      
      linear_X = 0
      linear_Y = 0
      # print("1")
      linear_Z = 1.0
      movePlane= "UP"
    if data_combin == "LF":
      if movePlane == "UP":
        linear_X = 1.0
        linear_Y = -1.0
        linear_Z = 1.0
      elif movePlane == "MIDDLE":
        linear_X = 1.0
        linear_Y = -1.0
        linear_Z = 0
      else:
        linear_X = 1.0
        linear_Y = -1.0
        linear_Z = -1.0
    if data_combin == "F":
      if movePlane == "UP":
        linear_X = 0
        linear_Y = -1.0
        linear_Z = 1.0
      elif movePlane == "MIDDLE":
        linear_X = 0
        linear_Y = -1.0
        linear_Z = 0
      else:
        linear_X = 0
        linear_Y = -1.0
        linear_Z = -1.0
    if data_combin == "RF":
      if movePlane == "UP":
        linear_X = -1.0
        linear_Y = -1.0
        linear_Z = 1.0
      elif movePlane == "MIDDLE":
        linear_X = -1.0
        linear_Y = -1.0
        linear_Z = 0
      else:
        linear_X = -1.0
        linear_Y = -1.0
        linear_Z = -1.0
    if data_combin == "MIDDLE":
      linear_X = 0
      linear_Y = 0
      linear_Z = 0
      movePlane= "MIDDLE"
    if data_combin == "L":
      if movePlane == "UP":
        linear_X = 1.0
        linear_Y = 0
        linear_Z = 1.0
      elif movePlane == "MIDDLE":
        linear_X = 1.0
        linear_Y = 0
        linear_Z = 0
      else:
        linear_X = 1.0
        linear_Y = 0
        linear_Z = -1.0
    if data_combin == "R":
      if movePlane == "UP":
        linear_X = -1.0
        linear_Y = 0
        linear_Z = 1.0
      elif movePlane == "MIDDLE":
        linear_X = -1.0
        linear_Y = 0
        linear_Z = 0
      else:
        linear_X = -1.0
        linear_Y = 0
        linear_Z = -1.0
    if data_combin == "DOWN":
      linear_X = 0
      linear_Y = 0
      linear_Z = -1.0
      movePlane= "DOWN"
    if data_combin == "LB":
      if movePlane == "UP":
        linear_X = 1.0
        linear_Y = 1.0
        linear_Z = 1.0
      elif movePlane == "MIDDLE":
        linear_X = 1.0
        linear_Y = 1.0
        linear_Z = 0
      else:
        linear_X = 1.0
        linear_Y = 1.0
        linear_Z = -1.0
    if data_combin == "B":
      if movePlane == "UP":
        linear_X = 0
        linear_Y = 1.0
        linear_Z = 1.0
      elif movePlane == "MIDDLE":
        linear_X = 0
        linear_Y = 1.0
        linear_Z = 0
      else:
        linear_X = 0
        linear_Y = 1.0
        linear_Z = -1.0
    if data_combin == "RB":
      if movePlane == "UP":
        linear_X = -1.0
        linear_Y = 1.0
        linear_Z = 1.0
      elif movePlane == "MIDDLE":
        linear_X = -1.0
        linear_Y = 1.0
        linear_Z = 0
      else:
        linear_X = -1.0
        linear_Y = 1.0
        linear_Z = -1.0
    if data_combin == "X+":
      linear_X = 1.0
      linear_Y = 0
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "X-":
      linear_X = -1.0
      linear_Y = 0
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
      # rospy.loginfo("X-")
    if data_combin == "Y+ ":
      linear_X = 0
      linear_Y = 1.0
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "Y-":
      linear_X = 0
      linear_Y = -1.0
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "Z+":
      linear_X = 0
      linear_Y = 0
      linear_Z = 1.0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "Z-":
      linear_X = 0
      linear_Y = 0
      linear_Z = -1.0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "AX+":
      linear_X = 0
      linear_Y = 0
      linear_Z = 0
      angular_X = 1.0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "AX-":
      linear_X = 0
      linear_Y = 0
      linear_Z = 0
      angular_X = -1.0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "AY+":
      linear_X = 0
      linear_Y = 0
      linear_Z = 0
      angular_X = 0
      angular_Y = 1.0
      angular_Z = 0
    if data_combin == "AY-":
      linear_X = 0
      linear_Y = 0
      linear_Z = 0
      angular_X = 0
      angular_Y = -1.0
      angular_Z = 0
    if data_combin == "AZ+":
      linear_X = 0
      linear_Y = 0
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 1.0
    if data_combin == "AZ-":
      linear_X = 0
      linear_Y = 0
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = -1.0
    if data_combin == "J4+":
      joint_4 = 1.0
      joint_5 = 0
      joint_6 = 0
      joint_7 = 0
      linear_X = 0
      linear_Y = 0 
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "J4-":
      joint_4 = -1.0
      joint_5 = 0
      joint_6 = 0
      joint_7 = 0
      linear_X = 0
      linear_Y = 0 
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "J5+":
      joint_4 = 0
      joint_5 = 1.0
      joint_6 = 0
      joint_7 = 0
      linear_X = 0
      linear_Y = 0 
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "J5-":
      joint_4 = 0
      joint_5 = -1.0
      joint_6 = 0
      joint_7 = 0
      linear_X = 0
      linear_Y = 0 
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "J6+":
      joint_4 = 0
      joint_5 = 0
      joint_6 = 1.0
      joint_7 = 0
      linear_X = 0
      linear_Y = 0 
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "J6-":
      joint_4 = 0
      joint_5 = 0
      joint_6 = -1.0
      joint_7 = 0
      linear_X = 0
      linear_Y = 0 
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "J7+":
      joint_4 = 0
      joint_5 = 0
      joint_6 = 0
      joint_7 = 1.0
      linear_X = 0
      linear_Y = 0 
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "J7-":
      joint_4 = 0
      joint_5 = 0
      joint_6 = 0
      joint_7 = -1.0
      linear_X = 0
      linear_Y = 0 
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    # if data_combin == "Stop":
    #     self._pub_stop.publish(self._empty_msg)
    #     # self._moving = False
    #     linear_X = 0
    #     linear_Y = 0 
    #     linear_Z = 0
    #     angular_X = 0
    #     angular_Y = 0
    #     angular_Z = 0
    #     joint_4 = 0
    #     joint_5 = 0
    #     joint_6 = 0
    return linear_X, linear_Y, linear_Z, angular_X, angular_Y, angular_Z, joint_4, joint_5, joint_6, joint_7, movePlane
    # return movePlane


  def getBaseMoveMsgGen3(self, data_combin, linear_X, linear_Y, linear_Z, angular_X, angular_Y, angular_Z, joint_4, joint_5, joint_6, joint_7, movePlane):
    if data_combin == "UP":
      # linear_X = 0
      # linear_Y = 0
      # print("1")
      linear_Z = 1.0
      movePlane= "UP"
    if data_combin == "LF":
      if movePlane == "UP":
        linear_X = 1.0
        linear_Y = 1.0
        linear_Z = 1.0
      elif movePlane == "MIDDLE":
        linear_X = 1.0
        linear_Y = 1.0
        linear_Z = 0
      else:
        linear_X = 1.0
        linear_Y = 1.0
        linear_Z = -1.0
    if data_combin == "F":
      if movePlane == "UP":
        linear_X = 1.0
        linear_Y = 0
        linear_Z = 1.0
      elif movePlane == "MIDDLE":
        linear_X = 1.0
        linear_Y = 0
        linear_Z = 0
      else:
        linear_X = 1.0
        linear_Y = 0
        linear_Z = -1.0
    if data_combin == "RF":
      if movePlane == "UP":
        linear_X = 1.0
        linear_Y = -1.0
        linear_Z = 1.0
      elif movePlane == "MIDDLE":
        linear_X = 1.0
        linear_Y = -1.0
        linear_Z = 0
      else:
        linear_X = 1.0
        linear_Y = -1.0
        linear_Z = -1.0
    if data_combin == "MIDDLE":
      # linear_X = 0.0
      # linear_Y = 0.0
      linear_Z = 0.0
      movePlane= "MIDDLE"
    if data_combin == "L":
      if movePlane == "UP":
        linear_X = 0
        linear_Y = 1.0
        linear_Z = 1.0
      elif movePlane == "MIDDLE":
        linear_X = 0
        linear_Y = 1.0
        linear_Z = 0
      else:
        linear_X = 0
        linear_Y = 1.0
        linear_Z = -1.0
    if data_combin == "R":
      if movePlane == "UP":
        linear_X = 0
        linear_Y = -1.0
        linear_Z = 1.0
      elif movePlane == "MIDDLE":
        linear_X = 0
        linear_Y = -1.0
        linear_Z = 0
      else:
        linear_X = 0
        linear_Y = -1.0
        linear_Z = -1.0
    if data_combin == "DOWN":
      # linear_X = 0
      # linear_Y = 0
      linear_Z = -1.0
      movePlane= "DOWN"
    if data_combin == "LB":
      if movePlane == "UP":
        linear_X = -1.0
        linear_Y = 1.0
        linear_Z = 1.0
      elif movePlane == "MIDDLE":
        linear_X = -1.0
        linear_Y = 1.0
        linear_Z = 0
      else:
        linear_X = -1.0
        linear_Y = 1.0
        linear_Z = -1.0
    if data_combin == "B":
      if movePlane == "UP":
        linear_X = -1.0
        linear_Y = 0
        linear_Z = 1.0
      elif movePlane == "MIDDLE":
        linear_X = -1.0
        linear_Y = 0
        linear_Z = 0
      else:
        linear_X = -1.0
        linear_Y = 0
        linear_Z = -1.0
    if data_combin == "RB":
      if movePlane == "UP":
        linear_X = -1.0
        linear_Y = -1.0
        linear_Z = 1.0
      elif movePlane == "MIDDLE":
        linear_X = -1.0
        linear_Y = -1.0
        linear_Z = 0
      else:
        linear_X = -1.0
        linear_Y = -1.0
        linear_Z = -1.0
    if data_combin == "X+":
      linear_X = 1.0
      linear_Y = 0
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "X-":
      linear_X = -1.0
      linear_Y = 0
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
      # rospy.loginfo("X-")
    if data_combin == "Y+ ":
      linear_X = 0
      linear_Y = 1.0
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "Y-":
      linear_X = 0
      linear_Y = -1.0
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "Z+":
      linear_X = 0
      linear_Y = 0
      linear_Z = 1.0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "Z-":
      linear_X = 0
      linear_Y = 0
      linear_Z = -1.0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "AX+":
      linear_X = 0
      linear_Y = 0
      linear_Z = 0
      angular_X = 1.0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "AX-":
      linear_X = 0
      linear_Y = 0
      linear_Z = 0
      angular_X = -1.0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "AY+":
      linear_X = 0
      linear_Y = 0
      linear_Z = 0
      angular_X = 0
      angular_Y = 1.0
      angular_Z = 0
    if data_combin == "AY-":
      linear_X = 0
      linear_Y = 0
      linear_Z = 0
      angular_X = 0
      angular_Y = -1.0
      angular_Z = 0
    if data_combin == "AZ+":
      linear_X = 0
      linear_Y = 0
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 1.0
    if data_combin == "AZ-":
      linear_X = 0
      linear_Y = 0
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = -1.0
    if data_combin == "J4+":
      joint_4 = 1.0
      joint_5 = 0
      joint_6 = 0
      joint_7 = 0
      linear_X = 0
      linear_Y = 0 
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "J4-":
      joint_4 = -1.0
      joint_5 = 0
      joint_6 = 0
      joint_7 = 0
      linear_X = 0
      linear_Y = 0 
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "J5+":
      joint_4 = 0
      joint_5 = 1.0
      joint_6 = 0
      joint_7 = 0
      linear_X = 0
      linear_Y = 0 
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "J5-":
      joint_4 = 0
      joint_5 = -1.0
      joint_6 = 0
      joint_7 = 0
      linear_X = 0
      linear_Y = 0 
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "J6+":
      joint_4 = 0
      joint_5 = 0
      joint_6 = 1.0
      joint_7 = 0
      linear_X = 0
      linear_Y = 0 
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "J6-":
      joint_4 = 0
      joint_5 = 0
      joint_6 = -1.0
      joint_7 = 0
      linear_X = 0
      linear_Y = 0 
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "J7+":
      joint_4 = 0
      joint_5 = 0
      joint_6 = 0
      joint_7 = 1.0
      linear_X = 0
      linear_Y = 0 
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "J7-":
      joint_4 = 0
      joint_5 = 0
      joint_6 = 0
      joint_7 = -1.0
      linear_X = 0
      linear_Y = 0 
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    # if data_combin == "Stop":
    #     self._pub_stop.publish(self._empty_msg)
    #     # self._moving = False
    #     linear_X = 0
    #     linear_Y = 0 
    #     linear_Z = 0
    #     angular_X = 0
    #     angular_Y = 0
    #     angular_Z = 0
    #     joint_4 = 0
    #     joint_5 = 0
    #     joint_6 = 0
    return linear_X, linear_Y, linear_Z, angular_X, angular_Y, angular_Z, joint_4, joint_5, joint_6, joint_7, movePlane
    # return movePlane  

def main():
  rospy.init_node('brain_control_interface')

  BCI = BrainControlInterface()
  time.sleep(0.5)
  # BCI.__move_by_pose()
  # BCI.UDPLinstenLoop()
  t0 = threading.Thread(target=BCI.UDPLinstenLoop,args=())
  t0.start()
  rospy.spin()

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass
