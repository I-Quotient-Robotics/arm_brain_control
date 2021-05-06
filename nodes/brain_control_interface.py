#!/usr/bin/env python
import time
import math
import rospy
import socket
import threading
import actionlib
import select
from kinova_msgs.srv import AddPoseToCartesianTrajectory, AddPoseToCartesianTrajectoryRequest
from brain_control.msg import brain_control_msg
from brain_control.srv import vision_grasp
from kinova_msgs.msg import SetFingersPositionAction
from kinova_msgs.msg import SetFingersPositionGoal
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from kinova_msgs.srv import HomeArm
from std_msgs.msg import UInt32MultiArray

class BrainControlInterface:
  def __init__(self):
    self._kinova_pose_init = Pose()
    self._sub = rospy.Subscriber("j2n6s300_driver/out/tool_pose", PoseStamped, self.kinovaPoseInitCallback)
    self._position_control_client = rospy.ServiceProxy('j2n6s300_driver/in/add_pose_to_Cartesian_trajectory', AddPoseToCartesianTrajectory)
    self._sub = rospy.Subscriber("aruco_marker_publisher/markers_list", UInt32MultiArray, self.arucoDetectCallback)
    self._pub = rospy.Publisher('brain_control_msg', brain_control_msg, queue_size=1)
    self._grasp_client = rospy.ServiceProxy('vision_grasp', vision_grasp)
    self._home_client = rospy.ServiceProxy('j2n6s300_driver/in/home_arm', HomeArm)
    self.client = actionlib.SimpleActionClient("j2n6s300_driver/fingers_action/finger_positions", SetFingersPositionAction)
    self.stop = False
    self.move_point = False
    try:
      self._udpSocket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
      self._udpSocket_send = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

      # computer_liuhang
      self._udpSocket.bind(("192.168.43.55", 8848)) #receive
      # self._udpSocket.bind(("192.168.3.82", 8848)) #receive
      self.server_address = ("192.186.43.29", 8848) #send
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
    mode = 1
    scale_vel = 0.04

    movePlane = "PRESENT"

    while not rospy.is_shutdown():

      rospy.loginfo("linste..")
      # rospy.loginfo("%s" ,linear_X)

      # try:
      #   data, addr = self._udpSocket.recvfrom(1024)
      #   # rospy.loginfo("linste end..")
      #   # rospy.loginfo("%s", type(data))
      # except :
      #   print 'recvfrom error'
      #   exit(-1)
      timeout=60*60*5
      self._udpSocket.settimeout(timeout)
      data, addr = self._udpSocket.recvfrom(1024)

      
      data_combin = self.dateCombin(data)
      rospy.loginfo("%s", data_combin)

      # rospy.loginfo("%s" ,linear_Y)

      if data_combin == "ModeSwitch":
        if mode == 0:
          rospy.loginfo("changed to mode 1 / step")
          mode = 1
          scale_vel = 2
        elif mode == 1:
          rospy.loginfo("changed to mode 0 / continuous")
          mode = 0
          scale_vel = 0.04

      self.stop = False

      # if data_combin == "X+" or data_combin == "LF" or data_combin == "RF":
      #   linear_X = 1
      # if data_combin == "X-" or data_combin == "LB" or data_combin == "RB":
      #   linear_X = -1
      #   # rospy.loginfo("X-")
      # if data_combin == "Y+" or data_combin == "LF" or data_combin == "RF":
      #   linear_Y = 1
      # if data_combin == "Y-" or data_combin == "LB" or data_combin == "RB":
      #   linear_Y = -1
      # if data_combin == "Z+" or data_combin == "UP":
      #   linear_Z = 1
      # if data_combin == "Z-" or data_combin == "DOWN":
      #   linear_Z = -1
      # if data_combin == "MIDDLE":
      #   linear_Z = 0
      # if data_combin == "AX+":
      #   angular_X = 1
      # if data_combin == "AX-":
      #   angular_X = -1
      # if data_combin == "AY+":
      #   angular_Y = 1
      # if data_combin == "AY-":
      #   angular_Y = -1
      # if data_combin == "AZ+":
      #   angular_Z = 1
      # if data_combin == "AZ-":
      #   angular_Z = -1
      # if data_combin == "J4+":
      #   joint_4 = 1
      # if data_combin == "J4-":
      #   joint_4 = -1
      # if data_combin == "J5+":
      #   joint_5 = 1
      # if data_combin == "J5-":
      #   joint_5 = -1
      # if data_combin == "J6+":
      #   joint_6 = 1
      # if data_combin == "J6-":
      #   joint_6 = -1
      if data_combin == "Stop":
        self.stop = True
        linear_X = 0
        linear_Y = 0 
        linear_Z = 0
        angular_X = 0
        angular_Y = 0
        angular_Z = 0
        joint_4 = 0
        joint_5 = 0
        joint_6 = 0
      
      # if mode == 0:
      #   rospy.loginfo("mode 0")
      #   self.pubThread(linear_X, linear_Y, linear_Z, angular_X, angular_Y, angular_Z, joint_4, joint_5, joint_6)
      # elif mode == 1:
      #   rospy.loginfo("mode 1")
      #   self.kinovaPoseClient(linear_X, linear_Y, linear_Z, angular_X, angular_Y, angular_Z, joint_4, joint_5, joint_6)
        # linear_X = 0
        # linear_Y = 0 
        # linear_Z = 0
        # angular_X = 0
        # angular_Y = 0
        # angular_Z = 0

      linear_X, linear_Y, linear_Z, angular_X, angular_Y, angular_Z, joint_4, joint_5, joint_6, movePlane \
      = self.getBaseMoveMsg(data_combin, linear_X, linear_Y, linear_Z, angular_X, angular_Y, angular_Z, joint_4, joint_5, joint_6, movePlane)

      # print (result)
      print ("linear_X: %f, linear_Y: %f, linear_Z: %f" % (linear_X, linear_Y, linear_Z))

      if data_combin == "FigOpen":
        self.fingerControl(0)
      elif data_combin == "FigClose":
        self.fingerControl(6000)
      if data_combin == "Object1":
        self.visionGrasp("10")
      if data_combin == "Object2":
        self.visionGrasp("63")
      if data_combin == "Object3":
        self.visionGrasp("46")
      if data_combin == "Home":
        self._home_client.call()
      if not self.move_point:
        self.pubThread(linear_X, linear_Y, linear_Z, angular_X, angular_Y, angular_Z, joint_4, joint_5, joint_6, mode, scale_vel)

      # linear_X = 0
      # linear_Y = 0
      # linear_Z = 0
      # angular_X = 0
      # angular_Y = 0
      # angular_Z = 0
      data_combin = "/"

      # self.visionGrasp()

      # self.fingerControl(position)
      
      # self.pubMsg(arm_enable, linear_X, linear_Y, linear_Z, angular_X, angular_Y, angular_Z, joint_4, joint_5, joint_6)

  def pubMsg(self, linear_X, linear_Y, linear_Z, angular_X, angular_Y, angular_Z, joint_4, joint_5, joint_6, mode, scale_vel):
    msg = brain_control_msg()
    msg.header.frame_id = ''
    msg.header.stamp = rospy.Time.now()
    msg.linearX = linear_X * scale_vel
    msg.linearY = linear_Y * scale_vel
    msg.linearZ = linear_Z * scale_vel
    msg.angularX = angular_X * scale_vel
    msg.angularY = angular_Y * scale_vel
    msg.angularZ = angular_Z * scale_vel
    msg.joint_4 = joint_4
    msg.joint_5 = joint_5
    msg.joint_6 = joint_6
    # rospy.loginfo("%s, %s", str(con_num), str(con_key))
    # rospy.loginfo("linearX, %f", msg.linearX)
    # rospy.loginfo("linearY, %f", msg.linearY)
    # rospy.loginfo("linearZ, %f", msg.linearZ)

    rospy.loginfo("publish..")
    if mode == 0:
      while not self.stop:
        self._pub.publish(msg)
      # rospy.loginfo("start publish..")
    elif mode == 1:
      self.move_point = True
      for index in range(10000):
        self._pub.publish(msg)
      self.move_point = False

  def getBaseMoveMsg(self, data_combin, linear_X, linear_Y, linear_Z, angular_X, angular_Y, angular_Z, joint_4, joint_5, joint_6, movePlane):
    if data_combin == "UP":
      linear_X = 0
      linear_Y = 0
      # print("1")
      linear_Z = 1
      movePlane= "UP"
    if data_combin == "LF":
      if movePlane == "UP":
        linear_X = 1
        linear_Y = -1
        linear_Z = 1
      elif movePlane == "MIDDLE":
        linear_X = 1
        linear_Y = -1
        linear_Z = 0
      else:
        linear_X = 1
        linear_Y = -1
        linear_Z = -1
    if data_combin == "F":
      if movePlane == "UP":
        linear_X = 0
        linear_Y = -1
        linear_Z = 1
      elif movePlane == "MIDDLE":
        linear_X = 0
        linear_Y = -1
        linear_Z = 0
      else:
        linear_X = 0
        linear_Y = -1
        linear_Z = -1
    if data_combin == "RF":
      if movePlane == "UP":
        linear_X = -1
        linear_Y = -1
        linear_Z = 1
      elif movePlane == "MIDDLE":
        linear_X = -1
        linear_Y = -1
        linear_Z = 0
      else:
        linear_X = -1
        linear_Y = -1
        linear_Z = -1
    if data_combin == "MIDDLE":
      linear_X = 0
      linear_Y = 0
      linear_Z = 0
      movePlane= "MIDDLE"
    if data_combin == "L":
      if movePlane == "UP":
        linear_X = 1
        linear_Y = 0
        linear_Z = 1
      elif movePlane == "MIDDLE":
        linear_X = 1
        linear_Y = 0
        linear_Z = 0
      else:
        linear_X = 1
        linear_Y = 0
        linear_Z = -1
    if data_combin == "R":
      if movePlane == "UP":
        linear_X = -1
        linear_Y = 0
        linear_Z = 1
      elif movePlane == "MIDDLE":
        linear_X = -1
        linear_Y = 0
        linear_Z = 0
      else:
        linear_X = -1
        linear_Y = 0
        linear_Z = -1
    if data_combin == "DOWN":
      linear_X = 0
      linear_Y = 0
      linear_Z = -1
      movePlane= "DOWN"
    if data_combin == "LB":
      if movePlane == "UP":
        linear_X = 1
        linear_Y = 1
        linear_Z = 1
      elif movePlane == "MIDDLE":
        linear_X = 1
        linear_Y = 1
        linear_Z = 0
      else:
        linear_X = 1
        linear_Y = 1
        linear_Z = -1
    if data_combin == "B":
      if movePlane == "UP":
        linear_X = 0
        linear_Y = 1
        linear_Z = 1
      elif movePlane == "MIDDLE":
        linear_X = 0
        linear_Y = 1
        linear_Z = 0
      else:
        linear_X = 0
        linear_Y = 1
        linear_Z = -1
    if data_combin == "RB":
      if movePlane == "UP":
        linear_X = -1
        linear_Y = 1
        linear_Z = 1
      elif movePlane == "MIDDLE":
        linear_X = -1
        linear_Y = 1
        linear_Z = 0
      else:
        linear_X = -1
        linear_Y = 1
        linear_Z = -1
    if data_combin == "X+":
      linear_X = 1
      linear_Y = 0
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "X-":
      linear_X = -1
      linear_Y = 0
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
      # rospy.loginfo("X-")
    if data_combin == "Y+":
      linear_X = 0
      linear_Y = 1
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "Y-":
      linear_X = 0
      linear_Y = -1
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "Z+":
      linear_X = 0
      linear_Y = 0
      linear_Z = 1
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "Z-":
      linear_X = 0
      linear_Y = 0
      linear_Z = -1
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "AX+":
      linear_X = 0
      linear_Y = 0
      linear_Z = 0
      angular_X = 1
      angular_Y = 0
      angular_Z = 0
    if data_combin == "AX-":
      linear_X = 0
      linear_Y = 0
      linear_Z = 0
      angular_X = -1
      angular_Y = 0
      angular_Z = 0
    if data_combin == "AY+":
      linear_X = 0
      linear_Y = 0
      linear_Z = 0
      angular_X = 0
      angular_Y = 1
      angular_Z = 0
    if data_combin == "AY-":
      linear_X = 0
      linear_Y = 0
      linear_Z = 0
      angular_X = 0
      angular_Y = -1
      angular_Z = 0
    if data_combin == "AZ+":
      linear_X = 0
      linear_Y = 0
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 1
    if data_combin == "AZ-":
      linear_X = 0
      linear_Y = 0
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = -1
    if data_combin == "J4+":
      joint_4 = 1
      joint_5 = 0
      joint_6 = 0
      linear_X = 0
      linear_Y = 0 
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "J4-":
      joint_4 = -1
      joint_5 = 0
      joint_6 = 0
      linear_X = 0
      linear_Y = 0 
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "J5+":
      joint_4 = 0
      joint_5 = 1
      joint_6 = 0
      linear_X = 0
      linear_Y = 0 
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "J5-":
      joint_4 = 0
      joint_5 = -1
      joint_6 = 0
      linear_X = 0
      linear_Y = 0 
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "J6+":
      joint_4 = 0
      joint_5 = 0
      joint_6 = 1
      linear_X = 0
      linear_Y = 0 
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "J6-":
      joint_4 = 0
      joint_5 = 0
      joint_6 = -1
      linear_X = 0
      linear_Y = 0 
      linear_Z = 0
      angular_X = 0
      angular_Y = 0
      angular_Z = 0
    if data_combin == "Stop":
        self.stop = True
        linear_X = 0
        linear_Y = 0 
        linear_Z = 0
        angular_X = 0
        angular_Y = 0
        angular_Z = 0
        joint_4 = 0
        joint_5 = 0
        joint_6 = 0
    return linear_X, linear_Y, linear_Z, angular_X, angular_Y, angular_Z, joint_4, joint_5, joint_6, movePlane
    # return movePlane

  def fingerControl(self, position):
    finger_goal = SetFingersPositionGoal()
    finger_goal.fingers.finger1 = position
    finger_goal.fingers.finger2 = position
    finger_goal.fingers.finger3 = position
    self.client.send_goal(finger_goal)
    # rospy.loginfo("finger action..")

  def visionGrasp(self, obj_number):
    resp = self._grasp_client.call(obj_number) # call   send_goal  _grasp_client
    # rospy.loginfo("Message From server:%s"%resp.grasp_finished)

  def dateCombin(self, data):
    data_combin = "" 
    for index in range(len(data)):
      data_combin = data_combin + data[index]
    return data_combin

  def pubThread(self, linear_X, linear_Y, linear_Z, angular_X, angular_Y, angular_Z, joint_4, joint_5, joint_6, mode, scale_vel):
    t1 = threading.Thread(target=self.pubMsg,args=(linear_X, linear_Y, linear_Z, angular_X, angular_Y, angular_Z, joint_4, joint_5, joint_6, mode, scale_vel))
    t1.start()

  def kinovaPoseInitCallback(self, msg):
    # rospy.loginfo("finger callback..")
    self._kinova_pose_init.position.x = msg.pose.position.x
    self._kinova_pose_init.position.y = msg.pose.position.y
    self._kinova_pose_init.position.z = msg.pose.position.z
    self._kinova_pose_init.orientation.x = msg.pose.orientation.x
    self._kinova_pose_init.orientation.y = msg.pose.orientation.y
    self._kinova_pose_init.orientation.z = msg.pose.orientation.z
    self._kinova_pose_init.orientation.w = msg.pose.orientation.w

  def arucoDetectCallback(self, msg):
    marker_id = "/"
    marker = msg.data
    if len(marker) == 1:
      self.stop = True
    for index in range(len(marker)):
      marker_id = str(marker[index]) + marker_id
    marker_id = marker_id + str(len(marker))
    # self.client_socket.sendto(marker_id, self.server_address)

  # def kinovaPoseClient(self,linear_X, linear_Y, linear_Z, angular_X, angular_Y, angular_Z, joint_4, joint_5, joint_6):
  #   scale_linear = 10.0
  #   scale_theta = 10.0
  #   # kinova_pose = AddPoseToCartesianTrajectory()
  #   thetaXYZ = Quaternion2EulerXYZ(self._kinova_pose_init)
  #   kinova_pose_X = self._kinova_pose_init.position.x + scale_linear * linear_X
  #   kinova_pose_Y = self._kinova_pose_init.position.y + scale_linear * linear_Y
  #   kinova_pose_Z = self._kinova_pose_init.position.z + scale_linear * linear_Z
  #   kinova_pose_ThetaX = thetaXYZ[0] + scale_theta * angular_X
  #   kinova_pose_ThetaY = thetaXYZ[1] + scale_theta * angular_Y
  #   kinova_pose_ThetaZ = thetaXYZ[2] + scale_theta * angular_Z
  #   rospy.loginfo("%f, %f, %f, %f, %f, %f", self._kinova_pose_init.position.x, self._kinova_pose_init.position.y, kinova_pose_Z, kinova_pose_ThetaX, kinova_pose_ThetaY,kinova_pose_ThetaZ)
  #   rospy.loginfo("%f, %f, %f, %f, %f, %f", kinova_pose_X, kinova_pose_Y, kinova_pose_Z, kinova_pose_ThetaX, kinova_pose_ThetaY,kinova_pose_ThetaZ)
  #   kinova_pose_request = AddPoseToCartesianTrajectoryRequest(kinova_pose_X, kinova_pose_Y, kinova_pose_Z, kinova_pose_ThetaX, kinova_pose_ThetaY,kinova_pose_ThetaZ)

  #   if linear_X != 0 or linear_Y != 0 or linear_Z != 0 or angular_X != 0 or angular_Y != 0 or angular_Z != 0:
  #     try:
  #       self._position_control_client.call(kinova_pose_request)
  #     except rospy.ServiceException as e:
  #           pass
  #     rospy.loginfo("position action..")

def Quaternion2EulerXYZ(Q_raw):
    qx_ = Q_raw.orientation.x
    qy_ = Q_raw.orientation.y
    qz_ = Q_raw.orientation.z
    qw_ = Q_raw.orientation.w

    tx_ = math.atan2((2 * qw_ * qx_ - 2 * qy_ * qz_), (qw_ * qw_ - qx_ * qx_ - qy_ * qy_ + qz_ * qz_))
    ty_ = math.asin(2 * qw_ * qy_ + 2 * qx_ * qz_)
    tz_ = math.atan2((2 * qw_ * qz_ - 2 * qx_ * qy_), (qw_ * qw_ + qx_ * qx_ - qy_ * qy_ - qz_ * qz_))
    EulerXYZ_ = [tx_,ty_,tz_]
    return EulerXYZ_

def main():
  rospy.init_node('brain_control_interface')

  BCI = BrainControlInterface()
  # BCI.UDPLinstenLoop()
  t0 = threading.Thread(target=BCI.UDPLinstenLoop,args=())
  t0.start()
  rospy.spin()

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass
