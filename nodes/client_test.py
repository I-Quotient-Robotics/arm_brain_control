#!/usr/bin/env python
import socket
import time
import rospy
from std_msgs.msg import UInt32MultiArray

class ArucoDetectClient:
  def __init__(self):
    try:
      self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      # self.server_address = ("192.168.43.40", 8848)
      self.server_address = ("127.0.0.1", 8848)
      # self.server_address = ("", 8848)

    except:
      print 'Build UDP socket error!! exit~'
      exit(-1)

def main():
  rospy.init_node('aruco_detect_client')
  ADC = ArucoDetectClient()
  # ADC.client_socket.sendto("UP", ADC.server_address)
  # ADC.client_socket.sendto("FigOpen", ADC.server_address)
  # ADC.client_socket.sendto("J5-", ADC.server_address)
  # time.sleep(3)
  # ADC.client_socket.sendto("Y-", ADC.server_address)
  ADC.client_socket.sendto("Object1", ADC.server_address)
  # ADC.client_socket.sendto("DrinkPose", ADC.server_address)
  rospy.spin()

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass