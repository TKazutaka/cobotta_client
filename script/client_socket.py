#!/usr/bin/env python
import socket
import struct
from time import sleep
import rospy
from geometry_msgs.msg import Vector3

if __name__ == "__main__":
    rospy.init_node("client_socket")
    object_pos_pub = rospy.Publisher("/object_position", Vector3, queue_size=10)
    port_number = rospy.get_param("~port_number", 50000)
    buf_size = rospy.get_param("~object_pose_buffer_size", 24)
    ip_addr = rospy.get_param("~master_ip_address", "192.168.0.100")

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((ip_addr, port_number))
    while not rospy.is_shutdown():
        data = s.recv(buf_size)
        try:
            res = struct.unpack('<ddd', data)
            print(res)
            msg = Vector3()
            msg.x = res[0]
            msg.y = res[1]
            msg.z = res[2]
            object_pos_pub.publish(msg)
        except:
            sleep(1.0)
    s.close()
