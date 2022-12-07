#!/usr/bin/env python3

import serial
import rospy
from std_msgs.msg import Int32
import time

ser=serial.Serial('/dev/ttyACM0',115200,timeout=1)
time.sleep(2)

rospy.init_node("driver_potentiometer")

value_pub=rospy.Publisher("joy_knob",Int32,queue_size=1)

waitline=10
waitline_cnt=0
while not rospy.is_shutdown():
    line=ser.readline()
    if waitline_cnt<waitline:
        waitline_cnt+=1
        continue
    try:
        if line:
            value=int(line.decode())
            msg=Int32()
            msg.data=value
            value_pub.publish(msg)
    except:
        continue
    
    # if line:
    #     line_str=line.decode()
    #     if line_str[:2]=='dd':
    #         value=int(line_str[2:])
    #         msg=Int32()
    #         msg.data=value
    #         value_pub.publish(msg)
    
    # time.sleep(0.001)
