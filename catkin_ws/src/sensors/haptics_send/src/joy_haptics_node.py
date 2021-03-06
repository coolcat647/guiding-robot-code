#!/usr/bin/python
import rospy
import sys
import math
from BluetoothService import *
from bitstring import BitArray
from haptics_msgs.msg import VibrationArray
from sensor_msgs.msg import Joy

class JoyHaptics(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.vb_msg = VibrationArray()

        # Publisher
        self.pub_vib = rospy.Publisher("vibrate_cmd", VibrationArray, queue_size=1)

        # Subscriber
        self.sub_joy = rospy.Subscriber("joy", Joy, self.joy_cb, queue_size=1)

    def joy_cb(self, msg):
        self.vb_msg = VibrationArray() # clear vibration msg
        self.vb_msg.frequencies = [0, 0, 0]
        self.vb_msg.intensities = [0, 0, 0]

        if msg.axes[1] > 0:                     # front
            self.vb_msg.frequencies[1], self.vb_msg.intensities[1] = self.vb_pattern_select(round(abs(msg.axes[1]) * 2))

        if msg.axes[0] > 0:                     # left
            self.vb_msg.frequencies[0], self.vb_msg.intensities[0] = self.vb_pattern_select(round(abs(msg.axes[0]) * 2))
        else:                                   # right 
            self.vb_msg.frequencies[2], self.vb_msg.intensities[2] = self.vb_pattern_select(round(abs(msg.axes[0]) * 2))    

        if msg.buttons[0] == 1:
            self.pub_vib.publish(self.vb_msg)

    def vb_pattern_select(self, level=0):
        freq = 0
        intensity = 0

        if level == 0:
            freq, intensity = 0, 0
        elif level == 1:
            freq, intensity = 1, 2
        elif level == 2:
            freq, intensity = 3, 4
        
        return freq, intensity
    
if __name__ == '__main__':
    rospy.init_node("joy_haptics_node", anonymous=False)
    node = JoyHaptics()
    rospy.spin()


