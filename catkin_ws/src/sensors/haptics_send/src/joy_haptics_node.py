#!/usr/bin/python
import rospy
import sys
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

        if msg.axes[0] == 0:
            self.vb_msg.frequencies[1] = 3
            self.vb_msg.intensities[1] = 3
        elif msg.axes[0] > 0:                   # left
            self.vb_msg.frequencies[0] = int(abs(msg.axes[0]) * 3)
            self.vb_msg.intensities[0] = int(abs(msg.axes[0]) * 4)
        else:                                   # right
            self.vb_msg.frequencies[2] = int(abs(msg.axes[0]) * 3)
            self.vb_msg.intensities[2] = int(abs(msg.axes[0]) * 4)

        if msg.buttons[0] == 1:
            self.pub_vib.publish(self.vb_msg)
    
if __name__ == '__main__':
    rospy.init_node("haptic_send",anonymous=False)
    node = JoyHaptics()
    rospy.spin()


