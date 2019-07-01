#!/usr/bin/env python
from pydub import AudioSegment
from pydub.playback import play
import numpy as np
import time
import rospy
import rospkg
from haptics_msgs.msg import VibrationArray

RIGHT_ALARM_FILE = 'right_obs.mp3'
LEFT_ALARM_FILE = 'left_obs.mp3'
FRONT_ALARM_FILE = 'front_obs.mp3'

class BTSound(object):
    def __init__(self):
        
        rospack = rospkg.RosPack()
        self.sound_folder_path = rospack.get_path('sound_test') + '/src/'

        # Audio source init
        sound_right = AudioSegment.from_mp3(self.sound_folder_path + RIGHT_ALARM_FILE)
        self.stereo_right = sound_right.apply_gain_stereo(-20, +6)   # simulate right channel only sound
        sound_left = AudioSegment.from_mp3(self.sound_folder_path + LEFT_ALARM_FILE)
        self.stereo_left = sound_left.apply_gain_stereo(+6, -20)     # simulate left channel only sound
        self.stereo_front = AudioSegment.from_mp3(self.sound_folder_path + FRONT_ALARM_FILE)

        # Subscriber
        sub_vibration = rospy.Subscriber("vibrate_cmd", VibrationArray, self.vibration_cb)

        self.prev_alarm = -1
        # play(self.stereo_right)

    def vibration_cb(self, msg):
        if len(msg.frequencies) != len(msg.intensities):
            rospy.signal_shutdown('The length of freq array and the length of intensity are not equal')
        
        if msg.frequencies[1] >= 3 and self.prev_alarm != 1:
            play(self.stereo_front)
            self.prev_alarm = 1
        elif msg.frequencies[0] >= 3 and self.prev_alarm != 0:
            play(self.stereo_left)
            self.prev_alarm = 0
        elif msg.frequencies[2] >= 3 and self.prev_alarm != 2:
            play(self.stereo_right)
            self.prev_alarm = 2

    def shutdown_cb(self):
        rospy.loginfo("[%s] Shutting down..." % rospy.get_name())

if __name__ == '__main__':
    rospy.init_node('bt_sound_node', anonymous=False)
    node = BTSound()
    rospy.on_shutdown(node.shutdown_cb)
    rospy.spin()
