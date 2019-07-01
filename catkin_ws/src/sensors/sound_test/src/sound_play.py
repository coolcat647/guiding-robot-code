#!/usr/bin/env python
from pydub import AudioSegment
from pydub.playback import play
import numpy as np
import time
import rospy
import rospkg

rospy.init_node('sound_test_node', anonymous=False)
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('sound_test')
print(pkg_path)

sound_right = AudioSegment.from_mp3(pkg_path + "/src/right_obs.mp3")
stereo_right = sound_right.apply_gain_stereo(-20, +6)

sound_left = AudioSegment.from_mp3(pkg_path + "/src/left_obs.mp3")
stereo_left = sound_left.apply_gain_stereo(+6, -20)

sound_front = AudioSegment.from_mp3(pkg_path + "/src/front_obs.mp3")
# stereo_left = sound_left.apply_gain_stereo(+6, -20)

play(stereo_right)

# time.sleep(1)

play(stereo_left)
play(sound_front)
