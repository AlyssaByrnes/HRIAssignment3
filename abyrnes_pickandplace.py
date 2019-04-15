#THIS CODE IS A MODIFIED VERSION OF joint_position_keyboard.py FROM RETHINK ROBOTICS EXAMPLE CODE

#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Joint Position Example: keyboard
"""
import argparse

import rospy

import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION


def map_keyboard():
    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

##Starting Point
    neutral = {'right_s0': 0.6124418295632514, 'right_s1': -0.13729128051574452, 'right_w0': 2.77957318764837, 'right_w1': 1.5121215616580466, 'right_w2': -2.6384469551629115, 'right_e0': -0.502378708032473, 'right_e1': 1.5727138027795204}

    right_start = {'right_s0': 0.6438884357149024, 'right_s1': 0.04141748127290617, 'right_w0': 2.9410246655733094, 'right_w1': 1.4925633066125075, 'right_w2': -2.645349868708396, 'right_e0': -0.5135000687446423, 'right_e1': 1.423534171157664}


##Move
    waypoint = {'right_s0': 0.9123350735948498, 'right_s1': -0.8260486542762953, 'right_w0': 2.233092531964191, 'right_w1': 1.6482623565828771, 'right_w2': -2.677946960450961, 'right_e0': -0.7777282594579048, 'right_e1': 2.097335232236332}

##End Point
    right_end = {'right_s0': 1.3023496889147164, 'right_s1': 0.042184471666848876, 'right_w0': 3.0484033207252885, 'right_w1': 1.3406992086118517, 'right_w2': -2.743908134330034, 'right_e0': -0.388480634531981, 'right_e1': 1.2893108522176902}
    up ={'right_s0': 1.3372477518391095, 'right_s1': -0.13230584295511694, 'right_w0': 2.9640343773915907, 'right_w1': 1.2919953185964896, 'right_w2': -2.6288595752386277, 'right_e0': -0.44332044769888457, 'right_e1': 1.4434759214001744}
    right.move_to_joint_positions(neutral)
    grip_right.open()
    right.move_to_joint_positions(right_start)
    grip_right.close()
    right.move_to_joint_positions(waypoint)
    right.move_to_joint_positions(right_end)
    grip_right.open()
    right.move_to_joint_positions(up)
    right.move_to_joint_positions(right_start)

###Left arm option
    left_up = {'left_w0': 0.6389029981542749, 'left_w1': -1.1485681149292035, 'left_w2': -0.48397093857784806, 'left_e0': 0.2945243112739994, 'left_e1': 1.5140390376429034, 'left_s0': -1.015495281580144, 'left_s1': -0.2972087776527989}
    left_start = {'left_w0': 0.3236699462438223, 'left_w1': -1.4047429065060677, 'left_w2': -0.47169909227476475, 'left_e0': 0.2699806186678328, 'left_e1': 1.222966183141646, 'left_s0': -1.0204807191407714, 'left_s1': 0.05560680356084625}
    waypoint2 = {'left_w0': 1.030835089458998, 'left_w1': -1.291228328202547, 'left_w2': -0.9944030457467194, 'left_e0': 0.6784030034423242, 'left_e1': 1.7211264440074343, 'left_s0': -0.9165535207615347, 'left_s1': -0.8271991398672094}
    left_end = {'left_w0': -0.5342088093810954, 'left_w1': -1.1539370476868025, 'left_w2': -1.0208642143377429, 'left_e0': 0.9986214929134043, 'left_e1': 1.4162477624152083, 'left_s0': -1.037354507807511, 'left_s1': 0.39883500485020756}
    left_end_up = {'left_w0': -0.5453301700932646, 'left_w1': -1.1558545236716593, 'left_w2': -1.161990446823201, 'left_e0': 1.1044661672774978, 'left_e1': 1.5550730237188382, 'left_s0': -1.1213399559442374, 'left_s1': 0.3984515096532362}


    left.move_to_joint_positions(left_up)
    grip_left.open()
    left.move_to_joint_positions(left_start)
    grip_left.close()
    left.move_to_joint_positions(waypoint2)
    left.move_to_joint_positions(left_end)
    grip_left.open()
    left.move_to_joint_positions(left_end_up)

    


def main():
    """RSDK Joint Position Example: Keyboard Control

    Use your dev machine's keyboard to control joint positions.

    Each key corresponds to increasing or decreasing the angle
    of a joint on one of Baxter's arms. Each arm is represented
    by one side of the keyboard and inner/outer key pairings
    on each row for each joint.
    """
    epilog = """
See help inside the example with the '?' key for key bindings.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("rsdk_joint_position_keyboard")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)

    print("Enabling robot... ")
    rs.enable()

    map_keyboard()
    print("Done.")


if __name__ == '__main__':
    main()
