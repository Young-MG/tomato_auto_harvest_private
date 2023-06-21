#!/usr/bin/env python
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of {copyright_holder} nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Will Son

from math import exp
import os
import rclpy
import select
import sys
import threading

from open_manipulator_msgs.msg import KinematicsPose, OpenManipulatorState
from open_manipulator_msgs.srv import SetJointPosition, SetKinematicsPose
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState

from std_msgs.msg import String

import time

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

present_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0]
goal_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0]
prev_goal_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0]

present_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
goal_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
prev_goal_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

task_position_delta = 0.01  # meter
joint_angle_delta = 0.05  # radian

path_time = 2.0  # second

input_goal_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

camera_cordi_x = 0.0
camera_cordi_y = 0.0
camera_cordi_z = 0.0


class TeleopKeyboard(Node):

    qos = QoSProfile(depth=10)
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    def __init__(self):
        super().__init__('teleop_keyboard')
        key_value = ''

        # Create joint_states subscriber
        self.joint_state_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            self.qos)
        self.joint_state_subscription

        # Create kinematics_pose subscriber
        self.kinematics_pose_subscription = self.create_subscription(
            KinematicsPose,
            'kinematics_pose',
            self.kinematics_pose_callback,
            self.qos)
        self.kinematics_pose_subscription

        # Create manipulator state subscriber
        self.open_manipulator_state_subscription = self.create_subscription(
            OpenManipulatorState,
            'states',
            self.open_manipulator_state_callback,
            self.qos)
        self.open_manipulator_state_subscription

        #
        self.joint_msg_subscription = self.create_subscription(
            String,
            'joint_msg',
            self.joint_msg_callback,
            10)
        self.joint_msg_subscription

        self.loop_count = 0


        # Create Service Clients
        self.goal_joint_space = self.create_client(SetJointPosition, 'goal_joint_space_path')
        self.goal_task_space = self.create_client(SetKinematicsPose, 'goal_task_space_path')
        self.tool_control = self.create_client(SetJointPosition, 'goal_tool_control')
        self.goal_joint_space_req = SetJointPosition.Request()
        self.goal_task_space_req = SetKinematicsPose.Request()
        self.tool_control_req = SetJointPosition.Request()

    def send_goal_task_space(self):
        
        self.goal_task_space_req.end_effector_name = 'gripper'
        # if goal_kinematics_pose[0] > -0.1 :
        self.goal_task_space_req.kinematics_pose.pose.position.x = goal_kinematics_pose[0]
        # if goal_kinematics_pose[1] > 0.27 :
        self.goal_task_space_req.kinematics_pose.pose.position.y = goal_kinematics_pose[1]
        if (goal_kinematics_pose[2] < 0.58):
            self.goal_task_space_req.kinematics_pose.pose.position.z = goal_kinematics_pose[2]
        self.goal_task_space_req.kinematics_pose.pose.orientation.w = goal_kinematics_pose[3]
        self.goal_task_space_req.kinematics_pose.pose.orientation.x = goal_kinematics_pose[4]
        self.goal_task_space_req.kinematics_pose.pose.orientation.y = goal_kinematics_pose[5]
        self.goal_task_space_req.kinematics_pose.pose.orientation.z = goal_kinematics_pose[6]
        self.goal_task_space_req.path_time = 3.0

        try:
            self.goal_task_space.call_async(self.goal_task_space_req)
        except Exception as e:
            self.get_logger().info('Sending Goal Kinematic Pose failed %r' % (e,))

    def send_goal_joint_space(self, path_time):
        self.goal_joint_space_req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        self.goal_joint_space_req.joint_position.position = [goal_joint_angle[0], goal_joint_angle[1], goal_joint_angle[2], goal_joint_angle[3], goal_joint_angle[4]]
        self.goal_joint_space_req.path_time = path_time

        try:
            self.goal_joint_space.call_async(self.goal_joint_space_req)
        except Exception as e:
            self.get_logger().info('Sending Goal Joint failed %r' % (e,))

    def send_tool_control_request(self):
        self.tool_control_req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        self.tool_control_req.joint_position.position = [goal_joint_angle[0], goal_joint_angle[1], goal_joint_angle[2], goal_joint_angle[3], goal_joint_angle[4]]
        self.tool_control_req.path_time = path_time

        try:
            self.tool_control_result = self.tool_control.call_async(self.tool_control_req)

        except Exception as e:
            self.get_logger().info('Tool control failed %r' % (e,))

    def kinematics_pose_callback(self, msg):
        present_kinematics_pose[0] = msg.pose.position.x
        present_kinematics_pose[1] = msg.pose.position.y
        present_kinematics_pose[2] = msg.pose.position.z
        present_kinematics_pose[3] = msg.pose.orientation.w
        present_kinematics_pose[4] = msg.pose.orientation.x
        present_kinematics_pose[5] = msg.pose.orientation.y
        present_kinematics_pose[6] = msg.pose.orientation.z

    def joint_state_callback(self, msg):
        present_joint_angle[0] = msg.position[0]
        present_joint_angle[1] = msg.position[1]
        present_joint_angle[2] = msg.position[2]
        present_joint_angle[3] = msg.position[3]
        present_joint_angle[4] = msg.position[4]

    def open_manipulator_state_callback(self, msg):
        if msg.open_manipulator_moving_state == 'STOPPED':
            for index in range(0, 7):
                goal_kinematics_pose[index] = present_kinematics_pose[index]

                while True:
                    self.loop_count += 1
                    if input_goal_kinematics_pose[index] - present_kinematics_pose[index] > 0.1:
                        goal_kinematics_pose[index] += input_goal_kinematics_pose[index] - present_kinematics_pose[index]
                    else:
                        break
                    if self.loop_count >= 10:
                        break

            for index in range(0, 5):
                goal_joint_angle[index] = present_joint_angle[index]

    def joint_msg_callback(self, msg):

        global camera_cordi_x
        global camera_cordi_y
        global camera_cordi_z

        if msg.data != '[]' or msg.open_manipulator_moving_state == 'STOPPED':
            self.get_logger().info(' %s' % (msg.data))

            camera_cordi_all = msg.data[1:-1].split(",")

            camera_cordi_x = float(camera_cordi_all[0].strip())
            camera_cordi_y = float(camera_cordi_all[1].strip())
            camera_cordi_z = float(camera_cordi_all[2].strip())
            
def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    # print_present_values()
    return key

def print_present_values():
    print(usage)
    print('Joint Angle(Rad): [{:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}]'.format(
        present_joint_angle[0],
        present_joint_angle[1],
        present_joint_angle[2],
        present_joint_angle[3],
        present_joint_angle[4]))
    print('Kinematics Pose(Pose X, Y, Z | Orientation W, X, Y, Z): {:.3f}, {:.3f}, {:.3f} | {:.3f}, {:.3f}, {:.3f}, {:.3f}'.format(
        present_kinematics_pose[0],
        present_kinematics_pose[1],
        present_kinematics_pose[2],
        present_kinematics_pose[3],
        present_kinematics_pose[4],
        present_kinematics_pose[5],
        present_kinematics_pose[6]))
    # print("camera_cordi_x, camera_cordi_y, camera_cordi_z")    
    # print(camera_cordi_x, camera_cordi_y, camera_cordi_z)

def main():
    print("success")

    count = 0

    global camera_cordi_x
    global camera_cordi_y
    global camera_cordi_z

    settings = None

    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    try:
        rclpy.init()
    except Exception as e:
        print(e)

    try:
        teleop_keyboard = TeleopKeyboard()
    except Exception as e:
        print(e)

    try:
        while(rclpy.ok()):
            rclpy.spin_once(teleop_keyboard)

            if count == 0:
                goal_kinematics_pose[0] = 0.073-0.03
                goal_kinematics_pose[1] = 0.0
                goal_kinematics_pose[2] = 0.20
                goal_kinematics_pose[3] = 1.0
                goal_kinematics_pose[4] = 0.0
                goal_kinematics_pose[5] = 0.0
                goal_kinematics_pose[6] = 0.0
                
                teleop_keyboard.send_goal_task_space()

                count += 1

                time.sleep(5)

                print("a")
            elif count == 1:
                goal_kinematics_pose[0] = present_kinematics_pose[0] + camera_cordi_z - 0.05
                goal_kinematics_po6se[1] = present_kinematics_pose[1] - camera_cordi_x - 0.02
                goal_kinematics_pose[2] = present_kinematics_pose[2] + camera_cordi_y + 0.02
            
                teleop_keyboard.send_goal_task_space()

                count += 1

                time.sleep(5)

                print("b")
            elif count == 2:
                goal_joint_angle[4] = -0.01 + 0.005
                
                teleop_keyboard.send_tool_control_request()

                count += 1

                time.sleep(5)
                print("c")
            elif count == 3:
                goal_kinematics_pose[0] = 0.15
                goal_kinematics_pose[1] = 0.20
                goal_kinematics_pose[2] = 0.20
                goal_kinematics_pose[3] = 1.0
                goal_kinematics_pose[4] = 0.0
                goal_kinematics_pose[5] = 0.0
                goal_kinematics_pose[6] = 0.0

                teleop_keyboard.send_goal_task_space()

                count += 1

                time.sleep(5)
                print("d")
            elif count == 4:
                goal_joint_angle[4] = 0.01 - 0.005

                teleop_keyboard.send_tool_control_request()

                count += 1

                time.sleep(5)
                print("e")
            elif count == 5:
                goal_kinematics_pose[0] = 0.073-0.03
                goal_kinematics_pose[1] = 0.0
                goal_kinematics_pose[2] = 0.20
                goal_kinematics_pose[3] = 1.0
                goal_kinematics_pose[4] = 0.0
                goal_kinematics_pose[5] = 0.0
                goal_kinematics_pose[6] = 0.0
                
                teleop_keyboard.send_goal_task_space()

                count += 1

                time.sleep(5)

                print("a")
            else:
                count = 0

                time.sleep(5)
                print("f")

            for index in range(0, 7):
                prev_goal_kinematics_pose[index] = goal_kinematics_pose[index]
            for index in range(0, 5):
                prev_goal_joint_angle[index] = goal_joint_angle[index]
            
            # print("g")

    except Exception as e:
        print(e)

    finally:
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        teleop_keyboard.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
