#! /usr/bin/python3

import sys
import os, re
 
print(sys.exec_prefix)
sys.path.append('/home/wkdo/miniconda3/envs/cs234-torch/lib/python3.8/site-packages')

import rospy
import numpy as np
import IPython
from pynput import mouse, keyboard
import os
import message_filters 
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, JointState
import cv2
from copy import deepcopy
import glob
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf_conversions
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String


from utils import pipeutil as pu
from utils import jntutil as ju

class CtrlBridge(object):
    '''
    class for initiating control bridge and recording the data
    '''

    def __init__(self):

        self.br = CvBridge()
        # self.image_tactrgb_sub = message_filters.Subscriber('/RunCamera/image_raw_1', Image)
        self.image_tactrgb_sub = message_filters.Subscriber('/RunCamera/image_raw_1', Image)
        self.allegro_joint_sub = message_filters.Subscriber('/allegroHand/joint_states', JointState)


        # ts = message_filters.ApproximateTimeSynchronizer([self.image_tactrgb_sub,
        #                                                   self.allegro_joint_sub], queue_size=10, slop=0.2, allow_headerless=True)

        ts = message_filters.ApproximateTimeSynchronizer([self.allegro_joint_sub], queue_size=10, slop=0.2, allow_headerless=True)


        self.jtstatepub = rospy.Publisher('allegroHand/joint_cmd', JointState, queue_size=10)
        
        self.libcmdpub = rospy.Publisher('allegroHand/lib_cmd', String, queue_size=10)
            # <!-- <remap from="allegroHand/lib_cmd" to="allegroHand_$(arg NUM)/lib_cmd"/> -->

        ### another keyboard listener 
        listener = keyboard.Listener(on_press=self.on_press,
                                     on_release=self.on_release)
        listener.start()
        
        ##### defining jnt msg info 
        self.curr_jntinfo = JointState()
        self.jtstate_msg = JointState()
        self.jtstate_msg.name = ["joint_0.0", "joint_1.0", "joint_2.0", "joint_3.0", "joint_4.0", "joint_5.0", 
                                 "joint_6.0", "joint_7.0", "joint_8.0", "joint_9.0", "joint_10.0", "joint_11.0", 
                                 "joint_12.0", "joint_13.0", "joint_14.0", "joint_15.0"]
        self.jtstate_msg.position = ju.jntvalue('ready_w')
        self.jtstate_msg.header.frame_id = ''  # Assuming the frame_id is base_link
        self.jtstate_msg.velocity = [0.0]*16
        self.jtstate_msg.effort = [0.0]*16


        ###### for tracking time
        self.curr_time = rospy.Time.now().to_time()

        ts.registerCallback(self.callback)
        self.currflag = False

        # list length with zero: 9
        self.statusflag = [0]*9 



    # def callback(self, imgrgb, allegroJoint):
    def callback(self, allegroJoint):

        self.curr_jntinfo = allegroJoint
        # if self.currflag: 

        #     print("callback working")

        if self.statusflag[0] in (1,3):
            # print("processing trajectory")
            if self.trajectory == []:
                print('trajectory publishing done - trajectory is empty')
                if self.statusflag[0] == 1:
                    self.statusflag[0] = 0
                    return
                elif self.statusflag[0] == 3:
                    self.statusflag[0] = 2


                    return
            self.jtstate_msg.header.stamp = rospy.Time.now()

            self.jtstate_msg.position = self.trajectory[0]

            self.jtstatepub.publish(self.jtstate_msg)
            self.jtstate_msg.position = []
            # remove the first element in the trajectory until the trajectory is empty
            self.trajectory.pop(0)

    def on_press(self, key):
        
        try:
            if key.char == 'h':
                print("h :  existing command - home pose ")
                finalpos = ju.jntvalue('home')
                self.trajectory = ju.generate_trajectory(self.curr_jntinfo.position,finalpos)
                self.statusflag[0] = 1
                
            if key.char == 'o':
                print("o :  existing command - Servos Off ")
                self.libcmdpub.publish('off')
            if key.char == 'r':
                print("r :  existing command - ready")
                finalpos = ju.jntvalue('ready')
                self.trajectory = ju.generate_trajectory(self.curr_jntinfo.position,finalpos)
                self.statusflag[0] = 1
             
                # self.libcmdpub.publish('ready')

            # if key.char == 'a':
            #     print("start next process if each step is done")
            #     if self.statusflag[7] == 0:
            #         self.statusflag[7] = 1 
            #     else:
            #         self.statusflag[7] = 0


            # if key.char == 's':
            #     print("start /end initial state regardless of the current state")
            #     self.statusflag[4] = 0 if self.statusflag[4] == 1 else 1

            #     result_dir = 'screw_recognition/results/sorting_result'
            #     print("Saving the labels detected so far...")

            #     file_paths = {
            #         'pred_labels': os.path.join(result_dir, 'pred_labels.txt'),
            #         'pred_labels_one': os.path.join(result_dir, 'pred_labels_one.txt'),
            #         'cur_lst': os.path.join(result_dir, 'current_joint_value.txt'),
            #         'des_lst': os.path.join(result_dir, 'desired_joint_value.txt'),
            #         'cur_xyzlst': os.path.join(result_dir, 'current_ee_value.txt'),
            #         'des_xyzlst': os.path.join(result_dir, 'desired_ee_value.txt')
            #     }

            #     for name, path in file_paths.items():
            #         pu.save_list_to_file(path, getattr(self, name))


            # if key.char == 'd':
            #     self.classify_flag = 1

            if key.char == 'w':
                print(" move joint to open position")

                finalpos = ju.jntvalue('ready_w')

                self.trajectory = ju.generate_trajectory(self.curr_jntinfo.position,finalpos)
                self.statusflag[0] = 1
            if key.char == 'q':
                print(" move joint to fng ready position")

                # # self.single_record_flag = 0
                # # # self.jtp_msg.positions = [0.0,	0.5,	-1.57,	0.0,	0.0,	-0.5,	1.57,	0.0]
                finalpos = ju.jntvalue('fng_ready')

                self.trajectory = ju.generate_trajectory(self.curr_jntinfo.position,finalpos)
                self.statusflag[0] = 1                
            if key.char == 'e':
                print("fingernail grasping position")

                poslst = []
                poslst.append(list(self.curr_jntinfo.position))
                poslst.append(ju.jntvalue('fng_ready'))
                poslst.append(ju.jntvalue('fng_mid1'))
                poslst.append(ju.jntvalue('fng_mid2'))
                poslst.append(ju.jntvalue('fng_final'))
                # print(poslst)
                self.trajectory = ju.gen_traj_multiple(poslst)
                self.statusflag[0] = 1

            if key.char == 't':
                print(" move joint to fng tap ready position")
                poslst = []
                poslst.append(list(self.curr_jntinfo.position))
                poslst.append(ju.jntvalue('tap_ready1'))
                poslst.append(ju.jntvalue('tap_ready2'))
                self.trajectory = ju.gen_traj_multiple(poslst)
                self.statusflag[0] = 1
            if key.char == 'y':
                print("move joint to scratch after tapping")
                poslst = []

                poslst.append(list(self.curr_jntinfo.position))
                poslst.append(ju.jntvalue('tap_scratch0'))
                poslst.append(ju.jntvalue('tap_scratch1'))
                poslst.append(ju.jntvalue('tap_scratch2'))
                poslst.append(ju.jntvalue('tap_scratch3'))
                self.trajectory = ju.gen_traj_multiple(poslst)
                self.statusflag[0] = 1                

            if key.char == '/':
                print("help")
                print(" -----------------------------------------------------------------------------")
                print("  Keyboard Commands:" )
                print("  Commands:\t\t\t\tKey:")
                print(" -----------------------------------------------------------------------------")
                print("  Home Pose:\t\t\t'H'")
                print("  Ready Pose:\t\t\t'R'")
                print("  Motors Off (free motion):\t'O'")
                print("  Go to next state:\t\t'A'")
                print("  move to initial state of tapping:\t'S'")
                print("  move to initial state of fingernail-grasping:\t'D'")
                print("  move to initial state of general-grasping:\t'F'")
                print("  Debugging - ready pose:\t'W'")
                print("  Help (this message):\t\t'/'")
                print(" -----------------------------------------------------------------------------")

            self.handle_key_press( key.char)

        except AttributeError:
            print('special key {0} pressed'.format(
                key))
        if key == keyboard.Key.esc:
            # Stop listenereewwewe
            rospy.loginfo("Stop keyboard listener")
            return False
    def on_release(self, key):
        # print('{0} released'.format(
        #     key))
        if key == keyboard.Key.esc:
            # Stop listener
            rospy.loginfo("Stop keyboard listener")
            return False


if __name__ == '__main__':
    rospy.init_node("send_ctrlinput", anonymous=True)
    rec = CtrlBridge()
    rospy.spin()    
        