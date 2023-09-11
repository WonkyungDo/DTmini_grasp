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
        self.libcamerapub = rospy.Publisher('camera/blob_detect', String, queue_size=10)
            # <!-- <remap from="allegroHand/lib_cmd" to="allegroHand_$(arg NUM)/lib_cmd"/> -->


        ############### for communication btw manip and franka arm


        self.pub2_fr =  rospy.Publisher('/cmd_manip2frarm', String, queue_size=10)
        self.frsub = rospy.Subscriber('/cmd_frarm2manip', String, self.callback_frsub)
        self.blobsub = rospy.Subscriber('/cmd_camerablob2manip', String, self.callback_blobsub)



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

        ###############
        ### self.statusflag[0] = self.startpub_flag
        # startpub_flag: flag for start publishing the trajectory in jointmsg format (same as command from keyboard)
        # 0: not started 
        # 1: execute trajectory from keyboard command 
        ### self.statusflag[1] = self.pub_fr_flag
        # pub_fr_flag: flag for publishing the command to franka arm
        # 0: not started
        # 1: publish the command to franka arm once
        ### self.statusflag[2] = self.grasp_flag
        # whether the grasp strategy is in progress or not
        # grasp_flag: flag for grasping strategy
        # 0: not started
        # 1: grasping with fingernail / 2: grasping with tapping motion / 3: grasping with fingers
        ### self.statusflag[3] = self.pub_traj_flag
        # pub_traj_flag: flag for publishing the trajectory (for trajectory generation here)
        # 0: not started
        # 1: change the trajectory and run the trajectory once
        ### self.statusflag[4] = self.enable_pipeline_flag
        # enable_pipeline_flag: flag for enabling the pipeline in cu.run_pipeline ftn
        # 0: not started
        # 1: enable the pipeline
        ### self.statusflag[5] = self.allegrohand_run_flag
        # allegrohand_run_flag: flag for running the allegrohand
        # 0: allegrohand not running
        # 1: allegrohand running
        ### self.statusflag[6] = self.fr_run_flag
        # fr_run_flag: flag for running the franka arm
        # 0: franka arm not running
        # 1: franka arm running
        ### self.statusflag[7] = self.done_flag
        # done_flag: flag for finishing the execution of each status
        # 0: not finished
        # 1: finished
        ### self.statusflag[8] = self.debug_flag
        # debug_flag: flag for debugging - determines whether go to the next state or not
        # 0: don't go next state
        # 1: go to the next state (got it from 'a' key)
        ### self.statusflag[8] = self.indicator_flag
        # indicator_flag: governs communication status for each state.
        # a. during 'detect_and_grab', works as a flag for showing whether the detection is done or not 
        #   0: not done
        #   1: done
        # b. during 'detect_and_grab' and 'grasp' status in franka robot, indicates franka arm movement status  
        #   0: not done
        #   1: done
        # c. during 'grab_screw', indicates whether the screw is grabbed or not
        #   0: not grabbed
        #   1: grabbed


        ##### self.current_state 
        self.string_fr = 'initial'
        ##### make string_jtval as a list to handle multiple joint values
        self.string_jtval = ['home']
        self.curr_state = 'waiting_for_input'

        self.Pl = pu.PipelineState(self.statusflag, self.string_fr, self.string_jtval, self.curr_state)
        self.currflag = self.Pl.get_statusflag()



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
        # #################################### run the pipeline
        self.currflag, self.statusflag, self.string_fr, self.string_jtval, self.curr_state = self.Pl.run(self.statusflag, 
                                                                                        self.string_fr, 
                                                                                        self.string_jtval,
                                                                                        self.curr_state)
        if self.statusflag[1] == 1:
            #publish msg to franka arm
            self.pub2_fr.publish(self.string_fr)
            self.statusflag[1] = 0

        if self.statusflag[3] == 1:
            #publish msg to trajectory generator

            # self.trajectory = ju.generate_trajectory(self.curr_jntinfo.position, cu.jntvalue(self.string_jtval))
            poslst = []
            poslst.append(list(self.curr_jntinfo.position))
            for i in range(len(self.string_jtval)):
                poslst.append(ju.jntvalue(self.string_jtval[i]))
            self.trajectory = ju.gen_traj_multiple(poslst)
            self.statusflag[0] = 1
            self.statusflag[3] = 0

        # reseting the statusflag[5] by comparing current joint position and the target joint position
        # if the current joint position is close to the target joint position, then reset the flag
        if self.statusflag[4] == 1:
            curr_diff = np.linalg.norm(np.asarray(self.curr_jntinfo.position) - np.asarray(ju.jntvalue(self.string_jtval[-1])))
            # print('it\'s resetting, current jtval: ' , self.string_jtval[-1], ', curr_diff: ', curr_diff)
            if curr_diff < 1.8:
                self.statusflag[5] = ju.allegro_idx(self.string_jtval[-1])
            else:
                if self.curr_state != 'start_controller':
                    print('it\'s resetting, current jtval: ' , self.string_jtval[-1], ', curr_diff: ', curr_diff)
                    print('it\'s resetting, current jtval: ' , self.string_jtval[-1])
                    self.statusflag[5] = 0

        if self.curr_state == 'tap_check':
            # determine whether there is the object inside of the allegrohand or not, and update the statusflag[8]
            # if there is the object, then statusflag[8] = 1
            # if there is no object, then statusflag[8] = 2 (redo the motion from the capturing status)
            if self.statusflag[8] == 2:
                self.libcamerapub.publish('detect')
                self.statusflag[8] = 0



    def callback_blobsub(self, data):
        """
        determine whether there is object or not
        """
        frstatus = data.data
        # frstatus = frstatus.split("/")

        self.statusflag[8] = int(frstatus)


    def callback_frsub(self, data):
        frstatus = data.data
        frstatus = frstatus.split("/")

        self.statusflag[6] = int(ju.fridx(frstatus[0]))
        # self.statusflag[8] = int(frstatus[1])
    def on_press(self, key):
        
        try:
            if key.char == 'h':
                print("h :  existing command - home pose ")

                poslst = []
                poslst.append(list(self.curr_jntinfo.position))
                stexample = ['home', 'ready']
                for i in range(len(stexample)):
                    poslst.append(ju.jntvalue(stexample[i]))
                poslst.append(ju.jntvalue('home'))
                self.trajectory = ju.gen_traj_multiple(poslst)
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

            if key.char == 'a':
                print("start next process if each step is done")
                if self.statusflag[7] == 0:
                    self.statusflag[7] = 1 
                else:
                    self.statusflag[7] = 0


            if key.char == 's':
                print("reset statusflag and go to the initial state")
                self.statusflag[4] = 0 if self.statusflag[4] == 1 else 1
                self.statusflag[2] = 0
            if key.char == 'd':
                print("start /end fingernail-grasping state regardless of the current state")        
                self.statusflag[2] = 0 if self.statusflag[2] == 1 else 1
                print(self.statusflag)
            if key.char == 'f':
                print("start /end tapping state regardless of the current state")
                self.statusflag[2] = 0 if self.statusflag[2] == 2 else 2
                print(self.statusflag)

            if key.char == 'g':
                print("start /end general-grasping state regardless of the current state")
                self.statusflag[2] = 0 if self.statusflag[2] == 3 else 3
                print(self.statusflag)

            if key.char == 'j':
                print("start /end general-grasping state regardless of the current state")
                self.statusflag[8] = 0 if self.statusflag[8] == 3 else 3
                print(self.statusflag)


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


            if key.char == 'u':
                print("move joint to finger-grasping motion ready")
                finalpos = ju.jntvalue('finger_grasp_ready')

                self.trajectory = ju.generate_trajectory(self.curr_jntinfo.position,finalpos)
                self.statusflag[0] = 1
            if key.char == 'i':
                print("move joint to finger-grasping motion")
                finalpos = ju.jntvalue('finger_grasp_done')

                self.trajectory = ju.generate_trajectory(self.curr_jntinfo.position,finalpos)
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
                print("  Reset all the state and go to the initial state:\t'S'")
                print("  move to initial state of fingernail-grasping:\t'D'")
                print("  move to initial state of tapping:\t'F'")
                print("  move to initial state of general-grasping:\t'G'")
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
        