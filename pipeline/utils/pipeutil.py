
import numpy as np
import math
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from utils import jntutil as ju

class PipelineState:
    def __init__(self, statusflag, string_fr, string_jtval, curr_state):
        self.states = {
                'initial': self.initial_state,
                'start_controller': self.start_controller,
                'nail_graspready': self.nail_graspready,
                'nail_grasp': self.nail_grasp,
                'nail_done': self.nail_done,
                'tap_graspready': self.tap_graspready,
                'tap_grasp': self.tap_grasp, # tap and tapdone
                'tap_check': self.tap_check, # move and tapdone, 
                'tap_done': self.tap_done,
                'finger_graspready': self.finger_graspready,
                'finger_grasp': self.finger_grasp,
                'finger_done': self.finger_done,
                'waiting_for_input': self.waiting_for_input
            }
        self.statusflag = statusflag
        self.string_fr = string_fr
        self.string_jtval = string_jtval
        self.run_once = 0

        self.current_state = curr_state

    def get_statusflag(self):
        return self.current_state
    
    def run(self, statusflag, string_fr, string_jtval, curr_state):
        self.statusflag = statusflag
        self.string_fr = string_fr
        self.string_jtval = string_jtval
        self.current_state = curr_state
        # if self.statusflag[7] == 0:
        self.current_state = self.states[self.current_state]()
        # else:
            # print('waiting for finishing the previous task', self.statusflag, self.current_state)

        ##### hard-resetting the code #####
        if self.statusflag[4] == 0:
            self.current_state = 'waiting_for_input'
            self.states[self.current_state]()

        return self.current_state, self.statusflag, self.string_fr, self.string_jtval, self.current_state 

    def initial_state(self):
        """
        move the robot to the neutral position first, as well as move the gripper to the ready position
        """
        # self.move_franka_arm_neutral()
        
        current_state = 'initial'

        # move the gripper / fr to the ready position
        if self.run_once < 1:
            self.statusflag[3] = 1
            self.statusflag[1] = 1
            self.string_fr = 'readypose'
            self.string_jtval = ['ready_w']
            self.run_once += 1
        print('current fr status', ju.fridx(self.statusflag[6]), '  current gripper status', ju.allegro_idx(self.statusflag[5]))
        print(self.statusflag[6], self.statusflag[5])
        # if everything is done, then change the state. 
        # if franka status and gripper status are 'neutral and 'ready_w', then change the state
        if self.statusflag[6] == ju.fridx('readypose') and self.statusflag[5] == ju.allegro_idx('ready_w'):
            print("ready to move to the next state!!!!!", self.statusflag, self.current_state)

            # to make it seamlessly, make self.statusflag[7] = 1
            #############################
            if self.statusflag[7] == 1:
                print("move to the next state")
                if self.statusflag[2] == 1:
                    current_state = 'nail_graspready'
                elif self.statusflag[2] == 2:
                    current_state = 'tap_graspready'
                elif self.statusflag[2] == 3:
                    current_state = 'finger_graspready'

                self.run_once = 0
                self.statusflag[7] = 0
        return current_state

    def nail_graspready(self):
        # If successful, change the state. If not, stay in the same state
        print('nail_graspready state', self.statusflag, self.current_state)
        current_state = 'nail_graspready'
        if self.run_once < 1:
            self.statusflag[3] = 1
            self.statusflag[1] = 1
            self.string_fr = 'nailgrasp'
            self.string_jtval = ['fng_ready']
            self.run_once += 1
        

        # if the status is grasp / ready_w and it successfully move the robot base on the depthpt input, then change the state
        if self.statusflag[2] == 1 and self.statusflag[6] == ju.fridx('nailgrasp') and self.statusflag[5] == ju.allegro_idx('fng_ready'):
            #reset graspflag
            print("ready to move to the next state!!!!!", self.statusflag, self.current_state)
 
            if self.statusflag[7] == 1:
                print("move to the next state", self.statusflag)

                self.run_once = 0
                current_state = 'nail_grasp'
                self.statusflag[7] = 0

        return current_state

    def nail_grasp(self):

        curr_state = 'nail_grasp'
        if self.run_once < 1:
            self.statusflag[3] = 1
            self.statusflag[1] = 1

            self.string_jtval = ['fng_ready', 'fng_mid1', 'fng_mid2', 'fng_final']
            self.string_fr = 'nailgrasp_done/1'

            self.run_once += 1
            print('nail_grasp state - first move!!')
            curr_state = 'nail_grasp'

        print("current status: ",    self.statusflag, self.current_state)

        if self.statusflag[2] == 1 and self.statusflag[5] == ju.allegro_idx('fng_final') and self.run_once == 1:            
            
            print("ready to move to the next state!!!!!", self.statusflag, self.current_state)


            if self.statusflag[7] == 1:
                print("move to the next state", self.statusflag)

                self.run_once = 0
                curr_state = 'nail_done'
                self.statusflag[7] = 0

        return curr_state


    def nail_done(self):
        # If successful, change the state. If not, stay in the same state
        print('nail_done state', self.statusflag, self.current_state)
        current_state = 'nail_done'
        if self.run_once < 1:
            self.string_jtval = ['fng_ready']
            self.run_once += 1
            self.statusflag[3] = 1


        # if the status is grasp / ready_w and it successfully move the robot base on the depthpt input, then change the state
        if self.statusflag[2] == 1 and self.statusflag[6] == ju.fridx('nailgrasp_done') and self.statusflag[5] == ju.allegro_idx('fng_ready') :
            #reset graspflag
            print("ready to move to the next state!!!!!", self.statusflag, self.current_state)
 
            if self.statusflag[7] == 1:
                print("move to the next state", self.statusflag)

                self.run_once = 0
                current_state = 'initial'
                self.statusflag[2] = 0
                self.statusflag[7] = 0

        return current_state



    def tap_graspready(self):
        # If successful, change the state. If not, stay in the same state
        print('tap_graspready state', self.statusflag, self.current_state)
        current_state = 'tap_graspready'
        if self.run_once < 1:
            self.statusflag[3] = 1
            self.statusflag[1] = 1
            self.string_fr = 'tapready'
            self.string_jtval = ['tap_ready1', 'tap_ready2']
            self.run_once += 1
        

        # if the status is grasp / ready_w and it successfully move the robot base on the depthpt input, then change the state
        if self.statusflag[2] == 2 and self.statusflag[6] == ju.fridx('tapready') and self.statusflag[5] == ju.allegro_idx('tap_ready2'):
            #reset graspflag
            print("ready to move to the next state!!!!!", self.statusflag, self.current_state)
 
            if self.statusflag[7] == 1:
                print("move to the next state", self.statusflag)

                self.run_once = 0
                current_state = 'tap_grasp'
                self.statusflag[7] = 0

        return current_state

    def tap_grasp(self):

        curr_state = 'tap_grasp'
        if self.run_once < 1:
            self.statusflag[1] = 1
            self.string_fr = 'tap'

            self.run_once += 1
            print('tap_grasp state - first move!!')
            curr_state = 'tap_grasp'

        print("current status: ",    self.statusflag, self.current_state)

        if self.statusflag[2] == 2 and self.statusflag[6] == ju.fridx('tap') and self.statusflag[5] == ju.allegro_idx('tap_ready2') and self.run_once == 1:            
            self.statusflag[1] = 1
            self.string_fr = 'tapready'

            self.run_once += 1

        if self.statusflag[2] == 2 and self.statusflag[6] == ju.fridx('tapready') and self.statusflag[5] == ju.allegro_idx('tap_ready2') and self.run_once == 2:            
        
            print("ready to move to the next state!!!!!", self.statusflag, self.current_state)


            if self.statusflag[7] == 1:
                print("move to the next state", self.statusflag)
                self.statusflag[1] = 1
                self.string_fr = 'tap_verify_ready'

                self.run_once = 0
                curr_state = 'tap_check'
                self.statusflag[7] = 0

        return curr_state


    def tap_check(self):
        # If successful, change the state. If not, stay in the same state
        print('tap_check state', self.statusflag, self.current_state)
        current_state = 'tap_check'
        if self.run_once < 1:
            if self.statusflag[6] == ju.fridx('tap_verify_ready') :
                self.statusflag[1] = 1
                self.string_fr = 'tap_verify_tap'
                self.run_once += 1  
                # make selfstatusflag[8] as 2, so that we can publish the camera detection request
                self.statusflag[8] = 2


        # if the status is grasp / ready_w and it successfully move the robot base on the depthpt input, then change the state
        if self.statusflag[8] !=2 and self.statusflag[2] == 2 and self.run_once == 1 :
            #reset graspflag
            print("ready to move to the next state!!!!!", self.statusflag, self.current_state)
            #### maybe using self.statusflag[8]??
            if self.statusflag[8] == 3:
                print("object detected, move the seed", self.statusflag, self.current_state)
                print("move to the next state", self.statusflag)
                self.statusflag[1] = 1
                self.string_fr = 'tap_verify_ready'


                if self.statusflag[7] == 1:

                    self.run_once = 0
                    current_state = 'tap_done'
                    self.statusflag[2] = 0
                    self.statusflag[7] = 0


            if self.statusflag[8] == 4:
                print("object not detected, repeat the state", self.statusflag, self.current_state)
                self.statusflag[1] = 1
                self.string_fr = 'tap_verify_ready'


                if self.statusflag[7] == 1:

                    self.run_once = 0
                    current_state = 'tap_graspready'

                    self.statusflag[7] = 0


        return current_state


    def tap_done(self):
        # If successful, change the state. If not, stay in the same state
        print('tap_done state', self.statusflag, self.current_state)
        current_state = 'tap_done'
        if self.run_once < 1:
            self.string_jtval = ['tap_scratch0', 'tap_scratch1', 'tap_scratch2', 'tap_scratch3']
            self.run_once += 1
            self.statusflag[3] = 1


        # if the status is grasp / ready_w and it successfully move the robot base on the depthpt input, then change the state
        if self.statusflag[2] == 2 and self.statusflag[6] == ju.fridx('tap_verify_ready') and self.statusflag[5] == ju.allegro_idx('tap_scratch3') and self.run_once == 1:  
            print("ready to move to the next state!!!!!", self.statusflag, self.current_state)
 
            if self.statusflag[7] == 1:
                print("move to the next state", self.statusflag)

                self.run_once = 0
                current_state = 'initial'
                self.statusflag[2] = 0
                self.statusflag[7] = 0

        return current_state


    
    def finger_graspready(self):
        # If successful, change the state. If not, stay in the same state
        print('finger_graspready state', self.statusflag, self.current_state)
        current_state = 'finger_graspready'
        if self.run_once < 1:
            self.statusflag[3] = 1
            self.statusflag[1] = 1
            self.string_fr = 'fingergrasp'
            self.string_jtval = ['finger_grasp_ready']
            self.run_once += 1
        

        # if the status is grasp / ready_w and it successfully move the robot base on the depthpt input, then change the state
        if self.statusflag[2] == 3 and self.statusflag[6] == ju.fridx('fingergrasp') and self.statusflag[5] == ju.allegro_idx('finger_grasp_ready'):
            #reset graspflag
            print("ready to move to the next state!!!!!", self.statusflag, self.current_state)
 
            if self.statusflag[7] == 1:
                print("move to the next state", self.statusflag)

                self.run_once = 0
                current_state = 'finger_grasp'
                self.statusflag[7] = 0

        return current_state

    def finger_grasp(self):

        curr_state = 'finger_grasp'
        if self.run_once < 1:
            self.statusflag[3] = 1
            self.statusflag[1] = 1

            self.string_jtval = ['finger_grasp_done']
            self.string_fr = 'fingergrasp_done/1'

            self.run_once += 1
            print('finger_grasp state - first move!!')
            curr_state = 'finger_grasp'

        print("current status: ",    self.statusflag, self.current_state)

        if self.statusflag[2] == 3 and self.statusflag[6] == ju.fridx('fingergrasp_done') and self.statusflag[5] == ju.allegro_idx('finger_grasp_done') and self.run_once == 1:            
            
            print("ready to move to the next state!!!!!", self.statusflag, self.current_state)


            if self.statusflag[7] == 1:
                print("move to the next state", self.statusflag)

                self.run_once = 0
                curr_state = 'finger_done'
                self.statusflag[7] = 0

        return curr_state


    def finger_done(self):
        # If successful, change the state. If not, stay in the same state
        print('finger_done state', self.statusflag, self.current_state)
        current_state = 'finger_done'
        if self.run_once < 1:
            self.string_jtval = ['finger_grasp_ready']
            self.run_once += 1
            self.statusflag[3] = 1


        # if the status is grasp / ready_w and it successfully move the robot base on the depthpt input, then change the state
        if self.statusflag[2] == 3 and self.statusflag[6] == ju.fridx('fingergrasp_done') and self.statusflag[5] == ju.allegro_idx('finger_grasp_ready') :
            #reset graspflag
            print("ready to move to the next state!!!!!", self.statusflag, self.current_state)
 
            if self.statusflag[7] == 1:
                print("move to the next state", self.statusflag)

                self.run_once = 0
                current_state = 'initial'
                self.statusflag[2] = 0
                self.statusflag[7] = 0

        return current_state
    
    def reset_gripper(self):
        self.current_state = 'start_controller'

    def start_controller(self):
        curr_state = 'start_controller'
        print("turn on the controller mode", self.statusflag)
        print("turn on the controller mode", self.statusflag)
        print("turn on the controller mode", self.statusflag)
        print("turn on the controller mode", self.statusflag)
        if self.run_once < 1:
            self.statusflag[0] = 2
            self.run_once += 1

        if self.statusflag[7] == 1:
            print("move to the next state!!", self.statusflag)
            print("move to the next state!!", self.statusflag)
            print("move to the next state!!", self.statusflag)
            print("move to the next state!!", self.statusflag)
            print("move to the next state!!", self.statusflag)
            print("move to the next state!!", self.statusflag)
            print("move to the next state!!", self.statusflag)
            print("move to the next state!!", self.statusflag)
            self.statusflag[0] = 0
            self.statusflag[7] = 0
            curr_state = 'run_classifier'
            self.run_once = 0
        return curr_state





    def disable_robot(self):
        self.statusflag[4] = 0  # disable the pipeline
        self.current_state = 'waiting_for_input'

    def waiting_for_input(self):
        # print('waiting for input')
        current_state = 'waiting_for_input'

        if self.statusflag[4] == 1:  # enable the pipeline
            current_state = 'initial'
            print('go to initial state')
            self.run_once = 0
            self.statusflag[0] = 0
        return current_state
    

