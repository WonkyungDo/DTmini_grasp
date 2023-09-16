import rospy
import numpy as np
from franka_interface import ArmInterface
from franka_interface import GripperInterface
import IPython
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
#add pointcloud2
from sensor_msgs.msg import PointCloud2
import ros_numpy
import quaternion
import tf2_ros

"""
:info:

rosnode for mod-dt paper's pipeline for franka arm side. 

This code governs: 
    1. Based on the input from realsense camera, extracts the maximum depth point out of the pile of the small objects
    2. Move the robot arm to the maximum depth point. If there is the point that is impossible to grasp
        , regather the screw using the gripper arm without the sensor
    3. Publish a grasp-possible topic to move the gripper and grasp the screw. 
    4. if the gripper is not successfully grasp the screw, retry. 
    5. subscribe topic from controller node to determine which screw we grabbed.
    6. move the robot to the designated position.  


"""

class MoveRobot(object):
    '''
    class for initiating keyboard input
    '''

    def __init__(self):
        rospy.init_node("move_robot_rs")

        self.r = ArmInterface() # arm interface instance 
        self.cm = self.r.get_controller_manager() # controller manager instance associated with the robot 
        self.mvt = self.r.get_movegroup_interface() # moveit interface for planning and executing trajectories using moveit planners 
        self.frames = self.r.get_frames_interface() # frame interface for getting ee frame and calculate transformation

        self.elapsed_time_ = rospy.Duration(0.0)
        self.period = rospy.Duration(0.005)
        self.initial_pose = self.r.joint_angles() # get current joint angles of the robot


        # subscribe topic cmd_grasp_done
        self.sub = rospy.Subscriber("/cmd_manip2frarm", String, self.callback)
        # # subscribe topic camera/depth/color/points, which is sensor_msgs/PointCloud2
        # self.sub2 = rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.callback_pt)
        
        # make a publisher for the topic cmd_grasp_possible
        self.pub = rospy.Publisher("/cmd_frarm2manip", String, queue_size=10)
        self.r.reset_cmd()

        # self.r.move_to_neutral() # move robot to neutral pose

        self.qmax = self.r._joint_limits.position_upper
        self.qmin = self.r._joint_limits.position_lower
        self.vec = np.array([0,0,0,0,0,0])

        # self.currpipestatus = 'initial'
        # the vector needed to move from the center 
        self.vec_from_grasp = np.array([0,0,0])
        self.successful = False
        # self.frstatus = 'initial'
        # self.curstatus = 'initial'
        self.debug = False
        self.debug_pt = np.array([0,0,0])

        self.frstatus = 'readypose'
        self.curstatus = 'readypose'
        self.readyforgrasp = False
        # self.pose_run('capture')
        self.vec_available = False
        self.rotateang = 0
        self.runOnce = True
        IPython.embed()

    def pose_run(self, pose, duration=0, rotate = None):
        joint_values = {
            'nailgrasp': [-0.03937498216670855, -1.068049477710909, 0.05048099698150747, -2.8696537649873357, 0.027559908232755132, 3.581285130020441, 0.7734291850688422],
            'nailgrasp_done': [-0.03973148006597039, -0.6992343106939082, 0.06330780830508782, -2.5072743703261104, 0.02474625374045637, 3.4406369994901715, 0.7678589784224716],
            'fingergrasp': [-0.012109009914535876, -0.7974097161509502, -0.012014423031291185, -2.546695933636689, 0.10148282341824637, 3.148524012624071, 0.4852952626264511],
            'fingergrasp_done': [-0.017161933695344123, -0.6052486062049864, -0.02467725866118021, -2.236735367222838, -0.09219678607914182, 2.926245968756852, 0.6979356119894552],
            'tapready': [-0.023951281753286976, -0.5079533180437589, -0.13355196935879557, -2.122273677455737, -0.009476801121795692, 2.7754341093434225, 0.4596439049192848],
            'tap': [-0.024634827915411182, -0.4999734100284208, -0.1472490267251667, -2.198138052990562, -0.012707608332220538, 2.83271503828916, 0.45907563488766967],
            'tap_verify_ready': [-0.026215520090303448, -0.3758119174726504, -0.12827605100496844, -2.0032155311947464, -0.009258266491194567, 2.7736965846750468, 0.45968486973808864],
            'tap_verify_tap': [-0.027187629960085214, -0.3566004407447681, -0.13480249280205064, -2.0772760850562144, -0.009276356509088082, 2.873731215396564, 0.4604492306504701],
            'tap_done': [-0.024109670782198313, -0.2992506224272543, -0.13040193791765917, -1.894288752354702, 0.04248693028116913, 2.755549529367023, 0.46498885619109576],
            'readypose': [-0.03801349247400349, -0.8632944868071037, 0.03887387903060829, -2.3080252514395054, 0.027079712576336328, 2.6870713678730858, 0.7596867809628151]
           }

        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time) < rospy.Duration(duration):
            rospy.sleep(0.1)


        if pose in joint_values:
            self.pub.publish("moving/0")

            jointinfo = {f'panda_joint{i+1}': value for i, value in enumerate(joint_values[pose])}
            # if pose in ['center', 'grasp'] and rotate is not None: 
            #     # happens when we want to move the grasp angle 
            #     # print(jointinfo)
            #     jointinfo['panda_joint7'] += rotate / 180 * np.pi
            #     print('additional rotate when grasping: ', rotate)
            self.r.set_joint_position_speed(0.2)
            self.r.move_to_joint_positions(jointinfo)

            print("moving to " + pose + " pose is done")

        else:
            print("Invalid pose name. Please provide a valid pose name.")
            print("invalid name: "+ pose)
        self.pub.publish(pose + '/0')
        self.curstatus = pose




##### just for skipping realsense part

    def nextmove(self, vec):
        """
        vec: 3d vector(pos) for the next move
        vec_pos : vec (1,0,0) corresponds to moving in x direction about 4.41cm. with the calculation, now changing unit as cm
        """ 
        vec = vec / 4.41
        curr_eepos = np.asarray(self.r.endpoint_pose()['position'])
        curr_jtstate = [v for v in self.r.joint_angles().values()]
        desired_eepos = curr_eepos + vec
        nextjnt = next_joint(self.r._jacobian,
                                 curr_eepos, 
                                 np.asarray(curr_jtstate), 
                                 desired_eepos, 
                                 np.asarray(self.qmin), 
                                np.asarray(self.qmax), lambda_ = 0.1, Kp_=1)
        self.r.move_to_joint_positions(dict(zip(self.r.joint_names(), nextjnt)))

        print('next move done')

    def callback(self, data):
        '''
        callback function for the topic cmd_manip2frarm
        '''
        dataparse = data.data.split('/')
        print(dataparse)
        #no additional condition needed for status definition - also sleeping is not necessary, but just in case
        self.pose_run(dataparse[0], float(dataparse[1])) if len(dataparse) == 2 else self.pose_run(dataparse[0])
        



def next_joint(jac, curr_eepos, curr_jtstate, desired_eepos, q_min, q_max, lambda_ = 0.1, Kp_=1):
    """
     make a jacobian based controller that move to the desired ee position
    """

    # Compute pseudoinverse of J
    J_pinv = np.linalg.pinv(jac.T @ jac + lambda_**2 * np.eye(jac.shape[1])) @ jac.T
    
    # calculate the current end effector position from q_current        

    # Calculate position error in the task space
    pos_error = np.zeros(6)

    pos_error[:3] = desired_eepos - curr_eepos

    # pos_error = np.zeros(3)
    # pos_error = -p_desired + p_current

    # Task space control to get desired velocity

    v_desired = Kp_ * pos_error

    # minimize the movement of the joint angles
    joint_limit_penalty = 0.1 * (curr_jtstate - q_min) * (curr_jtstate - q_max)

    # Compute joint velocities with joint limit avoidance included
    q_dot = J_pinv @ v_desired + (np.eye(jac.shape[1]) - J_pinv @ jac) @ joint_limit_penalty
    # q_dot = J_pinv @ v_desired 

    # Integrate over time to get joint positions
    q_next = curr_jtstate + q_dot * 0.05
    return q_next

if __name__ == '__main__':
    

    mv = MoveRobot()


    

