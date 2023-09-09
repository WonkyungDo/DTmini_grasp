
import numpy as np
import math
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def gen_traj_multiple(jnt_lst, maxstep = 0.008):
    """
    Generates a linearly interpolated trajectory from the multiple state current joint state to the desired one
    :param jnt_lst: list of joint that we want to interpolate
    :param num_steps: Number of steps to interpolate
    :return: List of JointState messages
    """
    
    # Interpolate between current and desired joint states
    jnt_lst_np = np.array(jnt_lst)
    trajectory = []
    print(len(jnt_lst)  )
    print(jnt_lst_np.shape  )

    for i in range(jnt_lst_np.shape[0]-1):

        current_state_np = np.array(jnt_lst_np[i,:])
        desired_state_np = np.array(jnt_lst_np[i+1,:])
        # get the abs diff btw current and desired joint state
        diff = np.abs(desired_state_np - current_state_np)
        # get the max diff
        max_diff = np.max(diff)
        if (max_diff > maxstep):
            stepnum = math.floor(max_diff/(maxstep/2))
            #make the trajectory using stepnum, and for the last step, use the desired joint state
            step = (desired_state_np - current_state_np) / stepnum
            smalltrajectory = [current_state_np + step * i for i in range(stepnum+1)]
            smalltrajectory.append(desired_state_np)
        # append small trajectory to the big trajectory in one list 
            trajectory = trajectory + smalltrajectory

        # final step
        if i == jnt_lst_np.shape[0]-2:
            if (max_diff < maxstep):   

                print("max_diff < maxstep")
                print("max_diff < maxstep")
                print("max_diff < maxstep")
                print("max_diff < maxstep")
                print("max_diff < maxstep")
                trajectory = [np.array(jnt_lst_np[0]), desired_state_np]
    # print(trajectory)
    return trajectory
def generate_trajectory(cur_pos, desired_jntlst, maxstep = 0.008):
    """
    Generates a linearly interpolated trajectory from the current joint state to the desired one
    :param desired_jntmsg: Desired joint state message
    :param num_steps: Number of steps to interpolate
    :return: List of JointState messages
    """
    
    # Interpolate between current and desired joint states


    current_state_np = np.array(cur_pos)
    desired_state_np = np.array(desired_jntlst)
    # get the abs diff btw current and desired joint state
    diff = np.abs(desired_state_np - current_state_np)
    # get the max diff
    max_diff = np.max(diff)
    if (max_diff > maxstep):
        stepnum = math.floor(max_diff/(maxstep/2))
        #make the trajectory using stepnum, and for the last step, use the desired joint state
        step = (desired_state_np - current_state_np) / stepnum
        trajectory = [current_state_np + step * i for i in range(stepnum+1)]
        trajectory.append(desired_state_np)
    else: 
        print("max_diff < maxstep")
        print("max_diff < maxstep")
        print("max_diff < maxstep")
        print("max_diff < maxstep")
        print("max_diff < maxstep")
        trajectory = [current_state_np, desired_state_np]

    return trajectory

def jntvalue(str):
    """
        give jntvalue
    """
    joint_values = {
            'ready_w': [-0.044764117124454064, 1.0553570554570424, 0.4874653219333984, 0.6832663135573744, 
                        -0.0009940740530896984, 0.9069377560304106, 0.6876585850601755, 0.754008443356434, 
                        0.024138892556626632, 0.9325009233917427, 0.6468028436508633, 0.7539887693578724, 
                        1.0827907129446472, 0.6166716313494351, 0.518206624200143, 0.5431195938065412],
            'home': [-0.04228470595492494, -0.1020768671531391, 0.7899646401336801, 0.7429235042586014, 
                     -0.017261019180845855, -0.05715746776407815, 0.7731704444487273, 0.750649100390064, 
                     0.03855100779119551, 0.0014574671770184449, 0.8614652825295497, 0.7534848075504113, 
                     0.929444651042881, 0.42770854317418927, 0.39021241106722365, 0.7306176429222538],
            'ready': [-0.04246532321986563, 0.3184375722538081, 0.499457992289817, 0.6765922823654842, 
                      -0.004965380005428757, 0.2632155630398908, 0.6216076379278026, 0.7012687614070175, 
                      0.038055670891266616, 0.3306314747561553, 0.6871532651151792, 0.7533572545626258, 
                      1.1052162739978322, 0.566005692931742, 0.49351358788283084, 0.5540495012539528],
            'cheese': [0.11733491029997242, 0.07354928946935965, -0.24397337181900733, 0.0017473158686596632, 
                       -0.027115853306697087, 0.15915500092653498, -0.2711855892063133, 0.0011332476125650007, 
                       0.24507055363987906, 1.176202259516487, 0.5866812794417255, 0.7547888671544813, 
                       1.1050957866179116, 0.5658149005563357, 0.49307549629627745, 0.5539623401655251],
            'fng_ready': [0.031395162102881485, 0.29051870403074703, 0.7412438809073691, 0.3725976027408964, -0.06204673304411221, 0.5405290860275064, -0.2077514787078154, 0.12920157514445055, -0.3282076954510347, 0.2528506874944901, 0.32333781745191753, -0.25011335328295436, 1.1964682130665536, 0.021197045826337073, 0.966865657636359, -0.16510469689184745],
            # 'fng_final': [-0.02389555426483496, 0.6827506066978344, 1.4131729173791845, -0.24101919205773598, -0.06189804377905135, 0.5406956318434427, -0.20795191670162025, 0.12925007299207314, -0.32761117004179197, 0.2530190417982692, 0.32357431463878555, -0.2501276537644908, 1.1983870576116595, 0.02156719206218346, 0.9554347970847086, -0.16994576394934452],
            'fng_final': [-0.02389555426483496, 0.6827506066978344, 1.5131729173791845, -0.24101919205773598, -0.06189804377905135, 0.5406956318434427, -0.20795191670162025, 0.12925007299207314, -0.32761117004179197, 0.2530190417982692, 0.32357431463878555, -0.2501276537644908, 1.1983870576116595, 0.02156719206218346, 0.9554347970847086, -0.16994576394934452],
            'fng_mid2': [0.012694983177152933, 0.5821446143950997, 1.2696401530655503, -0.1599518965971972, -0.06194626725330836, 0.5405962682324936, -0.20786911525350782, 0.12920914594971636, -0.3290100423517698, 0.2529029881356828, 0.32306646897814295, -0.25011171131412446, 1.1966833029916268, 0.0212755820423615, 0.9666587397789771, -0.16514377845893308],
            'fng_mid1': [0.01785871475512751, 0.5799271826126393, 0.8758590796764707, 0.16395284211294547, -0.0620392774487082, 0.5402319328015809, -0.20785510267485, 0.12922496447392187, -0.33005042965986053, 0.25288392160748396, 0.32312064928583845, -0.25025283716061764, 1.1966283144117906, 0.021219134983278553, 0.9669766934576397, -0.1652593725434292]            
            }

    return joint_values[str]

def gripidx(val):
    """
        give grip index or vice versa
        input: either string or index
        output: either index or string
    """
    gripidx = {
        'moving':  0,
        'ready_w': 1,
        'grip_e':  2,
        'basepos_r': 3,
        'ready_drop': 4,
        'mixing_screw1' : 5,
        'mixing_screw2' : 6
    }

    if isinstance(val, str):
        return gripidx[val] if val in gripidx else None
    elif isinstance(val, int):
        for key, value in gripidx.items():
            if value == val:
                return key
    return None

def fridx(val):
    """
        give franka robot arm status index or vice versa
        input: either string or index
        output: either index or string
        This will be used for deciding which status franka is now in. 
    """
    fridx = {
        'moving':  0, 
        'neutral': 1,
        'capture':  2,
        'center': 3,  
        'box1': 4,
        'box2': 5,
        'box3' : 6,
        'box4' : 7,
        'grasp' : 8,
        'aftergrasp' : 9,
        'graspmovedone' : 10
    }

    if isinstance(val, str):
        return fridx[val] if val in fridx else None
    elif isinstance(val, int):
        for key, value in fridx.items():
            if value == val:
                return key
    return None