
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
            'fng_final': [-0.02389555426483496, 0.6827506066978344, 1.5131729173791845, -0.24101919205773598, -0.06189804377905135, 0.5406956318434427, -0.20795191670162025, 0.12925007299207314, -0.32761117004179197, 0.2530190417982692, 0.32357431463878555, -0.2501276537644908, 1.1983870576116595, 0.02156719206218346, 0.9354347970847086, -0.16994576394934452],
            # 'fng_final': [-0.021415059949526177, 0.6630041724176237, 1.1595315531923762, 0.3115358972149229, -0.031812726851405765, 0.6694778089576843, -0.2347366906354288, 0.1362074848053859, 0.01855480081142222, 0.4161294661663061, 0.30851939662438654, -0.2509292477326174, 1.1676497816373894, 0.05242168419798862, 0.9414848415441396, -0.05678423588895835],
            'fng_final2': [-0.011474376182450972, 0.7993284933696532, 0.639649201422191, 1.0047715763722715, -0.033157133704125825, 0.6313476748118201, -0.1977735654193032, 0.17069413009566925, 0.05484894832070614, 0.3832880290909609, 0.332832833604767, -0.22180133136244967, 1.1601523038501487, 0.05624977973455809, 0.935117057100346, -0.032720574427199195],
            'fng_mid2': [0.012694983177152933, 0.5821446143950997, 1.2696401530655503, -0.1599518965971972, -0.06194626725330836, 0.5405962682324936, -0.20786911525350782, 0.12920914594971636, -0.3290100423517698, 0.2529029881356828, 0.32306646897814295, -0.25011171131412446, 1.1966833029916268, 0.0212755820423615, 0.9366587397789771, -0.16514377845893308],
            'fng_mid1': [0.01785871475512751, 0.5799271826126393, 0.8758590796764707, 0.16395284211294547, -0.0620392774487082, 0.5402319328015809, -0.20785510267485, 0.12922496447392187, -0.33005042965986053, 0.25288392160748396, 0.32312064928583845, -0.25025283716061764, 1.1966283144117906, 0.021219134983278553, 0.9669766934576397, -0.1652593725434292],
            'tap_ready1': [0.03984671129154367, 0.4302074168252564, 0.2611949474712758, 0.5902316595780206, 0.05759877739785461, 0.4572604246585161, -0.01986146390423686, 0.8005746320815869, 0.14074299734362897, 0.35037780824743225, 0.08615700120357868, 0.6929502785184121, 0.702902750089233, 0.3955731251391541, 0.45020415046186557, 0.557574747392485],
            'tap_ready2': [-0.00027392574377409576, 1.272354525697206, 1.6526004369634608, 0.7590166030232702, 0.2052787360812826, 1.391666916208214, 1.376322134209486, 1.0621384894957502, 0.21341567231085426, 1.255860900526943, 1.1314156590339177, 0.7870133035800867, 0.299697458823045, 1.093259038196576, 0.6807550685714082, 0.2941515847021078],
            'tap_scratch0': [0.1541928439074483, 0.09247664860355138, 0.954739599617247, 1.0846959900675204, 0.16959451112464094, 1.237996707939489, 1.2532971166325513, 1.0314729315937652, 0.207158570377499, 1.1449451446839656, 1.0700254560654205, 0.7168671489569124, 0.47447317763561864, 0.9783550010807632, 0.6905274512678337, 0.32596672082229383],
            'tap_scratch1': [0.19155532589292903, 1.1095589452448706, 0.2595879107134029, 1.1590753613924547, 0.10150342179594254, 1.337403436557218, 1.2060174141147606, 1.016983661579242, 0.13851635184816802, 1.150557558578761, 1.137851662373195, 0.7252240170306592, 0.9831964750659513, 0.15365423596757313, 0.40153994986222574, 0.5596103697518692],
            'tap_scratch2': [0.14189644723794248, 0.9710442145635909, 0.25946195809905065, 1.269070537553296, 0.15135753209662733, 1.2637848464917694, 1.0959175925215772, 1.0243613020738025, 0.13996746822601366, 1.5661353671293905, 0.39632752424840656, 0.7128546643835867, 0.9733086137188489, 0.1824696949721601, 0.607358546407333, 0.5042535429971579],
            'tap_scratch3': [0.18140728099415268, 1.1113806171572609, 0.21967336166045928, 1.1937304866641907, 0.055987477354097884, 1.2765434971929763, 1.0902574319226201, 1.0211512677050745, 0.14315067199184003, 1.4154491779153149, 0.45298513091936654, 1.088138013629772, 0.9477619760902001, 0.1075426624030524, 0.7294227224046538, 0.49606112831030935],
            'finger_grasp_ready': [0.10555404829088236, 0.14601793392729626, -0.317486339967387, -0.04112545534349199, 0.2785040676328378, 0.5888069371054733, 0.5410923130883205, 1.6341641648706806, -0.004761433301088424, 0.06533626989938705, 0.17884103267335516, 0.137576160456312, 1.3190292286539762, 0.22824769014995466, 0.21269773226720032, 1.408972116459882],
            'finger_grasp_done': [0.10481912778850046, 0.14593099829035525, -0.31773073065490537, -0.04123725219527494, 0.28536034593037074, 0.9126614506052622, 0.8319054892133292, 1.1786612305784154, -0.009098171667059816, 0.06502642745755356, 0.17874165846236234, 0.13749365449171852, 1.2072997773609355, 0.24314981765146526, 0.515468650929741, 1.0005107092840813],
            'home': [-0.04228470595492494, -0.1020768671531391, 0.7899646401336801, 0.7429235042586014, 
            -0.017261019180845855, -0.05715746776407815, 0.7731704444487273, 0.750649100390064, 
            0.03855100779119551, 0.0014574671770184449, 0.8614652825295497, 0.7534848075504113, 
            0.929444651042881, 0.42770854317418927, 0.39021241106722365, 0.7306176429222538]
            }
            # 'finger_grasp_ready': [0.10175468872348954, -0.03383297385117495, 0.09897710791218513, -0.2409722143732875, 0.12993634552793454, 0.5899909200028569, 1.1118468535849135, 0.8976750355800872, -0.04475840597687032, -0.13231597018682412, 0.16438245898309953, 0.10110263895252833, 1.2237743337369849, 0.3348396135860019, 0.23312239554410868, 1.2196029670759951],
            # 'finger_grasp_done': [0.09944277402138095, -0.0013005778704736003, 0.0841159162939131, -0.2388797932340891, 0.10517272431330668, 0.7193254624787789, 1.0334412332376266, 1.0552898388332683, -0.0404642727440004, 0.2787641840949686, -0.21810943748384853, 0.0843155861166592, 1.209634049323468, 0.3805223462826223, 0.6205994174243882, 0.8262473906143502],

    return joint_values[str]

def allegro_idx(val):
    """
        give allegro index or vice versa
        input: either string or index
        output: either index or string
    """
    allegro_idx = {
        'moving':  0,
        'ready_w': 1,
        'ready':  2,
        'cheese': 3,
        'fng_ready': 4,
        'fng_final': 5,
        'fng_mid2': 6,
        'fng_mid1': 7,
        'tap_ready1': 8,
        'tap_ready2': 9,
        'tap_scratch0': 10,
        'tap_scratch1': 11,
        'tap_scratch2': 12,
        'tap_scratch3': 13,
        'finger_grasp_ready': 14,
        'finger_grasp_done': 15,        
        'home': 16        
    }

    if isinstance(val, str):
        return allegro_idx[val] if val in allegro_idx else None
    elif isinstance(val, int):
        for key, value in allegro_idx.items():
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
        'nailgrasp': 1,
        'nailgrasp_done': 2,
        'fingergrasp': 3,
        'fingergrasp_done': 4,
        'tapready': 5,
        'tap': 6,
        'tap_verify_ready': 7,
        'tap_verify_tap': 8,
        'tap_done': 9,
        'readypose': 10
        }
    if isinstance(val, str):
        return fridx[val] if val in fridx else None
    elif isinstance(val, int):
        for key, value in fridx.items():
            if value == val:
                return key
    return None




