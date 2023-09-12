#!/usr/bin/env python

# prompter file to command the object detector on what to do

import rospy
from std_msgs.msg import String

def prompter():
    # Initialize the node
    rospy.init_node('keyboard_input_publisher_node')
    pub = rospy.Publisher('camera/blob_detect', String)

    print("Initialize base image or detect an object Press Ctrl+C to exit.")
    
    while not rospy.is_shutdown():
        # choices d, n, o, p
        user_input = input("Enter your choice: \n    s: save camera stream \n    n: initialize not pressed base image\n    p: initialize pressed base image\n    o: initialize object pressed base image\n    d for detect object\n    ")

        pub.publish(user_input)

if __name__ == '__main__':
    try:
        prompter()
    except rospy.ROSInterruptException:
        pass
