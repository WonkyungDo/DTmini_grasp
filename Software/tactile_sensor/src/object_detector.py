#!usr/bin/env python

import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import os
import rospkg

def closest_index_to(target, num_list): # Return the index of the number in num_list that is closest to the target.
    return min(enumerate(num_list), key=lambda x: abs(x[1] - target))[0]

class ObjectDetector:
    def __init__(self, test_name="DEFAULT"):
        self.gray_frame = None
        self.display_frame = None
        self.detected = False # value for if the object is detected
        self.pkg_path = rospkg.RosPack().get_path('cam_streamer')
        self.pressed_image = cv2.imread(os.path.join(self.pkg_path, 'resources', 'cam_3_pressed.jpg')) # baseline
        self.not_pressed_image = cv2.imread(os.path.join(self.pkg_path, 'resources', 'cam_3_not_pressed.jpg')) # baseline
        self.object_pressed_image = cv2.imread(os.path.join(self.pkg_path, 'resources', 'cam_3_object_pressed.jpg')) # baseline
        self.start_saving_stream = False # boolean to stream
        self.stream_save_count = 0 # index for naming saved images
        self.test_name = test_name # initialize the input test name for saving images

    def obj_detect_callback(self, data): # data is our image to show here
        self.display_frame = bridge.imgmsg_to_cv2(data, "bgr8") # convert back to image format from image message
        frame_sum = sum(cv2.sumElems(self.display_frame))
        pressed_sum = sum(cv2.sumElems(self.pressed_image))
        object_sum = sum(cv2.sumElems(self.object_pressed_image))

        bias_val = 19000 # for nut M1.6
        # bias_val = 50000 # for nut M2
        #bias_val = 12000 # for basil

        # determine if object is present
        cl_idx = closest_index_to(frame_sum, [pressed_sum - bias_val, object_sum])
        if cl_idx == 1: # means an object present
            self.detected = True
        else:
            self.detected = False

        # save stream if most recent command is "s"
        if self.start_saving_stream:
            print("saving image")
            impath = os.path.join(self.pkg_path, 'stream_save_failure_tapping', f"{self.test_name}_img_{self.stream_save_count}.jpg")
            print(impath)
            cv2.imwrite(impath, self.display_frame)
            self.stream_save_count += 1

    
    def command_callback(self, data): # control callback for initialization and saving images
        if data.data == "p":
            self.pressed_image = self.display_frame
            rospy.loginfo("Base Pressed Image Initialized")
        if data.data == "n":
            self.not_pressed_image = self.display_frame
            rospy.loginfo("Base Not Pressed Image Initialized")
        if data.data == "o":
            self.object_pressed_image = self.display_frame
            rospy.loginfo("Object Pressed Image Initialized")
        if data.data == "d":
            if self.detected: 
                self.detector_pub.publish("3")
                rospy.loginfo("Object Present")
            else:
                self.detector_pub.publish("4")
                rospy.loginfo("Object Not Present")
        if data.data == "s":
            self.start_saving_stream = not self.start_saving_stream
    
    def run_object_detector(self):
        rospy.init_node("object_detector")
        obj_sub = rospy.Subscriber("RunCamera/image_raw_1", Image, self.obj_detect_callback)
        detect_command = rospy.Subscriber("camera/blob_detect", String, self.command_callback)
        self.detector_pub = rospy.Publisher("cmd_camerablob2manip", String)
        rospy.spin() # keeps python running

if __name__ == "__main__":
    bridge = CvBridge()
    test_name = "tapping_failure" # name for saving images
    object_detector = ObjectDetector(test_name)
    object_detector.run_object_detector()