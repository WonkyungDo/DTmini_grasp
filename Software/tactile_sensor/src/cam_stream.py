#!/usr/bin/env python

# creates a camera stream from a pipeline for the Arducam and sends images to the object detector

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# CSI Arducam pipeline to stream with GStreamer
def create_camera_pipeline(
    sensor_id=0,
    capture_width=640,
    capture_height=480,
    display_width=640,
    display_height=480,
    framerate=60,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

def streamer():
    # initialize stream publisher
    stream_pub = rospy.Publisher("RunCamera/image_raw_1", Image) #publishes an Image to the RunCamera topic
    rospy.init_node('cam_streamer')
    rate = rospy.Rate(60) # Hertz

    #start the camera stream capture
    bridge = CvBridge()
    stream = cv2.VideoCapture(create_camera_pipeline(), cv2.CAP_GSTREAMER) 
    rospy.loginfo("----------STREAM CREATED----------")
    if not stream.isOpened():
        print("Error: Unable to open camera stream.")
        exit()

    while not rospy.is_shutdown():
        ret, frame = stream.read()
        
        if not ret:
            print("Failed to grab frame.")
            continue  # Skip the rest of the loop and try again
        
        cv2.imshow("Stream", frame) # will show our stream, but blob detector will handle the showing
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
        #cv_image = cv2.imread("path_to_image.jpg")  # or however you capture your image
        frame_msg = bridge.cv2_to_imgmsg(frame, "bgr8")  # Convert OpenCV image to ROS Image message
        stream_pub.publish(frame_msg) # publishes the frame Image to the RunCamera topic
        #rospy.loginfo("image published")
        rate.sleep() # process every RATE frames
    
    stream.release()
    cv2.destroyAllWindows() 

if __name__ == '__main__':
    try:
        streamer()
    except rospy.ROSInterruptException:
        pass

