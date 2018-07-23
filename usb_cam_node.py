#!/usr/bin/env python

import argparse
import rospy
from sensor_msgs.msg import Image
from usb_cam import UsbCam

if __name__ == "__main__":
    # Parse args
    parser = argparse.ArgumentParser()
    parser.add_argument('--device', '-d', default='/dev/video0', help='Camera device')
    parser.add_argument('--topic',  '-t', default='/camera/rgb/image', help='ROS topic (publish)')
    parser.add_argument('--rate', '-r', default=30, help='Rate [Hz]')
    args, unknown = parser.parse_known_args()

    # Initialize publisher node
    rospy.init_node("usb_cam_node")
    image_pub = rospy.Publisher(args.topic, Image, queue_size=10)
    rate = rospy.Rate(int(args.rate)) # 30Hz
    camera = UsbCam(args.device)
    while not rospy.is_shutdown():
    	img = camera.capture()
        image_pub.publish(img)
        rate.sleep()
    camera.close()
