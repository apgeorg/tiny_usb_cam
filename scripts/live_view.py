#!/usr/bin/env python

import argparse
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from tiny_usb_cam.usb_cam import UsbCam

class LiveView(object):
    def __init__(self, device, topic_name, rate):
        rospy.init_node("live_view")
        rospy.loginfo('LiveView started...')
        self.image_pub = rospy.Publisher(topic_name, Image, queue_size=10)
        self.camera = UsbCam(device)
        self.bridge = CvBridge()
        self.rate = rospy.Rate(rate) # Hz
        self.run()

    def run(self):
        while not rospy.is_shutdown():
            img = self.camera.capture()
            try:
                msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
                msg.header.stamp = rospy.Time.now()
                self.image_pub.publish(msg)
            except CvBridgeError as e:
                rospy.logerr(e)
            self.rate.sleep()
        self.camera.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--device', '-d', default='/dev/video0', help='Camera device')
    parser.add_argument('--topic',  '-t', default='/camera/rgb/image', help='ROS topic (publish)')
    parser.add_argument('--rate', '-r', default=30, help='Rate [Hz]')
    args, unknown = parser.parse_known_args()
    try:
        LiveView(args.device, args.topic, int(args.rate))
    except rospy.ROSInterruptException:
        pass
