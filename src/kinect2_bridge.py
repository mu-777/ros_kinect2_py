#!/usr/bin/env python
# -*- coding: utf-8 -*-


import pyfreenect2

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# remap-able
DEFAULT_NODE_NAME = 'my_kinect2_bridge'
DEFAULT_COLOR_IMG_TOPIC_NAME = 'my_kinect2_rgb'
DEFAULT_IR_IMG_TOPIC_NAME = 'my_kinect2_ir'
DEFAULT_DEPTH_IMG_TOPIC_NAME = 'my_kinect2_depth'


class Kinect2Device(object):
    default_serial_number = pyfreenect2.getDefaultDeviceSerialNumber()

    def __init__(self, serial_number=None):

        serial = self.default_serial_number if serial_number is None else serial_number
        self._kinect = pyfreenect2.Freenect2Device(serial)
        self.is_activated = False

        # Set up frame listener
        self._frameListener = pyfreenect2.SyncMultiFrameListener(pyfreenect2.Frame.COLOR,
                                                                 pyfreenect2.Frame.IR,
                                                                 pyfreenect2.Frame.DEPTH)
        self._kinect.setColorFrameListener(self._frameListener)
        self._kinect.setIrAndDepthFrameListener(self._frameListener)

    def __del__(self):
        if self.is_activated:
            self.deactivate()

    def activate(self):
        self.is_activated = True
        self._kinect.start()
        registration = pyfreenect2.Registration(self._kinect.ir_camera_params,
                                                self._kinect.color_camera_params)
        rospy.loginfo("Kinect serial: " + self._kinect.serial_number)
        rospy.loginfo("Kinect firmware: " + self._kinect.firmware_version)

    def deactivate(self):
        self.is_activated = False
        self._kinect.stop()
        self._kinect.close()

    def update_frame(self):
        if not self.is_activated:
            self.activate()
        self._frame = self._frameListener.waitForNewFrame()

    def get_all_frames(self):
        '''
        :return: cv::Mat, cv::Mat, cv::Mat
        '''
        rgb_data = self._frame.getFrame(pyfreenect2.Frame.COLOR).getData()
        ir_data = self._frame.getFrame(pyfreenect2.Frame.IR).getData()
        depth_data = self._frame.getFrame(pyfreenect2.Frame.DEPTH).getData()
        return rgb_data, ir_data, depth_data

    def get_color_frame(self):
        '''
        :return: cv::Mat
        '''
        return self._frame.getFrame(pyfreenect2.Frame.COLOR).getData()

    def get_ir_frame(self):
        '''
        :return: cv::Mat
        '''
        return self._frame.getFrame(pyfreenect2.Frame.IR).getData()

    def get_depth_frame(self):
        '''
        :return: cv::Mat
        '''
        return self._frame.getFrame(pyfreenect2.Frame.DEPTH).getData()


class Kinect2Bridge(object):
    def __init__(self):
        self._kinect2device = Kinect2Device()
        try:
            self._color_img_pub = rospy.Publisher(DEFAULT_COLOR_IMG_TOPIC_NAME, Image, queue_size=1)
        except ValueError:
            print('color')
        try:
            self._ir_img_pub = rospy.Publisher(DEFAULT_IR_IMG_TOPIC_NAME, Image, queue_size=1)
        except ValueError:
            print('ir')
        try:
            self._depth_img_pub = rospy.Publisher(DEFAULT_DEPTH_IMG_TOPIC_NAME, Image, queue_size=1)
        except ValueError:
            print('depth')
        self._cvbridge = CvBridge()

    def __del__(self):
        if self._kinect2device.is_activated:
            self._kinect2device.deactivate()

    def activate(self):
        self._kinect2device.activate()

    def deactivate(self):
        self._kinect2device.deactivate()

    def publish_imgs(self):
        self._kinect2device.update_frame()
        color_img, ir_img, depth_img = self._kinect2device.get_all_frames()
        try:
            self._color_img_pub.publish(self._cvbridge.cv2_to_imgmsg(color_img, "bgr8"))
            self._ir_img_pub.publish(self._cvbridge.cv2_to_imgmsg(ir_img))
            self._depth_img_pub.publish(self._cvbridge.cv2_to_imgmsg(depth_img))
        except CvBridgeError, e:
            print e


# --------------------------------------------
if __name__ == '__main__':
    rospy.init_node(DEFAULT_NODE_NAME, anonymous=True)
    rate_mgr = rospy.Rate(30)  # Hz

    kinect2_bridge = Kinect2Bridge()
    kinect2_bridge.activate()

    while not rospy.is_shutdown():
        kinect2_bridge.publish_imgs()
        rate_mgr.sleep()

    kinect2_bridge.deactivate()



