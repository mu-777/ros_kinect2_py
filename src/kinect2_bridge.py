#!/usr/bin/env python
# -*- coding: utf-8 -*-


import signal
import pyfreenect2

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# remap-able
DEFAULT_NODE_NAME = 'kinect2_bridge'
DEFAULT_COLOR_IMG_TOPIC_NAME = 'color_img'
DEFAULT_IR_IMG_TOPIC_NAME = 'ir_img'
DEFAULT_DEPTH_IMG_TOPIC_NAME = 'depth_img'


class Kinect2Device(object):
    default_serial_number = pyfreenect2.getDefaultDeviceSerialNumber()

    def __init__(self, serial_number=None):
        serial = self.default_serial_number if serial_number is None else serial_number
        self._kinect = pyfreenect2.Freenect2Device(serial)
        signal.signal(signal.SIGINT, self._sigint_handler)
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

    def _sigint_handler(self, signum, frame):
        rospy.loginfo("Got SIGINT, shutting down...")
        self.deactivate()

    def activate(self):
        self.is_activated = True
        self._kinect.start()
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
        rgb_data = self._frame.getFrame(pyfreenect2.Frame.COLOR).getData()
        ir_data = self._frame.getFrame(pyfreenect2.Frame.IR).getData()
        depth_data = self._frame.getFrame(pyfreenect2.Frame.DEPTH).getData()
        return rgb_data, ir_data, depth_data

    def get_color_frame(self):
        return self._frame.getFrame(pyfreenect2.Frame.COLOR).getData()

    def get_ir_frame(self):
        return self._frame.getFrame(pyfreenect2.Frame.IR).getData()

    def get_depth_frame(self):
        return self._frame.getFrame(pyfreenect2.Frame.DEPTH).getData()


class Kinect2Bridge(object):
    class Frame:
        ALL, COLOR, IR, DEPTH = 0, pyfreenect2.Frame.COLOR, pyfreenect2.Frame.IR, pyfreenect2.Frame.DEPTH

    def __init__(self, serial_numbers=[], suffixes=[]):
        self._kinect2devices = []
        if len(serial_numbers) is 0:
            self._kinect2devices.append(Kinect2Device())
        else:
            for serial_number in serial_numbers:
                self._kinect2devices.append(Kinect2Device(serial_number))

        self.color_img_pubs, self.ir_img_pubs, self.depth_img_pubs = [], [], []
        if suffixes == [] or len(suffixes) is not len(self._kinect2devices):
            suffixes = [str(i + 1) for i in xrange(len(self._kinect2devices))]

        for suffix in suffixes:
            _suffix = suffix if suffix.startswith('_') else '_' + suffix
            self.color_img_pubs.append(rospy.Publisher(DEFAULT_COLOR_IMG_TOPIC_NAME + _suffix, Image, queue_size=1))
            self.ir_img_pubs.append(rospy.Publisher(DEFAULT_IR_IMG_TOPIC_NAME + _suffix, Image, queue_size=1))
            self.depth_img_pubs.append(rospy.Publisher(DEFAULT_DEPTH_IMG_TOPIC_NAME + _suffix, Image, queue_size=1))

        self._cvbridge = CvBridge()

    def __del__(self):
        for device in self._kinect2devices:
            if device.is_activated:
                device.deactivate()

    def activate(self):
        for device in self._kinect2devices:
            device.activate()

    def deactivate(self):
        for device in self._kinect2devices:
            device.deactivate()

    def publish_imgs(self, *target_frames):
        # deviceごとにpublishする
        for device, color_pub, ir_pub, depth_pub in zip(self._kinect2devices, self.color_img_pubs,
                                                        self.ir_img_pubs, self.depth_img_pubs):
            device.update_frame()
            if self.Frame.ALL in target_frames:
                cv_images = device.get_all_frames()
                try:
                    color_pub.publish(self._cvbridge.cv2_to_imgmsg(cv_images(0), "bgr8"))
                    ir_pub.publish(self._cvbridge.cv2_to_imgmsg(cv_images(1)))
                    depth_pub.publish(self._cvbridge.cv2_to_imgmsg(cv_images(2)))
                except CvBridgeError, e:
                    print e

            else:
                if self.Frame.COLOR in target_frames:
                    cv_image = device.get_color_frame()
                    try:
                        color_pub.publish(self._cvbridge.cv2_to_imgmsg(cv_image, "bgr8"))
                    except CvBridgeError, e:
                        print e

                if self.Frame.IR in target_frames:
                    cv_image = device.get_ir_frame()
                    try:
                        ir_pub.publish(self._cvbridge.cv2_to_imgmsg(cv_image))
                    except CvBridgeError, e:
                        print e

                if self.Frame.COLOR in target_frames:
                    cv_image = device.get_depth_frame()
                    try:
                        depth_pub.publish(self._cvbridge.cv2_to_imgmsg(cv_image))
                    except CvBridgeError, e:
                        print e


# --------------------------------------------
if __name__ == '__main__':
    rospy.init_node(DEFAULT_NODE_NAME, anonymous=True)
    rate_mgr = rospy.Rate(30)  # Hz

    kinect2_bridge = Kinect2Bridge(['017748151147', '017779451147'], ['A', 'B'])
    kinect2_bridge.activate()

    while not rospy.is_shutdown():
        kinect2_bridge.publish_imgs(Kinect2Bridge.Frame.ALL)
        rate_mgr.sleep()

    kinect2_bridge.deactivate()




















