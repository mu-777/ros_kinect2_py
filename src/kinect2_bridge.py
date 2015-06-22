#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import signal
import pyfreenect2


class Kinect2Device(object):
    def __init__(self, serial_number=None):
        serial = pyfreenect2.getDefaultDeviceSerialNumber() if serial_number is None else serial_number
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

    def get_all_frames(self):
        if not self.is_activated:
            self.activate()
        frames = self._frameListener.waitForNewFrame()
        rgb_data = frames.getFrame(pyfreenect2.Frame.COLOR).getData()
        ir_data = frames.getFrame(pyfreenect2.Frame.IR).getData()
        depth_data = frames.getFrame(pyfreenect2.Frame.DEPTH).getData()
        return rgb_data, ir_data, depth_data

    def get_rgb_frame(self):
        if not self.is_activated:
            self.activate()
        frames = self._frameListener.waitForNewFrame()
        return frames.getFrame(pyfreenect2.Frame.COLOR).getData()

    def get_ir_frame(self):
        if not self.is_activated:
            self.activate()
        frames = self._frameListener.waitForNewFrame()
        return frames.getFrame(pyfreenect2.Frame.IR).getData()

    def get_depth_frame(self):
        if not self.is_activated:
            self.activate()
        frames = self._frameListener.waitForNewFrame()
        return frames.getFrame(pyfreenect2.Frame.DEPTH).getData()


class Kinect2Bridge(object):
    def __init__(self, serial_numbers=[]):
        self._kinect2devices = []
        if len(serial_numbers) is 0:
            self._kinect2devices.append(Kinect2Device())
        else:
            for serial_number in serial_numbers:
                self._kinect2devices.append(Kinect2Device(serial_number))


    def activate(self):
        for device in self._kinect2devices:
            device.activate()

    def deactivate(self):
        for device in self._kinect2devices:
            device.deactivate()




