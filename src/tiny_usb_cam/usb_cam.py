#!/usr/bin/env python

import cv2
import threading

class UsbCam(object):
    def __init__(self, device, fps=30, width=640, height=480):
        self.device = device
        self.camera = cv2.VideoCapture(self.device)
        self.set_resolution(width, height)
        self.set_fps(fps)
        self.video_recorder = None
        self.lock = threading.Lock()

    def is_camera_opened(self):
        if self.camera and self.camera.isOpened():
            return True
        else:
            print("Unable to read camera feed")
            return False

    def is_video_recorder_opened(self):
        if self.video_recorder and self.video_recorder.isOpened():
            return True
        else:
            print("Unable to write camera feed")
            return False

    def start_video_recording(self, filename="output.avi"):
        with self.lock:
            self.video_recorder = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc('X','V','I','D'), self.get_fps(), self.get_resolution())

    def stop_video_recording(self):
        with self.lock:
            if not self.is_video_recorder_opened():
                return
            self.video_recorder.release()

    def write(self, frame):
        with self.lock:
            if not self.is_video_recorder_opened():
                return
            self.video_recorder.write(frame)

    def capture(self):
        if not self.is_camera_opened():
            return
        ret, frame = self.camera.read()
        return frame

    def close(self):
        if not self.is_camera_opened():
            return
        else:	
            self.camera.release()

    def get_resolution(self):
        width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return (width, height)

    def set_resolution(self, width, height):
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def get_fps(self):
        return self.camera.get(cv2.CAP_PROP_FPS)

    def set_fps(self, fps):
        self.camera.set(cv2.CAP_PROP_FPS, fps)