import cv2
from cv_bridge import CvBridge

class UsbCam(object):
    def __init__(self, device, live_view_active=True):
        self.camera = cv2.VideoCapture(device)
        self.bridge = CvBridge()
        self.isLiveViewActive = live_view_active

    """
    def show(self):
        if self.isLiveViewActive is False or \
                self.camera is None or \
                self.camera.isOpened() is False:
            return
        while(True):
            ret, frame = self.camera.read()
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            cv2.imshow('Live View', rgb)

        self.close()
    """

    def capture(self):
        if self.camera is None or self.camera.isOpened() is False:
            return
        ret, frame = self.camera.read()
        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        return self.bridge.cv2_to_imgmsg(image_rgb, "rgb8")
        	
    def close(self):
        if self.camera is None or self.camera.isOpened() is False:
            return
        else:	
            self.camera.release()
            if self.isLiveViewActive:
                cv2.destroyAllWindows()
                self.isLiveViewActive = False


