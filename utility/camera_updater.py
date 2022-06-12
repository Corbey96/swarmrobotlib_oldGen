import cv2 as cv

class Camera_updater:  
    def __init__(self, bot, preview=False, debug=False):
        self._bot = bot
        
    def update_camera(self):
        if self._bot._camera is not None:
            _, image = self._bot._camera.read()
            while image is None:
                _, image = self._bot._camera.read()