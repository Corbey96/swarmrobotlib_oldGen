import cv2
import numpy as np


class ParkingSpaceDetection:
        
    def detect_red_line(self, img):
        # threshold on red color
        lowcolor = (0, 36, 235)
        highcolor = (67, 112, 255)
        thresh = cv2.inRange(img, lowcolor, highcolor)

        # apply morphology close
        kernel = np.ones((5,5), np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

        # get contours and filter on area
        contours = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours[0] if len(contours) == 2 else contours[1]

        if len(contours)>0:
            return True
        
        return False
