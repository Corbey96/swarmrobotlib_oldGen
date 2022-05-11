from intersectionDetection import IntersectionDetection
from qrDetection import QrDetection
from barCodeDetection import BarCodeDetection
from datetime import datetime
from turn_assistant import Turn_Assistant
from parkingSpaceDetection import ParkingSpaceDetection

from scipy.ndimage import gaussian_filter
from skimage.restoration import denoise_nl_means, estimate_sigma

import cv2 as cv
import numpy as np
import time
import sys

class Navigator:
    def __init__(self, width, height, bot, kernel_size=(5,5), preview=False, debug=False):    
        # Define Region of interest
        self.resolution = (int(width), int(height))
        w = self.resolution[0]//3
        h = self.resolution[1]//2
        self.roi_x1 = self.resolution[0]//2 - w//2
        self.roi_y1 = self.resolution[1] - h
        self.roi_x2 = self.roi_x1 + w
        self.roi_y2 = self.roi_y1 + h

        # Constants
        self.kernel_size = kernel_size
        self.preview = preview
        self.debug = debug
        
        self.intDetector = IntersectionDetection()
        self.qrDetector = QrDetection()
        self.barCodeDetector = BarCodeDetection()
        self.parkingDetector = ParkingSpaceDetection()
        
        self.bot = bot
        self.ta = Turn_Assistant(bot)
        self._event=""
        self._detected = False
        
    def navigate(self, image, event):
        if self.preview:
            cv.imshow('preview',image)
        self._event = event
        # Detect Intersection in image
        intersection = self.bot.intersection
        # When intersection is detected and wsn't allready detected
        if len(intersection)>0 and self._detected == False:
            # Pause line tracking
            self._event.set()
            self.bot.change_drive_power(0)
            
            # calculate where intersection is
            n = self.intDetector.get_intersection_coordinates(intersection)
            # Sort intersection points and delete the bad ones
            #intersection.sort(key=lambda y: y[0])
            #intersection = list(filter(lambda x: x[0][0]>0 and x[0][0]<2000, intersection))
            if n>=0:
                time.sleep(1)
                # Get better img while not moving for scanning
                _, image = self.bot._camera.read()
                while image is None:
                    _, image = self.bot._camera.read()
                # Get part of image that contains relevant data
                qrImg = self.intDetector.get_right_upper_corner_intersection(image, intersection[n])
                # make image sharper and colours intenser/brighter
                kernel = np.array([[-1,-1,-1], [-1,11,-1], [-1,-1,-1]])
                qrImg = cv.filter2D(qrImg, -1, kernel)
                qrImg = self.automatic_brightness_and_contrast(qrImg)

                # Show and save image
                cv.imshow('qrImg',qrImg)
                date = "/home/pi/qrImg/qrImg_"+datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]+ ".jpg"                   
                cv.imwrite(date,qrImg)
                
                # When it contains parking space
                is_parking_space = self.parkingDetector.detect_red_line(qrImg)
                if is_parking_space:
                    self._detected = True
                    # Start parking procedure
                    self.turn_intersection('parking')
                    sys.exit()
                else:
                
                    lable = ""
                    # Detect and read barcode
                    lable, self._detected = self.barCodeDetector.detect_barcode(qrImg)
                    print("Lable: ", lable)
                    if self._detected:
                        # Start turning procedure
                        self.turn_intersection(self.analyse_barcode(lable))
                        # Unpause line tracking
                        self._event.clear()
                        self.bot.change_drive_power(self.bot.power_lvl)
                self._detected = False
                
        # If robot waits at intersection, but doesn't see intersection anymore        
        #elif self._event.isSet() and len(intersection)==0:
        #    self.bot._drive_motor.rotate_motor(-0.2*self.bot.full_rotation_deg)
            
        if cv.waitKey(1) == ord("d"):
            cv.destroyAllWindows()
            exit()
        
    # 0=Rundkurs, 1=Parkplatz1, 2=Parkplatz2    
    def analyse_barcode(self, lable):
        lable = lable.decode()+""
        directions = list(lable)
        print('direction:',directions)
        if directions[0] == self.bot.goal:
            return 'l'
        elif directions[1] == self.bot.goal:
            return 'r'
        else:
            return 'g'
                    
    def turn_intersection(self, direction):
        steering_angle = 0
        self.bot.set_drive_steer(steering_angle)
        print(direction)
        if direction == "r":
            self.ta.turn_90_deg(-1)
        elif direction == "l":
            self.ta.turn_90_deg(1)
        elif direction == "g":
            self.ta.turn_0_deg()
        elif direction == "parking":
            self.ta.park_backwards()
        elif direction == "stop":
            self.bot.stop_all()
        return

    # Automatic brightness and contrast optimization with optional histogram clipping
    def automatic_brightness_and_contrast(self, image, clip_hist_percent=1):
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    
        # Calculate grayscale histogram
        hist = cv.calcHist([gray],[0],None,[256],[0,256])
        hist_size = len(hist)
    
        # Calculate cumulative distribution from the histogram
        accumulator = []
        accumulator.append(float(hist[0]))
        for index in range(1, hist_size):
            accumulator.append(accumulator[index -1] + float(hist[index]))
    
        # Locate points to clip
        maximum = accumulator[-1]
        clip_hist_percent *= (maximum/100.0)
        clip_hist_percent /= 2.0
    
        # Locate left cut
        minimum_gray = 0
        while accumulator[minimum_gray] < clip_hist_percent:
            minimum_gray += 1
    
        # Locate right cut
        maximum_gray = hist_size -1
        while accumulator[maximum_gray] >= (maximum - clip_hist_percent):
            maximum_gray -= 1
    
        # Calculate alpha and beta values
        alpha = 255 / (maximum_gray - minimum_gray)
        beta = -minimum_gray * alpha
    
        '''
        # Calculate new histogram with desired range and show histogram 
        new_hist = cv2.calcHist([gray],[0],None,[256],[minimum_gray,maximum_gray])
        plt.plot(hist)
        plt.plot(new_hist)
        plt.xlim([0,256])
        plt.show()
        '''

        auto_result = cv.convertScaleAbs(image, alpha=alpha, beta=beta)
        return auto_result
        
        
        
        
        
        
        