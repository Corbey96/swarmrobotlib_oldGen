from intersectionDetection import IntersectionDetection
from qrDetection import QrDetection
from datetime import datetime
from turn_assistant import Turn_Assistant

import cv2 as cv
import numpy as np
import time

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
        
        #self._camera = cv.VideoCapture(0)

        # Constants
        self.kernel_size = kernel_size
        self.preview = preview
        self.debug = debug
        
        self.intDetector = IntersectionDetection()
        self.qrDetector = QrDetection()
        
        self.bot = bot
        self.ta = Turn_Assistant(bot)
        
    def navigate(self, image, event):
        if self.preview:
            cv.imshow('preview',image)
        intersection = self.intDetector.detect_intersection(image)
        print(intersection)
        if len(intersection)>0:
            print (intersection)
            event.set()
            self.bot.change_drive_power(0)
            n = self.intDetector.get_intersection_coordinates(intersection)
            if n>=0:
                #print(intersection[n])
                time.sleep(1)
                _, image = self.bot._camera.read()
                while image is None:
                    _, image = self.bot._camera.read()
                qrImg = self.intDetector.get_right_upper_corner_intersection(image, intersection[n])
                #print (qrImg)
                cv.imshow('qrImg',qrImg)
                date = "/home/pi/qrImg/qrImg_"+datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]+ ".jpg"                   
                cv.imwrite(date,qrImg)
                lable, points = self.qrDetector.detect_QR(qrImg)
                #time.sleep(3)
                #print("A")
                print("Lable: ", lable)
                print("Ponits: ", points)
                #bot.stop_all()
                #time.sleep(10)
                if points is not None:
                    #print("B")
                    #turn(lable)
                    self.turn_intersection(self.convert_qr(lable))
        if cv.waitKey(1) == ord("d"):
            cv.destroyAllWindows()
            exit()
                    
    def convert_qr(self, lable):
        lableDirections = self.analyse_lable(lable)
        for direction in lableDirections:
            if direction[0] == _self.goal:
                return direction[1]
        for direction in lableDirections:
            if direction[0] == 0:
                return direction[1]
        return 'g'
        
    # 0=Rundkurs, 1=Parkplatz1, 2=Parkplatz2
    def analyse_lable(self, lable):
        lableDirections = {}
        directions = lable.split(';')
        for direction in directions:
            data = direction.split(':')
            lableDirections[data[1]] = data[0]
        return lableDirections
                    
    def turn_intersection(self, direction):
        steering_angle = 0
        bot.drive_steer(steering_angle)
        print(direction)
        if direction == "r":
            self.ta.turn_90_deg(1)
        elif direction == "l":
            self.ta.turn_90_deg(-1)
        elif direction == "g":
            self.ta.turn_0_deg()
        elif direction == "stop":
            bot.stop_all()
        event.clear()
        self.bot.change_drive_power(23)
        return