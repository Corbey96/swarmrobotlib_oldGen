from swarmrobot import SwarmRobot
from time import sleep
from PIL import Image
from turn_assistant import Turn_Assistant
import numpy as np

from motor import CalibratedMotor, Motor

from qrDetection import QrDetection
from barCodeDetection import BarCodeDetection

import qr_test as reader
import cv2
import time

def main():
    bot = SwarmRobot()
    bot.calibrate(False, True)
    motor = Motor(Motor._bp.PORT_B)
    time.sleep(1)
    motor.rotate_motor(-610)
    
    #qrdetect = QrDetection()
    
    #ta = Turn_Assistant(bot)
    #ta.turn_90_deg(1)
    #ta.turn_180_deg_on_spot()
    
    #barcodeDetect = BarCodeDetection()
    #image = cv2.imread('/home/pi/qrImg/qrImg_test_14.jpg')
    #image = cv2.imread('/home/pi/FTP/files/qrImg_test9.jpg')
    #image = cv2.imread('/home/pi/Pictures/first_image.jpg')
    #lable, detected = barcodeDetect.detect_barcode(image)
    #print(lable)
    #codes, frame = reader.extract(image, True)
    #lable = lable.decode()+""
    #directions = list(lable)
    #print('direction:',directions)
    #print(frame)
    #bot._navigator.parkingDetector.detect_red_line(image)

if __name__=='__main__':
    main()