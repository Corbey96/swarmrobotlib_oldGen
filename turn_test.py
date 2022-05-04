from swarmrobot import SwarmRobot
from time import sleep
from PIL import Image
from turn_assistant import Turn_Assistant
import numpy as np

from motor import CalibratedMotor, Motor

from qrDetection import QrDetection

import qr_test as reader
import cv2

def main():
    #bot = SwarmRobot()
    #bot.calibrate(False, True)
    #qrdetect = QrDetection()
    #img = Image.open('/home/pi/qrImg/l0;g0;r1.png')
    #img = Image.open('/home/pi/qrImg/qrImg_test4.jpg')
    #img = np.array(img)
    #lable, points = qrdetect.detect_QR(img)
    #print("Lable: ", lable)
    #print("Ponits: ", points)
    
    #motor = Motor(Motor._bp.PORT_B)
    #motor.rotate_motor(510)
    
    #ta = Turn_Assistant(bot)
    #ta.turn_90_deg(-1)
    #ta.turn_180_deg_on_spot()
    
    image = cv2.imread('/home/pi/FTP/files/l0;g0;r1.png')#qrImg_test4.jpg')
    codes, frame = reader.extract(image, True)
    
    print(codes)
    #print(frame)

if __name__=='__main__':
    main()