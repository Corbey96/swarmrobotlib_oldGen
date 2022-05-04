from signDetection import SignDetector
from swarmrobot import SwarmRobot

from time import sleep
import cv2
import numpy as np
import sys

def main():
    bot = SwarmRobot()
    # calibrate bot
    bot.calibrate(False, True)
    
    signDetection = SignDetector()
    
    # init for sign detection
    distance_vars = signDetection.init_distance_to_signs()
    
    while True:
    
        # capture frames from the camera
        _, frame = bot._camera.read()
        draw_image = frame.copy()
        if not frame is None:
            signs = signDetection.detect_traffic_sign(frame, distance_vars)
            if signs is not None:
                print("Detected traffic signs ", end="")
                for sign_name, sign_pos, sign_distance in signs:
                    print(sign_name, end="\n")
                    print(sign_name)
                    print(sign_pos)
                    print(sign_distance)
                    draw_image = signDetection.label_image(draw_image, sign_name, sign_pos, sign_distance)
                    cv2.imshow("The world through olfas eye", draw_image)
        
        # press 'q' to quit program
        if cv2.waitKey(1) == ord("q"):
            print("End program")
            cv2.destroyAllWindows
            sys.exit("Erfolgreich beendet!")
    
    

if __name__=='__main__':
    main()