from swarmrobot.swarmrobot import SwarmRobot

from time import sleep
import cv2
import sys


def main():
    bot = SwarmRobot()
    # calibrate bot
    bot.calibrate(False, True)
    
    # activate sign detection
    bot.set_sign_detection_state(active=True, show_only=True, drive_and_show=False)
    
    sleep(15)
    
    cv2.destroyAllWindows
    sys.exit("Erfolgreich beendet!")
    

if __name__ == '__main__':
    main()
