from swarmrobot import SwarmRobot
from time import sleep
import sys

def main():
    bot = SwarmRobot()
    # calibrate bot
    bot.calibrate(False, True)
    # setup automatic lineDetection 
    # fo bot.set_autopilot_state(active = True)
    # fo bot._setup_autopilot()     
    # fo bot.change_drive_power(23)
    
    # setup sign detection
    bot._setup_sign_detection()
    
    sleep(15)
    bot.stop_all()
    sys.exit("Erfolgreich beendet!")
    
if __name__=='__main__':
    main()