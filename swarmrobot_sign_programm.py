from swarmrobot import SwarmRobot
from time import sleep
import sys

def main():
    bot = SwarmRobot()
    # calibrate bot
    bot.calibrate(False, True)
    # setup automatic lineDetection 
    bot.set_autopilot_state(active=True)
    bot._setup_autopilot()

    # activate sign detection
    bot.set_sign_detection_state(active=True)

    # set velocity of bot
    bot.change_drive_power(23)

    sleep(25)
    bot.stop_all()
    sys.exit("Erfolgreich beendet!")


if __name__=='__main__':
    main()