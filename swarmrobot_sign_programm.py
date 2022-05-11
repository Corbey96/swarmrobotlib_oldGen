from swarmrobot.swarmrobot import SwarmRobot
from time import sleep
import sys


def main():
    bot = SwarmRobot()
    # calibrate bot
    bot.calibrate(False, True)
    # setup automatic lineDetection 
    bot.set_autopilot_state(active=True)

    # activate sign detection
    bot.set_sign_detection_state(active=True, show_only=False, drive_and_show=False)

    # set velocity of bot
    bot.change_drive_power(25)

    sleep(11)
    bot.stop_all()
    sys.exit("Erfolgreich beendet!")


if __name__ == '__main__':
    main()
