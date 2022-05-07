from swarmrobot import SwarmRobot
from time import sleep
import sys

def main():
    bot = SwarmRobot()
    # calibrate bot
    bot.calibrate(False, True)
    # setup automatic line detection 
    bot.set_autopilot_state(active=True)

    # set velocity of bot
    bot.change_drive_power(28)

    sleep(15)
    bot.stop_all()
    sys.exit("Erfolgreich beendet!")


if __name__=='__main__':
    main()