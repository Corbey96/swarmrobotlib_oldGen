from swarmrobot.swarmrobot import SwarmRobot
from time import sleep
import sys


def main():
    bot = SwarmRobot()
    # calibrate bot
    bot.calibrate(False, True)
    # setup automatic line detection 
    bot.set_autopilot_state(active=True)
    
    # set velocity of bot
    bot.set_power_lvl(28)
    bot.change_drive_power_lvl()

    # duration of program in sec
    sleep(25)
    bot.stop_all()
    sys.exit("finished successfully")


if __name__ == '__main__':
    main()
