from swarmrobot.swarmrobot import SwarmRobot
from time import sleep


def main():
    bot = SwarmRobot()
    # Calibrate Bot
    bot.calibrate(False, True)
    # Setup automatic Linedetection
    bot.set_autopilot_state(active=True)
    bot.set_leave_line_reverse(True)
    # Setup Navigation
    bot.set_navigation_state(active=True)
    bot.set_goal('1')
    # Setup Intersectiondetection
    bot.set_intsecdet_state(active=True)
    # Set velocity of Bot
    bot.set_power_lvl(29)
    bot.change_drive_power(bot.power_lvl)
    sleep(300)
    bot.stop_all()
    

if __name__ == '__main__':
    main()
