from swarmrobot import SwarmRobot
from time import sleep

def main():
    bot = SwarmRobot()
    # Calibrate Bot
    bot.calibrate(False, True)
    # Setup automatic Linedetection
    bot.set_autopilot_state(active = True)
    bot._setup_autopilot()
    # Setup Navigation
    bot.set_navigaton_state(active = True)
    bot._setup_navigation()
    # Set velocity of Bot
    bot.change_drive_power(23)
    sleep(100)
    bot.stop_all()
    

if __name__=='__main__':
    main()