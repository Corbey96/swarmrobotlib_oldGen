import sys
sys.path.insert(1, '/home/pi/Studienarbeit/swarmrobotlib_oldGen')
from swarmrobot.swarmrobot import SwarmRobot

# checks if the motors are coupled up correctly

if __name__ == '__main__':
    print('stopping all...')
    SwarmRobot().stop_all()
    print('done...')
