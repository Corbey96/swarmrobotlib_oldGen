from motor import Motor
#from swarmrobot import SwarmRobot

import time

class Turn_Assistant:
    def __init__(self, bot):
        self._bot = bot
        
        self._full_rotation_deg = 510
        
    #direction 1 = links -1 = rechts
    def turn_90_deg(self, direction):
        self._bot.set_drive_steer(1.0*direction)
        self._bot._drive_motor.rotate_motor(-1*self._full_rotation_deg)
        time.sleep(1)
        self._bot.set_drive_steer(-1.0*direction)
        self._bot._drive_motor.rotate_motor(1.2*self._full_rotation_deg)
        
    def turn_180_deg(self):
        self._bot.set_drive_steer(1.0)
        self._bot._drive_motor.rotate_motor(-1.3*self._full_rotation_deg)
        time.sleep(1)
        self._bot.set_drive_steer(-1.0)
        self._bot._drive_motor.rotate_motor(1.3*self._full_rotation_deg)
        time.sleep(1)
        self._bot.set_drive_steer(1.0)
        self._bot._drive_motor.rotate_motor(-1.3*self._full_rotation_deg)
        time.sleep(1)
        self._bot.set_drive_steer(-1.0)
        self._bot._drive_motor.rotate_motor(1.4*self._full_rotation_deg)
        time.sleep(1)
        self._bot.set_drive_steer(0)
        
    def turn_180_deg_on_spot(self):
        self._bot.set_drive_steer(1.0)
        self._bot._drive_motor.rotate_motor(-0.6*self._full_rotation_deg)
        time.sleep(1)
        self._bot.set_drive_steer(-1.0)
        self._bot._drive_motor.rotate_motor(0.7*self._full_rotation_deg)
        time.sleep(1)
        self._bot.set_drive_steer(1.0)
        self._bot._drive_motor.rotate_motor(-0.6*self._full_rotation_deg)
        time.sleep(1)
        self._bot.set_drive_steer(-1.0)
        self._bot._drive_motor.rotate_motor(0.8*self._full_rotation_deg)
        time.sleep(1)
        self._bot.set_drive_steer(1.0)
        self._bot._drive_motor.rotate_motor(-0.6*self._full_rotation_deg)
        time.sleep(1)
        self._bot.set_drive_steer(-1.0)
        self._bot._drive_motor.rotate_motor(0.8*self._full_rotation_deg)
        time.sleep(1)
        self._bot.set_drive_steer(1.0)
        self._bot._drive_motor.rotate_motor(-0.6*self._full_rotation_deg)
        time.sleep(1)
        self._bot.set_drive_steer(-1.0)
        self._bot._drive_motor.rotate_motor(0.8*self._full_rotation_deg)
        time.sleep(1)
        self._bot.set_drive_steer(0)
        
    def turn_0_deg(self):
        self._bot._drive_motor.rotate_motor(1.3*self._full_rotation_deg)
