import cv2
from datetime import datetime
from swarmrobot.swarmrobot import SwarmRobot


def main():
    bot = SwarmRobot()
    # calibrate bot
    bot.calibrate(False, True)
    take_picture(bot, '/detectionsign/referencepictures/picture01.png')

    
def take_picture(bot, pic_path):
    _, picture = bot._camera.read()
    cv2.imwrite(pic_path, picture)
    print(str(datetime.now()) + " | bot took a picture")


if __name__ == '__main__':
    main()
