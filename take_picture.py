import cv2
from datetime import datetime
from swarmrobot.swarmrobot import SwarmRobot


def main():
    bot = SwarmRobot()
    # calibrate bot
    bot.calibrate(False, True)
    take_picture(bot, '/home/pi/Studienarbeit/swarmrobotlib_oldGen/detectionsign/referencepictures/picture01_test.png')

    
def take_picture(bot, pic_path):
    _, picture = bot._camera.read()
    val = cv2.imwrite(pic_path, picture)
    if val:
        print(str(datetime.now()) + " | bot took a picture")
    else:
        print("Error while taking picture. Please check picture path.")


if __name__ == '__main__':
    main()
