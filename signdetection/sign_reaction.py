import time
from datetime import datetime, timedelta

from turn_assistant import Turn_Assistant


class SignReactor:
    def __init__(self, bot):
        self.bot = bot
        self.ta = Turn_Assistant(bot)

        # initial time setup to prevent reaction on same sign over and over again
        self.last_stop_time = datetime(2022, 1, 1, 22, 22, 22)
        self.speed = 28
        self.is_busy = False

    def react_to_sign(self, signs, event):
        for sign_name, sign_pos, sign_distance in signs:
            if sign_name == "sign stop":
                if sign_distance <= 30 and not self.is_busy:
                    current_time = datetime.now()
                    print("current time : " + str(current_time))
                    print("last stop time : " + str(self.last_stop_time))
                    stop_time_blocked = self.last_stop_time + timedelta(seconds=6)
                    print("last stop time + 6s : " + str(stop_time_blocked))
                    if self.last_stop_time == datetime(2022, 1, 1, 22, 22, 22) or (current_time > stop_time_blocked):
                        # block other sign reactions
                        self.is_busy = True
                        print(
                            str(datetime.now()) + " | detected stop sign - distance OK - STOP | distance = " + str(sign_distance))

                        # event to stop follow_line
                        event.set()
                        self.bot.change_drive_power(0)

                        self.last_stop_time = datetime.now()
                        print("set last stop time to: " + str(self.last_stop_time))
                        time.sleep(3)

                        # reactivate follow_line
                        event.clear()
                        self.bot.change_drive_power(self.speed)

                        self.is_busy = False
                        # print("nimmer so gefragt: " + str(self.is_busy))
                        # print("-------------- fertig mit Stopp ----------------------------")
                    else:
                        print(str(datetime.now()) + " | too short time interval to last stop sign detection")
                else:
                    print(str(datetime.now()) + " | detected stop sign - distance to big or bot busy | distance = "
                          + str(sign_distance) + " | busy = " + str(self.is_busy))
            if sign_name == "sign no entry":
                if sign_distance <= 30 and not self.is_busy:
                    self.is_busy = True

                    print(
                        "follow line | detected no entry sign - distance OK - TURN | distance = " + str(sign_distance))

                    # event to stop follow_line
                    event.set()
                    self.bot.change_drive_power(0)

                    self.ta.turn_180_deg()

                    # reactivate follow_line
                    event.clear()
                    self.bot.change_drive_power(self.speed)

                    self.is_busy = False
                else:
                    print(
                        "follow line | detected no entry sign - distance to big or bot busy | distance = "
                        + str(sign_distance) + " | busy = " + str(self.is_busy))
