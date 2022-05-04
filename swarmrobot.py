from motor import CalibratedMotor, Motor
from pidcontroller import PIDController
from line_tracking import LineTracker
from navigation import Navigator
from signDetection import SignDetection
from threading import Thread, Event
import cv2

class SwarmRobot:
    def __init__(self):
        self._drive_motor = Motor(Motor._bp.PORT_B)
        self._steer_motor = CalibratedMotor(Motor._bp.PORT_D, calpow=40)
        self._fork_lift_motor = CalibratedMotor(Motor._bp.PORT_C, calpow=50)
        self._fork_tilt_motor = CalibratedMotor(Motor._bp.PORT_A, calpow=40)

        # Camera
        self._camera = cv2.VideoCapture(0)
        
        self._event = Event()
        
        self._goal = 1

        # Linetracking
        self._track_process = None
        self._track_active = False
        self._pid_controller = PIDController(verbose=False)
        self._line_tracker = LineTracker(self._camera.get(cv2.CAP_PROP_FRAME_WIDTH), self._camera.get(cv2.CAP_PROP_FRAME_HEIGHT), preview=False, debug=False)

        # Navigation
        self._navigation_process = None
        self._navigation_active = False
        self._navigator = Navigator(self._camera.get(cv2.CAP_PROP_FRAME_WIDTH), self._camera.get(cv2.CAP_PROP_FRAME_HEIGHT), self, preview=True, debug=False)
        
    def __del__(self):
        self._steer_motor.to_init_position()
        self.stop_all()

    def change_drive_power(self, pnew):
        self._drive_motor.change_power(pnew)

    def set_drive_power(self, pnew):
        self._drive_motor.set_power(pnew)

    def set_drive_steer(self, pnew):
        pos = self._steer_motor.position_from_factor(pnew)
        self._steer_motor.set_position(pos)

    def calibrate(self, calibrate_forklift=False, verbose=False):
        print('Calibrating steering')
        self._steer_motor.calibrate(verbose)
        if(calibrate_forklift):
            print('Calibrating forklift lift motor')
            self._fork_lift_motor.calibrate(verbose)
            print('Calibrating forklift tilt motor')
            self._fork_tilt_motor.calibrate_offset(53000,verbose)

    def stop_all(self):
        self._drive_motor.stop()
        self._steer_motor.stop()
        cv2.destroyAllWindows()

    def _setup_autopilot(self):
        from time import sleep

        def follow(event):
            try:
                while True:
                    if not self._track_active:
                        sleep(0.5)

                    if self._track_active:
                        _,frame = self._camera.read()
                        if not frame is None:
                            cv2.imshow('ultimativer Frame', frame)
                            if cv2.waitKey(1) == ord("q"):
                                break
                            pos = self._line_tracker.track_line(frame, event)
                            if pos != None:
                                steer = self._pid_controller.pid(pos)
                                self.set_drive_steer(steer)
                            else:
                                print('[!] no frame available')
            except KeyboardInterrupt:
                self.stop_all()
                # cv2.destroyAllWindows() - noch erforderlich?
                # self._bot.stop_all()
            finally:
                self.stop_all()
                # cv2.destroyAllWindows() - noch erforderlich?
                # self._bot.stop_all()

        self._track_process = Thread(group=None, target=follow, daemon=True, args=(self._event,))
        self._track_process.start()

    def get_autopilot_state(self):
        return self._track_active

    def set_autopilot_state(self, active:bool):
        self._track_active = active
        if(active and self._track_process == None):
            self._setup_autopilot()

    def _setup_classifier(self):
        from .classifier import Classifier
        
    def _setup_navigation(self):
        from time import sleep
        
        def navigate(event):
            try:
                while True:
                    if not self._navigation_active:
                        sleep(5)
                        
                    if self._navigation_active:
                        _,frame = self._camera.read()
                        if not frame is None:
                            self._navigator.navigate(frame, event)
            except KeyboardInterrupt:
                self.stop_all()
            finally:
                self.stop_all()
        
        self._navigation_process = Thread(group=None, target=navigate, daemon=True, args=(self._event,))
        self._navigation_process.start()
    
    def set_navigaton_state(self, active:bool):
        self._navigation_active = active
        if(active and self._navigation_process == None):
            self._setup_navigation()
            
            
    #def set_goal(self, ):
    #    self.

    def _setup_sign_detection(self):
        signDetection = SignDetection()
        distance_vars = signDetection.init_distance_to_signs()

        def detect():
            try:
                while True:
                    # capture frames from the camera
                    _, frame = self._camera.read()
                    draw_image = frame.copy()
                    if not frame is None:
                        signs = signDetection.detect_traffic_sign(frame, distance_vars)
                        if signs is not None:
                            print("Detected traffic signs ", end="")
                            for sign_name, sign_pos, sign_distance in signs:
                                print(sign_name, end="\n")
                                draw_image = signDetection.label_image(draw_image, sign_name, sign_pos, sign_distance)
                                cv2.imshow("The world through olfas eye", draw_image)
                    else:
                        print("I hob köi Bild")

                    if cv2.waitKey(1) == ord("q"):
                        print("irgendwas")

            except KeyboardInterrupt:
                self.stop_all()
            finally:
                self.stop_all()

        self._track_process = Thread(group=None, target=detect, daemon=True)
        self._track_process.start()