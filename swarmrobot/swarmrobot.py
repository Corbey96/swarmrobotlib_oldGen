from utility.motor import CalibratedMotor, Motor
from linetracking.pidcontroller import PIDController
from linetracking.line_tracking import LineTracker
from navigation.navigation import Navigator
from detection.intersection_detection_threading import IntersectionDetection
from detectionsign.sign_detection import SignDetector
from detectionsign.sign_reaction import SignReactor
from threading import Thread, Event
import cv2
import sys


class SwarmRobot:
    def __init__(self):
        self._drive_motor = Motor(Motor._bp.PORT_B)
        self._steer_motor = CalibratedMotor(Motor._bp.PORT_D, calpow=40)
        self._fork_lift_motor = CalibratedMotor(Motor._bp.PORT_C, calpow=50)
        self._fork_tilt_motor = CalibratedMotor(Motor._bp.PORT_A, calpow=40)

        # Camera
        self._camera = cv2.VideoCapture(0)
        
        self._event = Event()
        
        self.goal = '1'
        self.steer = 0
        self.full_rotation_deg = 510
        self.last_line_tracking = 0
        self.power_lvl = 0

        # Linetracking
        self._track_process = None
        self._track_active = False
        self._pid_controller = PIDController(verbose=False)
        self._line_tracker = LineTracker(self._camera.get(cv2.CAP_PROP_FRAME_WIDTH), self._camera.get(cv2.CAP_PROP_FRAME_HEIGHT), preview=False, debug=False)

        # Navigation
        self._navigation_process = None
        self._navigation_active = False
        self._navigator = Navigator(self._camera.get(cv2.CAP_PROP_FRAME_WIDTH), self._camera.get(cv2.CAP_PROP_FRAME_HEIGHT), self, preview=True, debug=False)
        
        # Intersection detection
        self._intsecdet_process = None
        self._intsecdet_active = False
        self._intersection_detector = IntersectionDetection(self._camera.get(cv2.CAP_PROP_FRAME_WIDTH), self._camera.get(cv2.CAP_PROP_FRAME_HEIGHT), self, preview=True, debug=False)
        self.intersection = []

        # sign detection
        self._sign_detection_process = None
        self._sign_detection_active = False
        self._do_save_detection = False
        self._show_only = False
        self._drive_and_show = False
        self._sign_detector = SignDetector()

        self._sign_reactor = SignReactor(self)
        
    def __del__(self):
        self._steer_motor.to_init_position()
        self.stop_all()

    def change_drive_power(self, pnew):
        self._drive_motor.change_power(pnew)

    def set_drive_power(self, pnew):
        self._drive_motor.set_power(pnew)

    def set_power_lvl(self, lvl):
        self.power_lvl = lvl

    def change_drive_power_lvl(self):
        self._drive_motor.change_power(self.power_lvl)

    def set_drive_steer(self, pnew):
        pos = self._steer_motor.position_from_factor(pnew)
        self._steer_motor.set_position(pos)

    def calibrate(self, calibrate_forklift=False, verbose=False):
        print('Calibrating steering')
        self._steer_motor.calibrate(verbose)
        if calibrate_forklift:
            print('Calibrating forklift lift motor')
            self._fork_lift_motor.calibrate(verbose)
            print('Calibrating forklift tilt motor')
            self._fork_tilt_motor.calibrate_offset(53000, verbose)

    def stop_all(self):
        self._drive_motor.stop()
        self._steer_motor.stop()

    def _setup_autopilot(self):
        from time import sleep

        def follow(event):
            try:
                while True:
                    if not self._track_active:
                        sleep(0.5)

                    if self._track_active:
                        _, frame = self._camera.read()
                        if frame is not None:
                            # show current camara frame
                            #cv2.imshow('frame | line tracking', frame)
                            if cv2.waitKey(1) == ord("q"):
                                break
                            pos = self._line_tracker.track_line(frame, event, self)
                            self.last_line_tracking = pos
                            if pos is not None:
                                self.steer = self._pid_controller.pid(pos)
                                self.set_drive_steer(self.steer) 
            except KeyboardInterrupt:
                self.stop_all()
            finally:
                self.stop_all()

        self._track_process = Thread(group=None, target=follow, daemon=True, args=(self._event,))
        self._track_process.start()

    def get_autopilot_state(self):
        return self._track_active

    def set_autopilot_state(self, active: bool):
        self._track_active = active
        if active and self._track_process is None:
            self._setup_autopilot()
        
    def _setup_navigation(self):
        from time import sleep
        
        def navigate(event):
            try:
                while True:
                    if not self._navigation_active:
                        sleep(5)
                        
                    if self._navigation_active:
                        _, frame = self._camera.read()
                        if frame is not None:
                            self._navigator.navigate(frame, event)
            except KeyboardInterrupt:
                self.stop_all()
            finally:
                self.stop_all()
        
        self._navigation_process = Thread(group=None, target=navigate, daemon=True, args=(self._event,))
        self._navigation_process.start()
    
    def set_navigation_state(self, active: bool):
        self._navigation_active = active
        if active and self._navigation_process is None:
            self._setup_navigation()
            
    def _setup_intersection_detection(self):
        from time import sleep
        
        def detect_intersection():
            try:
                while True:
                    if not self._intsecdet_active:
                        sleep(5)
                        
                    if self._intsecdet_active:
                        _, frame = self._camera.read()
                        if frame is not None:
                            self._intersection_detector.detect_intersection(frame)
            except KeyboardInterrupt:
                self.stop_all()
            finally:
                self.stop_all()
        
        self._intsecdet_process = Thread(group=None, target=detect_intersection, daemon=True)
        self._intsecdet_process.start()
    
    def set_intsecdet_state(self, active: bool):
        self._intsecdet_active = active
        if active and self._intsecdet_process is None:
            self._setup_intersection_detection()
            
    def set_goal(self, goal):
        self.goal = goal

    def _setup_sign_detection(self):
        import time
        from time import sleep

        def detect(event):
            try:
                while True:
                    if not self._sign_detection_active:
                        sleep(5)
                    else:
                        # capture frames from the camera
                        _, frame = self._camera.read()
                        if frame is not None:
                            draw_image = frame.copy()
                            signs = self._sign_detector.detect_traffic_sign(frame)

                            if self._show_only or self._drive_and_show:
                                cv2.imshow("frame", draw_image)

                            if signs is not None:
                                print("Detected traffic signs ", end="")
                                # show images in camera stream
                                for sign_name, sign_pos, sign_distance in signs:
                                    print(sign_name, end="\n")
                                    draw_image = self._sign_detector.label_image(draw_image, sign_name, sign_pos, sign_distance)
                                    # save picures of detected signs
                                    if self._do_save_detection:
                                        cv2.imwrite("./detectionsign/signdetectionpictures/traffic_sign_detection_"+ sign_name + "_" + str(time.time()) + ".jpg", draw_image)
                                    # update image
                                    if self._show_only or self._drive_and_show:
                                        cv2.imshow("frame", draw_image)

                                if not self._show_only:
                                    self._sign_reactor.react_to_sign(signs, event)

                            # show video stream - eventually performance little lower cause of that
                            if self._show_only or self._drive_and_show:
                                cv2.imshow("frame", draw_image)

 
                        else:
                            print("------------ no picture ------------")

                        if cv2.waitKey(1) == ord("q"):
                            print("stopping program...")
                            self.stop_all()
                            sys.exit("stop program successfully")
            except KeyboardInterrupt:
                print("stopping program...")
                self.stop_all()
                sys.exit("stop program successfully")
            finally:
                print("stopping program...")
                self.stop_all()
                sys.exit("stop program successfully")
        self._track_process = Thread(group=None, target=detect, daemon=True, args=(self._event,))
        self._track_process.start()

    def set_sign_detection_state(self, active: bool, show_only: bool, drive_and_show: bool):
        self._sign_detection_active = active
        self._show_only = show_only
        self._drive_and_show = drive_and_show
        if active and self._sign_detection_process is None:
            self._setup_sign_detection()
