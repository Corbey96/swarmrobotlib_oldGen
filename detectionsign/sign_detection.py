import cv2


class SignDetector:
    def __init__(self):
        # init distance vars
        self.distance_vars = self.init_distance_to_signs()

    """
    Control instance for sign detection with distance measurement.
    """

    # focal length finder function
    def focal_length(self, measured_distance, real_width, width_in_rf_image):
        """
        This Function Calculate the Focal Length(distance between lens to CMOS sensor), it is simple constant we can find by using
        MEASURED_DISTACE, REAL_WIDTH(Actual width of object) and WIDTH_OF_OBJECT_IN_IMAGE
        :param1 Measure_Distance(int): It is distance measured from object to the Camera while Capturing Reference image
        :param2 Real_Width(int): It is Actual width of object, in real world (like width of a stop sign is = 4.0 centimeters)
        :param3 Width_In_Image(int): It is object width in the frame /image in our case in the reference image(found by sign detector)
        :retrun focal_length(Float):"""
        focal_length_value = (width_in_rf_image * measured_distance) / real_width
        return focal_length_value

    # distance estimation function
    def distance_finder(self, focal_length, real_object_width, object_width_in_frame):
        """
        This Function simply Estimates the distance between object and camera using arguments(focal_length, Actual_object_width, Object_width_in_the_image)
        :param1 focal_length(float): return by the focal_length_Finder function
        :param2 Real_Width(int): It is Actual width of object, in real world
        :param3 object_Width_Frame(int): width of object in the image(frame in our case, using Video feed)
        :return Distance(float) : distance Estimated
        """
        distance = (real_object_width * focal_length) / object_width_in_frame
        # print("real_object_width", real_object_width)
        # print("focal_length", focal_length)
        # print("object_width_in_frame", object_width_in_frame)
        return distance

    def extract_sign_width_in_frame(self, detected_sign):
        for (x, y, h, w) in detected_sign:
            sign_width_in_frame = w  # get sign width of sign in frame
            # print("sign_width_in_frame", sign_width_in_frame)
        return sign_width_in_frame

    def detect(self, image, cascade: str):
        """
        Analyses the given image for the occurrence of the given cascade.

        :param image:   image to analyse
        :param cascade: list with border points/vertices (or a trained classifier)
        :return: list of tuples (x,y,w,h)
        """
        cascade = cv2.CascadeClassifier(cascade)

        if image is not None:
            sign_width = 0
            # Converts the color range of the image to gray-tone
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            # Searches classifier in the gray-image
            sign = cascade.detectMultiScale(gray, 1.3, 5)  # 1.1 = scaleFactor, 3 = minNeighbors)

            return sign

        print("frame is invalid")
        return None
    
    def init_distance_to_signs(self):
        """
        Calculate the distance parameter of reference images.
        Add values to a dict to access the variables later.
        """
        distance_vars = {}
        # stop parameter
        stop_cascade_classifier = 'classifiers/stop_classifier_01.xml'
        stop_known_distance = 30.0  # cm
        stop_known_width = 4.0  # cm
        stop_ref_image = cv2.imread("detectionsign/referencepictures/stop_30_sr14.png")
        stop_ref_image_detected = self.detect(stop_ref_image, stop_cascade_classifier)
        stop_ref_sign_width = self.extract_sign_width_in_frame(stop_ref_image_detected)
        stop_focal_length_found = self.focal_length(stop_known_distance,
                                                    stop_known_width,
                                                    stop_ref_sign_width)
        # print("stop_focal_length_found: ", stop_focal_length_found)
        distance_vars["stop_cascade_classifier"] = stop_cascade_classifier
        distance_vars["stop_known_width"] = stop_known_width
        distance_vars["stop_focal_length_found"] = stop_focal_length_found

        # no entry parameter
        no_entry_cascade_classifier = 'classifiers/cascade_no_entry_06.xml'
        no_entry_known_distance = 30.0  # cm
        no_entry_known_width = 4.0  # cm
        no_entry_ref_image = cv2.imread("detectionsign/referencepictures/no_entry_30_sr14.png")
        no_entry_ref_image_detected = self.detect(no_entry_ref_image,
                                                  no_entry_cascade_classifier)
        no_entry_ref_sign_width = self.extract_sign_width_in_frame(no_entry_ref_image_detected)
        no_entry_focal_length_found = self.focal_length(no_entry_known_distance,
                                                        no_entry_known_width,
                                                        no_entry_ref_sign_width)
        # print("no_entry_focal_length_found: ", no_entry_focal_length_found)
        distance_vars["no_entry_cascade_classifier"] = no_entry_cascade_classifier
        distance_vars["no_entry_known_width"] = no_entry_known_width
        distance_vars["no_entry_focal_length_found"] = no_entry_focal_length_found
        
        return distance_vars
        

    def detect_traffic_sign(self, image):
        """
        Find stop signs, speed signs and no entry signs.

        :return: [("sign name", (x, y, h, w)), ...]
        """

        # display parameters
        font = cv2.FONT_HERSHEY_SIMPLEX
        color = (0, 255, 0)  # green

        signs = []
        # Detect stop sign in image
        for (x, y, w, h) in self.detect(image, self.distance_vars.get("stop_cascade_classifier")):
            stop_sign_width_in_frame = w
            if stop_sign_width_in_frame != 0:
                distance = self.distance_finder(self.distance_vars.get("stop_focal_length_found"),
                                                self.distance_vars.get("stop_known_width"),
                                                stop_sign_width_in_frame)
                # make distance to int cause accuracy without decimals is enough
                int_distance = int(distance)
                print("sign detection | distance to stop sign is: " + str(int_distance))
                signs.append(("sign stop", (x, y, w, h), int_distance))
            else:
                signs.append(("sign stop | distance UNKNOWN", (x, y, w, h), 0))

        # Detect no entry sign in image
        for (x, y, w, h) in self.detect(image,
                                        self.distance_vars.get("no_entry_cascade_classifier")):
            no_entry_sign_width_in_frame = w
            if no_entry_sign_width_in_frame != 0:
                distance = self.distance_finder(self.distance_vars.get("no_entry_focal_length_found"),
                                                self.distance_vars.get("no_entry_known_width"),
                                                no_entry_sign_width_in_frame)
                # make distance to int cause accuracy without decimals is enough
                int_distance = int(distance)
                print("no entry detection | distance to no entry sign is: " + str(int_distance))
                signs.append(("sign no entry", (x, y, w, h), int_distance))
            else:
                signs.append(("sign no entry | distance UNKNOWN", (x, y, w, h), 0))
        if signs:  # not empty?
            return signs
        return None
    
    def label_image(self, image, name, detection, distance):
        if detection is not None:
            x, y, w, h = detection
            startX = int(x)
            startY = int(y)
            endX = int(x + w)
            endY = int(y + h)
            thickness = 2
            color = (0, 255, 0)
            image = cv2.rectangle(image, (startX, startY), (endX, endY), color, thickness)
            font = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 0.75
            cv2.putText(image, name, (startX, startY-5), font, fontScale, color, thickness, cv2.LINE_AA)
            cv2.putText(image, f"distance = {round(distance,2)} cm", (startX, startY-20), font, fontScale, color, 2)
            
        return image
