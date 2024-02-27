# Core opencv code provided by Einsteinium Studios
# Juliette and Theresa P4
import RPi.GPIO as GPIO
import numpy as np
import cv2
from picamera2 import Picamera2
from libcamera import controls
import time
picam2 = Picamera2() # assigns camera variable
picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous}) # sets auto focus mode
picam2.start() # activates camera
time.sleep(1) # wait to give camera time to start up
####   MOTOR SET UP    ####
# Set up PWM for forward rotation
OUT1 = 12 #left
OUT2 = 11 #left
OUT3 = 13 #right
OUT4 = 15 #right
GPIO.setmode(GPIO.BOARD)
GPIO.setup(OUT1, GPIO.OUT)
GPIO.setup(OUT2, GPIO.OUT)
GPIO.setup(OUT3, GPIO.OUT)
GPIO.setup(OUT4, GPIO.OUT)
pwm_motorL1 = GPIO.PWM(OUT1, 50) # 50 Hz frequency
pwm_motorL2 = GPIO.PWM(OUT2, 50)
pwm_motorR1 = GPIO.PWM(OUT3, 50)
pwm_motorR2 = GPIO.PWM(OUT4, 50)
# for speed control
max_PWM = 100 # this is the maximum duty cycle possible for the dc motor with RasPi, which enables maximum speed (0.0 <= dc <= 100.0)
min_PWM = 20
# Start PWM at 0
pwm_motorL1.start(0)
pwm_motorL2.start(0)
pwm_motorR1.start(0)
pwm_motorR2.start(0)
class PIDController:
    def __init__(self, kp, kd, setpoint): #kd,
        self.kp = kp
        self.kd = kd  # New derivative term
        self.setpoint = setpoint
        self.prev_error = 0
    def update(self, curr_value):
        error = self.setpoint - curr_value
        p_term = self.kp * error
        d_term = self.kd * (error - self.prev_error)
        output = p_term + d_term
        print("output:", output)
        self.prev_error = error
        return output
def motor_run(output):
    if 0 <= (25 - output) <= 100 and 0 <= (25 + output) <= 100:
        pwm_motorL1.ChangeDutyCycle(25 - output)
        pwm_motorL2.ChangeDutyCycle(0)
        print("L speed - ", 20 - output)
        pwm_motorR1.ChangeDutyCycle(25 + output)
        pwm_motorR2.ChangeDutyCycle(0)
        print("R speed - ", 20 + output)
        #time.sleep(.2)
        #pwm_motorL1.ChangeDutyCycle(0)
        #pwm_motorR1.ChangeDutyCycle(0)
        #time.sleep(.1)
    else:
        print("Duty cycle out of range.")
        GPIO.cleanup()
        time.sleep(0.001)
pid_controller = PIDController(kp=.13, kd = 0.006, setpoint = 90) #Change set point kd = 0.001,
# Initial default values for cx and cy
cx, cy = None, None
try:
    while True:
        # Display camera input
        image = picam2.capture_array("main")
        cv2.imshow('img',image)
        # Crop the image
        #crop_img = image[60:120, 0:160]
        crop_img = image[30:240, 0:320]
        # Convert to grayscale
        gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
        # Gaussian blur
        blur = cv2.GaussianBlur(gray,(5,5),0)
        # Color thresholding
        input_threshold,comp_threshold = cv2.threshold(blur,60,255,cv2.THRESH_BINARY_INV)
        # Find the contours of the frame
        contours,hierarchy = cv2.findContours(comp_threshold.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        TARGET_GRAY_MIN = 5
        TARGET_GRAY_MAX = 65
        # Find the biggest contour (if detected)
        if (len(contours) > 0):
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c) # determine moment - weighted average of intensities
            if int(M['m00']) != 0:
                cx = int(M['m10']/M['m00']) # find x component of centroid location
                cy = int(M['m01']/M['m00']) # find y component of centroid location
                cx_int = int(cx)
                cy_int = int(cy)
                roi_size = 5  # Define the ROI size
                x_start = max(0, cx_int - roi_size) #left of the line
                x_end = min(gray.shape[1], cx_int + roi_size) #right of the line
                y_start = max(0, cy_int - roi_size)
                y_end = min(gray.shape[0], cy_int + roi_size)
                # Extract the ROI from the grayscale image
                roi_gray = gray[y_start:y_end, x_start:x_end]
                # Calculate the average grayscale value within the ROI
                gray_value = np.mean(roi_gray)
                print(f"Average Grayscale Value around centroid: {gray_value}")
            else:
                print("Centroid calculation error, looping to acquire new values")
                continue
            cv2.line(crop_img,(cx,0),(cx,720),(255,0,0),1) # display vertical line at x value of centroid
            cv2.line(crop_img,(0,cy),(1280,cy),(255,0,0),1) # display horizontal line at y value of centroid
            cv2.drawContours(crop_img, contours, -1, (0,255,0), 2) # display green lines for all contours
            # determine location of centroid in x direction and adjust steering recommendation
            if (TARGET_GRAY_MIN <= gray_value <= TARGET_GRAY_MAX):
                if cx >= 120:
                    print("Turn Left!")
                if cx < 120 and cx > 50:
                    print("On Track!")
                    #error = pid(cx)
                    #motor_run(error)
                if cx <= 50:
                    print("Turn Right")
                    #error = pid(cx)
                    #motor_run(error)
            pid_output = pid_controller.update(cx)
            motor_run(pid_output)
            #time.sleep(0.1)
        else:
            print("I don't see the line")
            pwm_motorR1.ChangeDutyCycle(0)
            pwm_motorR2.ChangeDutyCycle(20)
            pwm_motorL1.ChangeDutyCycle(0)
            pwm_motorL2.ChangeDutyCycle(20)
            #time.sleep(.1)
        #Process motor control after ignoring the first two readings
        # Adjust sleep time as needed
        # Display the resulting frame
        cv2.imshow('frame',crop_img)
        # Show image for 1 ms then continue to next image
        cv2.waitKey(1)
except KeyboardInterrupt:
    print('All done')
