#ME35 P3, LINE FOLLOWER
import RPi.GPIO as GPIO
import time
#left sensor, slightly buggy with blue
s2 = 36
s3 = 38
sig = 40
#s22 = 33
#s32 = 35
#sig2 = 37 #labeled "out" on your board
cycles = 10
# Setup GPIO and pins
GPIO.setmode(GPIO.BOARD)
GPIO.setup(s2, GPIO.OUT)
GPIO.setup(s3, GPIO.OUT)
GPIO.setup(sig, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
# Set up PWM for forward rotation
OUT1 = 12 #left
OUT2 = 11 #left
OUT3 = 13 #right
OUT4 = 15 #right
GPIO.setup(OUT1, GPIO.OUT)
GPIO.setup(OUT2, GPIO.OUT)
GPIO.setup(OUT3, GPIO.OUT)
GPIO.setup(OUT4, GPIO.OUT)
pwm_motorL1 = GPIO.PWM(OUT1, 50) # 50 Hz frequency
pwm_motorR1 = GPIO.PWM(OUT3, 50)
# for speed control
max_PWM = 100 # this is the maximum duty cycle possible for the dc motor with RasPi, which enables maximum speed (0.0 <= dc <= 100.0)
min_PWM = 20
# Start PWM at 0
pwm_motorL1.start(0)
pwm_motorR1.start(0)
class PIDController:
    def __init__(self, kp, kd, setpoint):
        self.kp = kp
        self.kd = kd  # New derivative term
        self.setpoint = setpoint
        self.prev_error = 0
    def update(self, curr_color):
        error = self.setpoint - curr_color
        #print("blue value:", curr_color)
        # Calculate the proportional term
        p_term = self.kp * error
        # Calculate the derivative term
        d_term = self.kd * (error - self.prev_error)
        # Calculate the total output
        output = p_term + d_term
        print("output:", output)
        self.prev_error = error
        return output
class ColorSensor:
    def __init__(self, s2, s3, out):
        self.S2 = s2
        self.S3 = s3
        self.OUT = out
        GPIO.setup(self.S2, GPIO.OUT)
        GPIO.setup(self.S3, GPIO.OUT)
        GPIO.setup(self.OUT, GPIO.IN)
    def DetectColor(self, s2, s3, sig):
        # Detect red values
        GPIO.output(s2, GPIO.LOW)
        GPIO.output(s3, GPIO.LOW)
        time.sleep(1)
        start_time = time.time()
        for count in range(cycles):
            GPIO.wait_for_edge(sig, GPIO.FALLING)
        duration = time.time() - start_time
        red = cycles / duration
        print("red value - ", red)
        return red
def motor_run(output):
    if 0 <= (25 - output) <= 100 and 0 <= (25 + output) <= 100:
        pwm_motorL1.ChangeDutyCycle(20 - output)
        print("L speed - ", 25 - output)
        pwm_motorR1.ChangeDutyCycle(25 + output)
        print("R speed - ", 25 + output)
        time.sleep(.2)
        pwm_motorL1.ChangeDutyCycle(0)
        pwm_motorR1.ChangeDutyCycle(0)
        time.sleep(.1)
    else:
        # Duty cycle is out of range
        print("Duty cycle out of range.")
        GPIO.cleanup()
        # Continue with normal operation
        time.sleep(0.001)
# Initialize the correction timer
#correction_timer = time.time()
# ... (rest of your code remains unchanged)
color_sensor = ColorSensor(s2, s3, sig)
pid_controller_red = PIDController(kp=.0015, kd = 0.0001, setpoint = 22000) #blue values ki=0.05, kd=1, setpoint = blue
try:
    read_counter = 0  # Initialize a counter to keep track of readings
    while True:
        # Read RGB values from sensors
        red = color_sensor.DetectColor(s2, s3, sig)
        # Ignore the first two readings
        if read_counter < 2:
            read_counter += 1
            continue
        # Update setpoint for blue PID controller
        pid_controller_red.setpoint = 22000
        # Calculate PID outputs for each color
        pid_output = pid_controller_red.update(red)
        # Process motor control after ignoring the first two readings
        motor_run(pid_output)
        time.sleep(0.1)  # Adjust sleep time as needed
except KeyboardInterrupt:
    GPIO.cleanup()










