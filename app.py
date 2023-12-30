# app.py
# ME30 P5 2023
from flask import Flask, render_template, request
app = Flask(__name__)
import RPi.GPIO as GPIO
import time


# Set up GPIO pins
GPIO.setmode(GPIO.BOARD)
# Define GPIO pins connected to the H-bridge inputs
motorL = 11 #blue
motorR = 16 #red
motorLB = 13 #blue
motorRB = 15


# Set up PWM for forward rotation
GPIO.setup(motorL, GPIO.OUT)
GPIO.setup(motorR, GPIO.OUT)
pwm_motorL = GPIO.PWM(motorL, 500) # 500 Hz frequency
pwm_motorR = GPIO.PWM(motorR, 500)
# Start PWM at 0
pwm_motorL.start(0)
pwm_motorR.start(0)


# Set up PWM for backwards rotation
GPIO.setup(motorLB, GPIO.OUT)
GPIO.setup(motorRB, GPIO.OUT)
pwm_motorLB = GPIO.PWM(motorLB, 500)
pwm_motorRB = GPIO.PWM(motorRB, 500)
pwm_motorLB.start(0)
pwm_motorRB.start(0)


#define routes for webpage
@app.route('/')
def index():
  return render_template('index.html')


@app.route('/control/left')
def controlleft():
  pwm_motorL.ChangeDutyCycle(0)
  pwm_motorR.ChangeDutyCycle(0)
  time.sleep(0.05)
  # set to 20% duty cycle: slow
  pwm_motorLB.ChangeDutyCycle(20)
  pwm_motorRB.ChangeDutyCycle(0)
  print('Left command received')
  time.sleep(0.05) # Run for 0.05 seconds
  # Stop the motors
  pwm_motorL.ChangeDutyCycle(0)
  pwm_motorR.ChangeDutyCycle(0)
  time.sleep(0.1)
  pwm_motorLB.ChangeDutyCycle(0)
  pwm_motorRB.ChangeDutyCycle(0)
  print('Left command complete')
  return 'left done'


@app.route('/control/right')
def controlright():
  pwm_motorL.ChangeDutyCycle(0)
  pwm_motorR.ChangeDutyCycle(0)
  time.sleep(0.05)
  pwm_motorLB.ChangeDutyCycle(0)
  pwm_motorRB.ChangeDutyCycle(20)
  print('Right command received')
  time.sleep(0.05)
  pwm_motorL.ChangeDutyCycle(0)
  pwm_motorR.ChangeDutyCycle(0)
  time.sleep(0.1)
  pwm_motorLB.ChangeDutyCycle(0)
  pwm_motorRB.ChangeDutyCycle(0)
  print('Right command complete')
  return 'right done'


@app.route('/control/forward')
def controlfor():
  # set to 80% duty cycle: fast
  pwm_motorL.ChangeDutyCycle(80)
  pwm_motorR.ChangeDutyCycle(80)
  time.sleep(0.1)
  pwm_motorLB.ChangeDutyCycle(0)
  pwm_motorRB.ChangeDutyCycle(0)
  print('Forward command received')
  return 'forward on'


@app.route('/control/stop')
def controlstop():
  pwm_motorL.ChangeDutyCycle(0)
  pwm_motorR.ChangeDutyCycle(0)
  pwm_motorLB.ChangeDutyCycle(0)
  pwm_motorRB.ChangeDutyCycle(0t)
  print('stopped')
  return 'stopped'


if __name__ == '__main__':
  app.run(host='0.0.0.0', port=80, debug=True)
