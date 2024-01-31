import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
#GPIO.cleanup()  # Add this line to release GPIO channels

# Define the GPIO pins for the L298N motor driver
OUT1 = 12
OUT2 = 11
OUT3 = 13
OUT4 = 15
channel = 16

# Set the GPIO pins as output
GPIO.setup(OUT1, GPIO.OUT)
GPIO.setup(OUT2, GPIO.OUT)
GPIO.setup(OUT3, GPIO.OUT)
GPIO.setup(OUT4, GPIO.OUT)

GPIO.output(OUT1,GPIO.LOW)
GPIO.output(OUT2,GPIO.LOW)
GPIO.output(OUT3,GPIO.LOW)
GPIO.output(OUT4,GPIO.LOW)

GPIO.setup(channel, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

GPIO.add_event_detect(channel, GPIO.RISING)  # add rising edge detection on a channel
        

num_steps = 200
step_delay = 0.03

def counterclockwise ():
     while True:   
        current_step = 0
        print ('in counter clockwise')
        for x in range(num_steps):
            if current_step == 3:
                GPIO.output(OUT1,GPIO.HIGH)
                GPIO.output(OUT2,GPIO.LOW)
                GPIO.output(OUT3,GPIO.HIGH)
                GPIO.output(OUT4,GPIO.LOW)
                time.sleep(step_delay)
                #print("step 0")
            elif current_step == 2:
                GPIO.output(OUT1,GPIO.LOW)
                GPIO.output(OUT2,GPIO.HIGH)
                GPIO.output(OUT3,GPIO.HIGH)
                GPIO.output(OUT4,GPIO.LOW)
                time.sleep(step_delay)
                #print("step 1")
            elif current_step == 1:
                GPIO.output(OUT1,GPIO.LOW)
                GPIO.output(OUT2,GPIO.HIGH)
                GPIO.output(OUT3,GPIO.LOW)
                GPIO.output(OUT4,GPIO.HIGH)
                time.sleep(step_delay)
            elif current_step == 0:
                GPIO.output(OUT1,GPIO.HIGH)
                GPIO.output(OUT2,GPIO.LOW)
                GPIO.output(OUT3,GPIO.LOW)
                GPIO.output(OUT4,GPIO.HIGH)
                time.sleep(step_delay)
            if current_step == 0:
                current_step = 3
                continue 
            current_step = current_step - 1
        GPIO.cleanup()
        break    
     
def clockwise ():
     while True:   
        current_step = 0
        print ('in clockwise')
        
        for x in range(num_steps):
            if current_step == 0:
                GPIO.output(OUT1,GPIO.HIGH)
                GPIO.output(OUT2,GPIO.LOW)
                GPIO.output(OUT3,GPIO.LOW)
                GPIO.output(OUT4,GPIO.HIGH)
                time.sleep(step_delay)
                #print("step 0")
            elif current_step == 1:
                GPIO.output(OUT1,GPIO.LOW)
                GPIO.output(OUT2,GPIO.HIGH)
                GPIO.output(OUT3,GPIO.LOW)
                GPIO.output(OUT4,GPIO.HIGH)
                time.sleep(step_delay)
                #print("step 1")
            elif current_step == 2:
                GPIO.output(OUT1,GPIO.LOW)
                GPIO.output(OUT2,GPIO.HIGH)
                GPIO.output(OUT3,GPIO.HIGH)
                GPIO.output(OUT4,GPIO.LOW)
                time.sleep(step_delay)

            elif current_step == 3:
                GPIO.output(OUT1,GPIO.HIGH)
                GPIO.output(OUT2,GPIO.LOW)
                GPIO.output(OUT3,GPIO.HIGH)
                GPIO.output(OUT4,GPIO.LOW)
                time.sleep(step_delay)

            if current_step == 3:
                current_step = 0
                continue 
            current_step = current_step + 1
            #GPIO.wait_for_edge(channel, GPIO.RISING)
            if GPIO.event_detected(channel):  #assign channel
                my_callback(channel)
                print ('switch pressed')

            print ('checked wait')
        GPIO.cleanup()
        break    


def my_callback(channel):
    print('This is a edge event callback function!')
    time.sleep(0.5)
    counterclockwise()

#GPIO.add_event_detect(channel, GPIO.RISING, callback=my_callback)  # add rising edge detection on a channel

try:

    while True:
        print ('in while true')
        #GPIO.wait_for_edge(channel, GPIO.RISING)
        clockwise()

        #GPIO.add_event_detect(channel, GPIO.RISING)  # add rising edge detection on a channel
        


   
except KeyboardInterrupt:
    GPIO.cleanup()

# Clean up GPIO on exit (if not interrupted)
finally:
    GPIO.cleanup()
