import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
#GPIO.cleanup()  # Add this line to release GPIO channels

# Define the GPIO pins for the L298N motor driver
OUT1 = 12
OUT2 = 11
OUT3 = 13
OUT4 = 15
stop_channel = 31
channel2 = 33

# Set the GPIO pins as output
GPIO.setup(OUT1, GPIO.OUT)
GPIO.setup(OUT2, GPIO.OUT)
GPIO.setup(OUT3, GPIO.OUT)
GPIO.setup(OUT4, GPIO.OUT)

GPIO.output(OUT1,GPIO.LOW)
GPIO.output(OUT2,GPIO.LOW)
GPIO.output(OUT3,GPIO.LOW)
GPIO.output(OUT4,GPIO.LOW)

GPIO.setup(stop_channel, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(channel2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


GPIO.add_event_detect(stop_channel, GPIO.RISING)  # add rising edge detection on a channel
GPIO.add_event_detect(channel2, GPIO.RISING)  # add rising edge detection on a channel
    

num_steps = 100000000000
step_delay = 0.03

def calibration_going_up ():
     while True:   
        current_step = 0
        print ('in going up')
        # Move up
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
            if GPIO.event_detected(channel2):  #assign channel
                time.sleep (0.001)
                print ('switch pressed')
                if GPIO.input(channel2) == GPIO.HIGH:
                    print('AT 10 CM')
                    going_down()

            # print ('checked wait')
            current_step = current_step + 1
        GPIO.cleanup()
        break    
     
def going_down ():
     while True:   
        current_step = 0
        print ('in going down')
        
        #going down
        for x in range(num_steps):
            if current_step == 0:
                GPIO.output(OUT1,GPIO.HIGH)
                GPIO.output(OUT2,GPIO.LOW)
                GPIO.output(OUT3,GPIO.HIGH)
                GPIO.output(OUT4,GPIO.LOW)
                time.sleep(step_delay)
                #print("step 0")
            elif current_step == 1:
                GPIO.output(OUT1,GPIO.LOW)
                GPIO.output(OUT2,GPIO.HIGH)
                GPIO.output(OUT3,GPIO.HIGH)
                GPIO.output(OUT4,GPIO.LOW)
                time.sleep(step_delay)
                #print("step 1")
            elif current_step == 2:
                GPIO.output(OUT1,GPIO.LOW)
                GPIO.output(OUT2,GPIO.HIGH)
                GPIO.output(OUT3,GPIO.LOW)
                GPIO.output(OUT4,GPIO.HIGH)
                time.sleep(step_delay)
            elif current_step == 3:
                GPIO.output(OUT1,GPIO.HIGH)
                GPIO.output(OUT2,GPIO.LOW)
                GPIO.output(OUT3,GPIO.LOW)
                GPIO.output(OUT4,GPIO.HIGH)
                time.sleep(step_delay)

            if current_step == 3:
                current_step = 0
                continue 
            if GPIO.event_detected(stop_channel):  #assign channel
                time.sleep (0.1)
                print ('stop switch pressed')
                if GPIO.input(stop_channel) == GPIO.HIGH:
                    current_step = num_steps
                    print('Switch stabilized. Performing action.')

            #print ('checked wait')
            current_step = current_step + 1
            #GPIO.wait_for_edge(channel, GPIO.RISING)
        GPIO.cleanup()
        break    

#GPIO.add_event_detect(channel, GPIO.RISING, callback=my_callback)  # add rising edge detection on a channel

try:

    while True:
        print ('in while true')
        #GPIO.wait_for_edge(channel, GPIO.RISING)
        calibration_going_up()

        #GPIO.add_event_detect(channel, GPIO.RISING)  # add rising edge detection on a channel
        


   
except KeyboardInterrupt:
    GPIO.cleanup()

# Clean up GPIO on exit (if not interrupted)
finally:
    GPIO.cleanup()
