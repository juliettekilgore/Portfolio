# multi.py
# ME30 P6
# Juliette Kilgore, Vivian Becker, Ry Baralt
# 12/18/2012


from flask import Flask, Response, request
import threading
import requests
import time


import RPi.GPIO as GPIO


app = Flask(__name__)


#-------------------- VARIABLES TO CHANGE BASED ON OTHER ROBOT -----------------------


# IP addresses
other_robot_ip = "10.243.87.126"
my_ip = "10.243.87.84"


# port
my_port = 5004
partner_port = 5000


start_speed = 60
agreed_delay = 1 # agreed upon delay with other team, also the delay that will send out




#--------------------------------- GLOBAL VARIABLES ----------------------------------


# Flag to control thread termination
terminate_threads = False
# Flag to signal Flask app is ready to exit
flask_ready_to_exit = threading.Event()




#for start delay
our_start_delay = 0.5 # time it takes our motors to start up
delay_response = "" # response from other robot to our delay request (ok or no)
delay_request_received = False
delay_completed = False
rejected_using_our_delay = False # true if our delay request is first, but other robot still says no




#for motor control
target_speed = 0
my_speed = 0
partner_speed = 0
partner_target_speed = 0
speed_increment = 5




# for speed control
max_speed = 500
max_PWM = 100
min_PWM = 35




# for counting thread, left over from testing and now used to output current speed
count = 0


#---------------------------------- THREADING LOCKS ----------------------------------


# Lock for thread synchronization
buttons_lock = threading.Lock()
count_lock = threading.Lock()
speed_lock = threading.Lock()


#---------------------------------------- GPIO ---------------------------------------


# Set GPIO mode
GPIO.setmode(GPIO.BCM)


# Define GPIO pins for buttons
left_button_pin = 3
right_button_pin = 27


# Set up button GPIO pins
GPIO.setup(left_button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(right_button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)




# Define GPIO pins connected to the H-bridge inputs
motorL = 6
motorR = 5


# Set up PWM for forward rotation
GPIO.setup(motorL, GPIO.OUT)
GPIO.setup(motorR, GPIO.OUT)
pwm_motorL = GPIO.PWM(motorL, 500) # 500 Hz frequency
pwm_motorR = GPIO.PWM(motorR, 500)


# Start PWM at 0
pwm_motorL.start(0)
pwm_motorR.start(0)


#------------------------------------ CLEANUP FUNCTION -------------------------------


# Cleanup function for GPIO cleanup
def cleanup():
   print("Cleaning up GPIO...")
   GPIO.cleanup()


#-------------------------------- SPEED CONTROL FUNCTIONS ----------------------------


# Function to convert speed in mm/s to PWM percent
def convert_mms_to_PWM(speed_mms):
   global max_speed, max_PWM, min_PWM


   #convert speed in mm/s to PWM percent
   speedPWM = (speed_mms / max_speed) * (max_PWM - min_PWM) + min_PWM


   return speedPWM




# Function to control motors based on target speed
def set_motors_to_target(speed_mms):
   global pwm_motorL, pwm_motorR


   speedPWM = convert_mms_to_PWM(speed_mms)


   pwm_motorL.ChangeDutyCycle(speedPWM)
   pwm_motorR.ChangeDutyCycle(speedPWM)


#----------------------------- SENDING REQUESTS FUNCTIONS ----------------------------


# Function to send target speed request to the other robot
def send_target_request(speed):
   url = f"http://{other_robot_ip}:{partner_port}/target/{speed}"


   try:
       # send HTTP GET request to the other robot
       response = requests.get(url)


       # capture the plain text response of ok or no
       response_text = response.text


       # print and return the response
       print(response_text)
       return response_text


   except requests.ConnectionError:
       print("Error: Unable to connect to the other robot.")
       return "no"  # Assuming "no" for unsuccessful connection




# Function to send delay request to the other robot
def send_delay_request(delay):
   global delay_response


   url = f"http://{other_robot_ip}:{partner_port}/start/{delay}"


   try:
       # send HTTP GET request to the other robot
       response = requests.get(url)


       # capture the response of ok or no
       delay_response = response.text
       print(delay_response)


       #decide whether to execute our or their delay
       handle_start_request(delay)


       return delay_response
    except requests.ConnectionError:
       print("Error: Unable to connect to the other robot.")
       return "no"  # Assuming "no" for unsuccessful connection
 #----------------------------------- DELAY FUNCTIONS --------------------------------


# Function to handle which /start/<delay> request to use
def handle_start_request(delay):
   global our_start_delay, delay_completed, rejected_using_our_delay
   with buttons_lock:
       #using their delay if we received their delay request,
       # but have not yet received a response to ours
       if delay_request_received and delay_response == "":
           print("using their delay")
           # if their requested delay > time it takes for our motots to start, say ok
           if delay >= our_start_delay:
               print("said ok to their delay")
               return Response("ok", status=200, content_type='text/plain')
           else:
               print("said no to their delay")
               return Response("no", status=400, content_type='text/plain')
        
       # using our delay if we received a response to our delay,
       # and we have not yet received a delay request
       elif not delay_response == "" and not delay_request_received:
           print("using our delay")
           if delay_response == "ok":
               time.sleep(agreed_delay+our_start_delay)
               print(f"our delay used: {agreed_delay+our_start_delay}")


               delay_completed = True
           else:
               rejected_using_our_delay = True


           # start the main function if our request was accepted
           if rejected_using_our_delay == False:
               button_thread.start()


#------------------------------------- BUTTON LOOP -----------------------------------


# Main loop for buttons
def button_loop():
   global terminate_threads, my_speed, partner_speed, max_speed, start_speed
   global left_button_pin, right_button_pin, partner_target_speed
   global speed_increment, our_start_delay, agreed_delay, flask_ready_to_exit


   print("in button loop")


   #set motors to start speed
   my_speed = start_speed
   partner_speed = start_speed
   set_motors_to_target(my_speed)


   print(f"speed changed to {my_speed}")


   while not terminate_threads:
       with buttons_lock:
           # Check if the right button is pressed, meaning we are behind (too slow)
           if GPIO.input(right_button_pin) == GPIO.LOW:
               # If not at max speed, speed up your robot
               with speed_lock:
                   if my_speed < max_speed:
                       print(f"my speed < max speed")
                       print(f"my speed: {my_speed}")


                       my_speed += speed_increment
                       set_motors_to_target(my_speed)


                       print(f"my new speed: {my_speed}")
                   else:
                       # If at max speed, tell the other robot to slow down
                       print(f"my speed at max speed (right button pressed)")
                       print(f"partner speed: {partner_speed}")


                       partner_target_speed = my_speed - speed_increment
                       response = send_target_request(partner_target_speed)


                       if response == "ok":
                           partner_speed = partner_target_speed
                           print(f"partner said ok, new partner speed:{partner_speed}")
                       else:
                           print(f"partner said no, partner sucks (old speed):{partner_speed}")
                           pass
                       time.sleep(0.1)  # Wait and request again if necessary
                          
           # Check if the left button is pressed, meaning we are ahead (too fast)
           elif GPIO.input(left_button_pin) == GPIO.LOW:
               # If not at min speed, slow down our robot
               with speed_lock:
                   if my_speed > 10:
                       print(f"my speed greater than 10")
                       print(f"my speed: {my_speed}")


                       my_speed -= speed_increment
                       set_motors_to_target(my_speed)


                       print(f"my new speed: {my_speed}")
                   else:
                       print(f"at min speed (left button pressed)")
                       print(f"partner speed: {partner_speed}")


                       partner_target_speed = my_speed + speed_increment
                       response = send_target_request(partner_target_speed)


                       if response == "ok":
                           partner_speed = partner_target_speed
                           print(f"partner said ok, new partner speed:{partner_speed}")
                       else:
                           print(f"partner said no, partner sucks (old speed):{partner_speed}")
                           pass
                       time.sleep(0.1)
                      
       # sleep briefly to accept another button press, adjust based on sensitivity of the buttons
       time.sleep(0.1)
    # Signal that the Flask app is ready to exit
   flask_ready_to_exit.wait()


# define the button thread
button_thread = threading.Thread(target=button_loop)




#----------------------------------- SPEED OUTPUT LOOP -------------------------------




# prints our currents speed every five seconds
def count_loop():
   global terminate_threads, count


   while not terminate_threads:
       with count_lock, speed_lock:
           # count increase not necessary, left over from testing
           count += 1
           print(f"Curr speed: {my_speed}")
       time.sleep(5)
  
   with count_lock:
       print("Count loop exiting...")
#--------------------------------- RUNNING BEFORE MAIN -------------------------------




print("this is running first")




#send_delay_request(agreed_delay)


#---------------------------------------- ROUTES -------------------------------------


# Null request
@app.route('/')
def home():
   return "Welcome to the Flask App"




# Browser communication test
@app.route('/test1')
def basic_test():
   print("basic test 1")
   return "basic test triggered!"




# Function to stop the motors via browser/curl
@app.route('/stop')
def stop():
   global my_speed
   my_speed = 0
   set_motors_to_target(my_speed)
   return "stopped!"




#route for delay request
@app.route('/start/<int:delay>', methods=['GET'])
def start_request(delay):
   global delay_request_received, our_start_delay, delay_completed
    delay_request_received = True


   response = handle_start_request(delay)


   # if responded ok, started a new thread to execute the start delay
   if response.data.decode() == "ok":
       #execute the delay (time it takes for our motors to start + requested delay)
       time.sleep(our_start_delay + delay)
       print(f"their delay used: {our_start_delay + delay}")
       #start the button thread
       button_thread.start()
       delay_completed = True


   #return ok or no to other robot
   #this will execute simultaneously with executing the start delay
   return response




#route for speed request from other robot
@app.route('/target/<int:speed>', methods=['GET'])
def target_request(speed):
   global my_speed


   #if the reuested speed is reasonable, change our speed and reply ok
   if max_speed > speed:
       with speed_lock:
           my_speed = speed


       response = "ok"


       set_motors_to_target(my_speed)
       print("changed to requested speed")


       # return our response to the other robot
       return response
  
   else:
       response = "no"
       return response




#---------------------------------------- MAIN ---------------------------------------


def main():
   global terminate_threads
   print("inside main")


   # Send delay request before starting Flask app
   # if our program is the first to run, its delay message will get lost,
   # but then flask will turn on and it can accept the other robot's delay message
   send_delay_request(agreed_delay)


   # Create and start count thread
   count_thread = threading.Thread(target=count_loop)
   count_thread.start()
    #run flask to start accepting requests
   try:
       print("try to run flask in main")
       app.run(debug=True, use_reloader=False, host='0.0.0.0', port=my_port)
     
   except Exception as e:
       # Handle exceptions here
       print(f"Exception: {e}")
  
   #close the threads and cleanup
   finally:
       # Set the terminate flag for threads
       terminate_threads = True


       # join count thread
       count_thread.join()
    
       # Check if the button thread is still running
       if button_thread.is_alive():
           # If the button thread is still running,
           # wait for it to finish or until the Flask app signals ready to exit
           button_thread.join(timeout=20)  # Adjust the timeout as needed


       # Ensure GPIO cleanup is called before exiting
       cleanup()


   print("Exiting main")


if __name__ == '__main__':
   try:
       main()
   except:
       cleanup()
   finally:
       cleanup()


