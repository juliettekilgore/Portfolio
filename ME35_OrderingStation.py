import RPi.GPIO as GPIO
import time
from keras.models import load_model
import cv2
import numpy as np
from picamera2 import Picamera2
from libcamera import controls
import requests

camera_button = 18
small_button = 17
large_button = 25
GPIO.setmode(GPIO.BCM)
GPIO.setup(camera_button, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(small_button, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(large_button, GPIO.IN, pull_up_down=GPIO.PUD_UP)

model = load_model("keras_model.h5", compile=False)
class_names = open("labels.txt", "r").readlines()

camera = Picamera2()
camera.set_controls({"AfMode": controls.AfModeEnum.Continuous})
camera.resolution = (224, 224)
camera.start()
time.sleep(1)

i=1

ACCESS_TOKEN = 'patnz4q3YD2b6QE5b.c888c6f15cb3bbb30c73cb0bcd9b186c09f7459d2e0569c8f80a34dd45a8651b'
HEADERS = {'Authorization': 'Bearer ' + ACCESS_TOKEN}

def PublishStencil(stencil, i ):
    design = stencil + str(i)
    customer_info = {"fields": {"Design": design}}
    update_url = 'https://api.airtable.com/v0/appZXeS3vQKy6x41E/Drive/recPm6ye2Mu5fELGP'
    response = requests.patch(update_url, json=customer_info, headers=HEADERS)

def PublishCoffee(size):
    customer_info = {"fields": {"Design": size}}
    update_url = 'https://api.airtable.com/v0/appZXeS3vQKy6x41E/Drive/recY2LOdyKyHehK5Y'
    response = requests.patch(update_url, json=customer_info, headers=HEADERS)

def button_callback(channel):
    global i   
    selected_size = None  

    print(f"Button {channel} pressed!")

    # Check which button was pressed
    if channel == small_button:
        selected_size = "Small"
        print ('small')
        PublishCoffee(selected_size)  # Update status to 1
    elif channel == large_button:
        selected_size = "Large"
        print('large')
        PublishCoffee(selected_size)  # Update status to 1

    if channel == camera_button:
        
        print('Camera button pushed!')

        # get web camera's image.
        image = camera.capture_array("main")

        # Convert image to numpy array & reshape
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = cv2.resize(image, (224, 224), interpolation=cv2.INTER_AREA)
        image = np.asarray(image, dtype=np.float32).reshape(1, 224, 224, 3)

        # normalize image array
        image = (image / 127.5) - 1

        # Predict w/ model
        prediction = model.predict(image)
        index = np.argmax(prediction)
        class_name = class_names[index]
        confidence_score = prediction[0][index]

        print("Class:", class_name[:], end="")
        print("Confidence Score:", str(np.round(confidence_score * 100))[:-2], "%")

        PublishStencil(class_name, i)

        # Reset 
        selected_size = None

        i = i+1

# event detection for buttons
GPIO.add_event_detect(camera_button, GPIO.FALLING, callback=button_callback, bouncetime=300)
GPIO.add_event_detect(small_button, GPIO.FALLING, callback=button_callback, bouncetime=300)
GPIO.add_event_detect(large_button, GPIO.FALLING, callback=button_callback, bouncetime=300)

try:
    while True:
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nExiting program")
    GPIO.cleanup()
    camera.stop()
