# Camera, distance sensor imports
import RPi.GPIO as GPIO
from picamera2 import Picamera2
import cv2
from libcamera import controls
import time
import numpy as np
# Keras imports
from keras.models import load_model  # TensorFlow is required for Keras to work
# ROS imports
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from irobot_create_msgs.action import RotateAngle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
TRIGGER_H = 32
ECHO_H = 36
TRIGGER_L = 40
ECHO_L = 38
GPIO.setmode(GPIO.BOARD)
GPIO.setup(TRIGGER_H, GPIO.OUT)
GPIO.setup(ECHO_H, GPIO.IN)
GPIO.setup(TRIGGER_L, GPIO.OUT)
GPIO.setup(ECHO_L, GPIO.IN)
np.set_printoptions(suppress=True)
# Load the model
model = load_model("keras_model.h5", compile=False)
# Load the labels
class_names = open("labels.txt", "r").readlines()
# Initialize the camera
camera = Picamera2()
# Configure the picamera
camera.set_controls({"AfMode": controls.AfModeEnum.Continuous})  # Sets auto focus mode
camera.start()
time.sleep(1)
img_name = 'img.jpg'
def measure_distance(trigger, echo):
    GPIO.output(trigger, True)
    time.sleep(0.00001)
    GPIO.output(trigger, False)
    start_time = time.time()
    stop_time = time.time()
    while GPIO.input(echo) == 0:
        start_time = time.time()
    while GPIO.input(echo) == 1:
        stop_time = time.time()
    time_elapsed = stop_time - start_time
    distance_cm = round((time_elapsed * 34300) / 2, 2)
    time.sleep(0.1)
    return distance_cm
def measure_both():
    dH = measure_distance(TRIGGER_H, ECHO_H)
    dL = measure_distance(TRIGGER_L, ECHO_L)
    return min(dH, dL)
def find_object():
    camera.capture_file(img_name)
    image = cv2.imread(img_name)
    image = cv2.resize(image, (224, 224), interpolation=cv2.INTER_AREA)
    image_array = np.asarray(image, dtype=np.float32).reshape(1, 224, 224, 3)
    image_normalized = (image_array / 127.5) - 1
    prediction = model.predict(image_normalized)
    index = np.argmax(prediction)
    class_name = class_names[index].strip()
    confidence_score = prediction[0][index]
    print(f"Class: {class_name}, Confidence Score: {confidence_score * 100:.2f}%")
    return class_name, confidence_score
def turn_direction(obj, conf_score):
    dB  = -1
    dC  = -1
    dE  = 1
    dK  = -1
    dMa = 1
    dMu = 1
    dV  = -1
    xB  = 0.7
    xC  = 0.75
    xE  = 0.92
    xK  = 0.75
    xMa = 0.75
    xMu = 0.75
    xV  = 0.9
    deg90 = np.pi/2
    if obj == "6 Bear" and conf_score > xB:
        return dB * deg90
    if obj == "1 Cube" and conf_score > xC:
        return dC * deg90
    if obj == "5 Elephant" and conf_score > xE:
        return dE* deg90
    if obj == "4 Kiwi" and conf_score > xK:
        return dK * deg90
    if obj == "0 Mario" and conf_score > xMa:
        return dMa * deg90
    if obj == "3 Mug" and conf_score > xMu:
        return dMu * deg90
    if obj == "2 Darth" and conf_score > xV:
        return dV * deg90
    else:
        print('no object')
        return 0
# Drive functions
class MovePublisher(Node):
    def __init__(self):
        super().__init__('moving')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rotate_action = ActionClient(self, RotateAngle, 'rotate_angle', callback_group=ReentrantCallbackGroup())
        self.timer = self.create_timer(0.1, self.timer_callback, callback_group=ReentrantCallbackGroup())
    def timer_callback(self):
        dist = measure_both()
        print(dist)
        if dist > 20:
            msg = Twist()
            msg.linear.x = 0.1
            msg.angular.z = 0.0
            self.publisher.publish(msg)
            self.get_logger().info(f'Publishing: "{msg.linear.x}"')
        else:
            thingy, conf = find_object()
            direction = turn_direction(thingy, conf)
            self.rotate_robot(direction)
    def rotate_robot(self, direction):
        goal = RotateAngle.Goal()
        goal.angle = float(direction)  # Rotate approximately 90 degrees
        goal.max_rotation_speed = float(0.5)
        self.rotate_action.wait_for_server()
        self.rotate_action.send_goal_async(goal)
        self.get_logger().info('Rotating...')
# Main function
def main(args=None):
    rclpy.init(args=args)
    moving = MovePublisher()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(moving, executor=executor)
    except KeyboardInterrupt:
        print('\nCaught Keyboard Interrupt')
    finally:
        # Cleanup
        moving.destroy_node()
        rclpy.shutdown()
        GPIO.cleanup()
        print('Shutting down')
if __name__ == '__main__':
    main()
