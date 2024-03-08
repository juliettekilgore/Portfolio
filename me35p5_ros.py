import requests
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  

CMD_VEL_TOPIC = '/cmd_vel'

URL = 'https://api.airtable.com/v0/apppLWB4dSvsOwlTL/Table1'
ACCESS_TOKEN = 'patV99gXKs9u1mdlM.a5c5af5886884c7772fb6705616ddc29294a8c133d0830871a3914be3edda780'
HEADERS = {'Authorization': 'Bearer ' + ACCESS_TOKEN}

class RobotController(Node):
   def __init__(self):
       super().__init__('robot_controller')
       self.publisher_ = self.create_publisher(Twist, CMD_VEL_TOPIC, 1)
       timer_period = 0.001
       self.timer = self.create_timer(timer_period, self.timer_callback)

   def fetch_commands_from_airtable(self):
       response = requests.get(url=URL, headers=HEADERS)
       print(response.json())
       data = response.json()
       return data['records']

   def execute_command(self, command, value, status):
       twist = Twist()
       if command == 'Forward':
           twist.linear.x = float(value) 
       elif command == 'Backwards':
           twist.linear.x = -float(value)
           print('in backwards statment')
       elif command == 'Left':
           twist.angular.z = float(value)
       elif command == 'Right':
           twist.angular.z = -float(value)
       self.publisher_.publish(twist)
       self.get_logger().info(f'Executing {command} with value {value}')

   def timer_callback(self):
       commands = self.fetch_commands_from_airtable()
       for command_record in commands:
           fields = command_record.get('fields', {})
           print(fields)
           command = fields.get('calls')
           value = fields.get('Xdirection')
           status = fields.get('On/Off')

           if (command and value is not None) and (status == '1'):
               self.execute_command(command, value, status)
           else:
               print("Command or value missing in record:", command_record)

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    try:
        while rclpy.ok():  # Run the loop as long as ROS is okay
            rclpy.spin_once(robot_controller, timeout_sec=1.0)
    except KeyboardInterrupt:
        print('Interrupted by keyboard')
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
   main()
