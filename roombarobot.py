import time
import traitlets
from traitlets.config.configurable import SingletonConfigurable
#from Adafruit_MotorHAT import Adafruit_MotorHAT
from .roombamotor import Motor
from jetbot import create
#ROOMBA_PORT = "COM13"

# May need to use this command for permission issues in Ubuntu:
# Add user to dialout group:
# sudo gpasswd --add ${USER} dialout
# Change permissions of ttyUSB0 resource:
# sudo chmod 666 /dev/ttyUSB0
ROOMBA_PORT = "/dev/ttyUSB0"

class Robot(SingletonConfigurable):
    
    left_motor = traitlets.Instance(Motor)
    right_motor = traitlets.Instance(Motor)

    # config
    i2c_bus = traitlets.Integer(default_value=1).tag(config=True)
    left_motor_channel = traitlets.Integer(default_value=1).tag(config=True)
    left_motor_alpha = traitlets.Float(default_value=1.0).tag(config=True)
    right_motor_channel = traitlets.Integer(default_value=2).tag(config=True)
    right_motor_alpha = traitlets.Float(default_value=1.0).tag(config=True)
    
    def __init__(self, *args, **kwargs):
        super(Robot, self).__init__(*args, **kwargs)
        self.motor_driver = create.Create(ROOMBA_PORT, BAUD_RATE=115200)
        #self.motor_driver = Adafruit_MotorHAT(i2c_bus=self.i2c_bus)
        self.left_motor = Motor(self.motor_driver, channel=self.left_motor_channel, alpha=self.left_motor_alpha)
        self.right_motor = Motor(self.motor_driver, channel=self.right_motor_channel, alpha=self.right_motor_alpha)
        
    def set_motors(self, left_speed, right_speed):
        self.left_motor.value = left_speed
        self.right_motor.value = right_speed
        
    def forward(self, speed=1.0, duration=None):
        self.left_motor.value = speed
        self.right_motor.value = speed

    def backward(self, speed=1.0):
        self.left_motor.value = -speed
        self.right_motor.value = -speed

    def left(self, speed=1.0):
        self.left_motor.value = -speed
        self.right_motor.value = speed

    def right(self, speed=1.0):
        self.left_motor.value = speed
        self.right_motor.value = -speed

    def stop(self):
        self.left_motor.value = 0
        self.right_motor.value = 0


# Example Usage:
# from roombarobot import Robot
# robot = Robot()
# robot.left(5)
# robot.left(0)
# robot.right(5)
# robot.right(0)
# robot.forward(5)
# robot.backward(5)
# robot.stop()
#
