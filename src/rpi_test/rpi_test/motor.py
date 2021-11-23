#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import RPi.GPIO as GPIO


FORWARD_LEVEL = GPIO.HIGH
BACKWARD_LEVEL = GPIO.LOW
PWM_FREQUENCY = 1000
DEFAULT_PWM_PIN = 17
DEFAULT_DIR_PIN = 27
CONTROL_TOPIC = 'motor_control'

class MotorNode(Node):
    def __init__(self):
        super().__init__("motor_node")
        self.declare_parameter("prop_motor_pwm_pin", DEFAULT_PWM_PIN)
        self.declare_parameter("prop_motor_dir_pin", DEFAULT_DIR_PIN)
        self.pwm_pin = self.get_parameter("prop_motor_pwm_pin").value
        self.direction_pin = self.get_parameter("prop_motor_dir_pin").value

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.direction_pin, GPIO.OUT)
        GPIO.setup(self.pwm_pin, GPIO.OUT)

        self.pwm_output = GPIO.PWM(self.pwm_pin, PWM_FREQUENCY)
        self.control_subscriber = self.create_subscription(
            Int32, CONTROL_TOPIC, self.on_pwm_control_message, 10)

        self.get_logger().info("Motor node has started with pins " + str(self.pwm_pin) + " and " + str(self.direction_pin))

    def forward(self, speed=100):
        GPIO.output(self.direction_pin, FORWARD_LEVEL)
        self.pwm_output.start(speed)
    
    
    def backward(self, speed=100):
        GPIO.output(self.direction_pin, BACKWARD_LEVEL)
        self.pwm_output.start(speed)

    # Do we need this?
    def stop_pwm(self):
        self.pwm_output.stop()


    def on_pwm_control_message(self, msg):
        val = abs(msg.data)
        is_forward = msg.data > 0
        
        if val > 100:
            val = 100

        if val > 0:
            if is_forward:
                self.get_logger().info("Going forward, speed " + str(val))
                self.forward(speed=val)
            else:
                self.get_logger().info("Going backward, speed " + str(val))
                self.backward(speed=val)
        else:
                self.get_logger().info("Stopping..")
                self.stop_pwm()



def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    rclpy.spin(node)
    rclpy.shutdown()
    GPIO.cleanup() # Should this be here? What does it do?


if __name__ == "__main__":
    main()
