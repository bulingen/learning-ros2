#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time


# servoPIN = 17
# GPIO.setmode(GPIO.BCM)
# GPIO.setup(servoPIN, GPIO.OUT)

# p = GPIO.PWM(servoPIN, 50) # GPIO 17 for PWM with 50Hz

class ServoNode(Node):
    def __init__(self):
        super().__init__("servo")
        print("tjena")
        print(GPIO)
        # self.timer_ = self.create_timer(2, self.do_stuff)

        self.do_stuff2()

    def do_stuff(self):
        print('korv')

    def do_stuff2(self):

        servoPIN = 17
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(servoPIN, GPIO.OUT)

        p = GPIO.PWM(servoPIN, 50) # GPIO 17 for PWM with 50Hz
        p.start(2.5) # Initialization
        try:
            while True:
                p.ChangeDutyCycle(5)
                time.sleep(0.5)
                p.ChangeDutyCycle(7.5)
                time.sleep(0.5)
                p.ChangeDutyCycle(10)
                time.sleep(0.5)
                p.ChangeDutyCycle(12.5)
                time.sleep(0.5)
                p.ChangeDutyCycle(10)
                time.sleep(0.5)
                p.ChangeDutyCycle(7.5)
                time.sleep(0.5)
                p.ChangeDutyCycle(5)
                time.sleep(0.5)
                p.ChangeDutyCycle(2.5)
                time.sleep(0.5)
        except KeyboardInterrupt:
            p.stop()
            GPIO.cleanup()



def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
