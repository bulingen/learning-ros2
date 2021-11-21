import RPi.GPIO as GPIO
from gpiozero import Servo, Motor
import time
import serial

from gpiozero.pins.pigpio import PiGPIOFactory

# ==== motor
FORWARD_LEVEL = GPIO.HIGH
BACKWARD_LEVEL = GPIO.LOW
PWM_FREQUENCY = 1000
DEFAULT_PWM_PIN = 12
DEFAULT_DIR_PIN = 13

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(DEFAULT_DIR_PIN, GPIO.OUT)
GPIO.setup(DEFAULT_PWM_PIN, GPIO.OUT)
pwm_output = GPIO.PWM(DEFAULT_PWM_PIN, PWM_FREQUENCY)

def forward(speed=100):
    GPIO.output(DEFAULT_DIR_PIN, FORWARD_LEVEL)
    pwm_output.start(speed)


def backward(speed=100):
    GPIO.output(DEFAULT_DIR_PIN, BACKWARD_LEVEL)
    pwm_output.start(speed)

def stop_pwm():
    pwm_output.stop()

# forward(speed=20)
# backward(speed=20)

factory = PiGPIOFactory()

# servo = Servo(17, pin_factory=factory, min_pulse_width=0.75/1000, max_pulse_width=2.25/1000)
servo = Servo(17, pin_factory=factory, min_pulse_width=0.81/1000, max_pulse_width=2.2/1000)

# motor = Motor(12, 12, enable=13, pin_factory=factory)

# print("start in the middle")
# servo.value = 0
# # servo.mid()
# time.sleep(1)
# print("go to min")
# # servo.min()
# servo.value = -1
# time.sleep(5)
# print("go to max")
# servo.value = 1
# # servo.max()
# time.sleep(5)
# print("back to middle")
# servo.value = 0
# # servo.mid()
# time.sleep(4)


def find_string(string, key):
    start = string.find(key)
    if start < 0:
        return None

    start += len(key)

    substring = string[start:]
    return substring


print('welcome')

# koda en liten grej:
# om den inte får instruktion på en viss tid
# stäng av motor och återgå till center
# ifall kontrollen lägger av mitt i ett kommando

ser = serial.Serial('/dev/ttyS0', baudrate=9600)

while True:
    line = ser.readline()
    if line:
        string = line.decode('utf-8')
        motor_stuff = find_string(string, 'M: ')
        if motor_stuff:
            try:
                number = float(motor_stuff)
                if number >= -1 and number <= 1:
                    if number == 0:
                        stop_pwm()
                    elif number > 0:
                        forward(speed=number*100)
                    elif number < 0:
                        backward(speed=abs(number)*100)

            except ValueError as err:
                print('found shit', err)
        
        
        rudder_stuff = find_string(string, 'R: ')
        if rudder_stuff:
            try:
                number = float(rudder_stuff)
                if number >= -1 and number <= 1:
                    servo.value = number

            except ValueError:
                print('found shit')
        # try:
        #     number = float(string)
        #     # print('received', number)
        #     if number >= -1 and number <= 1:
        #         servo.value = number

        # except ValueError:
        #     print('found shit')
    time.sleep(0.01)




