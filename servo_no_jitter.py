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

# ska timeout vara 0 eller None?
# "wait indefinetely", blockar det för writes?
# isåfall, kan man köra olika trådar för det?



# === this works ===

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

    time.sleep(0.01)


# === experimental for service

def listen():
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

    time.sleep(0.01)



# === experimental ===

# def current_time_millis():
#     return round(time.time() * 1000)

# command_timeout_millis = 1000
# has_been_reset = False
# last_command_received_at = current_time_millis()

# def reset_stuff():
#     stop_pwm()
#     servo.value = 0
#     global has_been_reset
#     has_been_reset = True

# ser = serial.Serial('/dev/ttyS0', baudrate=9600)

# def update_timestamp():
#     global last_command_received_at
#     last_command_received_at = current_time_millis()

# def been_a_while_since_last_command():
#     now = current_time_millis()
#     return now - last_command_received_at > command_timeout_millis

# def handle_motor_command(value):
#     try:
#         number = float(value)
#         if number >= -1 and number <= 1:
#             if number == 0:
#                 stop_pwm()
#             elif number > 0:
#                 forward(speed=number*100)
#             elif number < 0:
#                 backward(speed=abs(number)*100)

#             update_timestamp()
#             global has_been_reset
#             has_been_reset = False

#     except ValueError as err:
#         print('unrecognized motor command', err)


# def handle_rudder_command(value):
#     try:
#         number = float(value)
#         if number >= -1 and number <= 1:
#             servo.value = number

#             update_timestamp()
#             global has_been_reset
#             has_been_reset = False

#     except ValueError as err:
#         print('unrecognized rudder command', err)


# while True:
#     line = ser.readline()
#     if line:
#         string = line.decode('utf-8')
#         motor_stuff = find_string(string, 'M: ')
#         if motor_stuff:
#             handle_motor_command(motor_stuff)        
        
#         rudder_stuff = find_string(string, 'R: ')
#         if rudder_stuff:
#             handle_rudder_command(rudder_stuff)

#     if not has_been_reset and been_a_while_since_last_command():
#         reset_stuff()

    
#     # is this needed?
#     time.sleep(0.01)

