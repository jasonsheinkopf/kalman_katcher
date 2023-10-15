# sudo pigpiod
#650 min 2350 max

import RPi.GPIO as GPIO
import pigpio
import time
import math
import threading

# pin 13
servo = 13

pwm = pigpio.pi()
pwm.set_mode(servo, pigpio.OUTPUT)

pwm.set_PWM_frequency( servo, 50 )

min_pos = 650
max_pos = 2350

goal_position = min_pos
# current_position = min_pos

def go_to_position(position):

    print(f'Going to: {position}')
    pwm.set_servo_pulsewidth( servo, position ) ;
    time.sleep(.3)

    # turning off servo
    # pwm.set_PWM_dutycycle( servo, 0 )
    # pwm.set_PWM_frequency( servo, 0 )

go_to_position(min_pos)
# go_to_position(2000)

exit_event = threading.Event()

def servo_thread_fn():
    while not exit_event.is_set():
        print(f'Going to: {goal_position}')
        go_to_position(goal_position)

frame_num = 0

# create thread
servo_thread = threading.Thread(target=servo_thread_fn)
# start thread
servo_thread.start()

try:
    while goal_position < max_pos:
        # goal_position = int(input("Enter Angle: "))
        # goal_position = int(850 * math.sin(frame_num * 2 * math.pi / 100) + 1500)
        # frame_num += 1

        # if goal_position != current_position:
        #     go_to_position(goal_position)
        #     current_position = goal_position
        # go_to_position(goal_position)

        goal_position += 10
        time.sleep(.2)

except KeyboardInterrupt:
    print("Stopping thread and exiting program")
    exit_event.set()

# wait for thread to finish
servo_thread.join()
