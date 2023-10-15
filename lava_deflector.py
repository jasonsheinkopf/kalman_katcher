from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
from rpi_ws281x import Adafruit_NeoPixel, Color
from time import sleep
import RPi.GPIO as GPIO
import pigpio
import pickle
import math
# import threading
import sys

# class Target:
#     def __init__(self, id, x, y):
#         self.id = id
#         self.x = x
#         self.y = y
#         self.threshold = 10

#     def _repr_(self):
#         return f'{self.id}: ({self.x}, {self.y})'
    
#     def is_tracking(self, circle_x, circle_y):
#         # subtraction = circle_x - self.x
#         # subtractiony = circle_y - self.y
#         distance = math.sqrt((int(circle_x) - int(self.x))**2 + (int(circle_y) - int(self.y))**2)
#         # print(f'Distance: {distance}')

#         return True if distance < self.threshold else False
    
#     def find_distance(self, circle):
#         return math.sqrt((int(circle[0]) - int(self.x))**2 + (int(circle[0]) - int(self.y))**2)

# Set GPIO numbering mode
# GPIO.setmode(GPIO.BOARD)

# Set pin 38 as an output, and define as servo1 as PWM pin
# servo_pin = 38
# GPIO.setup(servo_pin,GPIO.OUT)
# servo1 = GPIO.PWM(servo_pin, 50)

# pgpio setup
# servo = 13
# pwm = pigpio.pi()
# pwm.set_mode(servo, pigpio.OUTPUT)
# pwm.set_PWM_frequency( servo, 50 )

# initial servo position
# goal_pos = 1000
# min_pos = 650
# max_pos = 2350

# Start PWM running, with value of 0 (pulse off)
# servo1.start(0)

# def servo_dot(servo_box_l, servo_box_r):
#     x1, y1 = servo_box_l
#     x2, y2 = servo_box_r
#     # sample pixels along line
#     line_values = []

#     # create sample points along servo path
#     for t in np.linspace(0, 1, 100):
#         x = int(x1 + t * (x2 - x1))
#         y = int(y1 + t * (y2 - y1))
#         line_values.append(image[y, x])

#     # points brighter than threshold are where the servo is
#     threshold = 210

#     # find indices of bright pixels
#     bright_indices = np.where(np.array(line_values) > threshold)

#     # if there are no bright regions
#     if bright_indices[0].size == 0:
#         print("Servo not in sight")
#     else:
#         # find centerpoint
#         center_index = np.array(bright_indices[0]).mean().astype(int)
#         x_center = int(x1 + np.linspace(0, 1, 100)[center_index] * (x2 - x1))
#         y_center = int(y1 + np.linspace(0, 1, 100)[center_index] * (y2 - y1))

#         # Draw a circle at the center point
#         cv2.circle(image, (x_center, y_center), 15, (0, 0, 255), -1)
#         # print(f'x: {x_center}, y: {y_center}')

# def go_to_position(position):

#     print(f'Going to: {position}')
#     pwm.set_servo_pulsewidth(servo, position) ;
#     time.sleep(.3)


# set exit event to end thread
# exit_event = threading.Event()

# def servo_thread_fn():
#     while not exit_event.is_set():
#         print(f'Going to: {goal_pos}')
#         go_to_position(goal_pos)



# Loop to allow user to set servo angle. Try/finally allows exit
# with execution of servo.stop and GPIO cleanup :)

# try:
#     while True:
#         #Ask user for angle and turn servo to it
#         angle = float(input('Enter angle between 0 & 180: '))
#         servo1.ChangeDutyCycle(2+(angle/18))
#         time.sleep(0.5)
#         servo1.ChangeDutyCycle(0)

# finally:
#     #Clean things up at the end
#     servo1.stop()
#     GPIO.cleanup()

# # LED config
# LED_COUNT = 12
# LED_PIN = 21
# LED_FREQ_HZ = 800000
# LED_DMA = 10
# LED_BRIGHTNESS = 255
# LED_INVERT = False

# def save_values(dict):
#     # Save the dictionary to a Pickle file
#     with open('memory.pkl', 'wb') as file:
#         pickle.dump(dict, file)

# def calibrate(gray):
#     # All the 6 methods for comparison in a list
#     methods = [cv2.TM_CCOEFF, cv2.TM_CCOEFF_NORMED, cv2.TM_CCORR,
#     cv2.TM_CCORR_NORMED, cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]

#     chosen_method = methods[1]

#     # distance for bounding box from marker
#     edge = 5

#     img2 = gray.copy()

#     # bottom left
#     result = cv2.matchTemplate(img2, bl_template, chosen_method)
#     min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
#     location = max_loc
#     bottom_left = (location[0] - edge, location[1] + bl_h)
#     cv2.rectangle(image, location, bottom_left, 255, 5)
#     # cv2.circle(image, bottom_left, 5, (0, 255, 0), 2)

#     # top left
#     result = cv2.matchTemplate(img2, tl_template, chosen_method)
#     min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
#     location = max_loc  
#     top_left = (location[0] - edge, location[1])
#     cv2.rectangle(image, location, top_left, 255, 5)
#     # cv2.circle(image, top_left, 5, (0, 255, 0), 2)

#     # top right
#     result = cv2.matchTemplate(img2, tr_template, chosen_method)
#     min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
#     location = max_loc
#     top_right = (location[0] + tr_w + edge, location[1])
#     cv2.rectangle(image, location, top_right, 255, 5)
#     # cv2.circle(image, top_left, 5, (0, 255, 0), 2)

#     # bottom right
#     result = cv2.matchTemplate(img2, br_template, chosen_method)
#     min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
#     location = max_loc  
#     bottom_right = (location[0] + br_w + edge, location[1] + br_h)
#     cv2.rectangle(image, location, bottom_right, 255, 5)
#     # cv2.circle(image, bottom_right, 5, (0, 255, 0), 2)

#     # Define a region of interest (ROI) based on the bounding box
#     roi = gray[top_left[1]:bottom_right[1], top_left[0]:bottom_right[0]]

#     # servo edge
#     servo_box_l = (bottom_left[0], bottom_left[1] + 20)
#     servo_box_r = (bottom_right[0], bottom_right[1] + 20)

#     return top_left, bottom_right, top_right, bottom_left, roi, servo_box_l, servo_box_r

# # create neopixel ring
# ring = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS)
# ring.begin()

# white = Color(255, 255, 255)
# for i in range(LED_COUNT):
#     ring.setPixelColor(i, white)
#     ring.show()

# # Define the frame dimensions
# width, height = 300, 480

# # Initialize the PiCamera
# camera = PiCamera()
# camera.resolution = (width, height)
# camera.rotation = 90

# camera.resolution = (300, 480) 
# camera.framerate = 30
# rawCapture = PiRGBArray(camera, size=(width, height))
# time.sleep(1)

# camera.capture('template.jpg')

# corner template images
# bl_template = cv2.imread('/home/jasonsheinkopf/Desktop/object_track/BL.jpg', 0).astype('uint8')
# tl_template = cv2.imread('/home/jasonsheinkopf/Desktop/object_track/TL.jpg', 0).astype('uint8')
# br_template = cv2.imread('/home/jasonsheinkopf/Desktop/object_track/BR.jpg', 0).astype('uint8')
# tr_template = cv2.imread('/home/jasonsheinkopf/Desktop/object_track/TR.jpg', 0).astype('uint8')

# # corner template sizes
# bl_h, bl_w = bl_template.shape
# tl_h, tl_w = tl_template.shape
# br_h, br_w = br_template.shape
# tr_h, tr_w = tr_template.shape

# # Set a threshold for matching
# gamma = .73

# # line threshold
# threshold = 800

# frame_num = 0

# Create a thread for moving the servo
# servo_thread = threading.Thread(target=servo_thread_fn)

# Start the thread
# servo_thread.start()

# try:
#     # Try to load the Pickle file
#     with open('memory.pkl', 'rb') as file:
#         circle_params = pickle.load(file)
# except FileNotFoundError:
#     # If the file doesn't exist, create an empty dictionary
#     circle_params = {
#     'dp': 1,
#     'minDist': 50,
#     'param1': 50,
#     'param2': 20,
#     'minRadius': 13,
#     'maxRadius': 22
# }


# # empty list to hold targets
# new_targets = []

# # sequential id for targets
# next_id = 0

try:
    for frame in camera.capture_continuous(rawCapture, format='bgr', use_video_port=True):
        # # list of active target objects
        # targets = new_targets

        # # empty new target list
        # new_targets = []

        # # image object
        # image = frame.array

        # # input key
        # key = cv2.waitKey(1) & 0xFF

        # Convert the frame to grayscale
        # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # # calibrate corners on first frame or key press
        # if frame_num == 0 or key == ord('c'):
        #     top_left, bottom_right, top_right, bottom_left, roi, servo_box_l, servo_box_r = calibrate(gray)   
        #     print('Calibrating')

        # show servo location dot
        # servo_dot(servo_box_l, servo_box_r)
        
        # # draw bounding box
        # cv2.line(image, top_left, top_right, (0, 255, 255), 1)
        # cv2.line(image, top_right, bottom_right, (0, 255, 255), 1)
        # cv2.line(image, bottom_right, bottom_left, (0, 255, 255), 1)
        # cv2.line(image, bottom_left, top_left, (0, 255, 255), 1)

        # # draw servo box
        # # cv2.line(image, bottom_right, servo_box_r, (255, 0, 0), 1)
        # # cv2.line(image, bottom_left, servo_box_l, (255, 0, 0), 1)
        # cv2.line(image, servo_box_r, servo_box_l, (255, 0, 0), 2)
        
        # Use Hough Circle Transform to detect circles with parameters
        # circles = cv2.HoughCircles(
        #     gray,
        #     cv2.HOUGH_GRADIENT,
        #     **circle_params
        # )

        # if circles is not None:
        #     circles = np.uint16(np.around(circles))

        #     circle_list = []

        #     # remove false circles
        #     for circle in circles[0, :]:
        #         # append to list if white
        #         try:
        #             if gray[circle[1], circle[0]] > 200:
        #                 circle_list.append(circle)
        #         except IndexError:
        #             pass
            
        #     num_targets = len(targets)
        #     num_circles = len(circle_list)
        #     tar_to_add = num_circles - num_targets

        #     print(f'Targets: {num_targets} | Circles: {num_circles} | Extra Circ: {tar_to_add} | Angle: {goal_pos}')

        #     # if there are no targets
        #     # if num_targets == 0:
        #     #     # iterate over all circles
        #     #     for circle in circle_list:
        #     #         # add all as new targets
        #     #         new_targets.append(Target(next_id, circle[0], circle[1]))
        #     #         # increment id count
        #     #         next_id += 1
        #     # if there are some targets but more circles
        #     if num_targets <= num_circles:
        #         # Draw the outer circle
        #         [cv2.circle(image, (circle[0], circle[1]), circle[2], (0, 255, 0), 2) for circle in circle_list]
                
        #         # iterate over existing targets
        #         for target in targets:
        #             # print(f'Targets: {len(targets)} | Circles: {len(circle_list)} | Extra Circ: {tar_to_add}')
        #             # sort circle list by distance from target (closest last)
        #             circle_list.sort(key=lambda circle: target.find_distance(circle), reverse=True)
        #             # pop closest circle
        #             closest_circle = circle_list.pop()
        #             # update target location
        #             target.x, target.y = closest_circle[0], closest_circle[1]
        #             # append updated target to new list
        #             new_targets.append(target)
        #         # iterate over remaining circles
        #         for circle in circle_list:
        #             # append as new targets
        #             new_targets.append(Target(next_id, circle[0], circle[1]))
        #             # increment id count
        #             next_id += 1
        #     elif num_targets > num_circles:
        #         # iterate over circles
        #         for circle in circle_list:
        #             # sort target list by distance from circle
        #             targets.sort(key=lambda target: target.find_distance(circle), reverse=True)
        #             # for target in targets:
        #             #     print(f'ID: {target.id} | Dist: {target.find_distance(circle)}')
        #             # time.sleep(1)
        #             # pop closest target
        #             closest_target = targets.pop()
        #             # update target location
        #             closest_target.x, closest_target.y = circle[0], circle[1]
        #             # append updated target to new list
        #             new_targets.append(closest_target)
        #             # Draw the outer circle
        #             cv2.circle(image, (circle[0], circle[1]), circle[2], (0, 255, 0), 2)

        #         # Draw the center of the circle
        #         # cv2.circle(image, (center_x, center_y), 2, (0, 0, 255), 3)
                
        #         # Display text
        #         # text = f"{center_x}, {center_y})"
        #         # cv2.putText(image, text, (center_x - 30, center_y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
        
        # # print(f'Num Targets: {len(new_targets)}')

        # for target in new_targets:
        #     cv2.putText(image, str(target.id), (target.x, target.y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)

        # go to angle
        # servo_track(servo_angle)

        # if key == ord('1'):
        #     circle_params['minDist'] += 1
        #     save_values(circle_params)
        # elif key == ord('a'):
        #     circle_params['minDist'] -= 1
        #     save_values(circle_params)
        # elif key == ord('2'):
        #     circle_params['param1'] += 1
        #     save_values(circle_params)
        # elif key == ord('s'):
        #     circle_params['param1'] -= 1
        #     save_values(circle_params)
        # elif key == ord('3'):
        #     circle_params['param2'] += 1
        #     save_values(circle_params)
        # elif key == ord('d'):
        #     circle_params['param2'] -= 1
        #     save_values(circle_params)
        # elif key == ord('4'):
        #     circle_params['minRadius'] += 1
        #     save_values(circle_params)
        # elif key == ord('f'):
        #     circle_params['minRadius'] -= 1
        #     save_values(circle_params)
        # elif key == ord('5'):
        #     circle_params['maxRadius'] += 1
        #     save_values(circle_params)
        # elif key == ord('g'):
        #     circle_params['maxRadius'] -= 1
        #     save_values(circle_params)
        # elif key == ord('q'):
        #     cv2.destroyAllWindows()
        #     camera.close()  # Close the PiCamera
        #     # sys.exit(0)
        #     break
        #     # print("Stopping thread and exiting program")
        #     # exit_event.set()
        #     # # wait for thread to finish
        #     # servo_thread.join()
        # elif key == ord('y'):
        #     if goal_pos < max_pos:
        #         goal_pos += 50
        # elif key == ord('h'):
        #     if goal_pos > min_pos:
        #         goal_pos -= 50

        # print(circle_params)

        # threshold_text = f'Threshold = {threshold:.5f}'
        # cv2.putText(image, threshold_text, (20, int(height/2)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1)
        
        # Display the image in a non-GUI window
        cv2.imshow('Circle Detection', image)
        
        # Clear the stream for the next frame
        rawCapture.truncate(0)
        
        # If 'q' is pressed, break the loop
        if key == ord('q'):
            break

        frame_num += 1

except KeyboardInterrupt:
    # print("Stopping thread and exiting program")
    # exit_event.set()
    cv2.destroyAllWindows()
    camera.close()  # Close the PiCamera
    sys.exit(0)

finally:
    cv2.destroyAllWindows()
    camera.close()  # Close the PiCamera
    sys.exit(0)

# wait for thread to finish
# servo_thread.join()

# Close all OpenCV windows and release resources
cv2.destroyAllWindows()
camera.close()  # Close the PiCamera
sys.exit(0)
