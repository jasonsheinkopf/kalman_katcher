from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
from time import sleep
import pigpio
import pickle
import math
import threading
from kalman_filter import Catcher

"""
This code was created by Jason Sheinkopf to track and catch ping-pong balls rolling down a slope using computer vision and
a Kalman filter. You may copy this code for personal use and citation would be appreciated. To calibrate corners, press 'c'.
To create new lookup table from servo location to PWM frequency, press 't'.
"""

class Target:
    """
    Class to track individual white ping-pong ball
    """
    def __init__(self, id, x, y, r):
        self.id = id
        self.x = x
        self.y = y
        # detected radius
        self.r = r
        # Kalman filter belief covariance matrix
        self.P = None
        # Kalman filter belief matrix
        self.kfx = None
        # list of observations
        self.observations = []
        # list of future points
        self.future_points = [(0, 0, 1000)]
        self.goal_x = None
        self.impact_time = 1000

    def __repr__(self):
        return f'{self.id}: ({self.x}, {self.y})'
    
    def find_distance(self, circle):
        return math.sqrt((int(circle[0]) - int(self.x))**2 + (int(circle[1]) - int(self.y))**2)

def servo_dot(goal_left_point, goal_right_point, color):
    """
    Used for creating a lookup table from catcher location to servo PWM frequency.
    Servo box are left endpoint of goal. Updates global servo_x.
    """
    global servo_x
    x1, y1 = goal_left_point
    x2, y2 = goal_right_point

    # list to hold sample points along goal line
    line_values = []

    # create equally spaced points along goal line
    for t in np.linspace(0, 1, 100):
        x = int(x1 + t * (x2 - x1))
        y = int(y1 + t * (y2 - y1))
        line_values.append(image[y, x])

    # points brighter than threshold are part of white servo arm
    BRIGHT_THRESHOLD = 200

    # find indices of bright pixels
    bright_indices = np.where(np.array(line_values) > BRIGHT_THRESHOLD)

    # if there are no bright regions
    if bright_indices[0].size == 0:
        print("Catcher arm not intersecting goal line.")
    else:
        # find centerpoint
        center_index = np.array(bright_indices[0]).mean().astype(int)
        x_center = int(x1 + np.linspace(0, 1, 100)[center_index] * (x2 - x1))
        y_center = int(y1 + np.linspace(0, 1, 100)[center_index] * (y2 - y1))
        # x distance from dot to center
        servo_x_distance_to_center = x_center - (width / 2)
        # normalize dist to center to max at 1
        norm_dist_to_center = servo_x_distance_to_center / (width / 2)
        # coefficient to adjust dot location to counteract camera perspective
        DOT_ADJ_COEFF = 10
        # adjustment
        dot_adj = int(norm_dist_to_center * DOT_ADJ_COEFF)
        # adjust dot x
        servo_x_adjusted = x_center + dot_adj
        # servo_x = x_center
        servo_x = servo_x_adjusted
        # Draw a circle at the center point
        cv2.circle(image, (servo_x, y_center), 10, color, -1)

def save_params(dict):
    # save circle detection parameters to pickle for key press adjustment
    with open('memory.pkl', 'wb') as file:
        pickle.dump(dict, file)

def calibrate_corners(gray):
    """
    Calibrates edges and goal line using template matching. Returns corner and goal endpoints.
    """
    # corner template images
    bl_template = cv2.imread('templates/BL.jpg', 0).astype('uint8')
    tl_template = cv2.imread('templates/TL.jpg', 0).astype('uint8')
    br_template = cv2.imread('templates/BR.jpg', 0).astype('uint8')
    tr_template = cv2.imread('templates/TR.jpg', 0).astype('uint8')

    # corner template sizes
    bl_h, bl_w = bl_template.shape
    tl_h, tl_w = tl_template.shape
    br_h, br_w = br_template.shape
    tr_h, tr_w = tr_template.shape

    # distance for bounding box from marker
    edge = 5

    # bottom left
    result = cv2.matchTemplate(gray, bl_template, cv2.TM_CCOEFF_NORMED)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
    location = max_loc
    bottom_left = (location[0] - edge, location[1] + bl_h)
    # top left
    result = cv2.matchTemplate(gray, tl_template, cv2.TM_CCOEFF_NORMED)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
    location = max_loc  
    top_left = (location[0] - edge, location[1])
    # top right
    result = cv2.matchTemplate(gray, tr_template, cv2.TM_CCOEFF_NORMED)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
    location = max_loc
    top_right = (location[0] + tr_w + edge, location[1])
    # bottom right
    result = cv2.matchTemplate(gray, br_template, cv2.TM_CCOEFF_NORMED)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
    location = max_loc  
    bottom_right = (location[0] + br_w + edge, location[1] + br_h)

    # servo edge
    goal_left_point = (bottom_left[0], bottom_left[1] + GOAL_Y_OFFSET)
    goal_right_point = (bottom_right[0], bottom_right[1] + GOAL_Y_OFFSET)

    return top_left, bottom_right, top_right, bottom_left, goal_left_point, goal_right_point

# set exit event to end thread
exit_event = threading.Event()

# camera frame rate
framerate = 40

# create instance of catcher class at 30 fps
catcher = Catcher(dt=1/framerate)

# list of colors to draw circles
colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255)]

# servo PWM frequencies
MIN_PWM = 750
MAX_PWM = 2100

# initialize goal position at middle or range
goal_pos = (MAX_PWM - MIN_PWM) / 2

# how many pixels below bottom goal line is
GOAL_Y_OFFSET = 30

# global variable for current target
current_target = None

# create lookup dict to convert servo_x to goal_pos
try:
    # load lookup talbe
    with open('lookup.pkl', 'rb') as file:
        lookup_dict = pickle.load(file)
except FileNotFoundError:
    # If the file doesn't exist, create an empty dictionary
    print("Servo lookup table lookup.pkl not found. Press 't' to create.")
    lookup_dict = {}

# servo dot location
servo_x = 0

# pgpio setup for servo GPIO pin
servo = 13
pwm = pigpio.pi()
pwm.set_mode(servo, pigpio.OUTPUT)
pwm.set_PWM_frequency(servo, 50)

# change this flag to False if you don't have a lookup.pkl
have_lookup_pickle = True

def choose_target(new_targets):
    """
    Returns target object that will imapct soonest based on Kalman filter prediction.
    """
    global current_target
    # if there are targets
    if len(new_targets) != 0:
        # set current target to the target whose last future point's t value is lowest (soonest impact)
        current_target = min(new_targets, key=lambda target: target.future_points[-1][2])
        # check if current target will never impact or if only one future point exists
        if current_target.future_points[-1][2] == 10000 or len(current_target.future_points) == 1:
            # select target lowest on the screen
            current_target = max(new_targets, key=lambda target: target.y)
            # set target's x to middle
            current_target.goal_x = int(width / 2)
            print('going to middle')
        else:
            # last 2 future points are before and then after crossing goal
            before = (current_target.future_points[-2][0], current_target.future_points[-2][1])
            after = (current_target.future_points[-1][0], current_target.future_points[-1][1])
            # left and right endpoints of the goal
            left_goal = boundaries['goal_l_point']
            right_goal = boundaries['goal_r_point']
            # try to find intersection of line connecting last 2 future points and goal line
            try:
                # calculate slopes
                m_points = (after[1] - before[1]) / (after[0] - before[0])
                m_goal = (left_goal[1] - right_goal[1]) / (left_goal[0] - right_goal[0])
                # calculate y-intercepts
                b_points = before[1] - m_points * before[0]
                b_goal = right_goal[1] - m_goal * right_goal[0]
                # calculate intersection
                x_goal_predict = int((b_goal - b_points) / (m_points - m_goal))
                y_goal_predict = int(m_points * x_goal_predict + b_points)
            # if the line is vertical
            except ZeroDivisionError:
                # set goal x to middle
                x_goal_predict = int(width / 2)
                # set goal y to middle height of goal
                y_goal_predict = int((left_goal[1] + right_goal[1]) / 2)
            # goal x is where the line between these points intersects goal
            current_target.goal_x = x_goal_predict
            # draw circle after 3rd observation
            if len(current_target.observations) > 2:
                cv2.circle(image, (x_goal_predict, y_goal_predict), 5, colors[current_target.id % 5], 5)
                cv2.circle(image, (x_goal_predict, y_goal_predict), 3, (255, 255, 255), 3)
                cv2.circle(image, (x_goal_predict, y_goal_predict), 2, (0, 0, 0), 2)
        # print(f'Catching target {current_target.id} at x = {round(current_target.goal_x)}')

    else:
        # print('no current targets')
        current_target = None

    return current_target

def go_to_position(position):
    # print(f'Going to: {position}')
    pwm.set_servo_pulsewidth(servo, position) ;
    time.sleep(.3)

def servo_thread_fn():
    print('servo thread')
    global goal_pos, lookup_dict, servo_x, current_target, have_lookup_pickle
    # delay to let camera start
    time.sleep(3)
    
    # continue look until thread exit event is detected
    while not exit_event.is_set():
        # if flag is triggered, servo sweep will create lookup table
        if have_lookup_pickle == False:
            # start servo at min position
            goal_pos = MIN_PWM
            # create empty lookup dict
            lookup_dict = {}
            # continue to sweep until it hits mas pos
            while goal_pos < MAX_PWM:
                # send servo to active position
                go_to_position(goal_pos)
                # wait for servo to move
                time.sleep(.05)
                # record goal position for given servo_x
                print(f'Adding {servo_x}: {goal_pos}')
                lookup_dict[servo_x] = goal_pos
                # delay for movement
                time.sleep(.05)
                # move servo
                goal_pos += 10
            # Save the dictionary to a pickle file
            with open('lookup.pkl', 'wb') as file:
                pickle.dump(lookup_dict, file)
            print('Lookup table pickled')
            have_lookup_pickle = True
        # set target position for servo to move to
        if current_target is not None:
            # x value of the target
            current_target_x = current_target.goal_x
            # find closest key in the lookup_dict
            closest_key = min(lookup_dict, key=lambda key: abs(current_target_x - key))
            # retrieve servo PWM frequency from lookup table
            goal_pos = lookup_dict[closest_key]
            print(f'Going to: {closest_key}: {goal_pos} from current target: {current_target_x} to target: {current_target.id}')
        else:
            # if no target is seen, return to middle point
            goal_pos = (MAX_PWM + MIN_PWM) / 2
        # send servo to active position
        go_to_position(goal_pos)

# Create a thread for moving the servo
servo_thread = threading.Thread(target=servo_thread_fn)

# Start the thread
servo_thread.start()

# set exit event to end thread
exit_event = threading.Event()

# Define the frame dimensions
width, height = 300, 480

# Initialize the PiCamera
camera = PiCamera()
camera.resolution = (width, height)
camera.rotation = 90
camera.framerate = framerate
camera.shutter_speed = 4500
rawCapture = PiRGBArray(camera, size=(width, height))
time.sleep(1)

# track frames since video began
frame_num = 0

try:
    # Try to load the circle detection parameter pickle
    with open('memory.pkl', 'rb') as file:
        circle_params = pickle.load(file)
except FileNotFoundError:
    # If the file doesn't exist, create a dict with circle params
    circle_params = {
    'dp': 1,
    'minDist': 50,
    'param1': 50,
    'param2': 20,
    'minRadius': 13,
    'maxRadius': 22
}

# empty list to hold targets
new_targets = []

# sequential id for targets
next_id = 0

# number of consecutive frames to constitute a change in circle count
update_threshold = 2

# number of consecutive frames circle count did not match target count
change_count = 0

# max change count to 
max_change_count = 0

try:
    # iterate over frames in continues capture
    for frame in camera.capture_continuous(rawCapture, format='bgr', use_video_port=True):
        # list of active target objects
        targets = new_targets

        # empty new target list
        new_targets = []
        
        # Image processing can be added here
        image = frame.array

        # convert to gray
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # set key press length
        key = cv2.waitKey(1) & 0xFF

        # calibrate_corners corners on first frame or key press
        if frame_num == 0 or key == ord('c'):
            top_left, bottom_right, top_right, bottom_left, goal_left_point, goal_right_point = calibrate_corners(gray)   
            # boundaries for bounce
            boundaries = {
                'left': (top_left[0] + bottom_left[0]) / 2,
                'right': (top_right[0] + bottom_right[0]) / 2,
                'top': (top_left[1] + top_right[1]) / 2,
                'goal': (goal_left_point[1] + goal_right_point[1]) / 2,
                'goal_l_point': goal_left_point,
                'goal_r_point': goal_right_point
            }
            goal_y = int((goal_left_point[1] + goal_right_point[1]) / 2)
            print('Calibrating')
        
        if not have_lookup_pickle:
            # set dot color to match current target color
            dot_color = (255, 0, 0)
            # show servo location dot
            servo_dot(goal_left_point, goal_right_point, dot_color)

        # draw bounding box
        cv2.line(image, top_left, top_right, (0, 255, 255), 1)
        cv2.line(image, top_right, bottom_right, (0, 255, 255), 1)
        cv2.line(image, bottom_right, bottom_left, (0, 255, 255), 1)
        cv2.line(image, bottom_left, top_left, (0, 255, 255), 1)

        # set goal color to match current target color
        goal_color = colors[current_target.id % 5] if current_target is not None else (0, 0, 0)
        # draw goal line
        cv2.line(image, goal_right_point, goal_left_point, (255, 255, 255), 2)

        # Use Hough Circle Transform to detect circles with parameters
        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            **circle_params
        )

        if circles is not None:
            circles = np.uint16(np.around(circles))

            circle_list = []

            # iterate over detected circles
            for circle in circles[0, :]:
                # cv2.circle(image, (circle[0], circle[1]), circle[2], (0, 0, 0), 5)
                try:
                    # check if circle has white center and is still above goal
                    if gray[circle[1], circle[0]] > 200 and circle[1] < goal_left_point[1]:
                        # append to list if white
                        circle_list.append(circle)
                except IndexError:
                    # print('Index error when checking for white circles.')
                    pass
            
            num_targets = len(targets)
            num_circles = len(circle_list)

            # check if number of circles changed
            if abs(num_targets - num_circles) != 0 and change_count < update_threshold:
                # pass on current targets
                new_targets = targets.copy()
                # set flag to do double check
                change_count += 1
                # print(f'Circles != Targets {change_count} times in a row')
                if change_count > max_change_count:
                    max_change_count = change_count
            else:
                # reset change count
                change_count = 0
                # if there are some targets but more circles
                if num_targets <= num_circles:
                    # iterate over existing targets
                    for target in targets:
                        # sort circle list by distance from target (closest last)
                        circle_list.sort(key=lambda circle: target.find_distance(circle), reverse=True)
                        # pop closest circle
                        closest_circle = circle_list.pop()
                        # update target location
                        target.x, target.y, target.r = closest_circle[0], closest_circle[1], closest_circle[2]
                        # append updated target to new list
                        new_targets.append(target)
                    # iterate over remaining circles
                    for circle in circle_list:
                        # append as new targets
                        new_targets.append(Target(next_id, circle[0], circle[1], circle[2]))
                        # increment id count
                        next_id += 1

                elif num_targets > num_circles:
                    # iterate over circles
                    for circle in circle_list:
                        # sort target list by distance from circle
                        targets.sort(key=lambda target: target.find_distance(circle), reverse=True)
                        # for target in targets:
                        #     print(f'ID: {target.id} | Dist: {target.find_distance(circle)}')
                        # time.sleep(1)
                        # pop closest target
                        closest_target = targets.pop()
                        # update target location
                        closest_target.x, closest_target.y, closest_target.r = circle[0], circle[1], circle[2]
                        # append updated target to new list
                        new_targets.append(closest_target)
                else:
                    # if a ball disappeared but you didn't check the next frame, keep all targets
                    new_targets = targets.copy()
        # iterate over targets
        for target in new_targets:
            # choose target color
            color = colors[target.id % 5]
            # draw circle
            cv2.circle(image, (target.x, target.y), target.r, color, 2)
            # append observation to history
            target.observations.append((target.x, target.y))
            # update belief and future points with Kalman filter
            target = catcher.predict(target=target, goal_y=goal_y, boundaries=boundaries)
            # if the KF has developed a belief
            if target.kfx is not None:
                # extract belief state kinematics parameters
                x, y = int(target.kfx[0].item()), int(target.kfx[1].item())
                vx, vy = int(target.kfx[2].item()), int(target.kfx[3].item())
                ay = int(target.kfx[3].item())
                # size of plus drawn around location belief
                plus_size = 20
                # draw belief marker
                cv2.line(image, (x + plus_size, y), (x - plus_size, y), color, 2)
                cv2.line(image, (x, y + plus_size), (x, y - plus_size), color, 2)
                cv2.circle(image, (x, y), 5, color, 5)
                cv2.circle(image, (x, y), 3, (255, 255, 255), 3)
                cv2.circle(image, (x, y), 2, (0, 0, 0), 2)
                # track previous point
                prev_future_point = (int(target.kfx[0].item()), int(target.kfx[1].item()))
                # wait until 3rd observation before drawing future points
                if len(target.observations) > 2:
                    # iterate over future points and draw them omitting first
                    for i, point in enumerate(target.future_points[1:]):
                        next_future_point = (point[0], point[1])
                        # draw line to connect future points
                        cv2.line(image, prev_future_point, next_future_point, color, 2)
                        # update prev
                        prev_future_point = next_future_point
                    # set catch_x as x value of predicted state belief before goal_y
                    target.catch_x = target.future_points[-1][0]
        # choose best target
        current_target = choose_target(new_targets)

        # keyboard commands to modify circle detection parameters and save to pickle
        if key == ord('q'):
            exit_event.set()
            servo_thread.join()
            cv2.destroyAllWindows()
            camera.close()
            break
        elif key == ord('1'):
            circle_params['minDist'] += 1
            save_params(circle_params)
            print(circle_params)
        elif key == ord('a'):
            circle_params['minDist'] -= 1
            save_params(circle_params)
            print(circle_params)
        elif key == ord('2'):
            circle_params['param1'] += 1
            save_params(circle_params)
            print(circle_params)
        elif key == ord('s'):
            circle_params['param1'] -= 1
            save_params(circle_params)
            print(circle_params)
        elif key == ord('3'):
            circle_params['param2'] += 1
            save_params(circle_params)
            print(circle_params)
        elif key == ord('d'):
            circle_params['param2'] -= 1
            save_params(circle_params)
            print(circle_params)
        elif key == ord('4'):
            circle_params['minRadius'] += 1
            save_params(circle_params)
            print(circle_params)
        elif key == ord('f'):
            circle_params['minRadius'] -= 1
            save_params(circle_params)
            print(circle_params)
        elif key == ord('5'):
            circle_params['maxRadius'] += 1
            save_params(circle_params)
            print(circle_params)
        elif key == ord('g'):
            circle_params['maxRadius'] -= 1
            save_params(circle_params)
            print(circle_params)
        elif key == ord('t'):
            have_lookup_pickle = False
            print('Regenerating lookup table')

        # Display the image
        cv2.imshow('Kalman Katcher', image)

        rawCapture.truncate(0)

        # increment frame count
        frame_num += 1

except KeyboardInterrupt:
    print("Stopping thread and exiting program")

finally:
    # Signal the servo thread to stop and wait for it to finish
    exit_event.set()
    servo_thread.join()
    cv2.destroyAllWindows()
    camera.close()