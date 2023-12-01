# Kalman Katcher
Raspberry Pi computer vision project that uses a Kalman Filter to track and catch ping pong balls with a servo.

# Demo
<img src="gif/kalman_catcher_short_demo.gif" alt="Your GIF" width="800">

[Kalman Katcher on YouTube](https://www.youtube.com/watch?v=kK27OdQG_vQ&ab_channel=JasonSheinkopf)

# How It Works
- Pi camera and Open CV capture video
- boundary box is detected with cv2 template matching
- lookup dict is created to match observed servo position with PWM frequency
- white circles are detected
- target objects are created for each new circle and tracked
- a Kalman Filter algorithm is implemented to track each target
- future positions are estimated using KF state model
- target with soonest impact is selected
- intersection with future trajectory and goal line sets servo position
- thread is started to control servo

# Creating the Lookup Dict
- servo sweeps from left to right (min to max PWM frequency)
- catcher location determined by center of white pixels intersecting goal line
- lookup dict values saved to pickle
<img src="gif/kalibration.gif" alt="Your GIF" width="800">

# Kalman Filter
A Kalman Filter is an algorithm used to track objects by performing a convolution of two Gaussians: the observations and a state model.
A covariance matrices containing confidence levels associated with each of these two distributions are set as hyperparameters. On each
step, the filter uses the current observation and an internalized state model to localize the object and track its kinematics.

# Installation on the Raspberry Pi
## 1. Clone the Repository
```bash
git clone https://github.com/jasonsheinkopf/kalman_katcher
```
## 2. Create virtual environment from requirements.txt and activate
```bash
python -m venv kalman_venv
```
## 3. Activate virtual environment and install dependencies
```base
source kalman_venv/bin/activate
pip install -r requirements.txt
```
