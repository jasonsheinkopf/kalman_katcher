import numpy as np
import sympy as sp

class Catcher():
    def __init__(self, dt):
        # variance hyperparameters
        self.Rvar_x = 0.001                      # measurement variance x
        self.Rvar_y = 0.001                      # measurement variance y
        self.x_pos_var = self.Rvar_x           # initial x position variance
        self.y_pos_var = self.Rvar_y            # initial y position variance
        self.x_vel_var = 20                   # initial x velocity variance
        self.y_vel_var = 10                   # initial y velocity variance
        self.y_acc_var = 1                   # initial x accel variance
        self.noise_x_pos = 0.1                  # x position noise variance
        self.noise_y_pos = 0.1                  # y position noise variance
        self.noise_x_vel = 0.1                 # x velocity noise variance
        self.noise_y_vel = 0.1                 # y velocity noise variance
        self.noise_y_accel = 0.01                # y accel noise variance
        self.damping = 0.5                      # percent of energy kept on bounce
        self.r = 1                            # percent of energy kept on roll (restitution)
        self.dt = dt                            # time increment
        self.F = np.array([[1, 0, self.dt * self.r, 0, 0],
                           [0, 1, 0, self.dt * self.r, self.dt**2 / 2],
                           [0, 0, self.r, 0, 0],
                           [0, 0, 0, self.r, self.dt],
                           [0, 0, 0, 0, 1]])
        self.u = np.array([[0], [0], [0], [0], [0]])    # no added thrust - included for completeness
        self.H = np.array([[1, 0, 0, 0, 0],             # observation transform matrix
                         [0, 1, 0, 0, 0]])
        self.R = np.array([[self.Rvar_x, 0],            # measurement covariance matrix
                           [0, self.Rvar_y]])
        self.I = np.eye(5)                              # identity matrix
        # state dynamice covariance matrix
        self.Q = np.diag([self.noise_x_pos, self.noise_y_pos, self.noise_x_vel, self.noise_y_vel, self.noise_y_accel])
        self.y = None
        self.observations = []

    def kf(self, x, P, observation):
        # measurement update
        Z = np.array([[observation[0], observation[1]]])
        # innovation - difference between the observation and the state
        self.y = Z.T - (self.H @ x)
        # innovation covariance
        S = self.H @ (P @ self.H.T) + self.R
        # Kalman gain - how much you should trust the observation (KG = 0) vs the state model (KG = 1)
        K = P @ self.H.T @ np.linalg.inv(S)
        # update belief based on KG and innovation
        x = x + (K @ self.y)
        # update state covariance
        P = (self.I - (K @ self.H)) @ P
        # update belief based on state model and added force (none)
        x = (self.F @ x) + self.u
        # update state covariance based on mstate odel covariance
        P = self.F @ P @ self.F.T + self.Q

        return x, P
    
    def predict(self, target, goal_y, boundaries):
        '''Update belief about individual target based on current observation.'''       
        # append first observation to list
        if len(target.observations) == 1:
            # set initial state values
            x2, y2 = target.observations[-1][0], target.observations[-1][1]
            vx2, vy2 = 0, -800
            ay2 = 5000

            # initial target belief
            target.kfx = np.array([x2,
                                    y2,
                                    vx2,
                                    vy2,
                                    ay2]).reshape(5, 1)
            
            # set initial covariance matrix
            target.P = np.array([[self.x_pos_var, 0, 0, 0, 0],
                                    [0, self.y_pos_var, 0, 0, 0],
                                    [0, 0, self.x_vel_var, 0, 0],
                                    [0, 0, 0, self.y_vel_var, 0],
                                    [0, 0, 0, 0, self.y_acc_var]])
        elif len(target.observations) > 1:
            # update state and covariance with KF using x and P and most recent observation
            target.kfx, target.P = self.kf(target.kfx, target.P, (target.x, target.y))
            # initialize number of time steps to predict into the future
            t = 1
            # time cutoff for no impact
            time_cutoff = 100
            # set initial prediction to current state
            next_kfx = target.kfx.copy()
            # reinitialize future points to empty list
            target.future_points = [(next_kfx[0].item(), next_kfx[1].item(), 10000)]
            # while kalman filter belief of y position is above goal
            while next_kfx[1] < goal_y and t != 100:
                # elapsed time until prediction
                e_time = t * self.dt
                # state dynamics model for future location
                p_F = np.array([[1, 0, e_time * self.r, 0, 0],
                                [0, 1, 0, e_time * self.r, e_time**2 / 2],
                                [0, 0, self.r, 0, 0],
                                [0, 0, 0, self.r, e_time],
                                [0, 0, 0, 0, 1]])
                # calculate predicted x belief matrix
                next_kfx = p_F @ next_kfx
                # boundary border
                border_buffer = 20
                # if future point x past right wall and moving to the right
                if next_kfx[0].item() > boundaries['right'] - border_buffer and next_kfx[2].item() > 0:
                    # invert vx to bounce
                    next_kfx[2] *= -self.damping
                    next_kfx[3] *= self.damping
                elif next_kfx[0].item() < boundaries['left'] + border_buffer and next_kfx[2].item() < 0:
                    # invert vx to bounce
                    next_kfx[2] *= -self.damping
                    next_kfx[3] *= self.damping
                # if the next point y is above the top
                elif next_kfx[1].item() < boundaries['top'] + border_buffer and next_kfx[3].item() < 0:
                    next_kfx[3] *= -self.damping
                # get predicted coordinate and time until that location
                predicted_coord = (int(next_kfx[0].item()), int(next_kfx[1].item()), t)
                # if time cutoff has reached target has no predicted impact
                if t > time_cutoff:
                    # indicate no predicted impact as 1000
                    t = 10000
                else:
                    # increment time step
                    t += 1
                # append to list of future coordinates
                target.future_points.append(predicted_coord)
            # check if last future point y position is above goal
            if next_kfx[1] < goal_y:
                # set point's time step to 10000 to show it won't impact
                prev_future_point = target.future_points[-1]
                target.future_points[-1] = (prev_future_point[0], prev_future_point[1], 10000)

        return target