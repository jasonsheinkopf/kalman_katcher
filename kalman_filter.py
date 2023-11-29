import numpy as np
# import math
import sympy as sp

class Catcher():
    def __init__(self, dt):
        
        # variance hyperparameters
        self.Rvar_x = .001                   # measurement variance x
        self.Rvar_y = .001                   # measurement variance y
        self.x_pos_var = self.Rvar_x        # initial x position variance
        self.y_pos_var = self.Rvar_y        # initial y position variance
        self.x_vel_var = 1000               # initial x velocity variance
        self.y_vel_var = 1000               # initial y velocity variance
        self.y_acc_var = 1000               # initial x accel variance
        self.noise_x_pos = 0.1                # x position noise variance
        self.noise_y_pos = 0.1                # y position noise variance
        self.noise_x_vel = 1.0                # x velocity noise variance
        self.noise_y_vel = 1.0                # y velocity noise variance
        self.noise_y_accel = 0.1              # y accel noise variance
        self.damping = 1                  # percent of energy kept on bounce
        self.r = 0.75                           # percent of energy kept on roll (restitution)
        # self.x_pos = init_pos[0]
        # self.y_pos = init_pos[1]
        self.dt = dt
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
        # dict to hold current target
        # self.current_target = None
        # dictionary to hold all targets
        # self.targets = {}
        self.observations = []

    def kf(self, x, P, observation):
        # measurement update
        Z = np.array([[observation[0], observation[1]]])
        # print(f'Z.T shape: {Z.T.shape}')
        # print(f'H shape: {self.H.shape}')
        # innovation - difference between the observation and the state
        self.y = Z.T - (self.H @ x)
        # print(f'y shape: {self.y.shape}')
        # innovation covariance
        S = self.H @ (P @ self.H.T) + self.R
        # print(f'S shape: {S.shape}')
        # print(f'P shape: {P.shape}')
        # print(P)
        # Kalman gain - how much you should trust the observation (KG = 0) vs the state model (KG = 1)
        K = P @ self.H.T @ np.linalg.inv(S)
        # print(f'K shape: {K.shape}')
        # update belief based on KG and innovation
        # print(f'x shape: {x.shape}')
        # print(x)
        # print(f'K @ self.y: {(K @ self.y).shape}')
        # print(K @ self.y)
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
        # if len(target.observations) < 3:
        if len(target.observations) < 0:
            pass

        # append current observation to list
        # elif len(target.observations) == 3:
        elif len(target.observations) <= 3:
            # x initial state calculations
            # x0 = target.observations[0][0]
            # x1 = target.observations[1][0]
            # x2 = target.observations[2][0]
            # vx1 = x1 - x0
            # vx2 = x2 - x1
            # vxavg = (vx1 + vx2) / 2

            # # y initial state calculations
            # y0 = target.observations[0][1]
            # y1 = target.observations[1][1]
            # y2 = target.observations[2][1]
            # vy1 = y0 - y1
            # vy2 = y1 - y2
            # ay2 = vy1 - vy2

            x2, y2 = target.observations[0][0], target.observations[0][1]
            vx2, vy2 = 0, -100
            ay2 = 3000

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
            
        # starting on 4th observation, use KF to predict
        if len(target.observations) >= 3:
            # bounce off walls
            # if target.x > 100

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
            
            # print(f'Target vy: {round(target.kfx[3].item(), 1)}')

        return target