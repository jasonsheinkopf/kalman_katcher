import numpy as np
# import math
import sympy as sp

class Catcher():
    def __init__(self, dt):
        
        # variance hyperparameters
        self.Rvar_x = .01                   # measurement variance x
        self.Rvar_y = .01                   # measurement variance y
        self.x_pos_var = self.Rvar_x        # initial x position variance
        self.y_pos_var = self.Rvar_y        # initial y position variance
        self.x_vel_var = 1000               # initial x velocity variance
        self.y_vel_var = 1000               # initial y velocity variance
        self.y_acc_var = 1000               # initial x accel variance
        self.noise_x_pos = 0.01                # x position noise variance
        self.noise_y_pos = 0.01                # y position noise variance
        self.noise_x_vel = 0.01                # x velocity noise variance
        self.noise_y_vel = 0.01                # y velocity noise variance
        self.noise_y_accel = 0.01              # y accel noise variance
        # self.x_pos = init_pos[0]
        # self.y_pos = init_pos[1]
        self.dt = dt
        self.F = np.array([[1, 0, self.dt, 0, 0],
                           [0, 1, 0, self.dt, self.dt**2 / 2],
                           [0, 0, 1, 0, 0],
                           [0, 0, 0, 1, self.dt],
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
    
    def predict(self, target, goal_y):
        '''Update belief about individual target based on current observation.'''       
        if len(target.observations) < 3:
            kfx, P, future_points, t, goal_x = None, None, [], None, None

        # append current observation to list
        elif len(target.observations) == 3:
            # x initial state calculations
            x0 = target.observations[0][0]
            x1 = target.observations[1][0]
            x2 = target.observations[2][0]
            # vx1 = x1 - x0
            vx2 = x2 - x1

            # y initial state calculations
            y0 = target.observations[0][1]
            y1 = target.observations[1][1]
            y2 = target.observations[2][1]
            vy1 = y1 - y0
            vy2 = y2 - y1
            ay2 = vy2 - vy1

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
            # update state and covariance with KF using x and P and most recent observation
            kfx, P = self.kf(target.kfx, target.P, (target.x, target.y))

            # # get kinematics variables
            # x, y = kfx[0].item(), kfx[1].item()
            # vx, vy = kfx[2].item(), kfx[3].item()
            # ay = kfx[4].item()

            # # define variables
            # t = sp.symbols('t')
            # goal_x = sp.symbols('goal_x')

            # # kinematics equations
            # eq_1 = sp.Eq(goal_x, x + vx * t)
            # eq_2 = sp.Eq(goal_y, y + vy * t + ay**2 / 2)

            # # solve equations
            # solutions = sp.solve((eq_1, eq_2), (t, goal_x))
            # # Extract solutions
            # # positive_t_solutions = [(sol[t], sol[goal_x]) for sol in solutions if sol[t] > 0]
            # print(solutions)

            # # extract solutions
            # t = solutions[t]
            # goal_x = solutions[goal_x]
            goal_x = None

            # # initialize future points as empty list
            future_points = []

            # initialize number of time steps to predict into the future
            t = 1
            
            # predict future points until ball's predicted location is off screen
            while t < 1:
                # elapsed time until prediction
                e_time = t * self.dt
                # state dynamics model for future location
                p_F = np.array([[1, 0, e_time, 0, 0],
                            [0, 1, 0, e_time, e_time**2 / 2],
                            [0, 0, 1, 0, 0],
                            [0, 0, 0, 1, e_time],
                            [0, 0, 0, 0, 1]])

                # calculate predicted x belief matrix
                p_kfx = p_F @ target.kfx
                # get predicted coordinate
                predicted_coord = (int(p_kfx[0].item()), int(p_kfx[1].item()))
                # append to list of future coordinates
                future_points.append(predicted_coord)
                # increment time step
                t += 1



        return kfx, P, future_points, t, goal_x