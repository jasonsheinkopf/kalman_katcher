import numpy as np
import math

class Catcher(object):
    def __init__(self, init_pos, dt):
        
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
        self.g = -9.81                      # gravitational constant
        self.theta = 5                      # ramp incline degrees
        self.theta_rad = math.radians(self.theta)   # ramp incline radians
        self.ga = self.g * math.cos(self.theta_rad)  # gravity along the incline
        self.x_pos = init_pos[0]
        self.y_pos = init_pos[1]
        self.dt = dt
        self.F = np.array([[1, 0, self.dt, 0, 0],
                           [0, 1, 0, self.dt, self.ga * self.dt**2 / 2],
                           [0, 0, 1, 0, 0],
                           [0, 0, 0, 1, self.ga * self.dt],
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
        self.current_target = None
        # dictionary to hold all targets
        self.targets = {}
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
    
    def predict(self, targets):
        '''Pass a list of target objects. Update predicted landing time and location and return.'''       
        # iterate over list of targets
        for target in targets:
            # if target id not already in dictionary, then add it with first observation
            if id not in self.targets.keys():
                self.targets[id] = {'P': None, 'x': None, 'observations': [(xobs, yobs)]}
                # make alias to observations
                obs = self.targets[id]['observations']
                # set initial observation to initial position value
                self.targets[id]['x'] = np.array([[obs[0][0]], 
                                                  [obs[0][1]],
                                                  [0],
                                                  [-.1],
                                                  [-.1]])
                # set initial covariance matrix
                self.targets[id]['P'] = np.array([[self.x_pos_var, 0, 0, 0, 0],
                                                  [0, self.y_pos_var, 0, 0, 0],
                                                  [0, 0, self.x_vel_var, 0, 0],
                                                  [0, 0, 0, self.y_vel_var, 0],
                                                  [0, 0, 0, 0, self.y_acc_var]])
            # update state and covariance with KF
            self.targets[id]['x'], self.targets[id]['P'] = self.kf(self.targets[id]['x'], self.targets[id]['P'], (xobs, yobs))
            # append meteorite location to list
            target_locations.append((id, self.targets[id]['x'][0][0], self.targets[id]['x'][1][0]))
        
                # record data for plotting for test meteorite
                # if id == self.test_meteorite_id:
                #     self.record_values(xobs, yobs)
       
        # increment elapsed time
        self.time += 1

        # list of meteorite ids that were destroyed
        destroyed_ids = []

        # Iterate over ids of my meteorites dict
        for id, info in self.targets.items():
            # Check if they are no longer in the new observation list
            if not any(id == observation[0] for observation in observations):
                # Add to list of destroyed ids
                destroyed_ids.append(id)

        # create a copy to iterate over
        destroyed_ids_copy = destroyed_ids.copy()
        
        # iterate over destroyed meteorite list and remove from dict
        for id in destroyed_ids_copy:
            if id in self.targets:
                if self.current_target is not None:
                    if self.current_target['id'] == id:
                        self.current_target = None
                del self.targets[id]
                # print(f'{id} destroyed')
                destroyed_ids.remove(id)

        # print(f'Time: {self.time} | {self.targets[self.test_meteorite_id]=}')
        # print(f'Time: {self.time} | {self.targets[self.test_meteorite_id]["P"][0][0]} | {self.targets[self.test_meteorite_id]["P"][1][1]}')
        
        # print(target_locations)
        return target_locations

    def get_laser_action(self, current_aim_rad):
        # wait until time 3 until x matrix is composed
        if self.time > 2 and len(self.targets) != 0:
            self.find_target(current_aim_rad)

            # rads from current aim to target
            try:
                print(self.current_target)
                rads_to_target = self.current_target['angle'] - current_aim_rad
            except TypeError:
                rads_to_target = math.pi/2 - current_aim_rad
                print("No current targets within range")
            except KeyError:
                rads_to_target = math.pi/2 - current_aim_rad
                print("No current targets within range")

            # print(f"CurAim: {current_aim_rad} | TarAng: {self.current_target['angle']}| AngDif: {rads_to_target} | Max: {self.max_angle_change}")

            fire = False

            if self.max_angle_change <= rads_to_target:
                angle_change_rad = self.max_angle_change
            elif rads_to_target <= -self.max_angle_change:
                angle_change_rad = -self.max_angle_change
            elif -self.max_angle_change < rads_to_target < self.max_angle_change:
                angle_change_rad = rads_to_target
                if self.current_target is not None:
                    if self.target_within_range(self.current_target['x'][0], self.current_target['x'][1]):
                        fire = True

        else:
            angle_change_rad = 0
            fire = False

        return angle_change_rad, fire
    
    def find_target(self, current_aim_rad):
        # iterate over all meteorite observations
        print(f'{len(self.observations)} meteorites observed')
        for observation in self.observations:
            # ignore observations with id -1 (destroyed)
            if observation[0] != -1 and observation[0] in self.targets.keys():
                # only consider targets that have 'x' recorded
                if self.targets[observation[0]]['x'] is not None:
                    obs_id, obs_x, obs_y = observation[0], observation[1], observation[2]  

                    if self.current_target is None:
                        self.current_target = self.targets[obs_id]
                        self.current_target['id'] = obs_id
                
                    # refer to current target's state
                    current_target_x_pos = self.current_target['x'][0].item()
                    current_target_y_pos = self.current_target['x'][1].item()
                    current_target_x_vel = self.current_target['x'][2].item()
                    current_target_y_vel = self.current_target['x'][3].item()
                    target_future_x_pos = current_target_x_pos + current_target_x_vel * self.dt
                    target_future_y_pos = current_target_y_pos + current_target_y_vel * self.dt
                    self.current_target['angle'] = math.atan2((target_future_y_pos+ 1), target_future_x_pos)
                    # calculate impact time - consider ground is at -1
                    current_target_impact_time = current_target_y_vel / (target_future_y_pos + 1)

                    print(f"TarId: {self.current_target['id']} | x: {current_target_x_pos} | y: {current_target_y_pos}")

                    # remove target if it will go out of range
                    if self.target_within_range(target_future_x_pos, target_future_y_pos) == False or target_future_y_pos < -1:
                        self.current_target = None

                    # # refer to current target's y position
                    # tar_bel_y_pos = self.current_target['x'][1]
                    
                    # calculate impact time for current observation
                    print(f'Is ID in targets?: {obs_id in self.targets.keys()}')
                    print(f'{self.targets[obs_id]=}')
                    obs_y_vel = self.targets[obs_id]['x'][3].item()
                    # calculate impact time - consider ground is at -1
                    obs_impact_time = obs_y_vel / (obs_y + 1)

                    if obs_impact_time < current_target_impact_time:
                    # if -1 < obs_y < current_target_y_pos and self.target_within_range(obs_x, obs_y):
                        
                        # make this observation current target
                        self.current_target = self.targets[obs_id]
                        self.current_target['id'] = obs_id

        