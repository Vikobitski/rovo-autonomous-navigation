import numpy as np

class PIDController:
    def __init__(self, params, logger):
        self.p = params # Configuration struct/dict
        self.logger = logger
        
        # Persistent States (Memory)
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_d_term = 0.0
        self.last_v = 0.0
        self.last_w = 0.0
        self.last_closest_idx = 0
        self.finished = False

    def compute(self, current_pose, path_points, dt):
        if self.finished:
            return 0.0, 0.0

        x_rob, y_rob, theta = current_pose
        path_x, path_y = path_points[:, 0], path_points[:, 1]
        n_points = len(path_x)

        # 1. Sliding Window Search (Prevents jumping back in closed loops)
        search_start = self.last_closest_idx
        search_end = min(search_start + 50, n_points)
        
        dists = np.sqrt((path_x[search_start:search_end] - x_rob)**2 + 
                        (path_y[search_start:search_end] - y_rob)**2)
        local_min_idx = np.argmin(dists)
        closest_idx = search_start + local_min_idx
        self.last_closest_idx = closest_idx

        # 2. Lookahead Logic
        dist_to_end = np.sqrt((path_x[-1] - x_rob)**2 + (path_y[-1] - y_rob)**2)
        has_progressed = closest_idx > (0.8 * n_points)

        if has_progressed and dist_to_end <= self.p['lookahead_dist']:
            x_target, y_target = path_x[-1], path_y[-1]
            target_idx = n_points - 1
        else:
            target_idx = closest_idx
            for k in range(closest_idx, n_points):
                d = np.sqrt((path_x[k] - x_rob)**2 + (path_y[k] - y_rob)**2)
                if d >= self.p['lookahead_dist']:
                    target_idx = k
                    break
            x_target, y_target = path_x[target_idx], path_y[target_idx]

        # 3. Heading Error & PID
        target_yaw = np.arctan2(y_target - y_rob, x_target - x_rob)
        error_yaw = np.arctan2(np.sin(target_yaw - theta), np.cos(target_yaw - theta))

        # if self.logger:
        #     self.logger.info(f"Error Yaw: {error_yaw}") 
        #     self.logger.info(f"Target Yaw: {target_yaw}")
        
        if abs(error_yaw) < np.radians(1.5): error_yaw = 0.0 # Deadband

        # PID Terms
        p_term = self.p['Kp'] * error_yaw
        self.integral = np.clip(self.integral + (error_yaw * dt), -self.p['I_max'], self.p['I_max'])
        d_raw = (error_yaw - self.prev_error) / dt
        alpha = 0.3
        d_filtered = alpha * d_raw + (1 - alpha) * self.prev_d_term
        self.prev_d_term = d_filtered
        self.prev_error = error_yaw

        w_target = p_term + (self.p['Ki'] * self.integral) + (self.p['Kd'] * d_filtered)

        # 4. Motion Strategy & Arrival
        if has_progressed and dist_to_end < self.p['dist_thresh']:
            self.finished = True
            return 0.0, 0.0

        if has_progressed and target_idx == (n_points - 1):
            w_target = 0.0 # Straight parking
            v_target = np.clip(self.p['v_des'] * (dist_to_end / self.p['lookahead_dist']), 0.05, self.p['v_des'])
        else:
            if abs(error_yaw) > np.radians(10): # Pivot Turn
                v_target = 0.0
                w_target = np.sign(w_target) * max(abs(w_target), 0.8)
            else: # Cruise
                v_target = self.p['v_des'] * np.cos(error_yaw)

        # 5. Acceleration Limiting (Ramps)
        v_final = self.last_v + np.clip(v_target - self.last_v, -self.p['accel_lin_max']*dt, self.p['accel_lin_max']*dt)
        w_ramp = self.last_w + np.clip(w_target - self.last_w, -self.p['accel_ang_max']*dt, self.p['accel_ang_max']*dt)
        w_final = 0.6 * w_ramp + 0.4 * self.last_w # Butter smoothing
        
        self.last_v, self.last_w = v_final, w_final
        return np.clip(v_final, -self.p['v_max'], self.p['v_max']), np.clip(w_final, -self.p['w_sat'], self.p['w_sat'])