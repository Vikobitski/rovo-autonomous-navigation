import numpy as np

class GeoFilter:
    def __init__(self, r_wheel=0.033, l_base=0.160):
        """
        Initializes the State Estimator.
        :param r_wheel: Wheel radius in meters (default TurtleBot3)
        :param l_base: Distance between wheels in meters
        """
        self.r_wheel = r_wheel
        self.l_base = l_base
        
        self.ref_lat = None
        self.ref_lon = None
        self.x_prev = 0.0
        self.y_prev = 0.0
        self.yaw_prev = 0.0
        self.is_initialized = False

    def update(self, lat, lon, w_left, w_right, ts, tau, use_filter=True, reset=False):
        """
        Updates the estimate based on new GPS and Encoder data.
        """
        # 1. Initialization / Reset Logic
        if reset or not self.is_initialized:
            self.ref_lat = lat
            self.ref_lon = lon
            self.x_prev, self.y_prev, self.yaw_prev = 0.0, 0.0, 0.0
            self.is_initialized = True
            return 0.0, 0.0, 0.0

        # 2. Geo to Cartesian Conversion (Local Approximation)
        r_earth = 6378137.0
        d_lat_rad = np.radians(lat - self.ref_lat)
        d_lon_rad = np.radians(lon - self.ref_lon)
        lat_rad = np.radians(self.ref_lat)

        x_raw = r_earth * d_lon_rad * np.cos(lat_rad)
        y_raw = r_earth * d_lat_rad

        # 3. PT1 (Low Pass) Filter Implementation
        if use_filter:
            alpha = ts / (ts + tau)
            x_filt = self.x_prev + alpha * (x_raw - self.x_prev)
            y_filt = self.y_prev + alpha * (y_raw - self.y_prev)
            self.x_prev, self.y_prev = x_filt, y_filt
        else:
            self.x_prev, self.y_prev = x_raw, y_raw

        # 4. Differential Drive Kinematics (Dead Reckoning for Yaw)
        v_left = w_left * self.r_wheel
        v_right = w_right * self.r_wheel
        omega_robot = (v_right - v_left) / self.l_base
        
        # Integrate Yaw using Forward Euler
        new_yaw = self.yaw_prev + omega_robot * ts
        self.yaw_prev = np.arctan2(np.sin(new_yaw), np.cos(new_yaw)) # Normalize to [-pi, pi]

        return self.x_prev, self.y_prev, self.yaw_prev
