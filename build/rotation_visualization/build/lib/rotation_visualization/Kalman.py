import numpy as np
from scipy.linalg import block_diag

class QuaternionKalmanFilter:

        
    dt = 1/37 # Time step
    Q = np.eye(4) * 1  # Process noise covariance (example)
    R = np.eye(4) * 1
    state = np.array([1, 0, 0, 0], dtype=np.float64)  # Initial quaternion (w, x, y, z)
    P = np.eye(4)  # Initial state covariance

    def predict(self, gyro_meas):
        gyro_meas = np.radians(gyro_meas)
        #Body frame gyro measurements to a quaternion
        omega = np.hstack(([0], gyro_meas))

        #Quaternion frame rates
        q_dot = 0.5 * self.quaternion_multiply(self.state, omega)
        
        # Integrate to predict new state
        self.state += q_dot * self.dt
        self.state /= np.linalg.norm(self.state)  # Normalize the quaternion
        
        # Predict new covariance
        F = np.eye(4)  # Approximation of state transition matrix
        self.P = F @ self.P @ F.T + self.Q

    def update(self, q_meas):
        
        z = q_meas
        H = np.eye(4)  # Measurement model
        y = z - H @ self.state  # Measurement residual
        
        S = H @ self.P @ H.T + self.R  # Residual covariance
        K = self.P @ H.T @ np.linalg.inv(S)  # Kalman gain
        
        self.state += K @ y  # Update state
        self.state /= np.linalg.norm(self.state)  # Normalize the quaternion
        
        I = np.eye(4)
        self.P = (I - K @ H) @ self.P  # Update covariance

    @staticmethod
    def quaternion_multiply(q, r):
        
        w0, x0, y0, z0 = q
        w1, x1, y1, z1 = r
        return np.array([
            -x0*x1 - y0*y1 - z0*z1 + w0*w1,
             x0*w1 + y0*z1 - z0*y1 + w0*x1,
            -x0*z1 + y0*w1 + z0*x1 + w0*y1,
             x0*y1 - y0*x1 + z0*w1 + w0*z1,
        ], dtype=np.float64)