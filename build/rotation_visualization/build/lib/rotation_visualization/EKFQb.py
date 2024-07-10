import numpy as np

class QuaternionKalmanFilter:

    def __init__(self):
        self.dt = 1/37  # Time step
        self.Q = np.eye(7) * 1  # Process noise covariance
        self.Q[4:, 4:] = np.eye(3) * 1  # Smaller noise for biases
        self.R = np.eye(7) * 0.1  # Measurement noise covariance
        self.state = np.array([1, 0, 0, 0, 0.3, 0.2, 0.1], dtype=np.float64)  # Initial quaternion (w, x, y, z) and biases
        self.P = np.eye(7)  # Initial state covariance
        self.static_past = np.array([1, 0, 0, 0])

    def predict(self, gyro_meas):
        time_elpased = 1/37
        omega = np.hstack(([0], [gyro_meas[0] - self.state[4]], [gyro_meas[1] - self.state[5]], [gyro_meas[2] - self.state[6]]))
        q_dot = 0.5 * self.quaternion_multiply(self.state[:4], omega)
        q_dot = np.pad(q_dot, (0, 3), 'constant')  # Pad q_dot to match the size of the state vector

        self.state_past = self.state.copy()
        self.state[:4] += q_dot[:4] * time_elpased
        self.state[:4] /= np.linalg.norm(self.state[:4])  # Normalize the quaternion

        F = np.eye(7)  # State transition matrix
        self.P = F @ self.P @ F.T + self.Q

    def update(self, q_meas, gyro):
        time_elpased = 1/37
        rate_estimate = np.array(q_meas - self.static_past)
        self.static_past = q_meas
        rate_estimate /= time_elpased
        rate_estimate = self.quaternion_to_body_rates(self.state[:4], rate_estimate)
        
        bias = gyro - rate_estimate

        print(f"Estimates: Rate Estimate - {rate_estimate} Bias Estimate - {bias}")
        z = np.hstack([q_meas, bias])

        H = np.eye(7)  # Measurement model
        y = z - H @ self.state  # Measurement residual
        
        S = H @ self.P @ H.T + self.R  # Residual covariance
        K = self.P @ H.T @ np.linalg.inv(S)  # Kalman gain

        self.state += K @ y  # Update state
        self.state[:4] /= np.linalg.norm(self.state[:4])  # Normalize the quaternion
        
        I = np.eye(7)
        self.P = (I - K @ H) @ self.P  # Update covariance

    @staticmethod
    def quaternion_multiply(q, r):
        w0, x0, y0, z0 = q[:4]
        w1, x1, y1, z1 = r
        return np.array([
            -x0*x1 - y0*y1 - z0*z1 + w0*w1,
            x0*w1 + y0*z1 - z0*y1 + w0*x1,
            -x0*z1 + y0*w1 + z0*x1 + w0*y1,
            x0*y1 - y0*x1 + z0*w1 + w0*z1,
        ], dtype=np.float64)

    def quaternion_to_body_rates(self, q, q_dot):
        q = np.asarray(q)
        q_dot = np.asarray(q_dot)
        
        omega = 2 * np.array([
            -q[1] * q_dot[0] - q[2] * q_dot[1] - q[3] * q_dot[2] + q[0] * q_dot[3],
            q[0] * q_dot[0] + q[1] * q_dot[3] + q[2] * q_dot[2] - q[3] * q_dot[1],
            q[0] * q_dot[1] - q[1] * q_dot[2] + q[2] * q_dot[3] + q[3] * q_dot[0],
            q[0] * q_dot[2] + q[1] * q_dot[1] - q[2] * q_dot[0] + q[3] * q_dot[3]
        ])
        return omega[1:]
