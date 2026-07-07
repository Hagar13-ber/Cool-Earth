import time
import math
import numpy as np
import board
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

# --- Mathematical Helper Functions for Quaternions and Vectors ---

def quat_normalize(q):
    q = np.asarray(q, dtype=float)
    return q / np.linalg.norm(q)

def quat_mul(q1, q2):
    q1 = np.asarray(q1, dtype=float)
    q2 = np.asarray(q2, dtype=float)
    s1, v1 = q1[0], q1[1:4]
    s2, v2 = q2[0], q2[1:4]
    s = s1 * s2 - np.dot(v1, v2)
    v = s1 * v2 + s2 * v1 + np.cross(v1, v2)
    return np.concatenate(([s], v))

def quat2rotm(q):
    q = np.asarray(q, dtype=float)
    w, x, y, z = q[0], q[1], q[2], q[3]
    # Rotation matrix from body to inertial frame (fully compatible with MATLAB)
    return np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - z*w),        2*(x*z + y*w)       ],
        [2*(x*y + z*w),        1 - 2*(x**2 + z**2),  2*(y*z - x*w)       ],
        [2*(x*z - y*w),        2*(y*z + x*w),         1 - 2*(x**2 + y**2)]
    ])


class SolarSailSingleStepExecution:
    def __init__(self):
        # ---------------------------------------------------
        # Hardware Configuration (PCA9685 over I2C)
        # ---------------------------------------------------
        self.i2c = board.I2C()
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = 50

        self.fins = [
            servo.Servo(self.pca.channels[0], min_pulse=500, max_pulse=2400),
            servo.Servo(self.pca.channels[1], min_pulse=500, max_pulse=2400),
            servo.Servo(self.pca.channels[2], min_pulse=500, max_pulse=2400),
            servo.Servo(self.pca.channels[3], min_pulse=500, max_pulse=2400)
        ]

        # ---------------------------------------------------
        # Geometric Parameters and Control Gains
        # ---------------------------------------------------
        self.Kp = 2.0e-3
        self.Kd = 0.5
        self.bias = 45.0          # Operational bias fixed at 45 degrees
        self.cg_max = 0.5         # Maximum physical CoM shift [m]
        self.A_main = 100.0       # Main sail area [m^2]
        self.P_srp = 9.0e-6       # Solar Radiation Pressure at L1 [N/m^2]
        self.sigma = 1.8          # Reflectivity coefficient
        self.L = 7.0              # Moment arm / Boom length [m]

        self.J  = np.array([50.0, 50.0, 100.0])  # Moments of Inertia (X, Y, Z axes)
        self.dt = 0.5                              # Integration time step [seconds]

    def set_vane_angles(self, angles):
        """Physically commands the 4 servo motors with strict saturation bounds."""
        safe_angles = np.clip(np.asarray(angles, dtype=float), 5.0, 85.0)
        for i in range(4):
            self.fins[i].angle = float(safe_angles[i])

    def execute_mixer(self, tau_req, scale):
        """Mixer Matrix: Transforms generalized torque demand into differential vane angles."""
        u = np.asarray(tau_req, dtype=float) * scale
        delta = np.array([
            -u[1] + u[2] * 2.0,
             u[0] + u[2] * 2.0,
             u[1] + u[2] * 2.0,
            -u[0] + u[2] * 2.0
        ])
        return self.bias + delta

    def run_step(self, q_initial, target_inertial, sun_inertial=None):
        """Executes a single control cycle and physics propagation step based on a defined state."""
        if sun_inertial is None:
            sun_inertial = np.array([0.0, 0.0, 1.0])

        # 1. Enforce initial angular velocity to zero as required
        w_curr = np.zeros(3)
        q_curr = quat_normalize(q_initial)

        # 2. Transform reference vectors to body coordinate system
        R_b2i = quat2rotm(q_curr)
        R_i2b = R_b2i.T

        S_body = R_i2b @ np.asarray(sun_inertial, dtype=float)
        T_body = R_i2b @ np.asarray(target_inertial, dtype=float)
        Z_body = np.array([0.0, 0.0, 1.0])

        # 3. Calculate pointing error components and magnitudes
        err_vec = np.cross(Z_body, T_body)
        err_deg = math.acos(float(np.clip(np.dot(Z_body, T_body), -1.0, 1.0))) * (180.0 / math.pi)

        # 4. State Machine and steering command allocation
        r_cg = np.zeros(3)

        if err_deg > 50.0:
            mode_str = "LARGE ANGLE (CoM + FINS)"
            scale_factor = 10000

            # Raw required control torque from tracking error
            tau_req = -self.Kp * err_vec - self.Kd * w_curr

            # Active Center of Mass (CoM) shifting allocation
            cos_theta = np.dot(S_body, Z_body)
            F_z_mag = -self.sigma * self.P_srp * self.A_main * (max(cos_theta, 0.05)**2)

            r_cg[0] = np.clip(-tau_req[1] / F_z_mag, -self.cg_max, self.cg_max)
            r_cg[1] = np.clip( tau_req[0] / F_z_mag, -self.cg_max, self.cg_max)

            alpha_cmd = self.execute_mixer(tau_req, scale_factor)
        else:
            mode_str = "NOMINAL (FINS ONLY)"
            scale_factor = 25000

            tau_req = -self.Kp * err_vec - self.Kd * w_curr

            alpha_cmd = self.execute_mixer(tau_req, scale_factor)

        # 5. Issue physical angle commands to hardware actuators
        self.set_vane_angles(alpha_cmd)

        # 6. Physics environment model for single-step integration
        cos_theta_main = np.dot(S_body, Z_body)
        F_z_main = -self.sigma * self.P_srp * self.A_main * (max(cos_theta_main, 0.05)**2)

        # Real-time torque calculation based on active inputs
        tau_applied = np.array([
            r_cg[1] * F_z_main  + (alpha_cmd[1] - alpha_cmd[3]) * 5e-6,
            -r_cg[0] * F_z_main + (alpha_cmd[2] - alpha_cmd[0]) * 5e-6,
            (np.sum(alpha_cmd) - 180.0) * 1e-6
        ])

        # Angular acceleration derivation
        w_dot = tau_applied / self.J

        # Update angular velocity via Euler forward step
        w_next = w_curr + w_dot * self.dt

        # Quaternion kinematics propagation
        q_dot = 0.5 * quat_mul(q_curr, np.concatenate(([0.0], w_curr)))
        q_final = quat_normalize(q_curr + q_dot * self.dt)

        # 7. Print summary telemetry to the terminal
        print("\n====================================================")
        print(f"      EXECUTION MODE: {mode_str}")
        print("====================================================")
        print(f"Initial Error: {err_deg:.2f} degrees")
        print(f"Vane Angles Commanded (Fins 1-4): {np.round(alpha_cmd, 2).tolist()} [deg]")
        print(f"CoM Shift Applied: X={r_cg[0]:.3f} m, Y={r_cg[1]:.3f} m")
        print(f"Calculated Torques: X={tau_applied[0]:.2e}, Y={tau_applied[1]:.2e}, Z={tau_applied[2]:.2e} [Nm]")
        print(f"Output Final Quaternion: [{', '.join([f'{x:.6f}' for x in q_final])}]")
        print("====================================================\n")

        return q_final


if __name__ == "__main__":
    # Define inertial target orientation (pointing directly at the Sun vector)
    target_vector = np.array([0.0, 0.0, 1.0])

    # Define initial attitude with exactly 5 degrees of tracking error around the X-axis
    # Quaternion format: [cos(ang/2), sin(ang/2)*ax, sin(ang/2)*ay, sin(ang/2)*az]
    ang_test = math.radians(5)
    q_start = np.array([math.cos(ang_test / 2), math.sin(ang_test / 2), 0.0, 0.0])

    # Instantiate flight computer module and run a single integration step
    sail_computer = SolarSailSingleStepExecution()
    q_end = sail_computer.run_step(q_initial=q_start, target_inertial=target_vector)

    # Safely de-initialize I2C and release hardware channels
    try:
        sail_computer.pca.deinit()
        print("[FSW] Hardware bus released cleanly.")
    except Exception as e:
        print(f"[Warning] Failed to release I2C bus: {e}")

