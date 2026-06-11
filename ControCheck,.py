import time
import math
import board
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

print("Initializing Cool Earth ADCS Test with PCA9685...")

# =======================================================
# 1. Hardware Initialization (From your working code)
# =======================================================
i2c = board.I2C()  # uses board.SCL and board.SDA
pca = PCA9685(i2c)
pca.frequency = 50

# Initialize the 4 servos on channels 0, 1, 2, 3
# Min and Max pulse tuned for the Micro Servos you tested
servo1 = servo.Servo(pca.channels[0], min_pulse=500, max_pulse=2400)
servo2 = servo.Servo(pca.channels[1], min_pulse=500, max_pulse=2400)
servo3 = servo.Servo(pca.channels[2], min_pulse=500, max_pulse=2400)
servo4 = servo.Servo(pca.channels[3], min_pulse=500, max_pulse=2400)

# =======================================================
# 2. ADCS Control Parameters (PID)
# =======================================================
Kp = 1.5
Ki = 0.05
Kd = 0.5
max_integral = 20.0

error_integral = 0.0
previous_error = 0.0

# =======================================================
# 3. ADCS Helper Functions
# =======================================================
def set_all_fins(angle):
    """
    Sets all 4 fins to the same angle, strictly within 
    the aerodynamic saturation limits (5 to 85 degrees).
    """
    safe_angle = max(5, min(85, angle))
    servo1.angle = safe_angle
    servo2.angle = safe_angle
    servo3.angle = safe_angle
    servo4.angle = safe_angle

def correct_mixer(tau_x, tau_y, tau_z):
    """
    Mixer Matrix: Converts calculated torque commands 
    (from the PID) into differential fin angles.
    """
    bias = 45.0
    scale = 2.0  # Conversion scale (Torque to Angle)
    
    u_x = tau_x * scale
    u_y = tau_y * scale
    u_z = tau_z * scale

    # Decoupling logic and Z-axis multiplication (Pinwheel configuration)
    delta1 = -u_y + (u_z * 2.0)
    delta2 =  u_x + (u_z * 2.0)
    delta3 =  u_y + (u_z * 2.0)
    delta4 = -u_x + (u_z * 2.0)

    # Apply physical limits (5 to 85 degrees) and write directly to servos
    servo1.angle = max(5, min(85, bias + delta1))
    servo2.angle = max(5, min(85, bias + delta2))
    servo3.angle = max(5, min(85, bias + delta3))
    servo4.angle = max(5, min(85, bias + delta4))

# =======================================================
# 4. Main Terminal Loop (Dry Run Mode)
# =======================================================
def main():
    global error_integral, previous_error
    
    print("\n--- Project Cool Earth: Hardware-in-the-Loop Dry Run ---")
    print("Moving all fins to nominal Bias (45 degrees)...")
    set_all_fins(45.0)
    time.sleep(2)
    
    try:
        while True:
            # Wait for manual error input from the user via terminal
            user_input = input("\nEnter angle error (in degrees) for the Z-axis, or press Ctrl+C to exit: ")
            error = float(user_input)
            
            # ----------------------------------------
            # Rescue Mode Check (Shake / Open Loop)
            # ----------------------------------------
            if abs(error) > 50.0:
                print(f"[!] Error {error} > 50: Activating RESCUE SHAKE mode for 5 seconds...")
                start_time = time.time()
                
                # Run the sine wave for 5 seconds
                while time.time() - start_time < 5.0:
                    t = time.time()
                    # Sine wave generator: Amp = 40 deg, Freq = 0.8 rad/s
                    shake_val = 40.0 * math.sin(0.8 * t)
                    set_all_fins(45.0 + shake_val)
                    time.sleep(0.02) # Hardware refresh rate
                
                error_integral = 0.0 # Reset integrator to prevent windup after shaking
                print("[V] Shake completed. Motors returning to 45 degrees.")
                set_all_fins(45.0)
                
            # ----------------------------------------
            # Static Stabilization Check (PID Loop)
            # ----------------------------------------
            else:
                print(f"[*] Error {error} <= 50: Activating normal PID control.")
                dt = 0.1 # Simulated time step
                rate = (error - previous_error) / dt
                
                # Update and constrain the integral component
                error_integral += error * dt
                error_integral = max(-max_integral, min(max_integral, error_integral))
                
                # Calculate required torque around the Z-axis
                tau_z = (Kp * error) + (Ki * error_integral) + (Kd * rate)
                print(f"    -> Calculated Torque (Tau Z): {tau_z:.2f}")
                
                # Feed the torque command to the mixer (X=0, Y=0, Z=tau_z)
                correct_mixer(0, 0, tau_z)
                previous_error = error

    except ValueError:
        print("Error: Please enter numbers only.")
    except KeyboardInterrupt:
        print("\n\nEnding test. Returning the system to the nominal angle (45 degrees)...")
        set_all_fins(45.0)
        time.sleep(1)
        pca.deinit() # Safely de-initialize the I2C board and release pins

if __name__ == "__main__":
    main()