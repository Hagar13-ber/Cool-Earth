import os
import sys
import time
import csv
import subprocess
import board
import busio
import smbus2
import RPi.GPIO as GPIO
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from gpiozero import LED

# ==========================================
#      OPNAV VISUALIZATION SETTINGS
# ==========================================
# Set OPNAV_SHOW_MATCHED_STARS to True to display a matched-stars popup
# after every OpNav pipeline call during the NAVIGATION state.
# This overrides config.PROC_SHOW_MATCHED_STARS_POPUP in Cool-Earth-NAV/config.py.
OPNAV_SHOW_MATCHED_STARS = False  # <-- change here to enable

# ==========================================
#         HARDWARE SETUP FUNCTIONS
# ==========================================

def setup_servos():
    """Initializes the I2C bus and the PCA9685 for servo control."""
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c)
    pca.frequency = 50
    # Optional: Initializing 4 channels as defined in your setup
    servos = [servo.Servo(pca.channels[i]) for i in range(4)]
    return pca, servos

def setup_dc_motor():
    """Initializes the GPIO pins and PWM for the DC motor."""
    if os.geteuid() != 0:
        print("Error: This script requires root privileges to access GPIO (/dev/mem). Run with sudo.")
        sys.exit(1)
        
    in1 = 24
    in2 = 23
    en = 25
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(in1, GPIO.OUT)
    GPIO.setup(in2, GPIO.OUT)
    GPIO.setup(en, GPIO.OUT)
    
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    
    p = GPIO.PWM(en, 1000)
    p.start(0) # Start with 0% duty cycle
    
    return p, in1, in2

def setup_leds():
    """Initializes the LEDs using gpiozero based on the PDF pinout."""
    leds = {
        'deployment': LED(27),
        'navigation': LED(26),
        'adcs': LED(22),
        'nominal': LED(16)
    }
    return leds

# --- IMU Configuration ---
L3GD20H_ADDR = 0x6B
L3G_CTRL1 = 0x20
L3G_OUT_X_L = 0x28

# Navigation status mapping (kept in 1/2/3 format):
# 1 = bad, 2 = degraded, 3 = good.
NAV_STATUS_BAD = 1
NAV_STATUS_DEGRADED = 2
NAV_STATUS_GOOD = 3

def setup_imu():
    """Initializes the I2C bus for the IMU and wakes up the sensor."""
    bus = smbus2.SMBus(1)
    # Enable X, Y, Z axes, set to Normal Mode
    bus.write_byte_data(L3GD20H_ADDR, L3G_CTRL1, 0x0F)
    return bus

def read_raw_axis(bus, addr, reg):
    """Helper function to read a 16-bit value from the IMU."""
    low = bus.read_byte_data(addr, reg)
    high = bus.read_byte_data(addr, reg + 1)
    value = (high << 8) | low
    if value > 32767:
        value -= 65536
    return value

def read_gyro(bus):
    """Reads the X, Y, and Z axes from the IMU."""
    x = read_raw_axis(bus, L3GD20H_ADDR, L3G_OUT_X_L)
    y = read_raw_axis(bus, L3GD20H_ADDR, L3G_OUT_X_L + 2)
    z = read_raw_axis(bus, L3GD20H_ADDR, L3G_OUT_X_L + 4)
    return x, y, z


def _capture_frame(frame_index=0):
    """
    Capture one frame for NAVIGATION.

    Two implementation sections are kept here on purpose:
    1) Real camera capture path (stub for future RT hardware integration).
    2) Scenario CSV fallback path (current placeholder using pre-saved FITS files).
    """
    use_real_capture = False

    # Section 1: Real camera capture path (future implementation).
    if use_real_capture:
        # TODO: Add actual camera capture logic.
        # Expected behavior:
        # - Capture one frame from the onboard camera
        # - Save it under Cool-Earth-NAV/data/captured/
        # - Return the saved FITS path
        pass

    # Section 2: Scenario CSV fallback (current placeholder behavior).
    project_root = os.path.dirname(os.path.abspath(__file__))
    scenarios_csv = os.path.join(project_root, "Cool-Earth-NAV", "data", "scenarios.csv")
    captured_dir = os.path.join(project_root, "Cool-Earth-NAV", "data", "captured")

    frame_paths = []
    try:
        with open(scenarios_csv, 'r', newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                filename = (row.get('filename') or '').strip()
                if filename:
                    frame_paths.append(os.path.join(captured_dir, filename))
    except Exception as e:
        print(f"Warning: Could not load scenarios.csv ({e}).")
        return None

    if not frame_paths:
        print("Warning: scenarios.csv loaded but no valid frame filenames were found.")
        return None

    frame_path = frame_paths[frame_index % len(frame_paths)]
    print(f"Captured frame: {frame_path}")
    return frame_path


def _run_opnav_navigation_once(frame_path):
    """
    Runs one non-interactive OpNav attempt and maps result to nav_status semantics.

    Intended purpose:
    Use camera-based OpNav measurements to calibrate/estimate gyro drift quality.

    Current placeholder behavior:
    - GOOD (3): pipeline execution succeeded.
    - DEGRADED (2): pipeline ran but did not reach full quality (for example, too few matches).
    - BAD (1): pipeline failed or call failed.
    """
    try:
        project_root = os.path.dirname(os.path.abspath(__file__))
        opnav_dir = os.path.join(project_root, "Cool-Earth-NAV")
        if opnav_dir not in sys.path:
            sys.path.insert(0, opnav_dir)

        import process_star_image

        result = process_star_image.run_single_image_pipeline(
            image_path=frame_path,
            show_plot=False,
            verbose=False,
            # t: seconds since reference epoch (used to compute ecliptic longitude)
            # lambda_0: initial ecliptic longitude in DEGREES (converted to radians internally)
            t=0.0,
            lambda_0=0.0,
            # Matched-stars popup — controlled by OPNAV_SHOW_MATCHED_STARS above
            show_matched_stars_popup=OPNAV_SHOW_MATCHED_STARS,
        )
        if result.get('status') == 'ok':
            return NAV_STATUS_GOOD, result

        if result.get('message') == 'not_enough_matches':
            return NAV_STATUS_DEGRADED, result

        return NAV_STATUS_BAD, result

    except Exception as e:
        return NAV_STATUS_BAD, {'status': 'failed', 'message': f'opnav_call_failed: {e}'}

# ==========================================
#           STATE MACHINE LOGIC
# ==========================================

def main():
    print("Setting up hardware connections...")
    
    # Run setup functions
    pca, servos = setup_servos()
    dc_pwm, dc_in1, dc_in2 = setup_dc_motor()
    leds = setup_leds()
    imu_bus = setup_imu()
    
    # Initialize state variable
    state = "INIT"
    
    try:
        while state != "DONE":
            
            if state == "INIT":
                print("--- STATE: INITIALIZATION ---")
                consecutive_count = 0
                
                # Check IMU until condition met for 50 straight samples
                while consecutive_count < 50:
                    x, y, z = read_gyro(imu_bus)
                    magnitude_squared = (x**2) + (y**2) + (z**2)
                    
                    if magnitude_squared <= 10000:  # 10^4
                        consecutive_count += 1
                    else:
                        consecutive_count = 0
                        
                    # Small delay to prevent I2C bus flooding
                    time.sleep(0.01) 
                    
                print("IMU condition verified for 50 samples.")
                state = "DEPLOYMENT"
                
            elif state == "DEPLOYMENT":
                print("--- STATE: DEPLOYMENT ---")
                
                # Activate GPIO27 LED for 2 seconds
                leds['deployment'].on()
                time.sleep(2)
                leds['deployment'].off()
                
                # Activate DC Motor for 2 seconds on 50% duty cycle
                GPIO.output(dc_in1, GPIO.HIGH)
                GPIO.output(dc_in2, GPIO.LOW)
                dc_pwm.ChangeDutyCycle(50)
                time.sleep(2)
                
                # Stop motor
                dc_pwm.ChangeDutyCycle(0)
                GPIO.output(dc_in1, GPIO.LOW)
                
                state = "NAVIGATION"
                
            elif state == "NAVIGATION":
                print("--- STATE: NAVIGATION ---")
                
                # Activate GPIO26 LED for 2 seconds
                leds['navigation'].on()
                time.sleep(2)
                leds['navigation'].off()
                
                nav_status = NAV_STATUS_BAD
                frame_index = 0
                # Placeholder loop purpose:
                # keep running OpNav camera measurements until navigation quality is
                # good enough to represent successful gyro-drift calibration input.
                while nav_status != NAV_STATUS_GOOD:
                    frame_path = _capture_frame(frame_index)
                    if not frame_path:
                        nav_status = NAV_STATUS_BAD
                        print("OpNav skipped: no captured frame available.")
                        frame_index += 1
                        time.sleep(1)
                        continue

                    nav_status, nav_result = _run_opnav_navigation_once(frame_path)
                    print(
                        "OpNav result: "
                        f"status={nav_result.get('status')} "
                        f"message={nav_result.get('message')} "
                        f"matched={nav_result.get('num_matched')} "
                        f"nav_status={nav_status}"
                    )
                    frame_index += 1
                    time.sleep(1)
                    
                print("Nav status is GOOD. Proceeding to ADCS.")
                state = "ADCS"
                
            elif state == "ADCS":
                print("--- STATE: ADCS ---")
                
                # Activate GPIO22 LED for 2 seconds
                leds['adcs'].on()
                time.sleep(2)
                leds['adcs'].off()
                
                # Activate external script
                print("Activating ServoRun.py...")
                try:
                    subprocess.run(["python3", "ServoRun.py"], check=True)
                except Exception as e:
                    print(f"Warning: Issue running ServoRun.py: {e}")
                
                state = "NOMINAL"
                
            elif state == "NOMINAL":
                print("--- STATE: TRANSITION TO NOMINAL ---")
                
                # Activate GPIO16 LED for 2 seconds
                leds['nominal'].on()
                time.sleep(2)
                leds['nominal'].off()
                
                print("State machine execution finished.")
                state = "DONE"

    except KeyboardInterrupt:
        print("\nProcess interrupted by user.")
        
    finally:
        # Cleanup routine
        print("Cleaning up GPIO resources...")
        dc_pwm.stop()
        GPIO.cleanup()
        # LEDs managed by gpiozero are generally cleaned up automatically 
        # on exit, but we close them for best practice.
        for led in leds.values():
            led.close()

if __name__ == "__main__":
    main()