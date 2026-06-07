"""
CoolEarth Project Spring 2026
Authors: Liran & Hagar
Description: State Machine Initialization and Deployment Code translated from MATLAB.
"""

import subprocess
import platform
import time
import RPi.GPIO as GPIO
from gpiozero import LED

# ==============================================================================
# HARDWARE CONFIGURATION & GPIO PIN ASSIGNMENTS (CHANGE THESE WHEN WIRED)
# ==============================================================================

# LED Pins (BCM numbering)
# TODO: Connect LEDs to these pins and update if necessary
PIN_LED_YELLOW = 17   # Mode 1 - Yellow
PIN_LED_GREEN = 27    # Mode 2 - Green
PIN_LED_ORANGE = 22   # Mode 3 - Orange

# DC Motor Pins (L298N) for Sail Deployment
# TODO: Connect L298N to these pins
PIN_IN1 = 24
PIN_IN2 = 23
PIN_EN = 25

# Servo Pins for Stabilization Dummy Movement
# TODO: Connect Servos to these pins
PIN_SERVO_1 = 5
PIN_SERVO_2 = 6
PIN_SERVO_3 = 13
PIN_SERVO_4 = 19

# Initialize LEDs
led_yellow = LED(PIN_LED_YELLOW)
led_green = LED(PIN_LED_GREEN)
led_orange = LED(PIN_LED_ORANGE)

# Setup RPi.GPIO for Motors and Servos
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# ==============================================================================
# ARTIFICIAL CONSTANTS & THRESHOLDS
# ==============================================================================
MIN_TEMP = 5      # [C]
MAX_TEMP = 45     # [C]
MAX_ANGULAR_RATE = 0.0174  # deg2rad(1)

# Dummy sensor values to pass the health check
dummy_temp_meas = 25       # Ideal temperature
dummy_rates_meas = 0.005   # Acceptable angular rate

# ==============================================================================
# SENSOR & SYSTEM CHECK FUNCTIONS
# ==============================================================================

def check_core_voltage():
    """Checks if Raspberry Pi core voltage is sufficient using vcgencmd."""
    try:
        # Runs: vcgencmd measure_volts core
        result = subprocess.run(['vcgencmd', 'measure_volts', 'core'], capture_output=True, text=True)
        # Output looks like: volt=1.2000V
        output = result.stdout.strip()
        volts_str = output.replace('volt=', '').replace('V', '')
        volts = float(volts_str)
        print(f"[Health Check] Core Voltage: {volts}V")
        # Assuming core voltage drops below 1.1V is bad for Pi stability
        return volts > 1.1
    except Exception as e:
        print(f"[Health Check] Warning: Could not read voltage ({e}). Defaulting to True.")
        return True

def check_comms(ip="8.8.8.8"):
    """Pings an IP address to check internet/comms connectivity."""
    param = '-n' if platform.system().lower() == 'windows' else '-c'
    command = ['ping', param, '1', ip]
    response = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    if response.returncode == 0:
        print(f"[Health Check] Ping successful! Connected to {ip}.")
        return True
    else:
        print(f"[Health Check] Ping failed. Cannot reach {ip}.")
        return False

# ==============================================================================
# STATE ACTION FUNCTIONS
# ==============================================================================

def set_mode_led(mode):
    """Lights the corresponding LED based on the current mode."""
    led_yellow.off()
    led_green.off()
    led_orange.off()
    
    if mode == 1:
        led_yellow.on()
        print("--> LED Indicator: YELLOW (Mode 1)")
    elif mode == 2:
        led_green.on()
        print("--> LED Indicator: GREEN (Mode 2)")
    elif mode == 3:
        led_orange.on()
        print("--> LED Indicator: ORANGE (Mode 3)")

def sail_deployment():
    """Activates the DC motor for 2 seconds to the right (forward) to deploy sail."""
    print("Executing Sail Deployment (DC Motor Forward for 2 seconds)...")
    
    GPIO.setup(PIN_IN1, GPIO.OUT)
    GPIO.setup(PIN_IN2, GPIO.OUT)
    GPIO.setup(PIN_EN, GPIO.OUT)
    
    GPIO.output(PIN_IN1, GPIO.LOW)
    GPIO.output(PIN_IN2, GPIO.LOW)
    p = GPIO.PWM(PIN_EN, 1000)
    p.start(75) # High speed
    
    # Move forward (Right)
    GPIO.output(PIN_IN1, GPIO.HIGH)
    GPIO.output(PIN_IN2, GPIO.LOW)
    
    time.sleep(2) # Run for 2 seconds
    
    # Stop
    GPIO.output(PIN_IN1, GPIO.LOW)
    GPIO.output(PIN_IN2, GPIO.LOW)
    p.stop()
    
    print("Sail Deployment Complete.")
    return True

def navigation_sensors_calibration():
    """Placeholder for actual camera calibration function."""
    print("Calibrating Navigation Sensors (Camera)...")
    time.sleep(1) # Simulating calibration time
    print("Calibration complete.")
    return 3 # 1/2/3 by result, returning 3 to exit loop

def dummy_stabilize():
    """Simple movement of 4 servos for 2 seconds if real function is missing."""
    print("Executing dummy servo stabilization sequence...")
    servo_pins = [PIN_SERVO_1, PIN_SERVO_2, PIN_SERVO_3, PIN_SERVO_4]
    pwms = []
    
    for pin in servo_pins:
        GPIO.setup(pin, GPIO.OUT)
        pwm = GPIO.PWM(pin, 50) # 50Hz for servos
        pwm.start(0)
        pwms.append(pwm)
    
    # Move servos to an arbitrary position (~90 degrees)
    for pwm in pwms:
        pwm.ChangeDutyCycle(7.5) 
    
    time.sleep(2) # Wait 2 seconds
    
    # Stop servos
    for pwm in pwms:
        pwm.stop()
    
    print("Dummy stabilization complete.")
    return True

def init_stabilize():
    """Attempts to call the real stabilization function, falls back to dummy if missing."""
    try:
        # If you have another file with this function, import it at the top 
        # and call it here. For now, we simulate its absence.
        raise NameError("Real init_stabilize function not found.")
    except NameError:
        print("************************************************************")
        print("WARNING: Real 'init_stabilize' function not found in scope!")
        print("Falling back to simple 4-servo dummy movement.")
        print("************************************************************")
        return dummy_stabilize()

# ==============================================================================
# MAIN STATE MACHINE
# ==============================================================================

def main():
    mode = 0 # Starting Mode
    print("Starting CoolEarth Spacecraft State Machine...")
    
    try:
        while True:
            # ---------------------------------------------------------
            # MODE 0: Initial Health Check State Loop
            # ---------------------------------------------------------
            if mode == 0:
                print("\n--- MODE 0: Initializing Health Checks ---")
                
                # 1. Battery/Voltage Level
                bat_stat = 1 if check_core_voltage() else 0
                
                # 2. Components Temperature (Using artificial constants)
                if MIN_TEMP <= dummy_temp_meas <= MAX_TEMP:
                    temp_stat = 1
                else:
                    temp_stat = 0
                    
                # 3. Comms Check
                comms_stat = 1 if check_comms() else 0
                
                # 4. Angular Rates Check (Using artificial constants)
                if abs(dummy_rates_meas) <= MAX_ANGULAR_RATE:
                    rate_stat = 1
                else:
                    rate_stat = 0
                
                # Final Check
                if bat_stat == 1 and temp_stat == 1 and comms_stat == 1 and rate_stat == 1:
                    print("All health checks passed. Transitioning to Mode 1.")
                    mode = 1
                    set_mode_led(mode)
                else:
                    print(f"Health check failed. States -> Bat:{bat_stat}, Temp:{temp_stat}, Comms:{comms_stat}, Rate:{rate_stat}")
                    time.sleep(2) # Retry after delay
            
            # ---------------------------------------------------------
            # MODE 1: Deployment State Loop
            # ---------------------------------------------------------
            elif mode == 1:
                print("\n--- MODE 1: Sail Deployment ---")
                is_deployed = sail_deployment()
                
                if not is_deployed:
                    print("Intervention error = 1: Must interfere from groundstation.")
                    # In reality, you'd wait for a groundstation command here to break or retry
                    time.sleep(5) 
                else:
                    print("Deployment successful. Transitioning to Mode 2.")
                    mode = 2
                    set_mode_led(mode)
            
            # ---------------------------------------------------------
            # MODE 2: Vision Aided Navigation State Loop
            # ---------------------------------------------------------
            elif mode == 2:
                print("\n--- MODE 2: Vision Aided Navigation Calibration ---")
                nav_quality = 0
                
                while nav_quality != 3:
                    nav_quality = navigation_sensors_calibration()
                    if nav_quality != 3:
                        print("Calibration pending... retrying.")
                        time.sleep(1)
                
                print("Navigation calibrated. Transitioning to Mode 3.")
                mode = 3
                set_mode_led(mode)
            
            # ---------------------------------------------------------
            # MODE 3: ADCS Stabilization State Loop
            # ---------------------------------------------------------
            elif mode == 3:
                print("\n--- MODE 3: ADCS Stabilization ---")
                is_stabilized = init_stabilize()
                
                if not is_stabilized:
                    print("Intervention error = 1: Must interfere from groundstation.")
                    time.sleep(5)
                else:
                    print("Stabilization successful. Transitioning to Mode 4.")
                    mode = 4
                    # set_mode_led(mode) # Optional: Add a 4th LED if needed later
            
            # ---------------------------------------------------------
            # MODE 4: Nominal State
            # ---------------------------------------------------------
            elif mode == 4:
                print("\n--- MODE 4: NOMINAL STATE ---")
                print("Spacecraft is in nominal operations. State Machine Initialization complete.")
                break # Exit the initialization sequence
                
    except KeyboardInterrupt:
        print("\nProcess interrupted by user.")
    
    finally:
        print("Cleaning up GPIO resources...")
        GPIO.cleanup()
        led_yellow.off()
        led_green.off()
        led_orange.off()

if __name__ == '__main__':
    main()