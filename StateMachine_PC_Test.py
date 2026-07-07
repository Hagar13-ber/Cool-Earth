"""
StateMachine_PC_Test.py
-----------------------
PC-compatible test version of StateMachine.py.
All Raspberry Pi hardware (GPIO, I2C, servos, LEDs, IMU) is replaced with
lightweight mocks so the OpNav / Cool-Earth-NAV pipeline can be tested on
a Windows or Linux development machine without any hardware attached.

Only the NAVIGATION state logic (_capture_frame + _run_opnav_navigation_once)
is real — everything else prints a [MOCK] notice and moves on.
"""

import os
import sys
import time
import csv
import subprocess

# ==========================================
#         HARDWARE MOCKS  (PC only)
# ==========================================

class _MockLED:
    def __init__(self, pin):
        self._pin = pin
    def on(self):
        print(f"[MOCK] LED(GPIO{self._pin}) ON")
    def off(self):
        print(f"[MOCK] LED(GPIO{self._pin}) OFF")
    def close(self):
        pass


class _MockPWM:
    def __init__(self, pin, freq):
        self._pin = pin
    def start(self, dc):
        print(f"[MOCK] PWM(pin={self._pin}) started at {dc}% duty cycle")
    def ChangeDutyCycle(self, dc):
        print(f"[MOCK] PWM(pin={self._pin}) duty cycle -> {dc}%")
    def stop(self):
        print(f"[MOCK] PWM(pin={self._pin}) stopped")


class _MockGPIO:
    BCM = "BCM"
    OUT = "OUT"
    HIGH = 1
    LOW = 0

    @staticmethod
    def setmode(mode):
        pass

    @staticmethod
    def setup(pin, mode):
        pass

    @staticmethod
    def output(pin, state):
        label = "HIGH" if state else "LOW"
        print(f"[MOCK] GPIO pin {pin} -> {label}")

    @staticmethod
    def cleanup():
        print("[MOCK] GPIO cleanup")

    @staticmethod
    def PWM(pin, freq):
        return _MockPWM(pin, freq)


class _MockIMUBus:
    """Always returns gyro values of (0, 0, 0) — satisfies the INIT condition immediately."""
    def read_byte_data(self, addr, reg):
        return 0

    def write_byte_data(self, addr, reg, value):
        pass


# Inject mocks so the rest of the file can use the same names as the Pi version
GPIO = _MockGPIO()

# ==========================================
#         HARDWARE SETUP FUNCTIONS
# ==========================================

def setup_servos():
    """Mock: returns placeholder objects — no I2C or PCA9685 needed."""
    print("[MOCK] setup_servos: PCA9685 + 4 servo channels initialised")
    pca = object()
    servos = [object() for _ in range(4)]
    return pca, servos


def setup_dc_motor():
    """Mock: returns a fake PWM object and pin numbers."""
    in1 = 24
    in2 = 23
    en  = 25
    print(f"[MOCK] setup_dc_motor: in1={in1}, in2={in2}, en={en}")
    p = _MockPWM(en, 1000)
    p.start(0)
    return p, in1, in2


def setup_leds():
    """Mock: returns _MockLED objects for each LED channel."""
    leds = {
        'deployment': _MockLED(27),
        'navigation': _MockLED(26),
        'adcs':       _MockLED(22),
        'nominal':    _MockLED(16),
    }
    print("[MOCK] setup_leds: 4 LEDs initialised")
    return leds


# --- IMU Configuration (kept for reference) ---
L3GD20H_ADDR = 0x6B
L3G_CTRL1    = 0x20
L3G_OUT_X_L  = 0x28

# Navigation status mapping (1/2/3 format):
# 1 = bad, 2 = degraded, 3 = good.
NAV_STATUS_BAD      = 1
NAV_STATUS_DEGRADED = 2
NAV_STATUS_GOOD     = 3


def setup_imu():
    """Mock: returns a fake I2C bus that always reads zero."""
    print("[MOCK] setup_imu: L3GD20H initialised (always returns 0)")
    return _MockIMUBus()


def read_raw_axis(bus, addr, reg):
    """Reads a 16-bit value from the IMU (works with real or mock bus)."""
    low  = bus.read_byte_data(addr, reg)
    high = bus.read_byte_data(addr, reg + 1)
    value = (high << 8) | low
    if value > 32767:
        value -= 65536
    return value


def read_gyro(bus):
    """Reads X, Y, Z axes from the IMU."""
    x = read_raw_axis(bus, L3GD20H_ADDR, L3G_OUT_X_L)
    y = read_raw_axis(bus, L3GD20H_ADDR, L3G_OUT_X_L + 2)
    z = read_raw_axis(bus, L3GD20H_ADDR, L3G_OUT_X_L + 4)
    return x, y, z


# ==========================================
#         NAVIGATION PIPELINE  (real)
# ==========================================

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
        # Expected behaviour:
        # - Capture one frame from the onboard camera
        # - Save it under Cool-Earth-NAV/data/captured/
        # - Return the saved FITS path
        pass

    # Section 2: Scenario CSV fallback (current placeholder behaviour).
    project_root  = os.path.dirname(os.path.abspath(__file__))
    scenarios_csv = os.path.join(project_root, "Cool-Earth-NAV", "data", "scenarios.csv")
    captured_dir  = os.path.join(project_root, "Cool-Earth-NAV", "data", "captured")

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

    Current placeholder behaviour:
    - GOOD (3): pipeline execution succeeded.
    - DEGRADED (2): pipeline ran but did not reach full quality (e.g. too few matches).
    - BAD (1): pipeline failed or call failed.
    """
    try:
        project_root = os.path.dirname(os.path.abspath(__file__))
        opnav_dir    = os.path.join(project_root, "Cool-Earth-NAV")
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
    print("=" * 55)
    print("  StateMachine_PC_Test  —  hardware mocked")
    print("=" * 55)
    print("Setting up hardware connections...")

    pca, servos          = setup_servos()
    dc_pwm, dc_in1, dc_in2 = setup_dc_motor()
    leds                 = setup_leds()
    imu_bus              = setup_imu()

    state = "INIT"

    try:
        while state != "DONE":

            if state == "INIT":
                print("\n--- STATE: INITIALIZATION ---")
                consecutive_count = 0

                # Mock IMU always returns (0,0,0) so this completes in ~0.5 s
                while consecutive_count < 50:
                    x, y, z = read_gyro(imu_bus)
                    magnitude_squared = (x**2) + (y**2) + (z**2)

                    if magnitude_squared <= 10000:
                        consecutive_count += 1
                    else:
                        consecutive_count = 0

                    time.sleep(0.01)

                print("IMU condition verified for 50 samples.")
                state = "DEPLOYMENT"

            elif state == "DEPLOYMENT":
                print("\n--- STATE: DEPLOYMENT ---")

                leds['deployment'].on()
                time.sleep(2)
                leds['deployment'].off()

                GPIO.output(dc_in1, GPIO.HIGH)
                GPIO.output(dc_in2, GPIO.LOW)
                dc_pwm.ChangeDutyCycle(50)
                time.sleep(2)

                dc_pwm.ChangeDutyCycle(0)
                GPIO.output(dc_in1, GPIO.LOW)

                state = "NAVIGATION"

            elif state == "NAVIGATION":
                print("\n--- STATE: NAVIGATION ---")

                leds['navigation'].on()
                time.sleep(2)
                leds['navigation'].off()

                nav_status  = NAV_STATUS_BAD
                frame_index = 0

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
                print("\n--- STATE: ADCS ---")

                leds['adcs'].on()
                time.sleep(2)
                leds['adcs'].off()

                # ServoRun.py is Pi-only; skipped in PC test mode
                print("[MOCK] Skipping ServoRun.py (Pi hardware not available)")

                state = "NOMINAL"

            elif state == "NOMINAL":
                print("\n--- STATE: TRANSITION TO NOMINAL ---")

                leds['nominal'].on()
                time.sleep(2)
                leds['nominal'].off()

                print("State machine execution finished.")
                state = "DONE"

    except KeyboardInterrupt:
        print("\nProcess interrupted by user.")

    finally:
        print("\nCleaning up (mock) resources...")
        dc_pwm.stop()
        GPIO.cleanup()
        for led in leds.values():
            led.close()


if __name__ == "__main__":
    main()
