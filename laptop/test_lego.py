"""
Hardware test for the LEGO side of the sumo robot.

Connects to the Double Motor, Color Sensor and Controller that are paired
with our Connection Card, then shows live readings at ~10 Hz.

Keys:
    m      spin the motors briefly
    c      calibrate the color sensor (floor vs. blue tape)
    d      toggle drive mode: Controller levers tank-drive the motors
    w/a/s/d  drive (only when the Controller can't be read)
    q      quit

Run from the repo root:
    .venv/bin/python laptop/test_lego.py
"""

import asyncio
import math
import select
import statistics
import sys
import termios
import time
import tty

import legoeducation as le
from bleak import BleakScanner

# ---------------- Tunable constants ----------------
CARD_SERIAL = "0999"             # our Connection Card (orange diamond)
CARD_COLOR = le.LEGO_COLOR_ORANGE  # only used to double-check the scan printout

SCAN_SECONDS = 5                 # how long to list nearby LEGO devices
NOTIFY_MS = 50                   # how often each device sends readings
LOOP_HZ = 10                     # live print rate

MOTOR_TEST_SPEED = 40            # percent, for the 'm' key
MOTOR_TEST_SECONDS = 0.5

TELEOP_SPEED = 50                # percent, for WASD fallback
TELEOP_TURN_SPEED = 35
KEY_HOLD_SECONDS = 0.5           # stop if no key repeat within this time
                                 # (must be longer than macOS key-repeat delay)

LEVER_DEADZONE = 5               # percent; lever values smaller than this count as 0
LEFT_LEVER_SIGN = 1              # set to -1 if pushing a lever forward drives backward
RIGHT_LEVER_SIGN = 1

CALIBRATION_SAMPLES = 20         # samples per surface (2 s at 10 Hz)
CALIBRATION_FIELDS = ["hue", "saturation", "reflection"]
HUE_MARGIN = 15                  # degrees added around the tape's hue range

# Values copied from legoeducation so the scan can decode advertisements
LEGO_SERVICE_UUID = "0000FD02-0000-1000-8000-00805F9B34FB"
LEGO_COMPANY_ID = 0x0397
PRODUCT_NAMES = {
    le.PRODUCT_GROUP_DEVICE_SINGLE_MOTOR: "Single Motor",
    le.PRODUCT_GROUP_DEVICE_DOUBLE_MOTOR: "Double Motor",
    le.PRODUCT_GROUP_DEVICE_COLOR_SENSOR: "Color Sensor",
    le.PRODUCT_GROUP_DEVICE_CONTROLLER: "Controller",
}
# ---------------------------------------------------


def color_name(color):
    name = le.LEGO_COLOR_NAME_MAP.get(color, "UNKNOWN")
    return name.replace("LEGO_COLOR_", "").lower()


# ---------------- Scanning ----------------

async def _scan(seconds):
    found = await BleakScanner.discover(timeout=seconds, return_adv=True,
                                        service_uuids=[LEGO_SERVICE_UUID])
    results = []
    for device, adv in found.values():
        data = adv.manufacturer_data.get(LEGO_COMPANY_ID)
        if data is None or len(data) < 5:
            continue
        product_id = (data[0] << 8) | data[1]
        # Card color in the advertisement is a firmware value; this helper
        # converts it to the le.LEGO_COLOR_* numbering.
        card_color = le.color_map._firmware_to_app(data[2])
        card_serial = data[3] | (data[4] << 8)
        results.append((PRODUCT_NAMES.get(product_id, f"product {product_id}"),
                        card_color, card_serial, device.address))
    return results


def list_nearby_devices():
    """Print every LEGO device nearby so we can check which ones are ours."""
    print(f"Scanning {SCAN_SECONDS} s for nearby LEGO devices...")
    try:
        devices = asyncio.run(_scan(SCAN_SECONDS))
    except Exception as e:
        print(f"  (scan failed: {e}) -- continuing to connect anyway")
        return
    if not devices:
        print("  No LEGO devices found. Are they switched on?")
        return
    for product, card_color, card_serial, address in sorted(devices):
        serial = f"{card_serial:04d}"
        if serial == CARD_SERIAL:
            mark = "<-- OURS"
            if card_color != CARD_COLOR:
                mark += f" (but card color is {color_name(card_color)}, expected {color_name(CARD_COLOR)}!)"
        else:
            mark = "ignored"
        print(f"  {product:13s} card {color_name(card_color):8s} serial {serial}  {mark}")


def connect(device, label):
    """Connect to the device of this type paired with OUR card only."""
    print(f"Connecting to {label} with card serial {CARD_SERIAL}...")
    try:
        device.connect(card_serial=CARD_SERIAL, device_notification_delay=NOTIFY_MS)
    except le.basic_device.VersionMismatchError as e:
        print(f"  {label}: {e}")
    if device.connected:
        print(f"  {label} connected.")
        return True
    print(f"  {label} NOT found with serial {CARD_SERIAL}.")
    return False


# ---------------- Keyboard ----------------

class Keyboard:
    """Reads single keypresses from the terminal without waiting for Enter."""

    def __enter__(self):
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, *args):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def get_key(self):
        """Return the key pressed since last call, or None."""
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1).lower()
        return None

    def wait_for_key(self):
        while True:
            key = self.get_key()
            if key:
                return key
            time.sleep(0.02)


# ---------------- Calibration ----------------

def collect_samples(color_sensor):
    samples = []
    while len(samples) < CALIBRATION_SAMPLES:
        reading = color_sensor.sensor
        if not math.isnan(reading.reflection):
            samples.append({f: getattr(reading, f) for f in CALIBRATION_FIELDS})
        time.sleep(1 / LOOP_HZ)
    return samples


def calibrate(color_sensor, keyboard):
    print("\n\n=== Color sensor calibration ===")
    print("Hold the sensor over the FLOOR, then press any key.")
    keyboard.wait_for_key()
    floor = collect_samples(color_sensor)
    print("Got floor. Now hold it over the BLUE TAPE, then press any key.")
    keyboard.wait_for_key()
    tape = collect_samples(color_sensor)

    print(f"\n{'':12s} {'---- floor ----':>21s}   {'---- tape ----':>21s}")
    print(f"{'field':12s} {'avg':>6s} {'min':>6s} {'max':>6s}   {'avg':>6s} {'min':>6s} {'max':>6s}")
    for field in CALIBRATION_FIELDS:
        f_vals = [s[field] for s in floor]
        t_vals = [s[field] for s in tape]
        print(f"{field:12s} {statistics.mean(f_vals):6.1f} {min(f_vals):6.0f} {max(f_vals):6.0f}"
              f"   {statistics.mean(t_vals):6.1f} {min(t_vals):6.0f} {max(t_vals):6.0f}")
    print("(floor hue is meaningless when its saturation is low -- that's why we "
          "also need a minimum saturation)")

    # Suggest: hue inside the tape's range (plus a margin) AND saturation above
    # the halfway point between the floor's highest and the tape's lowest.
    hue_min = max(0, min(s["hue"] for s in tape) - HUE_MARGIN)
    hue_max = min(360, max(s["hue"] for s in tape) + HUE_MARGIN)
    floor_sat_max = max(s["saturation"] for s in floor)
    tape_sat_min = min(s["saturation"] for s in tape)
    sat_min = (floor_sat_max + tape_sat_min) / 2

    print("\nSuggested values for the top of sumo.py:")
    print(f"    TAPE_HUE_MIN = {hue_min:.0f}")
    print(f"    TAPE_HUE_MAX = {hue_max:.0f}")
    print(f"    TAPE_MIN_SATURATION = {sat_min:.0f}")
    if tape_sat_min <= floor_sat_max:
        print("  WARNING: floor and tape saturation overlap, so saturation alone "
              "can't separate them. Check the hue ranges carefully.")

    def is_tape(s):
        return hue_min <= s["hue"] <= hue_max and s["saturation"] >= sat_min
    print(f"Check: {sum(map(is_tape, floor))}/{len(floor)} floor samples and "
          f"{sum(map(is_tape, tape))}/{len(tape)} tape samples count as tape "
          f"(want 0 and all).")
    print("Press any key to go back to live readings.\n")
    keyboard.wait_for_key()


# ---------------- Main ----------------

def wasd_to_tank(key):
    return {
        "w": (TELEOP_SPEED, TELEOP_SPEED),
        "s": (-TELEOP_SPEED, -TELEOP_SPEED),
        "a": (-TELEOP_TURN_SPEED, TELEOP_TURN_SPEED),
        "d": (TELEOP_TURN_SPEED, -TELEOP_TURN_SPEED),
    }.get(key)


def lever_to_speed(percent, sign):
    """Turn a lever reading into a motor speed, ignoring small resting drift."""
    if math.isnan(percent) or abs(percent) < LEVER_DEADZONE:
        return 0
    return max(-100, min(100, round(sign * percent)))


def levers_to_tank(controller):
    c = controller.sensor
    return (lever_to_speed(c.leftPercent, LEFT_LEVER_SIGN),
            lever_to_speed(c.rightPercent, RIGHT_LEVER_SIGN))


def main():
    list_nearby_devices()
    print()

    motor = le.DoubleMotor()
    color_sensor = le.ColorSensor()
    controller = le.Controller()

    motor_ok = connect(motor, "Double Motor")
    sensor_ok = connect(color_sensor, "Color Sensor")
    controller_ok = connect(controller, "Controller")

    # The library has a Controller API, but check we actually get numbers
    if controller_ok and math.isnan(controller.sensor.leftPercent):
        controller_ok = False
        print("  Controller connected but sends no readings.")

    print()
    if controller_ok:
        print("Controller readings OK via legoeducation.")
    else:
        print("*" * 60)
        print("  CONTROLLER NOT READABLE -- using WASD keyboard teleop instead")
        print("*" * 60)
    print("Keys: m = spin motors, c = calibrate, q = quit"
          + (", d = toggle drive mode" if controller_ok else ", w/a/s/d = drive"))
    print()

    last_drive = (0, 0)
    last_key_time = 0.0
    drive_mode = False

    try:
        with Keyboard() as keyboard:
            while True:
                key = keyboard.get_key()

                if key == "q":
                    break
                elif key == "m" and motor_ok:
                    print("\nSpinning motors...")
                    motor.movement_move_tank(MOTOR_TEST_SPEED, MOTOR_TEST_SPEED)
                    time.sleep(MOTOR_TEST_SECONDS)
                    motor.movement_stop()
                    last_drive = (0, 0)
                elif key == "c":
                    if sensor_ok:
                        calibrate(color_sensor, keyboard)
                    else:
                        print("\nColor sensor not connected, can't calibrate.")
                elif key == "d" and controller_ok:
                    if not motor_ok:
                        print("\nDouble Motor not connected, can't drive.")
                    elif drive_mode:
                        drive_mode = False
                        motor.movement_stop()
                        last_drive = (0, 0)
                        print("\nDrive mode OFF, motors stopped.")
                    else:
                        drive_mode = True
                        print("\nDrive mode ON: levers drive the motors. Press d to stop.")

                # Drive mode: left lever -> left motor, right lever -> right motor
                if drive_mode:
                    if controller.connected:
                        drive = levers_to_tank(controller)
                    else:
                        drive_mode = False
                        controller_ok = False
                        drive = (0, 0)
                        print("\nController disconnected! Drive mode OFF, motors stopped.")
                    if drive != last_drive:
                        motor.movement_move_tank(*drive)
                        last_drive = drive

                # WASD fallback: drive while a key is held (key repeat)
                if not controller_ok and motor_ok:
                    drive = wasd_to_tank(key)
                    if drive:
                        last_key_time = time.time()
                    elif time.time() - last_key_time > KEY_HOLD_SECONDS:
                        drive = (0, 0)
                    else:
                        drive = last_drive
                    if drive != last_drive:
                        motor.movement_move_tank(*drive)
                        last_drive = drive

                # Live readings on one line
                parts = []
                if controller_ok:
                    c = controller.sensor
                    parts.append(f"ctrl L {c.leftPercent:4.0f}% R {c.rightPercent:4.0f}% "
                                 f"angles {c.leftAngle:5.0f} {c.rightAngle:5.0f} "
                                 f"btn {controller.button.state}")
                if drive_mode:
                    mode = "DRIVE"
                elif controller_ok:
                    mode = "off"
                else:
                    mode = "wasd"
                parts.append(f"motors {mode:5s} L {last_drive[0]:4d} R {last_drive[1]:4d}")
                if sensor_ok:
                    s = color_sensor.sensor
                    parts.append(f"color {color_name(s.color):7s} refl {s.reflection:3.0f} "
                                 f"rgb {s.rawRed:4.0f} {s.rawGreen:4.0f} {s.rawBlue:4.0f} "
                                 f"hsv {s.hue:3.0f} {s.saturation:3.0f} {s.value:3.0f}")
                print("\r" + " | ".join(parts) + "   ", end="", flush=True)

                time.sleep(1 / LOOP_HZ)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nStopping and disconnecting...")
        if motor_ok:
            motor.movement_stop()
        for device in (motor, color_sensor, controller):
            if device.connected:
                device.disconnect()


if __name__ == "__main__":
    main()
