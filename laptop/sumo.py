"""
Sumo robot main control loop.

Reads the LEGO Color Sensor, the LEGO Controller and the HC-SR04 distance
(over MQTT from the Uno Q), runs the behavior tree, and drives the Double Motor.

Behavior tree (highest priority first):
    1. OUT       out button pressed: stop, sad song, publish out status
    2. ESCAPE    tape detected: override human, reverse then turn ~120 degrees
    3. ATTACK    opponent closer than 25 cm: full power forward
    4. HUMAN     controller levers moved: tank drive from the levers
    5. SEARCH    otherwise: slow spin

Run from the repo root:
    .venv/bin/python laptop/sumo.py             # real match
    .venv/bin/python laptop/sumo.py --dry-run   # motors never move

Ctrl-C to quit (motors stop).
"""

import argparse
import json
import math
import sys
import time

import legoeducation as le
import paho.mqtt.client as mqtt

from bt import Selector, Sequence, Condition, Action, SUCCESS, RUNNING
# Reuse the hardware code we already tested: card-serial connect and the
# lever -> tank-drive passthrough (deadzone and lever signs live there).
from test_lego import connect, levers_to_tank

# ---------------- Tunable constants ----------------
LOOP_HZ = 20

# Tape detection: blue hue AND enough saturation (a gray floor has a random
# hue but low saturation). Get values from test_lego.py calibration (press 'c').
# Calibrated: floor hue ~19 sat ~29, tape hue ~209 sat ~59 (0/20 floor, 20/20 tape)
TAPE_HUE_MIN = 185               # degrees
TAPE_HUE_MAX = 235
TAPE_MIN_SATURATION = 44         # None = not calibrated yet

ESCAPE_REVERSE_SPEED = 60        # percent
ESCAPE_REVERSE_SECONDS = 0.5
ESCAPE_TURN_SPEED = 50           # percent
ESCAPE_TURN_DEGREES = 120        # measured with the Double Motor's IMU yaw
ESCAPE_TURN_MAX_SECONDS = 1.0    # stop turning after this even if IMU says less
                                 # (also the whole turn if there's no IMU data)

ATTACK_DISTANCE_CM = 25
ATTACK_SPEED = 100

HUMAN_HOLD_SECONDS = 1.5         # keep human control (motors stopped) this long
                                 # after the levers are released, before SEARCH

SEARCH_SPEED = 25                # spin in place

SAD_SONG = [(523, 0.35), (494, 0.35), (466, 0.35), (440, 0.9)]  # (Hz, seconds)

MQTT_BROKER = "broker.hivemq.com"
MQTT_PORT = 1883
DIST_TOPIC = "ME193/ceci/sumo/dist"
STATUS_TOPIC = "ME193/ceci/sumo/status"
DIST_STALE_SECONDS = 0.5         # older distance data counts as "no opponent"
# ---------------------------------------------------


class Blackboard:
    """Shared data the behavior tree reads (inputs) and writes (outputs)."""

    def __init__(self):
        # inputs, refreshed every loop
        self.now = 0.0
        self.out_pressed = False
        self.tape = False
        self.opponent_cm = None      # None = no opponent / no fresh data
        self.levers = (0, 0)
        self.last_lever_time = -math.inf  # when the levers were last moved
        self.yaw = float("nan")
        # outputs
        self.drive = (0, 0)          # (left, right) percent
        self.requests = []           # one-time jobs for the main loop
        self.active_node = None
        # memory
        self.is_out = False


# ---------------- Actions with state ----------------

class GoOut(Action):
    """We lost: stop forever, play the sad song once, tell the world."""

    def run(self, bb):
        bb.drive = (0, 0)
        if not bb.is_out:
            bb.is_out = True
            bb.requests += ["publish_out", "sad_song"]
        return RUNNING


class EscapeTape(Action):
    """Reverse for a bit, then turn ~120 degrees, then hand control back."""

    def __init__(self, name):
        super().__init__(name)
        self.phase = None        # None, "reverse" or "turn"
        self.phase_start = 0.0
        self.start_yaw = float("nan")

    @property
    def active(self):
        return self.phase is not None

    def run(self, bb):
        if self.phase is None:
            self.phase, self.phase_start = "reverse", bb.now

        if self.phase == "reverse":
            bb.active_node = "ESCAPE: reversing"
            bb.drive = (-ESCAPE_REVERSE_SPEED, -ESCAPE_REVERSE_SPEED)
            if bb.now - self.phase_start >= ESCAPE_REVERSE_SECONDS:
                self.phase, self.phase_start = "turn", bb.now
                self.start_yaw = bb.yaw
            return RUNNING

        # phase == "turn"
        bb.active_node = "ESCAPE: turning"
        bb.drive = (ESCAPE_TURN_SPEED, -ESCAPE_TURN_SPEED)
        turned = yaw_change(self.start_yaw, bb.yaw)
        timed_out = bb.now - self.phase_start >= ESCAPE_TURN_MAX_SECONDS
        if turned >= ESCAPE_TURN_DEGREES or timed_out:
            self.phase = None
            bb.drive = (0, 0)
            return SUCCESS
        return RUNNING


def yaw_change(start, current):
    """Degrees turned since start, handling the jump at +/-180. 0 if no IMU data."""
    if math.isnan(start) or math.isnan(current):
        return 0
    return abs((current - start + 180) % 360 - 180)


def set_drive(speeds):
    def action(bb):
        bb.drive = speeds
        return RUNNING
    return action


def human_in_control(bb):
    """Levers moved now, or recently enough that it's just a brief pause."""
    return bb.now - bb.last_lever_time < HUMAN_HOLD_SECONDS


def follow_human(bb):
    bb.drive = bb.levers
    if bb.levers == (0, 0):
        bb.active_node = "HUMAN: paused"
    return RUNNING


def build_tree():
    escape = EscapeTape("ESCAPE")
    return Selector("root", [
        Sequence("out", [
            Condition("out button pressed?", lambda bb: bb.out_pressed or bb.is_out),
            GoOut("OUT: stopped, sad song"),
        ]),
        Sequence("tape", [
            # keep escaping after we've backed off the tape
            Condition("tape detected?", lambda bb: bb.tape or escape.active),
            escape,
        ]),
        Sequence("attack", [
            Condition("opponent close?", lambda bb: bb.opponent_cm is not None
                      and bb.opponent_cm < ATTACK_DISTANCE_CM),
            Action("ATTACK: full forward", set_drive((ATTACK_SPEED, ATTACK_SPEED))),
        ]),
        Sequence("human", [
            Condition("controller input?", human_in_control),
            Action("HUMAN: tank drive", follow_human),
        ]),
        Action("SEARCH: slow spin", set_drive((SEARCH_SPEED, -SEARCH_SPEED))),
    ])


# ---------------- MQTT ----------------

class DistanceListener:
    """Keeps the latest distance from the Uno Q. Works fine with no messages."""

    def __init__(self):
        self.dist_cm = None
        self.received_at = 0.0
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.connect_async(MQTT_BROKER, MQTT_PORT)
        self.client.loop_start()

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        print(f"\nMQTT connected ({reason_code}), subscribing to {DIST_TOPIC}")
        client.subscribe(DIST_TOPIC)

    def _on_message(self, client, userdata, msg):
        try:
            self.dist_cm = int(json.loads(msg.payload)["dist_cm"])
            self.received_at = time.time()
        except (ValueError, KeyError, TypeError):
            pass  # ignore malformed messages

    def opponent_cm(self):
        """Distance in cm, or None if no target / no data / data too old."""
        if self.dist_cm is None or self.dist_cm < 0:
            return None
        if time.time() - self.received_at > DIST_STALE_SECONDS:
            return None
        return self.dist_cm

    def publish_out(self):
        self.client.publish(STATUS_TOPIC, json.dumps({"state": "out"}), qos=1)

    def stop(self):
        self.client.loop_stop()
        self.client.disconnect()


# ---------------- Main ----------------

def read_tape(color_sensor):
    if TAPE_MIN_SATURATION is None or color_sensor is None:
        return False
    s = color_sensor.sensor
    if math.isnan(s.hue) or math.isnan(s.saturation):
        return False
    return TAPE_HUE_MIN <= s.hue <= TAPE_HUE_MAX and s.saturation >= TAPE_MIN_SATURATION


def play_sad_song(motor):
    for freq, seconds in SAD_SONG:
        motor.beep(frequency=freq)
        time.sleep(seconds)
    motor.stop_beep()


def main():
    parser = argparse.ArgumentParser(description="ME193 sumo robot")
    parser.add_argument("--dry-run", action="store_true",
                        help="run the tree and print decisions, but never move the "
                             "motors, beep or publish")
    args = parser.parse_args()
    dry = args.dry_run

    if TAPE_MIN_SATURATION is None:
        if not dry:
            sys.exit("TAPE_MIN_SATURATION is not set. Run test_lego.py, press 'c' to "
                     "calibrate, and put the results at the top of sumo.py.")
        print("WARNING: TAPE_MIN_SATURATION not set, tape will never be detected.")

    # In dry-run we don't even connect the motor, so it physically can't move.
    motor = None
    if not dry:
        motor = le.DoubleMotor()
        if not connect(motor, "Double Motor"):
            sys.exit("Need the Double Motor.")
    color_sensor = le.ColorSensor()
    if not connect(color_sensor, "Color Sensor"):
        color_sensor = None
    controller = le.Controller()
    if not connect(controller, "Controller"):
        controller = None
    if not dry and (color_sensor is None or controller is None):
        sys.exit("Need the Color Sensor (tape) and Controller (out button) for a real run.")

    distance = DistanceListener()
    tree = build_tree()
    bb = Blackboard()
    sent_drive = None
    last_node = None

    print("\nRunning" + (" in DRY RUN (motors disabled)" if dry else "") + ". Ctrl-C to stop.\n")
    try:
        while True:
            # 1. Read inputs
            bb.now = time.time()
            bb.tape = read_tape(color_sensor)
            bb.opponent_cm = distance.opponent_cm()
            if controller is not None and controller.connected:
                bb.levers = levers_to_tank(controller)
                if bb.levers != (0, 0):
                    bb.last_lever_time = bb.now
                bb.out_pressed = controller.button.state == le.BUTTON_STATE_PRESSED
            else:
                bb.levers = (0, 0)  # never follow stale lever values
                bb.out_pressed = False
            bb.yaw = motor.imu_device.yaw if motor else float("nan")

            # 2. Decide
            tree.tick(bb)

            # 3. Act: motors first (only when the command changes), then one-time jobs
            if bb.drive != sent_drive:
                if motor:
                    if bb.drive == (0, 0):
                        motor.movement_stop()
                    else:
                        motor.movement_move_tank(*bb.drive)
                sent_drive = bb.drive
            for job in bb.requests:
                if dry:
                    print(f"\n[dry-run] would {job.replace('_', ' ')}")
                elif job == "publish_out":
                    distance.publish_out()
                elif job == "sad_song":
                    play_sad_song(motor)
            bb.requests = []

            # 4. Print: a permanent line when the node changes, plus a live line
            if bb.active_node != last_node:
                print(f"\n>>> {bb.active_node}")
                last_node = bb.active_node
            dist = f"{bb.opponent_cm:3d}cm" if bb.opponent_cm is not None else "  --"
            print(f"\r{bb.active_node:28s} | motors L {bb.drive[0]:4d} R {bb.drive[1]:4d} "
                  f"| tape {'YES' if bb.tape else 'no ':3s} | dist {dist} "
                  f"| levers {bb.levers[0]:4d} {bb.levers[1]:4d}   ", end="", flush=True)

            time.sleep(1 / LOOP_HZ)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nStopping and disconnecting...")
        if motor and motor.connected:
            motor.movement_stop()
        distance.stop()
        for device in (motor, color_sensor, controller):
            if device and device.connected:
                device.disconnect()


if __name__ == "__main__":
    main()
