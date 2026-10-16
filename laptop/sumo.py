"""
Sumo robot main control loop.

Reads the LEGO Color Sensor, the LEGO Controller and the HC-SR04 distance
(over MQTT from the Uno Q), runs the behavior tree, and drives the Double Motor.

The robot never moves unless the human is pushing the levers.

Behavior tree (highest priority first):
    1. OUT         out button tapped: stop, sad song, publish out status
    2. TAPE GUARD  tape detected: lever drive, but forward is clamped to 0
                   (reversing still allowed, so we can back off the edge)
    3. BOOST       opponent closer than 25 cm AND both levers pushed forward:
                   both motors full forward
    4. HUMAN       levers outside the deadzone: tank drive from the levers
    5. IDLE        levers at rest: motors stop

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

from bt import Selector, Sequence, Condition, Action, RUNNING
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

BOOST_DISTANCE_CM = 25
BOOST_SPEED = 100                # percent

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
        self.levers = (0, 0)         # after deadzone, (left, right) percent
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


# ---------------- Conditions and actions ----------------

def opponent_close(bb):
    return bb.opponent_cm is not None and bb.opponent_cm < BOOST_DISTANCE_CM


def both_levers_forward(bb):
    return bb.levers[0] > 0 and bb.levers[1] > 0


def guard_tape(bb):
    """Follow the levers, but never drive forward: only stop or reverse."""
    left, right = bb.levers
    bb.drive = (min(left, 0), min(right, 0))
    return RUNNING


def boost(bb):
    bb.drive = (BOOST_SPEED, BOOST_SPEED)
    return RUNNING


def follow_human(bb):
    bb.drive = bb.levers
    return RUNNING


def idle(bb):
    bb.drive = (0, 0)
    return RUNNING


def build_tree():
    # TAPE GUARD sits above BOOST, so boosting is also blocked on the tape.
    return Selector("root", [
        Sequence("out", [
            Condition("out button pressed?", lambda bb: bb.out_pressed or bb.is_out),
            GoOut("OUT: stopped, sad song"),
        ]),
        Sequence("tape guard", [
            Condition("tape detected?", lambda bb: bb.tape),
            Action("TAPE GUARD: no forward", guard_tape),
        ]),
        Sequence("boost", [
            Condition("opponent close?", opponent_close),
            Condition("both levers forward?", both_levers_forward),
            Action("BOOST: full forward", boost),
        ]),
        Sequence("human", [
            Condition("levers moved?", lambda bb: bb.levers != (0, 0)),
            Action("HUMAN: tank drive", follow_human),
        ]),
        Action("IDLE: stopped", idle),
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
                bb.out_pressed = controller.button.state == le.BUTTON_STATE_PRESSED
            else:
                bb.levers = (0, 0)  # never follow stale lever values
                bb.out_pressed = False

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
