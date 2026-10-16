"""
Sumo robot main control loop.

Reads the LEGO Color Sensor, the LEGO Controller and the HC-SR04 distance
(over MQTT from the Uno Q), runs the behavior tree, and drives the Double Motor.

The robot never moves unless the human is pushing the levers.

Behavior tree (highest priority first):
    1. OUT         out button tapped: stop, sad song, publish out status
    2. TAPE GUARD  tape detected: lever drive, but forward is clamped to 0
                   (reversing still allowed, so we can back off the edge).
                   Latches: stays on until the sensor is off the tape AND
                   we've pulled either lever back at some point since it latched.
    3. BOOST       opponent closer than 25 cm AND both levers pushed forward:
                   both motors full forward
    4. HUMAN       levers outside the deadzone: tank drive from the levers
    5. IDLE        levers at rest: motors stop

Nothing in the main loop waits on the hardware: sensor readings arrive by
callback, and motor and sound commands are sent with blocking=False.

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
# lever -> tank-drive passthrough. Lever and motor signs live there; in all
# logic here, positive speed = physically forward.
from test_lego import connect, levers_to_tank, send_tank

# ---------------- Tunable constants ----------------
LOOP_HZ = 50                     # behavior tree ticks per second
PRINT_HZ = 10                    # live line refresh (printing is slow-ish)
SENSOR_NOTIFY_MS = 20            # color sensor + controller send readings every 20 ms
                                 # (library minimum is 15)

# Tape detection: blue hue AND enough saturation (a gray floor has a random
# hue but low saturation). Get values from test_lego.py calibration (press 'c').
# Calibrated: floor hue ~19 sat ~29, tape hue ~209 sat ~59 (0/20 floor, 20/20 tape)
TAPE_HUE_MIN = 185               # degrees
TAPE_HUE_MAX = 235
TAPE_MIN_SATURATION = 44         # None = not calibrated yet

TAPE_BEEP_HZ = 880

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


# ---------------- Conditions and actions ----------------

class GoOut(Action):
    """We lost: stop forever, play the sad song once, tell the world."""

    def run(self, bb):
        bb.drive = (0, 0)
        if not bb.is_out:
            bb.is_out = True
            bb.requests += ["publish_out", "sad_song"]
        return RUNNING


class TapeGuard(Action):
    """Follow the levers but never drive forward. Once tape is seen, stay
    latched until the sensor is off the tape AND the human has pulled either
    lever back at some point since the latch started. If we overshoot the
    tape without reversing, forward stays blocked."""

    def __init__(self, name):
        super().__init__(name)
        self.latched = False
        self.has_reversed = False    # any reverse command since latching?

    def run(self, bb):
        if not self.latched:
            self.latched = True
            self.has_reversed = False
            bb.requests.append("tape_beep")

        left, right = bb.levers
        bb.drive = (min(left, 0), min(right, 0))

        if left < 0 or right < 0:
            self.has_reversed = True
        if self.has_reversed and not bb.tape:
            self.latched = False
        return RUNNING


def opponent_close(bb):
    return bb.opponent_cm is not None and bb.opponent_cm < BOOST_DISTANCE_CM


def both_levers_forward(bb):
    return bb.levers[0] > 0 and bb.levers[1] > 0


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
    guard = TapeGuard("TAPE GUARD (latched)")
    return Selector("root", [
        Sequence("out", [
            Condition("out button pressed?", lambda bb: bb.out_pressed or bb.is_out),
            GoOut("OUT: stopped, sad song"),
        ]),
        Sequence("tape guard", [
            Condition("tape detected or latched?", lambda bb: bb.tape or guard.latched),
            guard,
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


# ---------------- Sensors ----------------

def is_tape(reading):
    if TAPE_MIN_SATURATION is None:
        return False
    if math.isnan(reading.hue) or math.isnan(reading.saturation):
        return False
    return (TAPE_HUE_MIN <= reading.hue <= TAPE_HUE_MAX
            and reading.saturation >= TAPE_MIN_SATURATION)


class TapeWatcher:
    """Checks every color reading the moment it arrives (library callback),
    so a quick pass over the tape between two loop ticks is never missed.
    Also counts readings so we can show the real update rate."""

    def __init__(self, color_sensor):
        self.tape_now = False
        self.tape_seen = False       # sticky until the main loop reads it
        self.count = 0               # all readings
        self.new_count = 0           # readings whose values actually changed
        self._last_values = None
        if color_sensor is not None:
            color_sensor.set_notification_callback(self._on_notification)

    def _on_notification(self, data):
        # Runs on the library's Bluetooth thread: keep it short, no device calls.
        for item in le.device_notification_parser(data):
            if isinstance(item, le.ColorSensorNotification):
                self.count += 1
                values = (item.rawRed, item.rawGreen, item.rawBlue, item.reflection)
                if values != self._last_values:
                    self.new_count += 1
                    self._last_values = values
                self.tape_now = is_tape(item)
                if self.tape_now:
                    self.tape_seen = True

    def read(self):
        """True if tape is under the sensor now or was seen since the last call."""
        seen = self.tape_seen or self.tape_now
        self.tape_seen = False
        return seen


class RateMeter:
    """Turns an ever-increasing counter into updates per second."""

    def __init__(self):
        self.last_time = time.time()
        self.last_count = 0
        self.hz = 0.0

    def update(self, count, now):
        if now - self.last_time >= 1.0:
            self.hz = (count - self.last_count) / (now - self.last_time)
            self.last_time, self.last_count = now, count


class DistanceListener:
    """Keeps the latest distance from the Uno Q. Works fine with no messages.
    paho runs its network loop on its own thread, so nothing here blocks."""

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


# ---------------- Sound ----------------

class SongPlayer:
    """Plays a list of (Hz, seconds) notes without blocking the main loop:
    call update() every loop and it starts the next note when it's due."""

    def __init__(self, motor):
        self.motor = motor
        self.notes = []
        self.next_time = None        # None = not playing

    def play(self, notes, now):
        self.notes = list(notes)
        self.next_time = now

    def update(self, now):
        if self.next_time is None or now < self.next_time:
            return
        if self.notes:
            freq, seconds = self.notes.pop(0)
            self.motor.beep(frequency=freq, blocking=False)
            self.next_time = now + seconds
        else:
            self.motor.stop_beep(blocking=False)
            self.next_time = None    # finished


# ---------------- Main ----------------

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
    # The motor doesn't need to send us readings (notify 0 saves Bluetooth time).
    motor = None
    if not dry:
        motor = le.DoubleMotor()
        if not connect(motor, "Double Motor", notify_ms=0):
            sys.exit("Need the Double Motor.")
    color_sensor = le.ColorSensor()
    if not connect(color_sensor, "Color Sensor", notify_ms=SENSOR_NOTIFY_MS):
        color_sensor = None
    controller = le.Controller()
    if not connect(controller, "Controller", notify_ms=SENSOR_NOTIFY_MS):
        controller = None
    if not dry and (color_sensor is None or controller is None):
        sys.exit("Need the Color Sensor (tape) and Controller (out button) for a real run.")

    tape_watcher = TapeWatcher(color_sensor)
    distance = DistanceListener()
    song = SongPlayer(motor)

    if color_sensor is not None:
        start = tape_watcher.new_count
        time.sleep(1.0)
        print(f"\nColor sensor: {tape_watcher.new_count - start} new readings in 1 s "
              f"(asked for {1000 / SENSOR_NOTIFY_MS:.0f} Hz)")

    tree = build_tree()
    bb = Blackboard()
    sent_drive = None
    last_node = None
    loop_count = 0
    loop_rate, color_rate = RateMeter(), RateMeter()
    next_print = 0.0
    next_tick = time.time()

    print("\nRunning" + (" in DRY RUN (motors disabled)" if dry else "") + ". Ctrl-C to stop.\n")
    try:
        while True:
            # 1. Read inputs (all just reading variables, nothing waits)
            bb.now = time.time()
            bb.tape = tape_watcher.read()
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
                        motor.movement_stop(blocking=False)
                    else:
                        send_tank(motor, *bb.drive, blocking=False)
                sent_drive = bb.drive
            for job in bb.requests:
                if dry:
                    print(f"\n[dry-run] would {job.replace('_', ' ')}")
                elif job == "publish_out":
                    distance.publish_out()
                elif job == "sad_song":
                    song.play(SAD_SONG, bb.now)
                elif job == "tape_beep":
                    motor.beep(frequency=TAPE_BEEP_HZ, blocking=False)
            bb.requests = []
            if motor:
                song.update(bb.now)

            # 4. Print: a permanent line when the node changes, plus a live line
            loop_count += 1
            loop_rate.update(loop_count, bb.now)
            color_rate.update(tape_watcher.new_count, bb.now)
            if bb.active_node != last_node:
                print(f"\n>>> {bb.active_node}")
                last_node = bb.active_node
            if bb.now >= next_print:
                next_print = bb.now + 1 / PRINT_HZ
                dist = f"{bb.opponent_cm:3d}cm" if bb.opponent_cm is not None else "  --"
                print(f"\r{bb.active_node:24s} | motors L {bb.drive[0]:4d} R {bb.drive[1]:4d} "
                      f"| tape {'YES' if bb.tape else 'no ':3s} | dist {dist} "
                      f"| levers {bb.levers[0]:4d} {bb.levers[1]:4d} "
                      f"| loop {loop_rate.hz:3.0f} Hz color {color_rate.hz:3.0f} Hz   ",
                      end="", flush=True)

            # Sleep until the next tick, so the loop rate stays steady
            next_tick += 1 / LOOP_HZ
            delay = next_tick - time.time()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.time()  # running late: don't try to catch up
    except KeyboardInterrupt:
        pass
    finally:
        print("\nStopping and disconnecting...")
        if motor and motor.connected:
            motor.movement_stop()    # blocking here on purpose: make sure it stops
        distance.stop()
        for device in (motor, color_sensor, controller):
            if device and device.connected:
                device.disconnect()


if __name__ == "__main__":
    main()
