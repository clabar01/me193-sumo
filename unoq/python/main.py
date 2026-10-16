"""Sumo distance sensor (Linux side).

Reads the HC-SR04 distance from the sketch over the Bridge and publishes it
to ME193/ceci/sumo/dist at ~15 Hz:
    {"dist_cm": <int>, "t": <Unix time s>}     dist_cm -1 = no target

Also listens on ME193/ceci/sumo/status: {"state": "out"} shows "OUT" on the
LED matrix; any other state clears it.

DEBUG = True logs every single ping from the sketch: how long after the
trigger the echo went high (rise), the raw echo pulse in microseconds, the
ping's cm, and the median being published. A real HC-SR04 echo rises a few
hundred us after the trigger (it sends its 8-pulse burst first); a rise near
0 us or a pulse of only ~100-200 us (2-3 cm) suggests electrical noise.

MQTT uses connect_async + loop_start, so paho connects and reconnects in the
background and nothing here waits on the network. While disconnected,
readings are skipped (not queued), so the laptop never gets stale distances.
"""
import json
import time

import paho.mqtt.client as mqtt

from arduino.app_utils import App, Bridge, Logger

logger = Logger("sumo_distance")

# ---------------- Tunable constants ----------------
BROKER_HOST = "broker.hivemq.com"
BROKER_PORT = 1883
DIST_TOPIC = "ME193/ceci/sumo/dist"
STATUS_TOPIC = "ME193/ceci/sumo/status"

PUBLISH_HZ = 15
STATUS_LOG_SECONDS = 5           # log rate / MQTT state / last distance this often
RECONNECT_MIN_S, RECONNECT_MAX_S = 1, 10

DEBUG = False                    # True: log every ping's raw echo timing (see above)
# ---------------------------------------------------

# Codes the sketch sends instead of a pulse length (see ECHO_* in sketch.ino)
ECHO_CODES = {-1: "no echo", -2: "too far", -3: "busy"}

connected = False
debug_sent = False               # told the sketch about DEBUG yet?
want_out = False                 # set from MQTT messages
shown_out = None                 # last value sent to the sketch (None = never)

published = 0
skipped = 0
last_dist = None
last_status_log = time.monotonic()
next_publish = time.monotonic()


def on_connect(client, userdata, flags, reason_code, properties):
    global connected
    connected = not reason_code.is_failure
    if connected:
        client.subscribe(STATUS_TOPIC)
    logger.info(f"MQTT connected to {BROKER_HOST} ({reason_code})")


def on_disconnect(client, userdata, flags, reason_code, properties):
    global connected
    if connected:
        logger.warning(f"MQTT disconnected ({reason_code}); reconnecting in the background")
    connected = False


def on_message(client, userdata, msg):
    """Runs on paho's thread: only record what we want; loop() talks to the sketch."""
    global want_out
    try:
        state = json.loads(msg.payload).get("state")
    except (ValueError, AttributeError):
        logger.warning(f"Ignoring bad status message: {msg.payload[:80]!r}")
        return
    want_out = state == "out"
    logger.info(f"status: {state!r} -> matrix {'OUT' if want_out else 'clear'}")


def ping_debug(rise_us: int, echo_us: int, cm: int, median: int):
    """Called by the sketch (Bridge.notify) for every ping while DEBUG is on."""
    echo = ECHO_CODES.get(echo_us, f"{echo_us:5d} us")
    logger.info(f"ping: rise {rise_us:5d} us | echo {echo:>8s} | ping {cm:3d} cm | "
                f"median {median:3d} cm")


Bridge.provide("ping_debug", ping_debug)

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_disconnect = on_disconnect
client.on_message = on_message
client.reconnect_delay_set(RECONNECT_MIN_S, RECONNECT_MAX_S)
client.connect_async(BROKER_HOST, BROKER_PORT, keepalive=30)
client.loop_start()


def loop():
    global published, skipped, last_dist, shown_out, last_status_log, next_publish, debug_sent

    # 0. Tell the sketch whether to send per-ping debug info (once)
    if not debug_sent:
        try:
            Bridge.call("set_debug", DEBUG)
            debug_sent = True
            logger.info(f"ping debug {'ON' if DEBUG else 'off'}")
        except Exception as e:
            logger.warning(f"set_debug failed: {e}")

    # 1. Distance: sketch -> MQTT
    try:
        last_dist = int(Bridge.call("get_distance"))
        if connected:
            payload = json.dumps({"dist_cm": last_dist, "t": round(time.time(), 3)})
            client.publish(DIST_TOPIC, payload, qos=0)   # fire-and-forget
            published += 1
        else:
            skipped += 1
    except Exception as e:
        logger.warning(f"get_distance failed: {e}")

    # 2. LED matrix: only call the sketch when it changes
    if want_out != shown_out:
        try:
            Bridge.call("set_out", want_out)
            shown_out = want_out
        except Exception as e:
            logger.warning(f"set_out failed: {e}")

    # 3. Status line every few seconds
    now = time.monotonic()
    if now - last_status_log >= STATUS_LOG_SECONDS:
        rate = published / (now - last_status_log)
        logger.info(f"MQTT {'up' if connected else 'DOWN'} | {rate:4.1f} msgs/s | "
                    f"skipped {skipped} | last dist {last_dist} cm | "
                    f"matrix {'OUT' if shown_out else 'clear'}")
        published = skipped = 0
        last_status_log = now

    # 4. Keep a steady PUBLISH_HZ
    next_publish += 1 / PUBLISH_HZ
    delay = next_publish - time.monotonic()
    if delay > 0:
        time.sleep(delay)
    else:
        next_publish = time.monotonic()   # running late: don't try to catch up


try:
    App.run(user_loop=loop)
finally:
    client.loop_stop()
    client.disconnect()
