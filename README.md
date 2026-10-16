# ME193 Sumo Robot: Shared Control

A LEGO sumo robot for ME193. The human drives with the LEGO Controller, and the
robot shares control. It vetoes driving forward over the ring's blue tape edge,
and it adds full power when the opponent is close and the human is already
charging. The decisions come from a behavior tree that combines three inputs:
the human's levers, a LEGO color sensor watching for the tape, and an
ultrasonic distance sensor on an Arduino UNO Q that reports over MQTT.

The rules: the ring is bounded by blue tape, and the first robot pushed fully
out loses. The losing robot must stop and play a sad song.

## Architecture

```
 Arduino UNO Q ("AirFour", on the robot)           Laptop
 ┌──────────────────────────────────┐              ┌──────────────────────────────────┐
 │ MCU (sketch.ino)                 │              │ laptop/sumo.py                   │
 │   HC-SR04 ping, median of 3      │              │   behavior tree, 50 Hz           │
 │        │ Arduino_RouterBridge    │    MQTT      │     ▲ distance     ▼ motor cmds  │
 │ Linux (python/main.py)           │──── dist ───►│     │ (subscribe)  │              │
 │   publishes distance, 15 Hz      │◄─ status ────│  out status ◄──────┘              │
 │   shows "OUT" on LED matrix      │  HiveMQ      │                                  │
 └──────────────────────────────────┘  public      │   Bluetooth (legoeducation lib)  │
                                       broker      └───────┬──────────┬──────────┬────┘
                                                           ▼          ▼          ▼
                                                     Double Motor  Color     Controller
                                                     (drive)       Sensor    (levers +
                                                                   (tape)    out button)
```

- **Uno Q:** the sketch measures distance and passes it to the board's Linux
  side over `Arduino_RouterBridge`. A Python script there publishes it to MQTT,
  and shows "OUT" on the LED matrix when the robot is out.
- **Laptop:** `sumo.py` subscribes to the distance and reads the color sensor
  and controller over Bluetooth. It runs the behavior tree and commands the
  Double Motor.

## Repository layout

| Path | What it is |
|---|---|
| `laptop/sumo.py` | Main control loop and behavior tree (the match program) |
| `laptop/bt.py` | Small behavior tree library: `Selector`, `Sequence`, `Condition`, `Action` |
| `laptop/test_lego.py` | Hardware test: live readings, color calibration, lever debug, drive test |
| `unoq/` | Arduino App Lab app for the UNO Q (`sketch/` for the MCU, `python/` for Linux) |
| `logs/` | Reserved for tuning-run logs (currently empty) |
| `requirements.txt` | Laptop Python dependencies |
| `CLAUDE.md` | Project notes and conventions used while developing |

## Hardware and wiring

| Part | Role |
|---|---|
| LEGO Education Double Motor | Tank drive (left and right motors) |
| LEGO Education Color Sensor | Detects the blue tape (hue + saturation) |
| LEGO Education Controller | Two levers for tank drive; its button is the "out" button |
| Arduino UNO Q | Reads the HC-SR04, publishes distance over Wi-Fi/MQTT |
| HC-SR04 ultrasonic sensor | Opponent distance (forward-facing) |

The three LEGO devices are paired to one LEGO Connection Card (orange, serial
0999). The code connects only to devices advertising that serial, because other
teams' devices show up in the same Bluetooth scans.

The robot is limited to 1 kg and must fit in a 30 cm cube.

### HC-SR04 to UNO Q

The HC-SR04 runs at 5 V and its Echo output is 5 V. The UNO Q's MCU pins are
3.3 V, so Echo goes through a 1 kΩ / 2 kΩ voltage divider
(5 V × 2k / (1k + 2k) ≈ 3.3 V). The UNO Q's 3.3 V Trig output is enough to
trigger the sensor.

```
HC-SR04 VCC  ── 5V
HC-SR04 GND  ── GND
HC-SR04 Trig ── D2
HC-SR04 Echo ── 1 kΩ ──┬── D3
                       │
                      2 kΩ
                       │
                      GND
```

## Behavior tree

`sumo.py` builds the tree in `build_tree()`. The root is a `Selector`: every
tick (50 Hz) it tries its children in priority order and runs the first one
whose condition holds. Each branch is a `Sequence` of `Condition` node(s) and
then an `Action`. **The robot never moves unless the human is pushing the
levers.**

| Priority | Node | Condition | Action |
|---|---|---|---|
| 1 | **OUT** | Controller button pressed (latches until the program is restarted) | Stop the motors, play the sad song on the laptop speaker (and the Double Motor's beeper), publish `{"state": "out"}` |
| 2 | **TAPE GUARD** | Tape detected, or the guard is still latched (see below) | Follow the levers, but each wheel is clamped to ≤ 0, so it can stop or reverse but not go forward |
| 3 | **BOOST** | Opponent closer than 25 cm **and** both levers pushed forward | Both motors at 100 % forward |
| 4 | **HUMAN** | Either lever outside the 5 % deadzone | Tank drive: left lever drives the left motor, right lever the right motor |
| 5 | **IDLE** | (always) | Motors stop |

TAPE GUARD sits above BOOST, so the robot can never boost over the edge.

### Shared control

The human makes the strategic choices: where to go, when to attack, when to
retreat. The robot uses its sensors to change the human's commands in two ways.
At the edge it removes the human's forward command (TAPE GUARD). When the color
sensor and distance sensor together say charging is safe and useful, it
amplifies the human's charge to full power (BOOST). It never acts on its own:
BOOST also needs the human to be pushing both levers forward.

### Tape guard latch

A single color reading can't tell which side of the tape the robot is on. If
the robot drove across the strip quickly, the sensor would see floor again on
the outside and a simple "tape under the sensor" check would let it keep
driving out. So the guard latches:

1. **Latch:** when tape is detected, the guard turns on and a short beep plays.
   From then on, forward commands are blocked even after the sensor leaves the
   tape.
2. **Release:** the guard turns off once both of these are true:
   - the sensor no longer sees tape, and
   - the human has pulled either lever back since the guard turned on. A pivot
     with one lever back counts.
3. **Pushing forward resets it:** if the human pushes **both** levers forward
   while the guard is on, the earlier pull-back no longer counts and they must
   pull back again. This stops "tap reverse, then drive forward over the
   strip" from releasing the guard on the far side. A one-sided pivot doesn't
   reset it.

If the robot overshoots the tape without the human ever reversing, forward
stays blocked.

### Tape detection

The ring floor has a reddish hue (~19°) at low saturation (~29), and the blue
tape has a hue of ~209° at saturation ~59. Hue alone isn't reliable: low
saturation makes a gray floor's hue random, and hue wraps at 360°. So tape
means **hue between 185° and 235° and saturation ≥ 44**. These values came
from `test_lego.py`'s calibration, which counted 0/20 floor samples and 20/20
tape samples as tape.

Every color reading is checked the moment it arrives (by a Bluetooth
notification callback, about every 20 ms), not only on loop ticks. A brief
pass over the tape between two ticks is still caught.

### Sign convention

In all the logic, a positive speed means physically forward. On this robot,
pushing a lever forward reads negative, and a negative motor command drives
forward. So `test_lego.py` defines `LEFT/RIGHT_LEVER_SIGN = -1` and
`LEFT/RIGHT_MOTOR_SIGN = -1`. The motor signs are applied only in
`send_tank()`, the single place that sends motor commands.

## MQTT

Broker: `broker.hivemq.com`, port 1883 (public, no authentication).

| Topic | Publisher → subscriber | Payload |
|---|---|---|
| `ME193/ceci/sumo/dist` | UNO Q → laptop, 15 Hz | `{"dist_cm": 23, "t": 1792174827.298}`. `dist_cm` is an int in cm; `-1` = no target within 80 cm. `t` is Unix time in seconds |
| `ME193/ceci/sumo/status` | laptop → UNO Q | `{"state": "out"}` shows "OUT" on the UNO Q LED matrix. Any other state, e.g. `{"state": "in"}`, clears it |

On the laptop, a distance older than 0.5 s counts as "no opponent". If the
UNO Q or the network is down, BOOST is simply never triggered, and everything
else works. The UNO Q skips readings while disconnected rather than queuing
them, so the laptop never acts on stale distances.

### Distance measurement (UNO Q sketch)

- The sketch pings continuously, about every 20 ms. The published value is
  the **median of the last 3 pings**, so one missed echo doesn't hide a real
  target, but two do.
- Readings beyond **80 cm** are `-1`.
- The echo is timed with `micros()`. This core's `pulseIn()` takes its timeout
  in milliseconds and has coarse timing.
- Setting `DEBUG = True` in `unoq/python/main.py` logs every ping's raw echo
  timing (rise time and pulse length in µs), to tell real echoes from
  electrical noise.

## Setup and running

### 1. Laptop environment

Python 3 on macOS (developed with Python 3.14). From the repository root:

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt     # legoeducation, paho-mqtt
```

Turn on the three LEGO devices with the Connection Card paired. The first run
may ask for Bluetooth permission for the terminal.

### 2. Hardware test and color calibration: `laptop/test_lego.py`

```bash
.venv/bin/python laptop/test_lego.py
```

- **At startup:** lists every nearby LEGO device with its card color and
  serial (ours are marked `<-- OURS`), then connects to ours only.
- **Live line:** controller levers and color sensor readings at 10 Hz.
- **Keys:**
  - `c`: calibrate. Hold the sensor over the floor and press a key, then over
    the tape and press a key. It prints floor and tape statistics for hue,
    saturation and reflection, suggests `TAPE_HUE_MIN`, `TAPE_HUE_MAX` and
    `TAPE_MIN_SATURATION` for the top of `sumo.py`, and reports how many floor
    and tape samples those values would classify as tape.
  - `d`: toggle lever tank-drive.
  - `m`: short motor spin.
  - `q`: quit (motors stop).

Lever debug, controller only:

```bash
.venv/bin/python laptop/test_lego.py --levers
```

It prints raw lever percent and angle, plus each value's range, to check the
sign convention.

### 3. UNO Q app: `unoq/`

The UNO Q runs one App Lab app at a time. To install from the laptop over SSH
(user `arduino`), as app `sumo_distance`:

```bash
scp -r unoq arduino@<board-ip>:ArduinoApps/sumo_distance
ssh arduino@<board-ip> 'arduino-app-cli app start ~/ArduinoApps/sumo_distance'
```

To update an existing install, copy the folder's contents instead and restart:

```bash
scp -r unoq/* arduino@<board-ip>:ArduinoApps/sumo_distance/
ssh arduino@<board-ip> 'arduino-app-cli app stop ~/ArduinoApps/sumo_distance; arduino-app-cli app start ~/ArduinoApps/sumo_distance'
```

The first start compiles and flashes the sketch and installs `paho-mqtt`; the
board needs internet. To check it's running:

```bash
ssh arduino@<board-ip> 'arduino-app-cli app list | grep sumo'                   # "running"
ssh arduino@<board-ip> 'arduino-app-cli app logs ~/ArduinoApps/sumo_distance'   # status every 5 s
```

A healthy status line looks like
`MQTT up | 15.0 msgs/s | skipped 0 | last dist 24 cm | matrix clear`.
The app can also be opened and run from the Arduino App Lab desktop app.

### 4. The match: `laptop/sumo.py`

```bash
.venv/bin/python laptop/sumo.py --dry-run   # never connects the motor: safe to test logic
.venv/bin/python laptop/sumo.py             # real run
```

- A real run refuses to start without the motor, color sensor and controller.
  `--dry-run` works with whatever devices are present, and prints the motor
  beeps and MQTT publishes it would make instead of doing them. The sad song
  still plays on the laptop speaker, so it can be tested.
- The sad song is synthesized into a short `.wav` file at startup and played
  with macOS's `afplay` in the background, so it never pauses the control loop.
- At startup it prints the measured color sensor update rate.
- **Live line:** active node, motor commands, tape, distance, levers, and
  actual loop and color-sensor rates. A new line is printed each time the
  active node changes.
- **Ctrl-C** stops the motors and disconnects.
- Nothing in the loop waits on hardware: sensor data arrives by callback,
  motor and sound commands are non-blocking, and MQTT runs on its own thread.

## Known limitations

- **One forward-facing distance sensor.** The HC-SR04 has a narrow cone
  (roughly ±15°) and an 80 cm cap, so it only sees opponents in front. BOOST
  never triggers for opponents beside or behind the robot.
- **The tape guard only protects forward motion.** Reversing is always
  allowed, so the robot can still be driven backwards out of the ring. The one
  color sensor only sees the tape where it's mounted.
- **The robot can still be pushed out.** The guard blocks the human's forward
  commands, but it can't stop the opponent shoving the robot over the edge.
- **Calibration depends on the ring and lighting.** The hue and saturation
  thresholds should be re-checked with `test_lego.py` (`c`) in the actual
  arena.
- **The public MQTT broker.** `broker.hivemq.com` adds network latency and has
  no authentication, so anyone could publish to these topics, including a fake
  `"out"` status. If the broker or Wi-Fi drops, BOOST is unavailable; driving
  and the tape guard don't depend on MQTT.
- **The "out" button is the Controller's main button.** OUT latches until
  `sumo.py` is restarted.
- **Bluetooth dropouts.** If the controller disconnects, `sumo.py` treats the
  levers as released and stops. If the laptop loses the motor connection
  mid-command, behavior depends on the motor firmware. This hasn't been
  tested.
- **Lever input is passed straight through.** There is no speed ramping or
  smoothing beyond the 5 % deadzone.
- **No match logging yet.** `logs/` is reserved for tuning-run CSVs, but the
  current code doesn't write any.
