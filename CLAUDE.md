# ME193 Sumo Wrestling Robot

## Assignment
Robot sumo match in a blue tape ring. First robot pushed fully out loses and must stop and play a sad song.
- Built with LEGO hardware, Arduino Uno Q used as a sensor, communicating over MQTT
- Max 1 kg, fits in a 30 cm cube
- Must use one of: behavior trees, decision trees, KNN, or K-means. We are using a behavior tree.
- At least two sensors: at least one autonomous, at least one with teleoperation
- Must have meaningful shared control: human and robot make decisions together based on sensor fusion
- Deliverables: GitHub repo + Notion page explaining the policy, autonomous sensors, teleop/shared control, and strengths/weaknesses
- Due Monday 11:59pm

## Hardware
- LEGO Double Motor (drive), LEGO color sensor (blue tape detection), LEGO controller (teleop)
- Controlled from laptop using the legoeducation Python library
- LEGO Connection Card: orange diamond, serial 0999. Other students' devices show up in scans, so always connect with card_serial="0999" and never to the first device found
- Uno Q board named AirFour, SSH user: arduino
- Uno Q with HC-SR04: Trig on D2, Echo on D3 through a 1k/2k voltage divider
- legoeducation has a Controller class (controller.sensor.leftPercent / rightPercent / leftAngle / rightAngle), but not yet confirmed on our hardware. Fallback is WASD keyboard teleop on the laptop (see laptop/test_lego.py).

## Architecture
- Uno Q: reads HC-SR04, publishes distance over MQTT
- Laptop: subscribes to MQTT, reads LEGO color sensor and controller, runs the behavior tree, commands motors

## MQTT
- Broker: broker.hivemq.com, port 1883
- ME193/ceci/sumo/dist: {"dist_cm": int, "t": timestamp}, -1 = no target
- ME193/ceci/sumo/status: {"state": "out"}

## Behavior tree (priority order)
The robot never moves unless the human is pushing the levers.
1. OUT: controller button tapped: stop, sad song, publish out status
2. TAPE GUARD: tape detected: block any forward motor command (clamp to 0 or below) but allow reversing, so the human can back away from the edge. Latches until the sensor no longer sees tape AND either lever has been pulled back at some point since the latch started (overshooting the tape without reversing keeps forward blocked)
3. BOOST: opponent closer than 25 cm AND both levers pushed forward: both motors full forward
4. HUMAN: levers outside the deadzone: tank drive
5. IDLE: levers at rest: motors stop

## Coding preferences
- Keep code simple and readable, this is for a class
- All tunable values as constants at the top of files
- Behavior tree implemented as node classes so the structure is clear

## Repo
- GitHub: https://github.com/clabar01/me193-sumo (linked from our Notion page, so keep it clean and readable)
- Local folder: ~/Desktop/sumo
- Structure:
  - unoq/      Uno Q app (sketch + Python MQTT publisher)
  - laptop/    control script, behavior tree, tests
  - logs/      CSV logs from tuning runs
- Commit and push after each working milestone with clear commit messages
- Never commit secrets or credentials
