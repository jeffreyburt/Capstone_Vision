# Capstone head tracking and vision-guided driving

The robot has two operating modes. In parked mode, the robot-mounted camera and browser face detector aim the speaker using the existing head tracker. In drive mode, the overhead vision camera tracks the ArUco markers and the laptop commands the chassis. The speaker stays level and faces forward during drive mode because the markers are attached to it.

## Layout and setup

- Copy the repository to each robot Pi. `head_tracker/dashboard.py` and its assets run the parked head tracker. `drive_with_head_lock.py` and `robot/drive_server.py` run the wheels and verify servo feedback in drive mode.
- On the laptop, run the team's Capstone_Vision checkout. Use `control/` and `config.yaml` to configure the vision UDP port (5005), the robot drive port (5700), robot IDs, addresses, and neutral marker geometry. The integration uses the existing UDP protocol. Follow the [MVP runbook](MVP_RUNBOOK.md) for the laptop setup, dry run, simulation, and wheel calibration.
- Use the [head-tracker README](../head_tracker/README.md) to set up the Pi camera and UART. The integrated drive command runs in the `capstone-base` virtual environment and needs both `capstone_base` and `pyserial` importable there. Both robot programs use `/dev/serial0`; verify that it points to the BusLinker UART on each Pi.

## Parked head tracking

1. Stop the laptop follow controller, then stop the drive server on each robot. Stop any other programs controlling the wheels.
2. From the repository root on the robot, run `/usr/bin/python3 head_tracker/dashboard.py`, or start the installed service for this copy. Open `http://ROBOT_IP:8080/`.
3. Engage torque and start head tracking as described in the tracker README. Keep the browser tab visible. In this mode, the moving markers should not be used for chassis navigation.

## Vision-guided driving

1. Stop tracking on all robots visible to the overhead camera. Put each speaker in its calibrated neutral pose, level and facing the chassis front, and engage both servos to hold it there. Use the dashboard's position capture to record that robot's neutral pan and tilt in servo units. Keep the mount supported if you disengage torque for manual positioning.
2. Stop the dashboard process or service, and close other servo and wheel controllers. With the wheels lifted for the first run, start the integrated drive server from the repository root. Replace the uppercase placeholders with measured or configured values.

   ```sh
   ~/capstone-base/.venv/bin/python drive_with_head_lock.py \
     --neutral PAN_UNITS TILT_UNITS -- \
     --robot-id ROBOT_ID --allow-from LAPTOP_IP
   ```

   The wrapper enables hardware and passes the arguments after `--` to the handoff drive server. If needed, pass the robot's `--hiwonder-config`, `--counts-per-meter`, and `--turn-radius` there as well. The wrapper requires valid in-tolerance positions and engaged torque from both servos before starting. It holds exclusive access to the UART and monitors feedback during operation. A missing reply, loss of torque, or out-of-tolerance position stops the wheels and exits the drive server. Recheck neutral before restarting.
3. Start or restart `vision.main` on the laptop so it reacquires the markers in the neutral pose. Verify robot IDs, reported positions and headings, and the user position. Run `python -m control.main` for a dry run before following the runbook's armed tests.
4. To return to parked tracking, stop the laptop controller and the robot drive server before restarting the dashboard. Repeat the neutral and vision checks before every drive session.

For UDP simulation, use `python3 -m robot.drive_server` with the handoff's normal options and a valid wheel-mapping file. Direct hardware use of the handoff server bypasses the neutral checks, so use the wrapper for integrated hardware runs. Leave other serial tools closed during drive mode.

## Hardware-free checks

```sh
python3 -m unittest discover -s tests -p 'test_tracking_control.py' -v
python3 -m unittest discover -s tests -p 'test_head_lock.py' -v
python3 -m pytest -o pythonpath=. tests/test_drive_server.py
```
