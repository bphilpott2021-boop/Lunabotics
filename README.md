# Lunabotics 2026 – Software

This repo contains all software for our Lunabotics 2026 rover, including:
- ROS 2 (Jazzy) workspace for the rover + simulation
- Motor control / PID experiments
- Utility scripts

## Repo Structure

```text
Lunabotics/
  bash/                 # Existing scripts (from BP)
  pid_test_BP/          # PID experiments from BP
  ros2_ws/              # ROS 2 Jazzy workspace (main focus now)
    src/
      rover_teleop/     # Teleop + joystick config (Python ROS package)
      rover_arduino_bridge/  # Bridge between ROS 2 and Arduino over USB
      rover_sim/        # Simple simulation of Arduino + motors for testing
  README.md
  .gitignore



# Lunabotics – Phase 1 (Keyboard Teleop + Simple Sim)

This repo provides a minimal, hardware-free pipeline to exercise the rover control flow in software only.

What’s included (relevant files)
- Keyboard teleop (publishes Twist):
  - ros2_ws/src/rover_teleop/rover_teleop/keyboard_teleop.py
- Simple wheel sim (consumes Twist, publishes odom + ticks):
  - ros2_ws/src/rover_sim/rover_sim/simple_wheel_sim.py
- Package manifests:
  - ros2_ws/src/rover_sim/package.xml
  - ros2_ws/src/rover_teleop/package.xml
- Optional (not used in Phase 1): joystick launch
  - ros2_ws/src/rover_teleop/launch/teleop_joy.launch.py

Requirements
- Ubuntu 24.04 with ROS 2 Jazzy installed
- Python 3.12 (default on Ubuntu 24.04)

Build once per change
```bash
cd ~/Lunabotics/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Run (three terminals)
In each terminal:
```bash
cd ~/Lunabotics/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
# optional
export ROS_DOMAIN_ID=0
```

- Terminal A (sim):
```bash
ros2 run rover_sim simple_wheel_sim
# Prints: "SimpleWheelSim started: listening on /motor/cmd_vel"
```

- Terminal B (keyboard teleop → /motor/cmd_vel):
```bash
ros2 run rover_teleop keyboard_teleop --ros-args -p topic:=motor/cmd_vel
# Controls:
#   w: toggle forward ON/OFF (ramped)
#   s or space: stop forward
#   a/d: turn left/right (in-place)
#   x: stop rotation
#   q: quit
```

- Terminal C (observe):
```bash
# Check wiring
ros2 topic info /motor/cmd_vel
ros2 topic info /wheel_odom_raw

# Check rate and data
ros2 topic hz /wheel_odom_raw
ros2 topic echo /wheel_odom_raw
```

Expected behavior
- Terminal B shows key events (Forward ON/OFF, Turning…).
- Terminal A logs received commands (v, w) and a once-per-second state line: x, y, yaw.
- Terminal C shows Odometry messages at ~50 Hz; when moving forward (w ON), pose.position.x increases; when turning (a/d), orientation z/w changes.

Troubleshooting (Phase 1)
- If Terminal C shows zeros but Terminal A “State …” changes:
  - Ensure all terminals sourced both: /opt/ros/jazzy/setup.bash and ros2_ws/install/setup.bash
  - Verify the topic is correct:
    - ros2 param get /keyboard_teleop topic  # should be motor/cmd_vel
    - ros2 topic echo /motor/cmd_vel -n 3    # should show nonzero linear.x when w is ON
  - Confirm publishers/subscribers:
    - ros2 topic info /wheel_odom_raw        # should list 1 publisher (simple_wheel_sim)
  - Encoder sanity check:
    - ros2 topic echo /wheel_ticks -n 5      # should increment while moving

Notes
- Phase 1 is keyboard-only. The joystick launch file exists but is not used yet.
- Keep build/install/log out of git (see .gitignore).
```

How to commit and push the README changes
- From the repo root:
```bash
cd ~/Lunabotics
git pull           # get latest from origin/main first
git add README.md
git commit -m "docs: add Phase 1 (keyboard teleop + sim) quickstart and troubleshooting"
git push origin main