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
