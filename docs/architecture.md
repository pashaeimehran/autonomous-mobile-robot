# Software architecture

## Overview

The robot uses ROS 1 to connect a 2D laser scanner, Cartographer SLAM, navigation planning, and differential-drive motor control. A Raspberry Pi 4B hosts the motor interface and communicates with an L298N dual H-bridge through GPIO.

| Subsystem | Interface | Implementation |
|---|---|---|
| Laser scanner | `/scan` (`sensor_msgs/LaserScan`) | YDLIDAR X2 ROS driver |
| Mapping | `/scan`, `/map`, TF | Google Cartographer in 2D mode |
| Navigation | `/map`, `/scan`, `/cmd_vel` | `move_base`, Navfn global planner, DWA local planner |
| Low-level control | `/cmd_vel` (`geometry_msgs/Twist`) | C++ ROS node and `pigpio` motor driver |
| Manual operation | HTTP input to `/cmd_vel` | Python Flask interface |

## Coordinate frames

```text
map
└── base_link
    └── laser
```

Cartographer supplies the robot pose in the map frame. The URDF defines a fixed `base_link` to `laser` transform with translation `(0.1007, 0, 0.0558)` m. SLAM is configured without encoder odometry or an IMU. The local costmap uses `map` as its global frame, consistent with the available transform chain.

The motion controller is open-loop. It does not publish wheel odometry, estimate robot velocity, or implement a closed-loop wheel-speed controller. DWA motion estimates are consequently limited by the available ROS odometry inputs.

## Motor driver

The motor driver is split into three parts:

1. **`DifferentialDriveMixer`** combines bounded linear and angular demands into signed left/right wheel commands. It has no ROS or GPIO dependencies.
2. **`MotorDriver`** sets motor direction pins and PWM duty cycles through `pigpio`. Outputs are disabled before direction changes and when the driver stops.
3. **`motor_driver_node`** subscribes to `/cmd_vel`, loads private ROS parameters, and stops the driver when no valid message arrives within the configured timeout.

Default open-loop mixing gains are `240` PWM-% per m/s and `50` PWM-% per rad/s. Wheel commands are scaled together when their magnitudes exceed the PWM range, preserving the requested steering ratio.

## Operating modes

**Autonomous navigation:** `navigation.launch` includes robot bringup and Cartographer before starting `move_base` with the DWA local planner.

**Manual control:** `bringup.launch` starts the motor node. The Flask interface publishes velocity commands from browser button and keyboard input. Manual and navigation publishers must not drive `/cmd_vel` simultaneously; no command multiplexer is included.

## Configuration

- `config/cartographer_2d.lua`: scan matching and map generation.
- `config/costmap_common_params.yaml`: LiDAR obstacles, footprint, inflation.
- `config/global_costmap_params.yaml`: global costmap frame and publication.
- `config/local_costmap_params.yaml`: rolling local costmap.
- `config/dwa_local_planner_params.yaml`: local planner constraints.
- `launch/bringup.launch`: motor limits, PWM scaling, watchdog.

The motor watchdog stops on missing commands but is a software control mechanism, not a substitute for an electrical emergency stop.
