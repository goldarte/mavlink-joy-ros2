# mav_joy — mavlink-joystick → `/joy`

ROS 2 (Jazzy) node that turns the [mavlink-joystick] app into a ROS joystick: it
answers the app's MAVLink discovery with `HEARTBEAT` and republishes every
`MANUAL_CONTROL` (message 69) it receives as `sensor_msgs::msg::Joy` on `/joy`.

```
  phone (mavlink-joystick app)                    robot / PC
  ┌───────────────────────────┐                 ┌──────────────────────────┐
  │ listens on UDP 14550      │◀── HEARTBEAT ───│ mav_joy_node             │
  │ (auto-detect, sys 1 cmp 1)│   1 Hz, sys 1   │ UDP 14550                │
  │                           │                 │                          │
  │ MANUAL_CONTROL, 50 Hz     │─── UDP ────────▶│ ──▶ /joy (Joy) 50 Hz     │
  └───────────────────────────┘                 └──────────────────────────┘
```

The app is a MAVLink GCS: it starts streaming `MANUAL_CONTROL` only after it has
seen a `HEARTBEAT` from a vehicle whose system/component id match its settings
(1/1, auto-detect on by default). This node provides exactly that heartbeat, so
no setup is needed on the app side beyond being on the same network.

## Build

```bash
source /opt/ros/jazzy/setup.bash
export PATH="$HOME/.cargo/bin:$PATH"           # colcon-ros-cargo needs cargo
cd ~/projects/ros2_ws
colcon build --packages-select mav_joy
source install/setup.bash
```

Rust-only iteration (no colcon) works as well:

```bash
source /opt/ros/jazzy/setup.bash
cd src/mavlink-joy-ros2 && cargo test && cargo build
```

## Run

```bash
ros2 run mav_joy mav_joy_node
```

With parameters:

```bash
ros2 run mav_joy mav_joy_node --ros-args \
  -p bind_port:=14550 \
  -p app_address:=192.168.1.42 -p app_port:=14550
```

## App settings

The node works with the app defaults; the app only has to be told which network
to use (it follows the Wi-Fi network it is bound to).

| App setting | Value |
|---|---|
| Host / port (manual mode) | IP of the machine running this node, port `14550` |
| Listen port | `14550` (must match `bind_port`) |
| Auto-detect | on — the node is discovered from its heartbeats |
| Drone system id / component id | `1` / `1` (must match `system_id` / `component_id`) |

## Parameters

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `joy_topic` | string | `/joy` | Topic the joystick messages are published on |
| `frame_id` | string | `mavlink_joystick` | `header.frame_id` of the published messages |
| `publish_rate_hz` | double | `50.0` | Republish rate of `/joy` |
| `bind_address` | string | `0.0.0.0` | Local UDP address |
| `bind_port` | int | `14550` | Local UDP port the app sends to (`0` picks a free one) |
| `app_address` | string | `255.255.255.255` | Where heartbeats are sent until the app is heard |
| `app_port` | int | `14550` | UDP port the app listens on |
| `heartbeat_rate_hz` | double | `1.0` | Rate of the `HEARTBEAT` that enables the app's discovery |
| `system_id` | int | `1` | MAVLink system id of this node |
| `component_id` | int | `1` | MAVLink component id of this node |
| `timeout_sec` | double | `1.0` | Link timeout; `0` disables the failsafe |

`/joy` stays silent until the first `MANUAL_CONTROL` arrives (no invented stick
positions). After `timeout_sec` without a message the node publishes neutral
values (sticks centred, throttle zero) once per tick and logs a warning, so
consumers see a failsafe instead of frozen sticks.

## `/joy` mapping

| `/joy` | `MANUAL_CONTROL` | Range | Meaning |
|---|---|---|---|
| `axes[0]` | `y` | -1..1 | roll, positive right |
| `axes[1]` | `-x` | -1..1 | pitch, positive nose up |
| `axes[2]` | `z` | -1..1 | throttle (`0..1` with the app) |
| `axes[3]` | `r` | -1..1 | yaw, positive clockwise |
| `buttons[0..16]` | `buttons` | 0/1 | the 16 bits of the button bitfield |

Values are the raw MAVLink payload divided by 1000; the app's stick curves,
expo and deadzone are already applied before sending. An axis marked invalid
(`INT16_MAX`) becomes `0.0`.

## Test without a phone

`cargo test` covers the protocol mapping and talks to a real UDP socket. For a
full ROS check, `scripts/fake_app.py` plays the app:

```bash
# terminal 1
ros2 run mav_joy mav_joy_node --ros-args -p bind_port:=14551 -p app_address:=127.0.0.1 -p app_port:=14555
# terminal 2
python3 scripts/fake_app.py 14555 20
# terminal 3
ros2 topic echo /joy
```

## Troubleshooting

**The node runs (the log shows `MANUAL_CONTROL from system 255 component 190`)
but `ros2 topic list` has no `/joy` and `ros2 node list` is empty.**
That is the ROS 2 daemon, not the node: the daemon caches the graph and misses
participants that appeared after it was started (it also keeps stale discovery
state after a network change). Check without the daemon and restart it:

```bash
ros2 node list --no-daemon        # -> /mav_joy_node
ros2 topic list --no-daemon       # -> /joy
ros2 daemon stop                  # the next ros2 command starts a fresh daemon
```

**The log alternates between two `MAVLink peer is ...` addresses.**
Until the app is known, heartbeats are sent to a broadcast address and the
kernel delivers a copy back to this socket, so one of the two addresses is this
machine's own IP. Such self-echoed heartbeats are recognised and ignored now;
the remaining address is the app. If both addresses keep changing, two devices
are talking MAVLink on the same port.

**Nothing arrives from the app.** Check that `bind_port` is the app's listen
port (14550 by default), that the app's drone system/component id are `1`/`1`
and that the app is on the same network. On networks that isolate clients
(visitor Wi-Fi, some hotspots) broadcast may be dropped: set `app_address` to
the phone's IP and `app_port` to its listen port to talk unicast.

## Notes on the Rust/ROS 2 setup

* `rclrs 0.7` is used because it is the release that matches the Rust message
  crates of `ros-jazzy-rosidl-generator-rs` (both use `rosidl_runtime_rs` 0.6).
* The message crates are not registered in the ament index
  (`share/ament_index/resource_index/rust_packages`), so the `[patch.crates-io]`
  entries in `Cargo.toml` point at `/opt/ros/jazzy/share/<pkg>/rust` explicitly.
  Adjust the paths if another distribution is used.
* `build.rs` links empty stubs for `test_msgs`, because rclrs 0.7 asks the
  linker for every message package it vendors and `test_msgs` is not part of a
  standard desktop install. Installing `ros-jazzy-test-msgs` removes the need
  for the stubs.
* MAVLink over UDP is unauthenticated: anyone who can reach `bind_port` can
  drive `/joy`. Keep the robot network trusted.

[mavlink-joystick]: https://github.com/goldarte/mavlink-joystick
