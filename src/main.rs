//! Bridge between the [mavlink-joystick] app and the ROS 2 `/joy` topic.
//!
//! The app is a MAVLink GCS that sends `MANUAL_CONTROL` (message 69) over UDP
//! once it has discovered a vehicle by its `HEARTBEAT`. This node plays the
//! vehicle: it answers with heartbeats and republishes every received
//! `MANUAL_CONTROL` as [`sensor_msgs::msg::Joy`] on `/joy`:
//!
//! | `/joy` axes | MANUAL_CONTROL | range | meaning |
//! |---|---|---|---|
//! | `axes[0]` | `y` | -1..1 | roll, positive right |
//! | `axes[1]` | `-x` | -1..1 | pitch, positive nose up |
//! | `axes[2]` | `z` | -1..1 | throttle, the app sends 0..1 |
//! | `axes[3]` | `r` | -1..1 | yaw, positive clockwise |
//!
//! `buttons` carries the 16 bits of the `MANUAL_CONTROL.buttons` bitfield.
//!
//! When no `MANUAL_CONTROL` arrives for `timeout_sec`, the node keeps `/joy`
//! alive with a neutral (centred, zero throttle) message so that consumers see
//! a failsafe instead of stale sticks.
//!
//! [mavlink-joystick]: https://github.com/goldarte/mavlink-joystick

mod mavlink_link;

use std::net::{IpAddr, SocketAddr};
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::{Context as _, Result, anyhow, bail};
use rclrs::{CreateBasicExecutor, RclrsErrorFilter, SpinOptions};
use sensor_msgs::msg::Joy;

use mavlink_link::{LinkConfig, ManualControl, MavlinkLink};

/// `MANUAL_CONTROL.buttons` is a 16 bit field, so `/joy` reports 16 buttons.
const BUTTON_COUNT: usize = 16;

fn main() -> Result<()> {
    let context = rclrs::Context::new(std::env::args(), rclrs::InitOptions::new())?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("mav_joy_node")?;

    let parameters = Parameters::declare(&node)?;
    let logger = node.logger().clone();

    let link = Arc::new(MavlinkLink::open(parameters.link.clone())?);
    let local_address = link
        .local_addr()
        .context("cannot read the local MAVLink address")?;
    link.start()?;

    let publisher = node.create_publisher::<Joy>(parameters.joy_topic.as_str())?;
    let clock = node.get_clock();

    let mut link_was_up = false;
    let timer = node.create_timer_repeating(parameters.publish_period, {
        let link = Arc::clone(&link);
        let logger = logger.clone();
        let frame_id = parameters.frame_id.clone();
        let timeout = parameters.timeout;
        move || {
            let now = Instant::now();
            let reading = link.manual_control();
            let fresh = reading.is_fresh(now, timeout);

            // Nothing was ever received: stay silent instead of inventing
            // stick positions for a vehicle that has no joystick yet.
            if !fresh && reading.received_at.is_none() {
                return;
            }

            if fresh != link_was_up {
                link_was_up = fresh;
                if fresh {
                    rclrs::log_info!(
                        &logger,
                        "MANUAL_CONTROL received from system {} component {} (peer {:?})",
                        reading.system_id,
                        reading.component_id,
                        link.peer(),
                    );
                } else {
                    rclrs::log_warn!(
                        &logger,
                        "no MANUAL_CONTROL for {:?}, publishing neutral /joy values",
                        timeout,
                    );
                }
            }

            // The failsafe keeps control centred and the throttle at zero.
            let values = if fresh {
                reading
            } else {
                ManualControl::neutral()
            };
            let message = joy_message(&values, &frame_id, clock.now().nsec);
            if let Err(error) = publisher.publish(message) {
                rclrs::log_error!(&logger, "cannot publish /joy: {error}");
            }
        }
    })?;

    rclrs::log_info!(
        &logger,
        "mav_joy_node: MAVLink UDP {} -> app {}, HEARTBEAT {}:{} at {:.1} Hz, /joy '{}' at {:.1} Hz, failsafe {}",
        local_address,
        parameters.link.app,
        parameters.link.system_id,
        parameters.link.component_id,
        1.0 / parameters.link.heartbeat_period.as_secs_f64(),
        parameters.joy_topic,
        1.0 / parameters.publish_period.as_secs_f64(),
        match parameters.timeout {
            Some(timeout) => format!("after {timeout:?}"),
            None => "disabled".to_owned(),
        },
    );

    executor.spin(SpinOptions::default()).first_error()?;
    link.stop();
    drop(timer);
    Ok(())
}

/// Builds the `/joy` message from one `MANUAL_CONTROL` reading.
fn joy_message(values: &ManualControl, frame_id: &str, now_nanoseconds: i64) -> Joy {
    Joy {
        header: std_msgs::msg::Header {
            stamp: ros_time(now_nanoseconds),
            frame_id: frame_id.to_owned(),
        },
        axes: vec![values.roll, values.pitch, values.throttle, values.yaw],
        buttons: (0..BUTTON_COUNT)
            .map(|bit| i32::from((values.buttons >> bit) & 1))
            .collect(),
    }
}

/// Converts ROS nanoseconds into the `sec`/`nanosec` fields of a message header.
fn ros_time(nanoseconds: i64) -> builtin_interfaces::msg::Time {
    builtin_interfaces::msg::Time {
        sec: nanoseconds.div_euclid(1_000_000_000) as i32,
        nanosec: nanoseconds.rem_euclid(1_000_000_000) as u32,
    }
}

/// Node parameters, declared with the defaults documented in the README.
struct Parameters {
    joy_topic: String,
    frame_id: String,
    publish_period: Duration,
    link: LinkConfig,
    /// `None` disables the failsafe, i.e. the last reading never expires.
    timeout: Option<Duration>,
}

impl Parameters {
    fn declare(node: &rclrs::Node) -> Result<Self> {
        let joy_topic = string_parameter(node, "joy_topic", "/joy")?;
        let frame_id = string_parameter(node, "frame_id", "mavlink_joystick")?;
        let timeout_sec = f64_parameter(node, "timeout_sec", 1.0)?;

        let link = LinkConfig {
            bind: SocketAddr::new(
                address_parameter(node, "bind_address", "0.0.0.0")?,
                port_parameter(node, "bind_port", 14550, true)?,
            ),
            app: SocketAddr::new(
                address_parameter(node, "app_address", "255.255.255.255")?,
                port_parameter(node, "app_port", 14550, false)?,
            ),
            system_id: id_parameter(node, "system_id", 1)?,
            component_id: id_parameter(node, "component_id", 1)?,
            heartbeat_period: period_from_rate(
                f64_parameter(node, "heartbeat_rate_hz", 1.0)?,
                "heartbeat_rate_hz",
            )?,
        };

        if timeout_sec < 0.0 || !timeout_sec.is_finite() {
            bail!("timeout_sec must be a finite number >= 0, got {timeout_sec}");
        }

        Ok(Self {
            joy_topic,
            frame_id,
            publish_period: period_from_rate(
                f64_parameter(node, "publish_rate_hz", 50.0)?,
                "publish_rate_hz",
            )?,
            link,
            timeout: (timeout_sec > 0.0).then(|| Duration::from_secs_f64(timeout_sec)),
        })
    }
}

fn period_from_rate(rate_hz: f64, name: &str) -> Result<Duration> {
    if !rate_hz.is_finite() || rate_hz <= 0.0 {
        bail!("{name} must be a finite number > 0, got {rate_hz}");
    }
    Ok(Duration::from_secs_f64(1.0 / rate_hz))
}

fn port_parameter(node: &rclrs::Node, name: &str, default: i64, allow_zero: bool) -> Result<u16> {
    let value = i64_parameter(node, name, default)?;
    let minimum = if allow_zero { 0 } else { 1 };
    if !(minimum..=i64::from(u16::MAX)).contains(&value) {
        bail!("{name} must be in {minimum}..={}, got {value}", u16::MAX);
    }
    Ok(value as u16)
}

fn id_parameter(node: &rclrs::Node, name: &str, default: i64) -> Result<u8> {
    let value = i64_parameter(node, name, default)?;
    if !(1..=255).contains(&value) {
        bail!("{name} must be in 1..=255, got {value}");
    }
    Ok(value as u8)
}

fn address_parameter(node: &rclrs::Node, name: &str, default: &str) -> Result<IpAddr> {
    let value = string_parameter(node, name, default)?;
    value
        .parse()
        .with_context(|| format!("{name} is not an IP address: {value}"))
}

fn i64_parameter(node: &rclrs::Node, name: &str, default: i64) -> Result<i64> {
    node.declare_parameter::<i64>(name)
        .default(default)
        .mandatory()
        .map_err(|error| anyhow!("cannot declare {name}: {error}"))
        .map(|parameter| parameter.get())
}

fn f64_parameter(node: &rclrs::Node, name: &str, default: f64) -> Result<f64> {
    node.declare_parameter::<f64>(name)
        .default(default)
        .mandatory()
        .map_err(|error| anyhow!("cannot declare {name}: {error}"))
        .map(|parameter| parameter.get())
}

fn string_parameter(node: &rclrs::Node, name: &str, default: &str) -> Result<String> {
    node.declare_parameter::<Arc<str>>(name)
        .default(Arc::<str>::from(default))
        .mandatory()
        .map_err(|error| anyhow!("cannot declare {name}: {error}"))
        .map(|parameter| parameter.get().to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn joy_axes_follow_the_manual_control_mapping() {
        let reading = ManualControl {
            roll: 0.5,
            pitch: -0.25,
            throttle: 1.0,
            yaw: -1.0,
            buttons: 0b11,
            system_id: 255,
            component_id: 190,
            received_at: None,
        };
        let message = joy_message(&reading, "mavlink_joystick", 1_500_000_000);

        assert_eq!(message.header.frame_id, "mavlink_joystick");
        assert_eq!(message.header.stamp.sec, 1);
        assert_eq!(message.header.stamp.nanosec, 500_000_000);
        assert_eq!(message.axes, vec![0.5, -0.25, 1.0, -1.0]);
        assert_eq!(message.buttons.len(), BUTTON_COUNT);
        assert_eq!(message.buttons[0], 1);
        assert_eq!(message.buttons[1], 1);
        assert!(message.buttons[2..].iter().all(|&button| button == 0));
    }

    #[test]
    fn ros_time_handles_fractions_and_negatives() {
        let stamp = ros_time(2_000_000_001);
        assert_eq!((stamp.sec, stamp.nanosec), (2, 1));

        let before_epoch = ros_time(-1);
        assert_eq!((before_epoch.sec, before_epoch.nanosec), (-1, 999_999_999));
    }

    #[test]
    fn periods_must_be_positive() {
        assert!(period_from_rate(50.0, "publish_rate_hz").is_ok());
        assert!(period_from_rate(0.0, "publish_rate_hz").is_err());
        assert!(period_from_rate(-1.0, "publish_rate_hz").is_err());
        assert!(period_from_rate(f64::NAN, "publish_rate_hz").is_err());
    }
}
