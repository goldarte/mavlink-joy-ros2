//! MAVLink v2 UDP link to the [mavlink-joystick] app.
//!
//! The app is a MAVLink GCS: it listens on its configured port, waits for a
//! `HEARTBEAT` from a vehicle and then starts streaming `MANUAL_CONTROL`
//! (message 69, 50 Hz) to the address that heartbeat came from. This module
//! implements the vehicle side of that handshake:
//!
//! * a TX thread answers with `HEARTBEAT` at a fixed rate, both to the learned
//!   app address and to the configured one (a broadcast address by default, so
//!   the app can discover this node without any manual setup);
//! * one RX thread parses incoming MAVLink v1/v2 frames and keeps the newest
//!   `MANUAL_CONTROL` values for the ROS node to publish on `/joy`.
//!
//! Broadcast heartbeats are delivered back to this socket by the kernel, so the
//! RX thread recognises its own heartbeat by the configured system/component id
//! and does not mistake it for the app.
//!
//! [mavlink-joystick]: https://github.com/goldarte/mavlink-joystick

use std::io::{self, Read};
use std::net::{SocketAddr, UdpSocket};
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use anyhow::{Context as _, Result};
use mavlink::dialects::common::{
    HEARTBEAT_DATA, MANUAL_CONTROL_DATA, MavAutopilot, MavMessage, MavModeFlag, MavState, MavType,
};
use mavlink::error::MessageReadError;
use mavlink::{MavHeader, MavlinkReader, write_v2_msg};

/// Largest UDP datagram accepted. A MAVLink v2 frame is at most 280 bytes, so a
/// bigger datagram cannot be a valid frame and is dropped instead of buffered.
const DATAGRAM_LIMIT: usize = 2048;

/// Blocking socket read timeout, i.e. how fast the threads notice a shutdown.
const SOCKET_POLL: Duration = Duration::from_millis(200);

/// Value of an axis that the sender marked as invalid (MAVLink uses INT16_MAX).
const AXIS_INVALID: i16 = i16::MAX;

/// Silence from the app that is reported as a warning first.
const APP_SILENCE_WARNING: Duration = Duration::from_secs(10);

/// Interval of the repeated warning while the app keeps being silent.
const APP_SILENCE_WARNING_REPEAT: Duration = Duration::from_secs(60);

/// Newest `MANUAL_CONTROL` values received from the app.
///
/// The values are the MAVLink payload normalized to `-1.0..=1.0`; no stick
/// curve, expo or deadzone is applied here, the app already did that.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ManualControl {
    /// Roll axis (`MANUAL_CONTROL.y`), positive is right.
    pub roll: f32,
    /// Pitch axis (`MANUAL_CONTROL.x`), positive is nose up.
    pub pitch: f32,
    /// Throttle axis (`MANUAL_CONTROL.z`), the app sends `0.0..=1.0`.
    pub throttle: f32,
    /// Yaw axis (`MANUAL_CONTROL.r`), positive is clockwise.
    pub yaw: f32,
    /// Button bitfield, bit 0 is button 1.
    pub buttons: u16,
    /// MAVLink system id of the sender.
    pub system_id: u8,
    /// MAVLink component id of the sender.
    pub component_id: u8,
    /// When the message arrived; `None` until the first one is received.
    pub received_at: Option<Instant>,
}

impl Default for ManualControl {
    fn default() -> Self {
        Self {
            roll: 0.0,
            pitch: 0.0,
            throttle: 0.0,
            yaw: 0.0,
            buttons: 0,
            system_id: 0,
            component_id: 0,
            received_at: None,
        }
    }
}

impl ManualControl {
    /// Values of a joystick that is centred and has no button pressed.
    pub fn neutral() -> Self {
        Self::default()
    }

    /// Age of the reading, or `None` if nothing was received yet.
    pub fn age(&self, now: Instant) -> Option<Duration> {
        self.received_at.map(|at| now.saturating_duration_since(at))
    }

    /// Whether the reading is not older than `timeout`.
    ///
    /// `None` means "never time out", i.e. the last reading stays valid.
    pub fn is_fresh(&self, now: Instant, timeout: Option<Duration>) -> bool {
        match (self.age(now), timeout) {
            (Some(age), Some(timeout)) => age <= timeout,
            (Some(_), None) => true,
            (None, _) => false,
        }
    }
}

/// Settings of the MAVLink UDP link.
#[derive(Clone, Debug)]
pub struct LinkConfig {
    /// Local address the app sends `MANUAL_CONTROL` to. Port `0` lets the OS
    /// pick a free port; the app learns it from the source port of our
    /// heartbeats, so this keeps working.
    pub bind: SocketAddr,
    /// Where heartbeats are sent before the app address is known. A broadcast
    /// address (`255.255.255.255`) discovers an app on the local network, a
    /// unicast address talks to an app with a fixed IP.
    pub app: SocketAddr,
    /// MAVLink system id of this node. The app only accepts heartbeats from its
    /// configured drone system id (1 by default).
    pub system_id: u8,
    /// MAVLink component id of this node (1 by default in the app).
    pub component_id: u8,
    /// Interval between heartbeats.
    pub heartbeat_period: Duration,
}

/// State shared between the ROS node and the two transport threads.
struct Shared {
    running: AtomicBool,
    sequence: AtomicU8,
    manual_control: Mutex<ManualControl>,
    peer: Mutex<Option<SocketAddr>>,
    /// Source of the datagram that completed the message that was parsed last.
    last_source: Mutex<Option<SocketAddr>>,
    /// When a message from another MAVLink system was received last.
    last_peer_seen: Mutex<Option<Instant>>,
    /// When the link was opened; the reference point of the first silence.
    started_at: Instant,
}

/// UDP link that answers the app's discovery and receives `MANUAL_CONTROL`.
pub struct MavlinkLink {
    socket: Arc<UdpSocket>,
    config: LinkConfig,
    shared: Arc<Shared>,
    threads: Mutex<Vec<JoinHandle<()>>>,
}

impl MavlinkLink {
    /// Binds the UDP socket; the link starts sending and receiving only after
    /// [`MavlinkLink::start`].
    pub fn open(config: LinkConfig) -> Result<Self> {
        let socket = UdpSocket::bind(config.bind)
            .with_context(|| format!("cannot bind MAVLink UDP socket to {}", config.bind))?;
        // Needed to send heartbeats to a broadcast address.
        socket
            .set_broadcast(true)
            .context("cannot enable UDP broadcast on the MAVLink socket")?;
        socket
            .set_read_timeout(Some(SOCKET_POLL))
            .context("cannot set the MAVLink socket read timeout")?;

        Ok(Self {
            socket: Arc::new(socket),
            config,
            shared: Arc::new(Shared {
                running: AtomicBool::new(false),
                sequence: AtomicU8::new(0),
                manual_control: Mutex::new(ManualControl::neutral()),
                peer: Mutex::new(None),
                last_source: Mutex::new(None),
                last_peer_seen: Mutex::new(None),
                started_at: Instant::now(),
            }),
            threads: Mutex::new(Vec::new()),
        })
    }

    /// Local address the socket is bound to.
    pub fn local_addr(&self) -> io::Result<SocketAddr> {
        self.socket.local_addr()
    }

    /// Spawns the receive and heartbeat threads.
    pub fn start(&self) -> Result<()> {
        self.shared.running.store(true, Ordering::SeqCst);

        let receiver = self.spawn("mav-joy-rx", {
            let socket = Arc::clone(&self.socket);
            let shared = Arc::clone(&self.shared);
            let config = self.config.clone();
            move || receive_loop(socket, shared, config)
        })?;
        let heartbeat = self.spawn("mav-joy-tx", {
            let socket = Arc::clone(&self.socket);
            let shared = Arc::clone(&self.shared);
            let config = self.config.clone();
            move || heartbeat_loop(socket, shared, config)
        })?;

        lock(&self.threads).extend([receiver, heartbeat]);
        Ok(())
    }

    /// Stops both threads and waits for them to finish.
    pub fn stop(&self) {
        self.shared.running.store(false, Ordering::SeqCst);
        for thread in lock(&self.threads).drain(..) {
            let _ = thread.join();
        }
    }

    /// Newest `MANUAL_CONTROL` values, or neutral values with `received_at`
    /// unset if nothing arrived yet.
    pub fn manual_control(&self) -> ManualControl {
        *lock(&self.shared.manual_control)
    }

    /// Address the app is sending from, once it has been heard.
    pub fn peer(&self) -> Option<SocketAddr> {
        *lock(&self.shared.peer)
    }

    fn spawn(&self, name: &str, body: impl FnOnce() + Send + 'static) -> Result<JoinHandle<()>> {
        thread::Builder::new()
            .name(name.into())
            .spawn(body)
            .with_context(|| format!("cannot start the {name} thread"))
    }
}

impl Drop for MavlinkLink {
    fn drop(&mut self) {
        self.stop();
    }
}

/// Parses MAVLink frames until the link is stopped.
fn receive_loop(socket: Arc<UdpSocket>, shared: Arc<Shared>, config: LinkConfig) {
    let mut reader = MavlinkReader::new(UdpReader {
        socket,
        shared: Arc::clone(&shared),
        buffer: Vec::new(),
        position: 0,
    });

    while shared.is_running() {
        let (header, message) = match reader.read_any_message::<MavMessage>() {
            Ok(message) => message,
            Err(error) => {
                if !shared.is_running() {
                    break;
                }
                eprintln!("mav_joy: discarding MAVLink data: {error}");
                // A socket that keeps failing (interface gone, for example)
                // must not turn this thread into a busy loop.
                if matches!(error, MessageReadError::Io(_)) {
                    thread::sleep(SOCKET_POLL);
                }
                continue;
            }
        };

        // The kernel echoes our own broadcast heartbeats back to this socket;
        // that datagram is us, not the app.
        let own_heartbeat = matches!(message, MavMessage::HEARTBEAT(_))
            && (header.system_id, header.component_id) == (config.system_id, config.component_id);
        if !own_heartbeat && let Some(source) = shared.last_source() {
            shared.observe_peer(source, Instant::now());
        }

        if let MavMessage::MANUAL_CONTROL(payload) = message {
            let reading = manual_control_from(&payload, &header, Instant::now());
            let first = {
                let mut current = lock(&shared.manual_control);
                let first = current.received_at.is_none();
                *current = reading;
                first
            };
            if first {
                eprintln!(
                    "mav_joy: MANUAL_CONTROL from system {} component {}",
                    header.system_id, header.component_id
                );
            }
        }
    }
}

/// Sends heartbeats so the app discovers this node and starts streaming.
fn heartbeat_loop(socket: Arc<UdpSocket>, shared: Arc<Shared>, config: LinkConfig) {
    let mut frame = Vec::with_capacity(64);
    let mut send_failed = false;
    let mut next_silence_warning = Instant::now() + APP_SILENCE_WARNING;
    while shared.is_running() {
        let heartbeat = HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: MavType::MAV_TYPE_GENERIC,
            // The app shows "Flix" for MAV_AUTOPILOT_GENERIC, which is the
            // closest match for a joystick bridge that is not an autopilot.
            autopilot: MavAutopilot::MAV_AUTOPILOT_GENERIC,
            base_mode: MavModeFlag::MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
                | MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            system_status: MavState::MAV_STATE_ACTIVE,
            mavlink_version: 3,
        };
        let header = MavHeader {
            system_id: config.system_id,
            component_id: config.component_id,
            sequence: shared.next_sequence(),
        };

        frame.clear();
        if let Err(error) = write_v2_msg(&mut frame, header, &MavMessage::HEARTBEAT(heartbeat)) {
            eprintln!("mav_joy: cannot encode HEARTBEAT: {error}");
        } else {
            // Report the first failure after a working period: a broadcast that
            // the network rejects would otherwise leave the app undiscovered
            // without any hint.
            match socket.send_to(&frame, config.app) {
                Ok(_) => send_failed = false,
                Err(error) => {
                    if !send_failed {
                        send_failed = true;
                        eprintln!("mav_joy: cannot send HEARTBEAT to {}: {error}", config.app);
                    }
                }
            }
            // A phone that moved to another Wi-Fi network keeps talking to us
            // from a new address, so answer there as well.
            if let Some(peer) = shared.peer()
                && peer != config.app
            {
                let _ = socket.send_to(&frame, peer);
            }
        }

        warn_if_app_is_silent(&socket, &shared, &config, &mut next_silence_warning);
        shared.sleep_while_running(config.heartbeat_period);
    }
}

/// Warns when heartbeats go out but nothing comes back from the app.
///
/// The app can be silent for reasons this side cannot see: it may not be
/// running, it may still send to an address it detected earlier, or the network
/// may drop unicast between clients (a guest Wi-Fi often floods broadcast – which
/// is why the app still shows a link – but not client-to-client unicast). The
/// warning names the address the app has to target, so the two sides can be
/// compared.
fn warn_if_app_is_silent(
    socket: &UdpSocket,
    shared: &Shared,
    config: &LinkConfig,
    next_warning: &mut Instant,
) {
    let now = Instant::now();
    let silence = shared.silence(now);
    if !silence_warning_due(silence, now, next_warning) {
        return;
    }

    // The app sends to the address the heartbeat came from, i.e. the source
    // address of our own heartbeats.
    let local_port = socket
        .local_addr()
        .map_or(config.bind.port(), |address| address.port());
    let target = match heartbeat_source(local_port, config.app) {
        Some(local) => format!("point the app at {local}, "),
        None => String::new(),
    };
    eprintln!(
        "mav_joy: heartbeats are being sent to {}, but nothing has been received from the app for {}s: \
         {target}the app keeps the address it detected last and the Wi-Fi must not isolate clients \
         (see README, Troubleshooting)",
        config.app,
        silence.as_secs(),
    );
}

/// Whether the "app is silent" warning is due; advances [`APP_SILENCE_WARNING_REPEAT`].
fn silence_warning_due(silence: Duration, now: Instant, next_warning: &mut Instant) -> bool {
    if silence < APP_SILENCE_WARNING || now < *next_warning {
        return false;
    }
    *next_warning = now + APP_SILENCE_WARNING_REPEAT;
    true
}

/// Address the app sees as the source of our heartbeats, i.e. the address it
/// stores as the drone address. `None` when the routing table cannot tell.
///
/// The source address is only picked by the route lookup that `connect`
/// performs, so a throwaway socket bound to an ephemeral port is used and its
/// source address is combined with the port the heartbeats really leave from.
fn heartbeat_source(local_port: u16, app: SocketAddr) -> Option<SocketAddr> {
    let probe = UdpSocket::bind(("0.0.0.0", 0)).ok()?;
    // Connecting to a broadcast address needs SO_BROADCAST.
    probe.set_broadcast(true).ok()?;
    probe.connect(app).ok()?;
    let source = probe.local_addr().ok()?.ip();
    Some(SocketAddr::new(source, local_port))
}

impl Shared {
    fn is_running(&self) -> bool {
        self.running.load(Ordering::Relaxed)
    }

    fn next_sequence(&self) -> u8 {
        self.sequence.fetch_add(1, Ordering::Relaxed)
    }

    /// Address the app was last heard from.
    fn peer(&self) -> Option<SocketAddr> {
        *lock(&self.peer)
    }

    /// Source of the datagram that completed the last parsed message.
    fn last_source(&self) -> Option<SocketAddr> {
        *lock(&self.last_source)
    }

    /// Remembers the address datagrams are coming from.
    fn observe_peer(&self, peer: SocketAddr, now: Instant) {
        *lock(&self.last_peer_seen) = Some(now);

        let mut current = lock(&self.peer);
        if *current != Some(peer) {
            eprintln!("mav_joy: MAVLink peer is {peer}");
            *current = Some(peer);
        }
    }

    /// Time since a message from another MAVLink system was received, or since
    /// the link was opened if none was received yet.
    fn silence(&self, now: Instant) -> Duration {
        let since = lock(&self.last_peer_seen).unwrap_or(self.started_at);
        now.saturating_duration_since(since)
    }

    /// Sleeps `period` in short steps so `stop()` is not delayed.
    fn sleep_while_running(&self, period: Duration) {
        let step = Duration::from_millis(20);
        let deadline = Instant::now() + period;
        while self.is_running() {
            let now = Instant::now();
            if now >= deadline {
                break;
            }
            thread::sleep(step.min(deadline - now));
        }
    }
}

/// Adapts the UDP socket to the blocking `Read` interface of [`MavlinkReader`].
///
/// Each datagram is served byte by byte, so a frame split over several
/// datagrams is reassembled as well. The socket read timeout is used to notice
/// a shutdown request; `Ok(0)` is reported only to stop the reader.
struct UdpReader {
    socket: Arc<UdpSocket>,
    shared: Arc<Shared>,
    buffer: Vec<u8>,
    position: usize,
}

impl Read for UdpReader {
    fn read(&mut self, out: &mut [u8]) -> io::Result<usize> {
        loop {
            if self.position < self.buffer.len() {
                let count = out.len().min(self.buffer.len() - self.position);
                out[..count].copy_from_slice(&self.buffer[self.position..self.position + count]);
                self.position += count;
                return Ok(count);
            }
            if !self.shared.is_running() {
                // EOF: the receive loop sees the read error and exits.
                return Ok(0);
            }

            let mut datagram = [0u8; DATAGRAM_LIMIT];
            match self.socket.recv_from(&mut datagram) {
                Ok((count, peer)) => {
                    self.buffer.clear();
                    self.buffer.extend_from_slice(&datagram[..count]);
                    self.position = 0;
                    *lock(&self.shared.last_source) = Some(peer);
                }
                Err(error)
                    if matches!(
                        error.kind(),
                        io::ErrorKind::WouldBlock
                            | io::ErrorKind::TimedOut
                            | io::ErrorKind::Interrupted
                    ) => {}
                Err(error) => return Err(error),
            }
        }
    }
}

/// Converts a raw `MANUAL_CONTROL` payload into joystick values.
///
/// `MANUAL_CONTROL.x/y/z/r` are `-1000..=1000`; the app builds them from its
/// sticks as `x = -pitch`, `y = roll`, `z = throttle (0..=1000)`, `r = yaw`, so
/// the mapping is inverted here to get the app's stick positions back.
fn manual_control_from(
    payload: &MANUAL_CONTROL_DATA,
    header: &MavHeader,
    received_at: Instant,
) -> ManualControl {
    ManualControl {
        roll: axis(payload.y),
        pitch: -axis(payload.x),
        throttle: axis(payload.z),
        yaw: axis(payload.r),
        buttons: payload.buttons,
        system_id: header.system_id,
        component_id: header.component_id,
        received_at: Some(received_at),
    }
}

/// Normalizes a MAVLink axis to `-1.0..=1.0`; the "invalid" marker becomes 0.
fn axis(value: i16) -> f32 {
    if value == AXIS_INVALID {
        return 0.0;
    }
    (f32::from(value) / 1000.0).clamp(-1.0, 1.0)
}

/// Locks a mutex, ignoring poisoning: the protected values stay usable.
fn lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    fn header() -> MavHeader {
        MavHeader {
            system_id: 255,
            component_id: 190,
            sequence: 7,
        }
    }

    /// Encodes a message and reads it back the way the receive loop does.
    fn round_trip(message: &MavMessage) -> (MavHeader, MavMessage) {
        let mut frame = Vec::new();
        write_v2_msg(&mut frame, header(), message).unwrap();
        let mut reader = MavlinkReader::new(Cursor::new(frame));
        reader.read_any_message::<MavMessage>().unwrap()
    }

    #[test]
    fn heartbeat_round_trip_carries_our_ids() {
        let message = MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: MavType::MAV_TYPE_GENERIC,
            autopilot: MavAutopilot::MAV_AUTOPILOT_GENERIC,
            base_mode: MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            system_status: MavState::MAV_STATE_ACTIVE,
            mavlink_version: 3,
        });
        let (header, parsed) = round_trip(&message);
        assert_eq!(header.system_id, 255);
        assert!(matches!(parsed, MavMessage::HEARTBEAT(_)));
    }

    #[test]
    fn manual_control_maps_to_joystick_axes() {
        let payload = MANUAL_CONTROL_DATA {
            x: -750, // pitch up
            y: 500,  // roll right
            z: 250,  // quarter throttle
            r: -100, // yaw left
            buttons: 0b101,
            target: 1,
        };
        let (header, parsed) = round_trip(&MavMessage::MANUAL_CONTROL(payload));
        let MavMessage::MANUAL_CONTROL(payload) = parsed else {
            panic!("expected MANUAL_CONTROL");
        };
        let reading = manual_control_from(&payload, &header, Instant::now());
        assert_eq!(reading.roll, 0.5);
        assert_eq!(reading.pitch, 0.75);
        assert_eq!(reading.throttle, 0.25);
        assert_eq!(reading.yaw, -0.1);
        assert_eq!(reading.buttons, 0b101);
        assert_eq!(reading.system_id, 255);
        assert_eq!(reading.component_id, 190);
        assert!(reading.received_at.is_some());
    }

    #[test]
    fn out_of_range_and_invalid_axes_are_clamped() {
        assert_eq!(axis(AXIS_INVALID), 0.0);
        assert_eq!(axis(30_000), 1.0);
        assert_eq!(axis(-30_000), -1.0);
        assert_eq!(axis(0), 0.0);
    }

    #[test]
    fn freshness_follows_the_timeout() {
        let now = Instant::now();
        let mut reading = ManualControl::neutral();
        assert!(!reading.is_fresh(now, Some(Duration::from_secs(1))));

        reading.received_at = Some(now - Duration::from_millis(500));
        assert!(reading.is_fresh(now, Some(Duration::from_secs(1))));
        assert!(!reading.is_fresh(now, Some(Duration::from_millis(100))));
        // No timeout: the last reading never expires.
        assert!(reading.is_fresh(now, None));
    }

    /// Plays the app's side: it must see our heartbeats and its
    /// `MANUAL_CONTROL` must arrive as a joystick reading.
    #[test]
    fn link_answers_the_app_and_receives_manual_control() {
        let app = UdpSocket::bind("127.0.0.1:0").unwrap();
        let app_address = app.local_addr().unwrap();
        app.set_read_timeout(Some(Duration::from_secs(5))).unwrap();

        let link = MavlinkLink::open(LinkConfig {
            bind: "127.0.0.1:0".parse().unwrap(),
            app: app_address,
            system_id: 1,
            component_id: 1,
            heartbeat_period: Duration::from_millis(50),
        })
        .unwrap();
        let node_address = link.local_addr().unwrap();
        link.start().unwrap();

        // The app discovers the node from the source of the first heartbeat.
        let mut datagram = [0u8; DATAGRAM_LIMIT];
        let (count, from) = app.recv_from(&mut datagram).expect("no heartbeat");
        assert_eq!(from, node_address);
        let mut reader = MavlinkReader::new(Cursor::new(datagram[..count].to_vec()));
        let (from_header, message) = reader.read_any_message::<MavMessage>().unwrap();
        assert!(matches!(message, MavMessage::HEARTBEAT(_)));
        assert_eq!((from_header.system_id, from_header.component_id), (1, 1));

        // The app then streams MANUAL_CONTROL back to that address.
        let mut frame = Vec::new();
        write_v2_msg(
            &mut frame,
            header(),
            &MavMessage::MANUAL_CONTROL(MANUAL_CONTROL_DATA {
                x: -1000,
                y: 1000,
                z: 0,
                r: 500,
                buttons: 0b10,
                target: 1,
            }),
        )
        .unwrap();
        app.send_to(&frame, node_address).unwrap();

        let deadline = Instant::now() + Duration::from_secs(5);
        while link.manual_control().received_at.is_none() && Instant::now() < deadline {
            thread::sleep(Duration::from_millis(10));
        }

        let reading = link.manual_control();
        assert_eq!(reading.roll, 1.0);
        assert_eq!(reading.pitch, 1.0);
        assert_eq!(reading.throttle, 0.0);
        assert_eq!(reading.yaw, 0.5);
        assert_eq!(reading.buttons, 0b10);
        assert_eq!(reading.system_id, 255);
        assert_eq!(reading.component_id, 190);
        assert_eq!(link.peer(), Some(app_address));
    }

    /// A [`Shared`] whose silence window started at `started_at`.
    fn shared_state(started_at: Instant) -> Shared {
        Shared {
            running: AtomicBool::new(false),
            sequence: AtomicU8::new(0),
            manual_control: Mutex::new(ManualControl::neutral()),
            peer: Mutex::new(None),
            last_source: Mutex::new(None),
            last_peer_seen: Mutex::new(None),
            started_at,
        }
    }

    #[test]
    fn silence_runs_from_the_last_peer_or_from_the_start() {
        let now = Instant::now();
        let shared = shared_state(now - Duration::from_secs(30));
        assert_eq!(shared.silence(now), Duration::from_secs(30));

        let peer = "127.0.0.1:14550".parse().unwrap();
        shared.observe_peer(peer, now - Duration::from_secs(2));
        assert_eq!(shared.silence(now), Duration::from_secs(2));
        assert_eq!(shared.peer(), Some(peer));
    }

    #[test]
    fn the_silence_warning_is_rate_limited() {
        let now = Instant::now();
        let mut next = now + APP_SILENCE_WARNING;

        // Too early in every respect.
        assert!(!silence_warning_due(APP_SILENCE_WARNING, now, &mut next));
        // Silence long enough and the first deadline reached.
        assert!(silence_warning_due(
            APP_SILENCE_WARNING,
            now + APP_SILENCE_WARNING,
            &mut next
        ));
        // The repeat interval has not passed yet.
        assert!(!silence_warning_due(
            APP_SILENCE_WARNING,
            now + APP_SILENCE_WARNING + Duration::from_secs(1),
            &mut next
        ));
        // ... and it is reported again afterwards.
        assert!(silence_warning_due(
            APP_SILENCE_WARNING,
            now + APP_SILENCE_WARNING + APP_SILENCE_WARNING_REPEAT,
            &mut next
        ));
    }
}
