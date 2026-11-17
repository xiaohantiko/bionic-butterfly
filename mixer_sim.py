"""Host-only angle simulation. No transport, pin access, pulse generation or flashing."""

from dataclasses import asdict, dataclass, field
import math
from typing import Optional


def _number(value):
    if type(value) not in (int, float):
        return False
    try:
        return math.isfinite(value)
    except (OverflowError, ValueError):
        return False


@dataclass(frozen=True)
class SideBounds:
    min_angle_deg: float = -10.0
    max_angle_deg: float = 10.0
    max_amplitude_deg: float = 6.0
    max_mean_offset_deg: float = 2.0
    max_rate_deg_s: float = 40.0

    def validate(self):
        values = asdict(self)
        if not all(_number(v) for v in values.values()):
            raise ValueError("Side bounds must be finite numbers, not booleans")
        if not self.min_angle_deg < 0 < self.max_angle_deg:
            raise ValueError("Each simulation envelope must contain the assumed zero angle")
        if self.max_amplitude_deg <= 0 or self.max_mean_offset_deg < 0:
            raise ValueError("Amplitude must be positive and mean-offset allowance nonnegative")
        if self.max_rate_deg_s <= 0:
            raise ValueError("Rate allowance must be positive")
        excursion = self.max_amplitude_deg + self.max_mean_offset_deg
        if excursion > min(-self.min_angle_deg, self.max_angle_deg):
            raise ValueError("Amplitude plus offset exceeds a side's angle envelope")


@dataclass(frozen=True)
class SimConfig:
    frequency_hz: float = 1.0
    turn_fraction: float = 0.25
    arm_throttle_max: float = 0.05
    command_timeout_s: float = 0.25
    left: SideBounds = field(default_factory=SideBounds)
    right: SideBounds = field(default_factory=SideBounds)

    def __post_init__(self):
        scalars = (self.frequency_hz, self.turn_fraction, self.arm_throttle_max,
                   self.command_timeout_s)
        if not all(_number(v) for v in scalars):
            raise ValueError("Config values must be finite numbers, not booleans")
        if self.frequency_hz <= 0 or self.command_timeout_s <= 0:
            raise ValueError("Frequency and timeout must be positive")
        if not 0 <= self.turn_fraction <= 1 or not 0 <= self.arm_throttle_max <= 1:
            raise ValueError("Turn fraction and low-throttle threshold must be in [0, 1]")
        for bounds in (self.left, self.right):
            bounds.validate()
            if 2 * math.pi * self.frequency_hz * bounds.max_amplitude_deg > bounds.max_rate_deg_s:
                raise ValueError("Steady sinusoid exceeds a side's rate allowance")


@dataclass(frozen=True)
class CommandFrame:
    time_s: float
    throttle: float
    turn: float
    pitch: float
    arm: bool
    link_ok: bool
    usb_vbus_present: bool

    def valid_controls(self):
        if not all(type(v) is bool for v in (self.arm, self.link_ok, self.usb_vbus_present)):
            return False
        if not all(_number(v) for v in (self.throttle, self.turn, self.pitch)):
            return False
        return 0 <= self.throttle <= 1 and -1 <= self.turn <= 1 and -1 <= self.pitch <= 1


class MixerSimulation:
    """Synchronous state machine; callers must supply monotonically increasing time.

    A target is a simulated relative angle, not a calibrated actuator instruction.
    None means no target. It does not mean that real hardware is powered off.
    """

    def __init__(self, config: Optional[SimConfig] = None):
        self.config = config or SimConfig()
        self.reset()

    def reset(self):
        self._last_event_time = None
        self._last_valid_frame_time = None
        self._command = None
        self._trip("POWER_ON_NEEDS_ARM_LOW")

    def stop(self, reason="INPUT_STREAM_STOPPED"):
        """Clear targets and all arming qualification when an input stream fails."""
        self._trip(reason)
        return self._snapshot(self._last_event_time)

    def _trip(self, reason):
        self._armed = False
        self._seen_arm_low = False
        self._previous_arm = None
        self._arm_time = None
        self._render_time = None
        self._angles = {"left": 0.0, "right": 0.0}
        self._targets = None
        self._reason = reason

    def _accept_time(self, now):
        if not _number(now) or now < 0:
            self._trip("INVALID_TIME")
            return False
        if self._last_event_time is not None and now <= self._last_event_time:
            self._trip("NON_MONOTONIC_TIME")
            return False
        self._last_event_time = now
        return True

    def _expired(self, now):
        return ((self._armed or self._seen_arm_low)
                and self._last_valid_frame_time is not None
                and now > self._last_valid_frame_time + self.config.command_timeout_s)

    def step(self, frame: CommandFrame):
        """Accept one explicit simulated command; reject invalid values without clamping."""
        if not isinstance(frame, CommandFrame):
            self._trip("INVALID_FRAME_TYPE")
            return self._snapshot(None)
        now = frame.time_s
        if not self._accept_time(now):
            return self._snapshot(now)
        if not frame.valid_controls():
            self._trip("INVALID_CONTROL_INPUT")
            return self._snapshot(now)
        if frame.usb_vbus_present:
            self._trip("USB_VBUS_PRESENT_SIMULATED_LOCK")
            return self._snapshot(now)
        if not frame.link_ok:
            self._trip("LINK_LOST")
            return self._snapshot(now)
        expired = self._expired(now)
        self._last_valid_frame_time = now
        self._command = frame
        if expired:
            # This late frame is discarded for arming, even if arm is low.
            self._trip("COMMAND_TIMEOUT_FRAME_DISCARDED")
            return self._snapshot(now)
        if not frame.arm:
            self._trip("ARM_LOW_SEEN_WAIT_NEW_EDGE")
            self._seen_arm_low = True
            self._previous_arm = False
            return self._snapshot(now)
        rising = self._previous_arm is False
        self._previous_arm = True
        if not self._armed:
            if not self._seen_arm_low:
                self._reason = "NEEDS_VALID_ARM_LOW"
                return self._snapshot(now)
            if not rising:
                self._reason = "WAIT_NEW_ARM_EDGE"
                return self._snapshot(now)
            if frame.throttle > self.config.arm_throttle_max:
                self._trip("ARM_REJECTED_HIGH_THROTTLE")
                return self._snapshot(now)
            self._armed = True
            self._seen_arm_low = False
            self._arm_time = now
            self._render_time = now
        self._render(now)
        return self._snapshot(now)

    def tick(self, now):
        """Advance simulation without a fresh command; expire the command lease."""
        if not self._accept_time(now):
            return self._snapshot(now)
        if self._expired(now):
            self._trip("COMMAND_TIMEOUT")
        elif self._armed:
            self._render(now)
        return self._snapshot(now)

    def _render(self, now):
        command = self._command
        cfg = self.config
        elapsed = now - self._arm_time
        dt = now - self._render_time
        wave = math.sin(2 * math.pi * cfg.frequency_hz * elapsed)
        targets = {}
        for name, sign, bounds in (("left", -1, cfg.left), ("right", 1, cfg.right)):
            amplitude = (command.throttle * bounds.max_amplitude_deg
                         * (1 + sign * cfg.turn_fraction * command.turn)
                         / (1 + cfg.turn_fraction))
            mean = command.throttle * command.pitch * bounds.max_mean_offset_deg
            desired = mean + amplitude * wave
            previous = self._angles[name]
            allowed_delta = bounds.max_rate_deg_s * dt
            delta = max(-allowed_delta, min(allowed_delta, desired - previous))
            angle = previous + delta
            within = (bounds.min_angle_deg - 1e-9 <= angle <= bounds.max_angle_deg + 1e-9
                      and amplitude <= bounds.max_amplitude_deg + 1e-9
                      and abs(mean) <= bounds.max_mean_offset_deg + 1e-9
                      and abs(delta) <= allowed_delta + 1e-9)
            if not within:
                self._trip("SOFTWARE_ENVELOPE_VIOLATION")
                return
            targets[name] = {
                "angle_deg": angle,
                "unlimited_angle_deg": desired,
                "amplitude_deg": amplitude,
                "mean_offset_deg": mean,
                "rate_limited": abs(angle - desired) > 1e-9,
                "software_within_bounds": within,
                "bounds": asdict(bounds),
            }
        self._angles = {name: target["angle_deg"] for name, target in targets.items()}
        self._render_time = now
        self._targets = targets
        self._reason = "SIMULATED_TARGETS_ACTIVE"

    def _snapshot(self, now):
        state = "ARMED" if self._armed else (
            "READY_FOR_NEW_ARM_EDGE" if self._seen_arm_low else "NEEDS_ARM_LOW")
        # Build a fresh structure so a consumer cannot mutate internal state.
        targets = None if self._targets is None else {
            name: {**target, "bounds": dict(target["bounds"])}
            for name, target in self._targets.items()
        }
        return {
            "mode": "HOST_ONLY_GROUND_SIMULATION",
            "time_s": now if _number(now) and now >= 0 else None,
            "state": state,
            "reason": self._reason,
            "targets": targets,
            "valid_until_s": (self._last_valid_frame_time + self.config.command_timeout_s
                              if self._armed else None),
            "hardware_interlock_implemented": False,
            "hardware_angle_calibration_validated": False,
            "radio_channel_mapping_validated": False,
            "flight_gains_validated": False,
        }
