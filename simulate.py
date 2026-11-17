"""Offline CRSF replay, virtual PWM, and flapping feasibility screening."""

import argparse
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path

from mixer_sim import CommandFrame, MixerSimulation, SideBounds, SimConfig


PWM_HZ = 333
RC_HZ = 100
CENTER_US = 1520
US_PER_DEG = 8.0  # Virtual calibration. Must be replaced with measured slope.
CHANNEL_MIN, CHANNEL_CENTER, CHANNEL_MAX = 172, 992, 1811
SIDES = ("left", "right")


def crc8(body):
    """CRC-8/DVB-S2, 0xd5, initial zero; CRSF type and payload."""
    value = 0
    for byte in body:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ (0xD5 if value & 0x80 else 0)) & 0xFF
    return value


def rc_bytes(throttle=0., turn=0., pitch=0., arm=False):
    # CH1 roll unused, CH2 pitch, CH3 throttle, CH4 turn, CH5 arm.
    channels = [CHANNEL_CENTER] * 16
    for i, value in ((1, pitch), (3, turn)):
        end = CHANNEL_MAX if value >= 0 else CHANNEL_MIN
        channels[i] = CHANNEL_CENTER + round(abs(value) * (end - CHANNEL_CENTER))
    channels[2] = CHANNEL_MIN + round(throttle * (CHANNEL_MAX - CHANNEL_MIN))
    channels[4] = CHANNEL_MAX if arm else CHANNEL_MIN
    packed = sum(value << (11 * i) for i, value in enumerate(channels))
    body = b"\x16" + packed.to_bytes(22, "little")
    return b"\xc8\x18" + body + bytes((crc8(body),))


class RcInput:
    """Reassemble FC-destined CRSF; only valid 0x16 renews the command lease."""

    def __init__(self):
        self.buffer = bytearray()
        self.rejected = 0
        self.link_zero = False

    def feed(self, chunk, now):
        self.buffer.extend(chunk)
        commands = []
        while self.buffer:
            start = self.buffer.find(b"\xc8")
            if start < 0:
                self.buffer.clear()
                break
            del self.buffer[:start]
            if len(self.buffer) < 2:
                break
            length = self.buffer[1]
            if not 2 <= length <= 62:
                self.rejected += 1
                del self.buffer[0]
                continue
            if len(self.buffer) < length + 2:
                break
            body = self.buffer[2 : length + 1]
            if crc8(body) != self.buffer[length + 1]:
                self.rejected += 1
                del self.buffer[0]
                continue
            del self.buffer[: length + 2]
            if body[0] == 0x14 and length == 12:
                # Standard link statistics: third payload byte is uplink LQ.
                self.link_zero = body[3] == 0
            if body[0] != 0x16 or length != 24:
                continue
            packed = int.from_bytes(body[1:], "little")
            _, pitch, throttle, turn, arm = [(packed >> (11 * i)) & 2047 for i in range(5)]
            if not all(CHANNEL_MIN <= v <= CHANNEL_MAX for v in (pitch, throttle, turn, arm)):
                self.rejected += 1
                continue

            def centered(raw):
                span = CHANNEL_MAX - CHANNEL_CENTER if raw >= CHANNEL_CENTER else CHANNEL_CENTER - CHANNEL_MIN
                return (raw - CHANNEL_CENTER) / span
            commands.append(CommandFrame(now, (throttle - CHANNEL_MIN) / (CHANNEL_MAX - CHANNEL_MIN),
                                         centered(turn), centered(pitch), arm > 1400, True, False))
        return commands


def control_sample(decoder, mixer, chunk, now, usb=False):
    if usb:
        decoder.buffer.clear()
        return mixer.step(CommandFrame(now, 0., 0., 0., False, False, True))
    frames = decoder.feed(chunk, now)
    if decoder.link_zero:
        return mixer.step(CommandFrame(now, 0., 0., 0., False, False, False))
    if not frames:
        return mixer.tick(now)
    # When UART delivery batches edges at one timestamp, honor a disarm in the
    # batch and wait for a later frame before accepting a fresh arm edge.
    chosen = next((frame for frame in reversed(frames) if not frame.arm), frames[-1])
    return mixer.step(chosen)


def virtual_pwm(snapshot, usb=False):
    if usb or snapshot["targets"] is None:
        return None
    # Mirrored servos and integer microsecond timer, within virtual ±60° calibration.
    return {side: round(CENTER_US + sign * snapshot["targets"][side]["angle_deg"] * US_PER_DEG)
            for side, sign in (("left", 1), ("right", -1))}


@dataclass(frozen=True)
class Wing:
    area_m2: float
    area_r2_m: float
    area_moment3_m5: float
    weight_n: float
    rho: float

    @classmethod
    def synthetic_example(cls):
        """Two ideal triangular wings and an illustrative mass."""
        half_span_m = 0.22
        root_chord_m = 0.18
        mass_kg = 0.030
        return cls(
            area_m2=root_chord_m * half_span_m,
            area_r2_m=half_span_m / math.sqrt(6),
            area_moment3_m5=root_chord_m * half_span_m**4 / 20,
            weight_n=mass_kg * 9.80665,
            rho=1.17,
        )


@dataclass(frozen=True)
class Drive:
    voltage: float
    wing_mass_radius_m: float
    drag_coefficient: float
    servo_bandwidth_hz: float
    factor: float

    @property
    def rated_limits(self):
        # Datasheet endpoints only: no-load transit speed and listed torque.
        sec60, kgf_cm = {6.0: (0.08, 1.8), 7.4: (0.07, 2.1), 8.4: (0.06, 2.4)}[self.voltage]
        return math.pi / 3 / sec60, kgf_cm * 0.0980665


class Servo:
    def __init__(self, drive, wing):
        self.inertia = .0032 * drive.wing_mass_radius_m**2
        wn = math.tau * drive.servo_bandwidth_hz
        self.kp = self.inertia * wn**2
        self.kd = 1.6 * self.inertia * wn
        speed, torque = drive.rated_limits
        self.speed_limit = speed * drive.factor
        self.torque_limit = torque * drive.factor
        # Stroke-direction profile drag at zero body-relative airspeed.
        self.drag = .5 * wing.rho * drive.drag_coefficient * wing.area_moment3_m5
        self.angle = self.speed = 0.

    def step(self, target_rad, dt):
        drag = self.drag * self.speed * abs(self.speed)
        if target_rad is None:
            torque, limited = 0., False
        else:
            needed = self.kp * (target_rad - self.angle) - self.kd * self.speed
            # Assumed linear motor envelope; allow rated braking torque when reversing.
            aiding_motion = needed * self.speed > 0
            available = self.torque_limit * (max(0., 1 - abs(self.speed) / self.speed_limit)
                                            if aiding_motion else 1.)
            torque = max(-available, min(needed, available))
            limited = abs(torque - needed) > 1e-9
        last_speed = self.speed
        self.speed += dt * (torque - drag) / self.inertia
        mean_speed = (last_speed + self.speed) / 2
        self.angle += dt * mean_speed
        return limited, max(0., torque * mean_speed)


def new_controller(cap_deg, frequency):
    bounds = SideBounds(-60., 60., cap_deg, 3., 1300.)
    return MixerSimulation(SimConfig(frequency_hz=frequency, left=bounds, right=bounds))


def require(passed, description):
    if not passed:
        raise RuntimeError(description)


def run_faults():
    """One chronological replay through the same parser, mixer and PWM functions."""
    mixer, parser = new_controller(50., 2.5), RcInput()
    good_low, good_high = rc_bytes(), rc_bytes(arm=True)
    results = []

    def check(t, packet, expected, usb=False, has_pwm=False):
        snapshot = control_sample(parser, mixer, packet, t, usb)
        pulse = virtual_pwm(snapshot, usb)
        require(snapshot["reason"] == expected and (pulse is not None) == has_pwm, expected)
        results.append({"time_s": t, "reason": snapshot["reason"], "virtual_pwm_us": pulse})

    require(crc8(b"123456789") == 0xBC, "DVB-S2 standard check vector")
    check(0., good_high, "NEEDS_VALID_ARM_LOW")
    check(.01, good_low[:9], "NEEDS_VALID_ARM_LOW")
    check(.02, good_low[9:], "ARM_LOW_SEEN_WAIT_NEW_EDGE")
    check(.03, rc_bytes(.3, arm=True), "ARM_REJECTED_HIGH_THROTTLE")
    check(.04, good_high, "NEEDS_VALID_ARM_LOW")
    check(.05, good_low, "ARM_LOW_SEEN_WAIT_NEW_EDGE")
    check(.06, good_high, "SIMULATED_TARGETS_ACTIVE", has_pwm=True)
    check(.10, rc_bytes(.7, .4, -.3, True), "SIMULATED_TARGETS_ACTIVE", has_pwm=True)
    check(.12, good_high[:-1] + bytes((good_high[-1] ^ 1,)), "SIMULATED_TARGETS_ACTIVE", has_pwm=True)
    check(.36, b"", "COMMAND_TIMEOUT")
    check(.37, good_high, "NEEDS_VALID_ARM_LOW")
    check(.38, good_low, "ARM_LOW_SEEN_WAIT_NEW_EDGE")
    check(.39, good_high, "SIMULATED_TARGETS_ACTIVE", has_pwm=True)
    check(.4, b"", "USB_VBUS_PRESENT_SIMULATED_LOCK", usb=True)
    check(.41, good_high, "NEEDS_VALID_ARM_LOW")
    check(.42, good_low, "ARM_LOW_SEEN_WAIT_NEW_EDGE")
    check(.43, good_high, "SIMULATED_TARGETS_ACTIVE", has_pwm=True)
    check(.44, good_low, "ARM_LOW_SEEN_WAIT_NEW_EDGE")
    check(.45, good_high, "SIMULATED_TARGETS_ACTIVE", has_pwm=True)
    check(.46, good_low + good_high, "ARM_LOW_SEEN_WAIT_NEW_EDGE")
    check(.47, good_high, "SIMULATED_TARGETS_ACTIVE", has_pwm=True)
    lost = bytes((0x14, 90, 91, 0, 0, 0, 0, 0, 90, 0, 0))
    restored = lost[:3] + b"\x64" + lost[4:]
    check(.48, b"\xc8\x0c" + lost + bytes((crc8(lost),)) + good_high, "LINK_LOST")
    check(.49, good_high, "LINK_LOST")
    check(.50, b"\xc8\x0c" + restored + bytes((crc8(restored),)) + good_high, "NEEDS_VALID_ARM_LOW")
    check(.51, good_low, "ARM_LOW_SEEN_WAIT_NEW_EDGE")
    check(.52, good_high, "SIMULATED_TARGETS_ACTIVE", has_pwm=True)
    check(.53, good_low, "ARM_LOW_SEEN_WAIT_NEW_EDGE")
    require(parser.rejected >= 1, "Bad CRC did not register")
    return results


def replay(wing, duration_s, cap_deg, hz, drive, maneuver=False, trace=None, substeps=3):
    mixer, decoder = new_controller(cap_deg, hz), RcInput()
    servos = {side: Servo(drive, wing) for side in SIDES}
    accum = {side: dict(low=math.inf, high=-math.inf, target_low=math.inf,
                        target_high=-math.inf, squared_error=0., limited=0, omega2=0.,
                        peak_rate=0., work=0.)
             for side in SIDES}
    last_rc = -1
    left_dominant = right_dominant = pitch_exercised = 0
    samples = 0
    for i in range(round(duration_s * PWM_HZ)):
        now = i / PWM_HZ
        rc_index = i * RC_HZ // PWM_HZ
        packet = b""
        if rc_index != last_rc:
            active = now >= .10
            throttle = min(1., max(0., (now - .2) / .8))
            pattern = (now - 5.) % 12. if now >= 5. else 0.
            turn = 1. if maneuver and 3. <= pattern < 5. else -1. if maneuver and 7. <= pattern < 9. else 0.
            pitch = (.3 if turn > 0 else -.3 if turn < 0 else 0.) if maneuver else 0.
            packet = rc_bytes(throttle, turn, pitch, active)
            last_rc = rc_index
        snapshot = control_sample(decoder, mixer, packet, now)
        pulse = virtual_pwm(snapshot)
        if now >= .11:
            require(pulse is not None, "Unexpected pulse dropout")
        if pulse is not None:
            require(all(CENTER_US - 480 <= p <= CENTER_US + 480 and 0 < p < 1e6 / PWM_HZ
                        for p in pulse.values()), "Virtual pulse outside configured range")
        if now >= 5.:
            samples += 1
            targets = snapshot["targets"]
            diff = targets["left"]["amplitude_deg"] - targets["right"]["amplitude_deg"]
            left_dominant += diff > 1.
            right_dominant += diff < -1.
            pitch_exercised += abs(targets["left"]["mean_offset_deg"]) > .5
        for side, sign in (("left", 1), ("right", -1)):
            target = None if pulse is None else sign * (pulse[side] - CENTER_US) / US_PER_DEG
            servo = servos[side]
            limited, work, omega2 = 0, 0., 0.
            target_rad = None if target is None else math.radians(target)
            dt = 1. / (PWM_HZ * substeps)
            for _ in range(substeps):
                clip, power = servo.step(target_rad, dt)
                limited += clip
                work += power * dt
                omega2 += servo.speed**2
            if now < 5.:
                continue
            record = accum[side]
            angle = math.degrees(servo.angle)
            record["low"] = min(record["low"], angle)
            record["high"] = max(record["high"], angle)
            record["target_low"] = min(record["target_low"], target)
            record["target_high"] = max(record["target_high"], target)
            record["squared_error"] += (target - angle)**2
            record["limited"] += limited / substeps
            record["omega2"] += omega2 / substeps
            record["peak_rate"] = max(record["peak_rate"], abs(math.degrees(servo.speed)))
            record["work"] += work
        if trace is not None and i % 11 == 0:
            trace.writerow([round(now, 5), snapshot["state"],
                            "" if pulse is None else pulse["left"], "" if pulse is None else pulse["right"],
                            round(math.degrees(servos["left"].angle), 4),
                            round(math.degrees(servos["right"].angle), 4)])

    if maneuver:
        require(min(left_dominant, right_dominant, pitch_exercised) > 0, "Missing maneuver output")
    final = control_sample(decoder, mixer, rc_bytes(), duration_s)
    require(virtual_pwm(final) is None, "Disarm at the end left virtual PWM enabled")
    if trace is not None:
        trace.writerow([duration_s, final["state"], "", "",
                        *(round(math.degrees(servos[side].angle), 4) for side in SIDES)])
    q_stroke = .5 * wing.rho * wing.area_m2 * wing.area_r2_m**2 * (
        sum(record["omega2"] for record in accum.values()) / (2 * samples))
    side_results = {side: {"target_half_range_deg": (r["target_high"] - r["target_low"]) / 2,
                           "simulated_half_range_deg": (r["high"] - r["low"]) / 2,
                           "tracking_rmse_deg": math.sqrt(r["squared_error"] / samples),
                           "torque_clip_pct": 100 * r["limited"] / samples,
                           "peak_simulated_rate_deg_s": r["peak_rate"],
                           "positive_shaft_work_j": r["work"]}
                    for side, r in accum.items()}
    # Forward speed is a parameter, not a predicted trim. Omits cross terms and uses
    # cycle-averaged stroke motion orthogonal to forward flow just as a speed scale.
    force_screen = []
    for speed in (0., 1., 2., 3.):
        q_area = q_stroke + .5 * wing.rho * wing.area_m2 * speed**2
        force_screen.append({"assumed_forward_speed_m_s": speed, "qS_N": q_area,
                             "required_effective_Cz_weight": wing.weight_n / q_area,
                             "required_effective_Cz_20pct_reserve": 1.2 * wing.weight_n / q_area})
    return {"elapsed_simulated_s": duration_s, "analysis_start_s": 5,
            "commanded_wing_cap_deg": cap_deg, "neutral_command_half_amplitude_deg": cap_deg / 1.25,
            "frequency_hz": hz, "full_steering_required_rate_deg_s": math.tau * hz * cap_deg,
            "integrator_substeps_per_pwm_frame": substeps,
            "drive_assumptions": drive.__dict__, "sides": side_results, "force_screen": force_screen,
            "left_dominant_samples": left_dominant, "right_dominant_samples": right_dominant,
            "pitch_offset_samples": pitch_exercised, "lost_pwm_frames_after_0p11s": 0,
            "final_virtual_pwm_disabled": True}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("output"))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    wing = Wing.synthetic_example()
    faults = run_faults()
    nominal = Drive(7.4, .16, 1.2, 12., 1.)
    cases = {
        "cap40_nominal": (40., 2.5, nominal),
        "neutral40_nominal": (50., 2.5, nominal),
        "neutral40_sag_derated": (50., 2.5, Drive(6., .18, 1.6, 9., .75)),
        "neutral40_favorable": (50., 2.5, Drive(8.4, .14, .8, 16., 1.)),
        "neutral45_at3Hz": (56.25, 3., nominal),
    }
    assessment = {key: replay(wing, 65, cap, hz, drive) for key, (cap, hz, drive) in cases.items()}
    # Compare integration step sizes over the same settled interval with unchanged PWM.
    reference = replay(wing, 9, 50., 2.5, nominal)
    refined = replay(wing, 9, 50., 2.5, nominal, substeps=6)
    q1 = reference["force_screen"][0]["qS_N"]
    q2 = refined["force_screen"][0]["qS_N"]
    convergence = abs(q1 - q2) / q2
    require(convergence < .03, "Numerical integration changed qS by more than 3%")
    with (args.output / "simulation-trace.csv").open("w", newline="") as handle:
        trace = csv.writer(handle)
        trace.writerow(["time_s", "state", "left_pwm_us_virtual", "right_pwm_us_virtual",
                        "left_angle_deg_model", "right_angle_deg_model"])
        maneuver = replay(wing, 65, 50., 2.5, nominal, maneuver=True, trace=trace)
    result = {
        "control_simulation": "PASS",
        "flight_feasibility": "UNVERIFIED",
        "physical_data": None,
        "scope": "Offline example with synthetic wing geometry. No measured flight data.",
        "wing": wing.__dict__,
        "fault_replay": faults,
        "integration_step_relative_qS_change": convergence,
        "straight_cases": assessment,
        "maneuver": maneuver,
    }
    (args.output / "simulation-result.json").write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(f"{len(faults)} control checkpoints passed; {len(cases)} straight scenarios and one maneuver replay.")
    for name, case in assessment.items():
        side = case["sides"]["left"]
        still, _, glide, _ = case["force_screen"]
        print(f"{name}: {side['simulated_half_range_deg']:.1f}deg, clip={side['torque_clip_pct']:.1f}%, "
              f"Cz(0,2 m/s)={still['required_effective_Cz_20pct_reserve']:.2f},"
              f"{glide['required_effective_Cz_20pct_reserve']:.2f}")
    print("Modeled command path PASS; physical free flight UNVERIFIED.")


if __name__ == "__main__":
    main()
