"""Behavioral checks for the simulation; these are not hardware acceptance tests."""

import itertools
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from mixer_sim import CommandFrame, MixerSimulation, SideBounds, SimConfig
from run_sim import apply_event, demo_events


def frame(time_s, **changes):
    defaults = dict(time_s=time_s, throttle=0.0, turn=0.0, pitch=0.0,
                    arm=False, link_ok=True, usb_vbus_present=False)
    defaults.update(changes)
    return CommandFrame(**defaults)


class SimulationTests(unittest.TestCase):
    def assert_stopped(self, result):
        self.assertIsNone(result["targets"])
        self.assertIsNone(result["valid_until_s"])
        self.assertNotEqual(result["state"], "ARMED")

    def arm(self, sim):
        self.assert_stopped(sim.step(frame(0.0)))
        result = sim.step(frame(0.02, arm=True))
        self.assertEqual(result["state"], "ARMED")
        return result

    def test_startup_high_cannot_arm_even_at_zero_throttle(self):
        sim = MixerSimulation()
        for index in range(5):
            result = sim.step(frame(index * 0.02, arm=True))
            self.assert_stopped(result)
        self.assert_stopped(sim.step(frame(0.1)))
        self.assertEqual(sim.step(frame(0.12, arm=True))["state"], "ARMED")

    def test_high_throttle_rejection_consumes_edge(self):
        sim = MixerSimulation()
        sim.step(frame(0.0))
        rejected = sim.step(frame(0.02, arm=True, throttle=0.5))
        self.assertEqual(rejected["reason"], "ARM_REJECTED_HIGH_THROTTLE")
        self.assert_stopped(sim.step(frame(0.04, arm=True, throttle=0.0)))
        sim.step(frame(0.06))
        self.assertEqual(sim.step(frame(0.08, arm=True))["state"], "ARMED")

    def test_low_throttle_threshold_is_inclusive(self):
        for throttle, armed in ((0.05, True), (0.050001, False)):
            with self.subTest(throttle=throttle):
                sim = MixerSimulation()
                sim.step(frame(0.0))
                result = sim.step(frame(0.02, arm=True, throttle=throttle))
                self.assertEqual(result["state"] == "ARMED", armed)

    def test_loss_or_usb_clears_output_and_rearm_history(self):
        for fault in ({"link_ok": False}, {"usb_vbus_present": True}):
            with self.subTest(fault=fault):
                sim = MixerSimulation()
                self.arm(sim)
                active = sim.step(frame(0.1, arm=True, throttle=0.7))
                self.assertGreater(abs(active["targets"]["left"]["angle_deg"]), 0)
                self.assert_stopped(sim.step(frame(0.12, arm=False, **fault)))
                self.assert_stopped(sim.step(frame(0.14, arm=True)))
                self.assert_stopped(sim.step(frame(0.16)))
                self.assertEqual(sim.step(frame(0.18, arm=True))["state"], "ARMED")

    def test_fault_low_frames_cannot_qualify_startup(self):
        for fault in ({"link_ok": False}, {"usb_vbus_present": True}, {"turn": float("nan")}):
            with self.subTest(fault=fault):
                sim = MixerSimulation()
                self.assert_stopped(sim.step(frame(0.0, **fault)))
                self.assert_stopped(sim.step(frame(0.02, arm=True)))

    def test_explicit_disarm_clears_output_immediately(self):
        sim = MixerSimulation()
        self.arm(sim)
        sim.step(frame(0.1, arm=True, throttle=1))
        self.assert_stopped(sim.step(frame(0.12)))
        self.assert_stopped(sim.tick(0.14))

    def test_timeout_boundary_without_new_frames(self):
        sim = MixerSimulation()
        armed = self.arm(sim)
        deadline = armed["valid_until_s"]
        self.assertEqual(sim.tick(deadline)["state"], "ARMED")
        self.assert_stopped(sim.tick(deadline + 0.000001))
        self.assert_stopped(sim.step(frame(deadline + 0.02, arm=True)))

    def test_late_frame_does_not_rebuild_low_qualification(self):
        sim = MixerSimulation()
        self.arm(sim)
        late_low = sim.step(frame(0.30, arm=False))
        self.assertEqual(late_low["reason"], "COMMAND_TIMEOUT_FRAME_DISCARDED")
        self.assert_stopped(sim.step(frame(0.32, arm=True)))
        sim.step(frame(0.34))
        self.assertEqual(sim.step(frame(0.36, arm=True))["state"], "ARMED")

    def test_low_qualification_itself_expires(self):
        sim = MixerSimulation()
        sim.step(frame(0.0))
        self.assert_stopped(sim.tick(0.250001))
        self.assert_stopped(sim.step(frame(0.26, arm=True)))

    def test_invalid_inputs_trip_instead_of_clamping_or_coercing(self):
        cases = [{name: value} for name in ("throttle", "turn", "pitch")
                 for value in (float("nan"), float("inf"), -float("inf"), True, "0")]
        cases += [{"throttle": -0.1}, {"throttle": 1.1}, {"turn": 1.1}, {"pitch": -1.1}]
        cases += [{name: value} for name in ("arm", "link_ok", "usb_vbus_present")
                  for value in (0, 1, None, "false")]
        for bad in cases:
            with self.subTest(bad=bad):
                sim = MixerSimulation()
                self.arm(sim)
                result = sim.step(frame(0.04, **bad))
                self.assert_stopped(result)
                self.assertEqual(result["reason"], "INVALID_CONTROL_INPUT")
                json.dumps(result, allow_nan=False)
                self.assert_stopped(sim.step(frame(0.06, arm=True)))

    def test_oversized_integer_is_invalid_and_clears_an_active_state(self):
        for field in ("throttle", "turn", "pitch", "time_s"):
            with self.subTest(field=field):
                sim = MixerSimulation()
                self.arm(sim)
                values = dict(time_s=0.04, throttle=0.0, turn=0.0, pitch=0.0,
                              arm=True, link_ok=True, usb_vbus_present=False)
                values[field] = 10 ** 400
                self.assert_stopped(sim.step(CommandFrame(**values)))
                self.assert_stopped(sim.tick(0.06))

    def test_invalid_time_trips_and_does_not_poison_next_valid_time(self):
        for bad in (0.02, 0.01, -1, float("nan"), float("inf"), True, None):
            with self.subTest(bad=bad):
                sim = MixerSimulation()
                self.arm(sim)
                result = sim.tick(bad)
                self.assert_stopped(result)
                json.dumps(result, allow_nan=False)
                self.assert_stopped(sim.step(frame(0.04, arm=True)))
                sim.step(frame(0.06))
                self.assertEqual(sim.step(frame(0.08, arm=True))["state"], "ARMED")

    def test_reset_removes_arming_qualification(self):
        sim = MixerSimulation()
        self.arm(sim)
        sim.reset()
        self.assert_stopped(sim.step(frame(0.0, arm=True)))

    def test_virtual_mixing_direction_and_zero_throttle(self):
        sim = MixerSimulation()
        self.arm(sim)
        zero = sim.step(frame(0.04, arm=True, throttle=0, turn=1, pitch=1))
        self.assertEqual(zero["targets"]["left"]["angle_deg"], 0)
        result = sim.step(frame(0.06, arm=True, throttle=1, turn=1, pitch=1))
        self.assertLess(result["targets"]["left"]["amplitude_deg"],
                        result["targets"]["right"]["amplitude_deg"])
        self.assertEqual(result["targets"]["left"]["mean_offset_deg"], 2)
        self.assertEqual(result["targets"]["right"]["mean_offset_deg"], 2)

    def test_angle_rate_and_amplitude_bounds_during_command_reversals(self):
        sim = MixerSimulation()
        previous = self.arm(sim)
        time_s = 0.02
        samples = 0
        for throttle, turn, pitch in itertools.product((0, 0.05, 0.5, 1), (-1, 0, 1), (-1, 0, 1)):
            for _ in range(25):
                time_s = round(time_s + 0.01, 8)
                result = sim.step(frame(time_s, arm=True, throttle=throttle, turn=turn, pitch=pitch))
                self.assertEqual(result["state"], "ARMED")
                for side in ("left", "right"):
                    target = result["targets"][side]
                    angle = target["angle_deg"]
                    self.assertTrue(-10 <= angle <= 10)
                    self.assertLessEqual(target["amplitude_deg"], 6 + 1e-10)
                    self.assertLessEqual(abs(target["mean_offset_deg"]), 2 + 1e-10)
                    delta = abs(angle - previous["targets"][side]["angle_deg"])
                    self.assertLessEqual(delta, 40 * (time_s - previous["time_s"]) + 1e-9)
                    self.assertTrue(target["software_within_bounds"])
                previous = result
                samples += 1
        self.assertEqual(samples, 900)

    def test_each_side_uses_its_own_envelope(self):
        narrow = SideBounds(-4, 4, 2, 1, 15)
        sim = MixerSimulation(SimConfig(right=narrow))
        self.arm(sim)
        for index in range(1, 100):
            result = sim.step(frame(0.02 + index * 0.01, arm=True, throttle=1, turn=1, pitch=-1))
            target = result["targets"]["right"]
            self.assertLessEqual(abs(target["angle_deg"]), 4)
            self.assertLessEqual(target["amplitude_deg"], 2)
            self.assertEqual(target["bounds"]["max_rate_deg_s"], 15)

    def test_invalid_configuration_is_rejected(self):
        for changes in ({"frequency_hz": 0}, {"frequency_hz": 10},
                        {"command_timeout_s": -1}, {"turn_fraction": 1.1},
                        {"arm_throttle_max": float("nan")}, {"frequency_hz": True}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                SimConfig(**changes)
        for bounds in (SideBounds(10, -10), SideBounds(max_rate_deg_s=0),
                       SideBounds(max_angle_deg=5), SideBounds(max_mean_offset_deg=-1),
                       SideBounds(max_amplitude_deg=math.inf)):
            with self.subTest(bounds=bounds), self.assertRaises(ValueError):
                SimConfig(left=bounds)

    def test_snapshots_cannot_mutate_state_and_never_claim_hardware_validation(self):
        sim = MixerSimulation()
        first = self.arm(sim)
        first["targets"]["left"]["angle_deg"] = 10000
        first["targets"]["left"]["bounds"]["max_angle_deg"] = 10000
        result = sim.tick(0.04)
        self.assertEqual(result["targets"]["left"]["angle_deg"], 0)
        self.assertEqual(result["targets"]["left"]["bounds"]["max_angle_deg"], 10)
        for key in ("hardware_interlock_implemented", "hardware_angle_calibration_validated",
                    "radio_channel_mapping_validated", "flight_gains_validated"):
            self.assertIs(result[key], False)

    def test_demo_exercises_loss_usb_timeout_without_automatic_restart(self):
        sim = MixerSimulation()
        results = [apply_event(sim, event) for event in demo_events()]
        for index in (0, 1, 2, 25, 30, 33, 50, 55, 58, 83, 89):
            self.assert_stopped(results[index])
        for index in (4, 24, 35, 49, 60, 69):
            self.assertEqual(results[index]["state"], "ARMED")
        self.assertEqual(results[-1]["reason"], "COMMAND_TIMEOUT")

    def test_cli_outputs_json_and_rejects_incomplete_schema(self):
        runner = Path(__file__).with_name("run_sim.py")
        run = subprocess.run([sys.executable, str(runner), "--summary"],
                             capture_output=True, text=True, check=True)
        lines = [json.loads(line) for line in run.stdout.splitlines()]
        self.assertGreater(len(lines), 5)
        with tempfile.TemporaryDirectory() as directory:
            bad = Path(directory) / "bad.jsonl"
            bad.write_text('{"kind":"frame","time_s":0}\n', encoding="utf-8")
            result = subprocess.run([sys.executable, str(runner), "--input", str(bad)],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 2)
            self.assert_stopped(json.loads(result.stdout))

    def test_bad_event_schema_clears_active_state_before_raising(self):
        for bad in ({"kind": "frame", "time_s": 0.03},
                    {"kind": "tick", "time_s": 0.03, "unexpected": True},
                    ["frame"], None):
            with self.subTest(bad=bad):
                sim = MixerSimulation()
                self.arm(sim)
                with self.assertRaises(ValueError):
                    apply_event(sim, bad)
                self.assert_stopped(sim.tick(0.04))
                self.assert_stopped(sim.step(frame(0.06, arm=True)))

    def test_cli_bad_json_or_schema_after_activity_finishes_with_no_targets(self):
        runner = Path(__file__).with_name("run_sim.py")
        prefix = [dict(kind="frame", time_s=0.0, throttle=0, turn=0, pitch=0,
                       arm=False, link_ok=True, usb_vbus_present=False),
                  dict(kind="frame", time_s=0.02, throttle=0, turn=0, pitch=0,
                       arm=True, link_ok=True, usb_vbus_present=False)]
        for bad in ('{invalid json}\n', '{"kind":"frame","time_s":0.04}\n'):
            with self.subTest(bad=bad), tempfile.TemporaryDirectory() as directory:
                source = Path(directory) / "events.jsonl"
                source.write_text(''.join(json.dumps(x) + '\n' for x in prefix) + bad,
                                  encoding="utf-8")
                result = subprocess.run([sys.executable, str(runner), "--input", str(source)],
                                        capture_output=True, text=True)
                self.assertEqual(result.returncode, 2)
                rows = [json.loads(x) for x in result.stdout.splitlines()]
                self.assertEqual(rows[-2]["state"], "ARMED")
                self.assert_stopped(rows[-1])
                self.assertEqual(rows[-1]["reason"], "INPUT_STREAM_STOPPED")


if __name__ == "__main__":
    unittest.main()
