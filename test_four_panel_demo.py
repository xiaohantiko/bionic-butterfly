import math
import unittest
from dataclasses import replace

from four_panel_demo import CASES, NAMES, PARAMS, Case, energy, metrics, replay, rk4


class PanelDynamicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.traces = {name: replay(case, 0.0005) for name, case in CASES.items()}

    def test_unforced_frictionless_rigid_rotation_matches_analytic_solution(self):
        params = replace(PARAMS, root_damping=0, panel_damping=0, quadratic_drag=0)
        state = [0.6] * 6 + [-1.25] * 6 + [0.0, 0.0]
        for index in range(1000):
            state = rk4(state, index * 0.001, 0.001, Case(), False, params)
        for angle in state[:6]:
            self.assertAlmostEqual(angle, 0.6 - 1.25, places=12)
        for speed in state[6:12]:
            self.assertAlmostEqual(speed, -1.25, places=12)
        expected_energy = (params.root_inertia + params.fore_inertia + params.hind_inertia) * 1.25 ** 2
        self.assertAlmostEqual(energy(state, Case(), params), expected_energy, places=14)
        self.assertEqual(state[12:], [0.0, 0.0])

    def test_symmetric_commands_and_mirror_swapped_commands(self):
        for first_case, mirror_case in (("straight", "straight"), ("left_smaller", "right_smaller")):
            for first, mirror in zip(self.traces[first_case], self.traces[mirror_case]):
                for left, right in zip(NAMES[:3], NAMES[3:]):
                    self.assertEqual(first[left + "_deg"], mirror[right + "_deg"])
                    self.assertEqual(first[right + "_deg"], mirror[left + "_deg"])
        amplitudes = metrics(self.traces["left_smaller"])["pre_event_half_range_deg_1_to_2s"]
        self.assertLess(amplitudes["LF"], amplitudes["RF"])
        self.assertLess(amplitudes["LH"], amplitudes["RH"])

    def test_unilateral_contact_is_local_and_changes_same_side_front(self):
        free, stopped = self.traces["straight"], self.traces["right_hind_stop"]
        for normal, hit in zip(free, stopped):
            self.assertEqual(normal["LH_deg"], hit["LH_deg"])
            self.assertEqual(normal["LF_deg"], hit["LF_deg"])
            self.assertEqual(normal["RH_contact_Nm"], 0)
            if hit["RH_deg"] <= math.degrees(PARAMS.stop_angle):
                self.assertEqual(hit["RH_contact_Nm"], 0)
        self.assertGreater(max(row["RH_contact_Nm"] for row in stopped), 0)
        self.assertLess(max(row["RH_deg"] for row in stopped), max(row["RH_deg"] for row in free) - 10)
        self.assertGreater(max(abs(row["RF_deg"] - hit["RF_deg"]) for row, hit in zip(free, stopped)), 1)

    def test_drive_off_removes_external_work_and_dissipates_energy(self):
        decay = [row for row in self.traces["drive_off"] if row["time_s"] >= 2]
        self.assertGreater(decay[0]["stored_J"], 0)
        for row in decay:
            self.assertEqual((row["powered"], row["L_drive_Nm"], row["R_drive_Nm"]), (0, 0, 0))
            self.assertEqual(row["net_drive_work_J"], decay[0]["net_drive_work_J"])
        self.assertTrue(all(a["stored_J"] >= b["stored_J"] for a, b in zip(decay, decay[1:])))
        self.assertLess(decay[-1]["stored_J"], decay[0]["stored_J"] * 0.0001)

    def test_work_energy_closure_and_drive_bound(self):
        for name, rows in self.traces.items():
            residual = metrics(rows)["max_abs_energy_residual_J_sampled"]
            self.assertLess(residual, 1e-6 if name == "right_hind_stop" else 1e-8)
            self.assertTrue(all(a["dissipated_J"] <= b["dissipated_J"] for a, b in zip(rows, rows[1:])))
            self.assertTrue(all(abs(row[side + "_drive_Nm"]) <= PARAMS.torque_limit
                                for row in rows for side in ("L", "R")))

    def test_step_halving_repeats_sampled_angles_within_tolerance(self):
        for name, case in CASES.items():
            coarse = replay(case, 0.001)
            fine = self.traces[name]
            self.assertEqual([row["time_s"] for row in coarse], [row["time_s"] for row in fine])
            delta = max(abs(first[key + "_deg"] - second[key + "_deg"])
                        for first, second in zip(coarse, fine) for key in NAMES)
            self.assertLess(delta, 0.05 if case.obstacle else 0.001)


if __name__ == "__main__":
    unittest.main()
