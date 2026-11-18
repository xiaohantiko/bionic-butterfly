"""Six synthetic rotational DOFs, fixed body, SI units internally."""

import argparse
import csv
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path


@dataclass(frozen=True)
class Params:
    root_inertia: float = 0.000020
    fore_inertia: float = 0.000036
    hind_inertia: float = 0.000023
    fore_stiffness: float = 0.22
    hind_stiffness: float = 0.13
    fore_coupling_damping: float = 0.0014
    hind_coupling_damping: float = 0.00085
    root_damping: float = 0.00045
    panel_damping: float = 0.00012
    quadratic_drag: float = 0.000055
    tracking_stiffness: float = 0.72
    tracking_damping: float = 0.0032
    torque_limit: float = 0.090
    stop_angle: float = math.radians(14)
    stop_stiffness: float = 1.1
    stop_damping: float = 0.0020
    frequency_hz: float = 2.0
    ramp_s: float = 0.5


@dataclass(frozen=True)
class Case:
    left_deg: float = 32
    right_deg: float = 32
    obstacle: bool = False
    drive_off_s: float = math.inf


CASES = {
    "straight": Case(),
    "left_smaller": Case(22, 36),
    "right_smaller": Case(36, 22),
    "right_hind_stop": Case(obstacle=True),
    "drive_off": Case(drive_off_s=2),
}
NAMES = ("L_root", "LF", "LH", "R_root", "RF", "RH")
PARAMS = Params()


def reference(time_s, amplitude, params):
    ramp = min(1, time_s / params.ramp_s)
    return (math.radians(amplitude) * (0.5 - 0.5 * math.cos(math.pi * ramp)) *
            math.sin(2 * math.pi * params.frequency_hz * time_s))


def rates(state, time_s, case, powered, params):
    angles, speeds = state[:6], state[6:12]
    acceleration = [0.0] * 6
    input_power = loss_power = contact_force = 0.0
    torques = []
    for root, amplitude in ((0, case.left_deg), (3, case.right_deg)):
        target = reference(time_s, amplitude, params)
        drive = params.tracking_stiffness * (target - angles[root]) - params.tracking_damping * speeds[root]
        drive = max(-params.torque_limit, min(params.torque_limit, drive)) if powered else 0.0
        torques.append(drive)
        root_net = drive - params.root_damping * speeds[root]
        input_power += drive * speeds[root]
        loss_power += params.root_damping * speeds[root] ** 2
        for offset, inertia, stiffness, damping in (
                (1, params.fore_inertia, params.fore_stiffness, params.fore_coupling_damping),
                (2, params.hind_inertia, params.hind_stiffness, params.hind_coupling_damping)):
            panel = root + offset
            rel_speed = speeds[root] - speeds[panel]
            coupling = stiffness * (angles[root] - angles[panel]) + damping * rel_speed
            resistance = (params.panel_damping + params.quadratic_drag * abs(speeds[panel])) * speeds[panel]
            root_net -= coupling
            panel_net = coupling - resistance
            loss_power += damping * rel_speed ** 2 + resistance * speeds[panel]
            if case.obstacle and panel == 5 and angles[panel] > params.stop_angle:
                compression_rate = max(speeds[panel], 0.0)
                contact_force = (params.stop_stiffness * (angles[panel] - params.stop_angle) +
                                 params.stop_damping * compression_rate)
                panel_net -= contact_force
                loss_power += params.stop_damping * compression_rate ** 2
            acceleration[panel] = panel_net / inertia
        acceleration[root] = root_net / params.root_inertia
    return list(speeds) + acceleration + [input_power, loss_power], torques, contact_force


def rk4(state, time_s, step, case, powered, params):
    def shift(base, delta, scale):
        return [value + scale * rate for value, rate in zip(base, delta)]

    first = rates(state, time_s, case, powered, params)[0]
    second = rates(shift(state, first, step / 2), time_s + step / 2, case, powered, params)[0]
    third = rates(shift(state, second, step / 2), time_s + step / 2, case, powered, params)[0]
    fourth = rates(shift(state, third, step), time_s + step, case, powered, params)[0]
    return [value + step * (a + 2 * b + 2 * c + d) / 6
            for value, a, b, c, d in zip(state, first, second, third, fourth)]


def energy(state, case, params):
    inertias = (params.root_inertia, params.fore_inertia, params.hind_inertia) * 2
    kinetic = sum(inertia * speed ** 2 / 2 for inertia, speed in zip(inertias, state[6:12]))
    potential = sum((params.fore_stiffness * (state[root] - state[root + 1]) ** 2 +
                     params.hind_stiffness * (state[root] - state[root + 2]) ** 2) / 2
                    for root in (0, 3))
    if case.obstacle:
        potential += params.stop_stiffness * max(0.0, state[5] - params.stop_angle) ** 2 / 2
    return kinetic + potential


def replay(case, step=0.001, duration=3.0, params=PARAMS):
    cadence = round(0.01 / step)
    steps = round(duration / step)
    if cadence < 1 or abs(cadence * step - 0.01) > 1e-12 or abs(steps * step - duration) > 1e-12:
        raise ValueError("Step must divide the 10 ms output interval and simulation duration.")
    if math.isfinite(case.drive_off_s) and abs(round(case.drive_off_s / step) * step - case.drive_off_s) > 1e-12:
        raise ValueError("The drive-off event must coincide with an integration boundary.")
    state = [0.0] * 14
    rows = []
    for index in range(steps + 1):
        time_s = index * step
        powered = time_s < case.drive_off_s
        if index % cadence == 0:
            _, torque, contact = rates(state, time_s, case, powered, params)
            row = {"time_s": round(time_s, 6), "powered": int(powered)}
            row.update({name + "_deg": math.degrees(value) for name, value in zip(NAMES, state)})
            for side, amplitude, value in zip(("L", "R"), (case.left_deg, case.right_deg), torque):
                row[side + "_reference_deg"] = math.degrees(reference(time_s, amplitude, params))
                row[side + "_drive_Nm"] = value
            row.update(RH_contact_Nm=contact, stored_J=energy(state, case, params),
                       net_drive_work_J=state[12], dissipated_J=state[13])
            rows.append(row)
        if index != steps:
            state = rk4(state, time_s, step, case, powered, params)
    return rows


def metrics(rows):
    window = [row for row in rows if 1 <= row["time_s"] < 2]
    half_range = {}
    for name in NAMES:
        angles = [row[name + "_deg"] for row in window]
        half_range[name] = (max(angles) - min(angles)) / 2
    return {
        "pre_event_half_range_deg_1_to_2s": half_range,
        "RH_peak_deg_1_to_2s": max(row["RH_deg"] for row in window),
        "peak_RH_contact_Nm_sampled": max(row["RH_contact_Nm"] for row in rows),
        "max_abs_energy_residual_J_sampled":
            max(abs(row["stored_J"] + row["dissipated_J"] - row["net_drive_work_J"]) for row in rows),
        "stored_J_at_2s": next(row["stored_J"] for row in rows if row["time_s"] == 2),
        "stored_J_final": rows[-1]["stored_J"],
    }


def chart(all_rows):
    width, height = 850, 445
    colors = ("#147d87", "#c45b12", "#7147a4")
    markup = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img">',
              '<title>Synthetic right-side panel angles with and without a compliant stop</title>',
              '<rect width="100%" height="100%" fill="#fff"/>',
              '<g font-family="sans-serif" fill="#203345">',
              '<text x="65" y="35" font-size="23">Right-hand panels: free stroke vs. a compliant stop</text>',
              '<text x="65" y="60" font-size="14">Synthetic angular response; degrees versus time, fixed body</text>']
    def x_coord(time_s):
        return 72 + (time_s - 1) * 730

    def y_coord(angle):
        return 244 - angle * 4.1

    for angle in (-30, -15, 0, 15, 30):
        y = y_coord(angle)
        markup += [f'<path d="M72 {y} H802" stroke="#d4dde1"/>',
                   f'<text x="62" y="{y + 5}" text-anchor="end">{angle}°</text>']
    for tick in (1, 1.25, 1.5, 1.75, 2):
        x = x_coord(tick)
        markup += [f'<path d="M{x} 94 V387" stroke="#e3e8ec"/>',
                   f'<text x="{x}" y="408" text-anchor="middle">{tick:g} s</text>']
    plot_curves = (("straight", "RH_deg", "RH free"), ("right_hind_stop", "RH_deg", "RH with stop"),
                   ("right_hind_stop", "RF_deg", "RF on same drive"))
    for index, (scenario, field, label) in enumerate(plot_curves):
        points = " ".join(f'{x_coord(row["time_s"]):.2f},{y_coord(row[field]):.2f}'
                          for row in all_rows[scenario] if 1 <= row["time_s"] <= 2)
        color = colors[index]
        markup += [f'<polyline points="{points}" stroke="{color}" stroke-width="2.7" fill="none"/>',
                   f'<rect x="{78 + 215 * index}" y="78" width="23" height="4" fill="{color}"/>',
                   f'<text x="{108 + 215 * index}" y="85">{label}</text>']
    stop_y = y_coord(math.degrees(PARAMS.stop_angle))
    markup.append(f'<path d="M72 {stop_y} H802" stroke="#934423" stroke-dasharray="3 6"/>')
    markup.append('<text x="72" y="435" font-size="13">Dotted: start of stop compression (14°); all curves sampled at 100 Hz.</text></g></svg>')
    return "\n".join(markup) + "\n"


def animation(all_rows):
    markup = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 880 370" width="880" height="370" role="img">',
              '<title>Four synthetic panel angles, one simulated cycle at quarter speed</title>',
              '<rect width="100%" height="100%" fill="#fff"/><g font-family="sans-serif" fill="#203345">',
              '<text x="32" y="40" font-size="23">Four-panel angular response</text>',
              '<text x="32" y="66" font-size="15">One simulated cycle (1.00–1.50 s) shown at ¼ speed; fixed front view, separated rows for clarity.</text>']
    for center, name, title in ((230, "straight", "Free stroke"), (652, "right_hind_stop", "Right hind obstructed")):
        rows = [row for row in all_rows[name] if 1 <= row["time_s"] <= 1.5]
        markup.append(f'<text x="{center}" y="104" text-anchor="middle" font-size="18">{title}</text>')
        for y_pos, suffix, label in ((170, "F", "Fore"), (276, "H", "Hind")):
            markup.append(f'<path d="M{center - 160} {y_pos} h320" stroke="#d5dfe4" stroke-dasharray="4 5"/>')
            markup.append(f'<text x="{center}" y="{y_pos + 29}" text-anchor="middle" font-size="13">{label}</text>')
            for side, direction in (("L", -1), ("R", 1)):
                key = side + suffix + "_deg"
                rotation = ";".join(f'{-direction * row[key]:.4f}' for row in rows)
                color = "#c45b12" if key == "RH_deg" and name == "right_hind_stop" else "#147d87"
                markup.append(f'<g transform="translate({center + 6 * direction} {y_pos})"><g transform="rotate({-direction * rows[0][key]:.4f})">')
                markup.append(f'<animateTransform attributeName="transform" type="rotate" values="{rotation}" dur="2s" repeatCount="indefinite"/>')
                markup.append(f'<path d="M0 0 h{125 * direction}" stroke="{color}" stroke-width="5" stroke-linecap="round"/>')
                markup.append(f'<circle cx="{125 * direction}" cy="0" r="5" fill="{color}"/></g><circle r="4" fill="#203345"/></g>')
    markup.append('<text x="32" y="355" font-size="13">Each line shows an effective panel angle. Relative motion is modeled; the drawing does not represent physical spacing.</text></g></svg>')
    return "\n".join(markup) + "\n"


def generate(output):
    output.mkdir(parents=True, exist_ok=True)
    all_rows = {}
    case_results = {}
    for name, case in CASES.items():
        coarse, fine = replay(case), replay(case, step=0.0005)
        all_rows[name] = fine
        result = metrics(fine)
        result["max_abs_angle_diff_deg_1ms_vs_0p5ms_all_outputs"] = max(
            abs(first[field + "_deg"] - second[field + "_deg"])
            for first, second in zip(coarse, fine) for field in NAMES)
        result["drive_off_s"] = case.drive_off_s if math.isfinite(case.drive_off_s) else None
        result["left_command_deg"], result["right_command_deg"] = case.left_deg, case.right_deg
        result["obstacle"] = case.obstacle
        case_results[name] = result
    summary = {"source": "synthetic, not measured", "integrator": "RK4", "published_step_s": 0.0005,
               "comparison_step_s": 0.001, "force_model": "none; angular drag surrogate only",
               "trace_interval_s": 0.01, "duration_s_per_case": 3, "rows_per_case": 301,
               "parameters_si": asdict(PARAMS), "cases": case_results}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    with (output / "trace.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=["case"] + list(next(iter(all_rows.values()))[0]), lineterminator="\n")
        writer.writeheader()
        for name, rows in all_rows.items():
            for row in rows:
                writer.writerow({"case": name, **{key: f"{value:.10g}" if isinstance(value, float) else value
                                                   for key, value in row.items()}})
    (output / "hind-stop.svg").write_text(chart(all_rows), encoding="utf-8")
    (output / "four-panel-motion.svg").write_text(animation(all_rows), encoding="utf-8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("output/four-panel-demo"))
    generate(parser.parse_args().output)
