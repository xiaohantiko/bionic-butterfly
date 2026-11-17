#!/usr/bin/env python3
"""Run a deterministic demo or replay a local JSONL command file. Standard library only."""

import argparse
import json
from pathlib import Path
import sys

from mixer_sim import CommandFrame, MixerSimulation


def demo_events():
    for index in range(90):
        now = round(index * 0.02, 6)
        event = dict(kind="frame", time_s=now, throttle=0.0, turn=0.0, pitch=0.0,
                     arm=False, link_ok=True, usb_vbus_present=False)
        if index < 3:
            event["arm"] = True  # High on startup must not arm.
        elif index == 3:
            pass
        elif index < 25:
            event.update(arm=True, throttle=0.0 if index == 4 else 0.8,
                         turn=0.7, pitch=0.25)
        elif index < 30:
            event.update(arm=True, throttle=0.8, link_ok=False)
        elif index < 34:
            event.update(arm=True, throttle=0.0)  # Reconnection alone must not arm.
        elif index == 34:
            pass
        elif index < 50:
            event.update(arm=True, throttle=0.0 if index == 35 else 0.6, turn=-0.6)
        elif index < 55:
            event.update(arm=False, usb_vbus_present=True)
        elif index < 59:
            event.update(arm=True)  # Unplugging alone must not arm.
        elif index == 59:
            pass
        elif index < 70:
            event.update(arm=True, throttle=0.0 if index == 60 else 0.7)
        else:
            event = dict(kind="tick", time_s=now)  # No incoming frames: timeout.
        yield event


def read_events(path):
    with path.open(encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, 1):
            if line.strip():
                try:
                    yield json.loads(line)
                except json.JSONDecodeError as error:
                    raise ValueError(f"Line {line_number}: invalid JSON") from error


def apply_event(sim, event):
    def reject(message):
        sim.stop("INVALID_EVENT_SCHEMA")
        raise ValueError(message)

    if not isinstance(event, dict):
        reject("Each event must be a JSON object")
    kind = event.get("kind")
    if kind == "tick":
        if set(event) != {"kind", "time_s"}:
            reject("A tick requires only kind and time_s")
        return sim.tick(event["time_s"])
    expected = {"kind", "time_s", "throttle", "turn", "pitch", "arm", "link_ok",
                "usb_vbus_present"}
    if kind != "frame" or set(event) != expected:
        reject("A frame requires all documented fields, with no unknown fields")
    return sim.step(CommandFrame(**{key: value for key, value in event.items() if key != "kind"}))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, help="Local JSONL events; omit for deterministic demo")
    parser.add_argument("--summary", action="store_true", help="Print only changes in state/reason")
    args = parser.parse_args(argv)
    sim = MixerSimulation()
    previous = None
    try:
        events = read_events(args.input) if args.input else demo_events()
        for event in events:
            result = apply_event(sim, event)
            signature = (result["state"], result["reason"])
            if not args.summary or signature != previous:
                print(json.dumps(result, ensure_ascii=False, allow_nan=False))
            previous = signature
    except (OSError, ValueError, TypeError) as error:
        print(json.dumps(sim.stop("INPUT_STREAM_STOPPED"), ensure_ascii=False, allow_nan=False))
        print(f"Simulation stopped: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
