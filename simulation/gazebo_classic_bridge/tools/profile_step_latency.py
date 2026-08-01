"""Measure the wall-clock phases of the isolated Gazebo bridge.

This diagnostic intentionally bypasses the controller and CBF. It holds one
constant differential-drive command so bridge / physics synchronization can be
profiled independently from the research policy.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, replace
from hashlib import sha256
import json
import os
from pathlib import Path
from statistics import fmean, median
from time import perf_counter

from cbf_rover.adapters.gazebo.classic_bridge import GazeboClassicSession
from cbf_rover.adapters.gazebo.differential_road_contract import (
    load_gazebo_differential_road_contract,
)


def _file_sha256(path: Path) -> str:
    digest = sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _nearest_rank(values: list[float], probability: float) -> float:
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, int(probability * len(ordered) - 1)))
    return ordered[index]


def _latency_summary(values: list[float]) -> dict[str, float]:
    return {
        "minimum_ms": min(values),
        "median_ms": median(values),
        "mean_ms": fmean(values),
        "p95_ms": _nearest_rank(values, 0.95),
        "maximum_ms": max(values),
    }


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--world", type=Path, required=True)
    parser.add_argument("--bridge", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--master-uri", required=True)
    parser.add_argument("--steps", type=int, default=20)
    parser.add_argument("--linear-mps", type=float, default=0.10)
    parser.add_argument("--angular-radps", type=float, default=0.0)
    parser.add_argument("--gzserver-binary", default="gzserver")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    if args.steps <= 0:
        raise ValueError("--steps must be positive")
    config_path = args.config.resolve()
    world_path = args.world.resolve()
    bridge_path = args.bridge.resolve()
    destination = args.output_dir.resolve()
    destination.mkdir(parents=True, exist_ok=False)

    contract = load_gazebo_differential_road_contract(config_path)
    runtime = replace(contract.runtime, master_uri=str(args.master_uri))
    os.environ["CBF_ROVER_GAZEBO_BRIDGE_TIMING"] = "1"

    latencies_ms: list[float] = []
    with GazeboClassicSession(
        runtime=runtime,
        world_path=world_path,
        bridge_binary=bridge_path,
        log_dir=destination / "runtime_logs",
        gzserver_binary=args.gzserver_binary,
    ) as session:
        initial = session.initial_state
        assert initial is not None
        last = initial
        for _ in range(args.steps):
            started = perf_counter()
            last = session.step(
                linear_mps=args.linear_mps,
                angular_radps=args.angular_radps,
            )
            latencies_ms.append((perf_counter() - started) * 1000.0)

    report = {
        "schema_version": 1,
        "kind": "gazebo_bridge_step_latency_probe",
        "scope": (
            "bridge_and_physics_only_constant_command_no_controller_or_cbf"
        ),
        "inputs": {
            "config_path": str(config_path),
            "config_sha256": _file_sha256(config_path),
            "world_path": str(world_path),
            "world_sha256": _file_sha256(world_path),
            "bridge_path": str(bridge_path),
            "bridge_sha256": _file_sha256(bridge_path),
            "master_uri": runtime.master_uri,
            "steps": args.steps,
            "linear_mps": args.linear_mps,
            "angular_radps": args.angular_radps,
            "physics_step_s": runtime.physics_step_s,
            "physics_iterations_per_control_step": (
                runtime.control_iterations_per_step
            ),
        },
        "initial_state": asdict(initial),
        "terminal_state": asdict(last),
        "observed": {
            "wall_latency": _latency_summary(latencies_ms),
            "simulated_duration_s": last.sim_time_s - initial.sim_time_s,
            "physics_iterations": last.iterations - initial.iterations,
            "achieved_realtime_factor": (
                (last.sim_time_s - initial.sim_time_s)
                / (sum(latencies_ms) / 1000.0)
            ),
            "raw_step_latency_ms": latencies_ms,
        },
        "claim_boundary": (
            "diagnostic_only_not_a_controller_cbf_or_safety_result"
        ),
    }
    report_path = destination / "bridge_timing_probe.json"
    report_path.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(report_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
