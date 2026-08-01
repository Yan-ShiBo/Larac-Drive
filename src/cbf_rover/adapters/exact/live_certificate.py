"""Exact M1 simulation with identity-bound live q-SBC telemetry."""

from __future__ import annotations

import json
from math import sqrt
from pathlib import Path
from time import perf_counter, sleep
from typing import Any, Iterable, Mapping

from cbf_rover.adapters.training.sac_reference import file_sha256
from cbf_rover.domain.planar import (
    CANONICAL_SCENARIO,
    CANONICAL_SYSTEM,
    PlanarState,
)
from cbf_rover.methods.polynomial_sbc.certificate import QSBCertificate
from cbf_rover.methods.polynomial_sbc.polynomial import (
    FrozenController,
    content_sha256,
    load_frozen_controller,
)

from .environment import ExactPlanarEnvironment


def _write_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(
        json.dumps(
            payload,
            indent=2,
            sort_keys=True,
            ensure_ascii=False,
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _distance_to_box(state: PlanarState, box: Any) -> float:
    dx = max(box.x_min - state.x_m, 0.0, state.x_m - box.x_max)
    dy = max(box.y_min - state.y_m, 0.0, state.y_m - box.y_max)
    return sqrt(dx * dx + dy * dy)


def _state_facts(state: PlanarState) -> dict[str, Any]:
    scenario = CANONICAL_SCENARIO
    terminal_reason = scenario.terminal_reason(state)
    return {
        "x_m": state.x_m,
        "y_m": state.y_m,
        "frame_id": "world",
        "geometry_terminal_reason": terminal_reason,
        "in_goal": scenario.is_goal(state),
        "in_unsafe": scenario.is_unsafe(state),
        "in_domain": scenario.domain.contains(state),
        "goal_distance_m": sqrt(scenario.goal.squared_distance(state)),
        "minimum_unsafe_set_distance_m": min(
            _distance_to_box(state, obstacle)
            for obstacle in scenario.unsafe_components
        ),
    }


def _frame(
    *,
    step_index: int,
    state: PlanarState,
    certificate: QSBCertificate,
    action: Any | None,
    noise_m: list[float] | None,
    reward: float | None,
    termination_reason: str | None,
) -> dict[str, Any]:
    barrier_value = certificate.evaluate(state)
    return {
        "schema_version": 1,
        "kind": "exact_q_sbc_live_frame",
        "step_index": step_index,
        "simulated_time_s": step_index * CANONICAL_SYSTEM.dt_s,
        "state": _state_facts(state),
        "action": (
            None
            if action is None
            else {
                "u_x_mps": action.u_x_mps,
                "u_y_mps": action.u_y_mps,
                "speed_mps": sqrt(action.squared_speed_m2ps2),
            }
        ),
        "noise_m": noise_m,
        "reward": reward,
        "termination_reason": termination_reason,
        "certificate": {
            "certificate_id": certificate.certificate_id,
            "content_sha256": certificate.sha256,
            "theorem_id": certificate.theorem_id,
            "barrier_value_W": barrier_value,
            "q": certificate.q,
            "epsbar": certificate.epsbar,
            "unsafe_set_certificate_level": 1.0,
            "unsafe_set_certificate_level_gap": 1.0 - barrier_value,
            "q_is_not_an_instantaneous_probability": True,
            "level_gap_is_not_state_classification": True,
            "independent_validation_status": certificate.validation.get(
                "status"
            ),
            "first_slice_ready": bool(
                certificate.validation.get("first_slice_ready", False)
            ),
        },
        "runtime": {
            "backend": "M1_exact_stochastic_single_integrator",
            "mode": "monitor_only",
            "certificate_modified_action": False,
            "post_controller_clipping_applied": False,
        },
    }


def run_exact_certificate_episode(
    *,
    controller: FrozenController,
    certificate: QSBCertificate,
    seed: int,
    wall_clock: bool,
    emit_console: bool,
    max_steps: int = 2000,
) -> tuple[list[dict[str, Any]], float]:
    """Run one exact episode and emit each measured state at M1 cadence."""

    if certificate.controller_content_sha256 != controller.sha256:
        raise ValueError("certificate/controller identity mismatch")
    if certificate.validation.get("status") != "validated":
        raise ValueError("certificate must be independently validated")
    environment = ExactPlanarEnvironment(
        max_steps=max_steps,
        reset_mode="initial_disk",
    )
    state = environment.reset(seed=int(seed))
    rows = [
        _frame(
            step_index=0,
            state=state,
            certificate=certificate,
            action=None,
            noise_m=None,
            reward=None,
            termination_reason=None,
        )
    ]
    start = perf_counter()
    if emit_console:
        print(
            json.dumps(
                rows[-1],
                sort_keys=True,
                separators=(",", ":"),
                allow_nan=False,
            ),
            flush=True,
        )
    for _ in range(max_steps):
        action = controller.act(state)
        if not CANONICAL_SYSTEM.action_is_admissible(
            action,
            tolerance=1.0e-12,
        ):
            raise RuntimeError("frozen controller violated the S1 action disk")
        state, reward, terminated, truncated, info = environment.step(action)
        row = _frame(
            step_index=environment.step_count,
            state=state,
            certificate=certificate,
            action=action,
            noise_m=list(info.get("noise_m", [])),
            reward=reward,
            termination_reason=info.get("termination_reason"),
        )
        rows.append(row)
        if emit_console:
            print(
                json.dumps(
                    row,
                    sort_keys=True,
                    separators=(",", ":"),
                    allow_nan=False,
                ),
                flush=True,
            )
        if wall_clock:
            target = environment.step_count * CANONICAL_SYSTEM.dt_s
            remaining = target - (perf_counter() - start)
            if remaining > 0.0:
                sleep(remaining)
        if terminated or truncated:
            break
    return rows, perf_counter() - start


def _write_trace(path: Path, rows: Iterable[Mapping[str, Any]]) -> None:
    temporary = path.with_name(f".{path.name}.tmp")
    with temporary.open("w", encoding="utf-8", newline="\n") as stream:
        for row in rows:
            stream.write(
                json.dumps(
                    row,
                    sort_keys=True,
                    separators=(",", ":"),
                    ensure_ascii=False,
                    allow_nan=False,
                )
                + "\n"
            )
    temporary.replace(path)


def render_exact_certificate_animation(
    rows: list[Mapping[str, Any]],
    *,
    gif_path: Path,
    final_frame_path: Path,
    display_stride: int = 4,
) -> None:
    """Render a 1x-time GIF and a final-frame PNG from the exact trace."""

    if display_stride <= 0:
        raise ValueError("display_stride must be positive")
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import animation
    from matplotlib.patches import Circle, Rectangle

    scenario = CANONICAL_SCENARIO
    figure, (scene_axis, value_axis) = plt.subplots(
        1,
        2,
        figsize=(12.8, 6.4),
        gridspec_kw={"width_ratios": [1.45, 1.0]},
    )
    figure.patch.set_facecolor("#10141c")
    for axis in (scene_axis, value_axis):
        axis.set_facecolor("#151b26")
        axis.tick_params(colors="#d7deea")
        for spine in axis.spines.values():
            spine.set_color("#667085")

    scene_axis.set_xlim(scenario.domain.x_min, scenario.domain.x_max)
    scene_axis.set_ylim(scenario.domain.y_min, scenario.domain.y_max)
    scene_axis.set_aspect("equal", adjustable="box")
    scene_axis.set_xlabel("x [m]", color="#d7deea")
    scene_axis.set_ylabel("y [m]", color="#d7deea")
    scene_axis.set_title(
        "M1 exact stochastic trajectory",
        color="#f2f4f7",
    )
    for obstacle in scenario.unsafe_components:
        scene_axis.add_patch(
            Rectangle(
                (obstacle.x_min, obstacle.y_min),
                obstacle.x_max - obstacle.x_min,
                obstacle.y_max - obstacle.y_min,
                facecolor="#f04438",
                edgecolor="#ff8a80",
                alpha=0.65,
            )
        )
    scene_axis.add_patch(
        Circle(
            (scenario.initial.center_x, scenario.initial.center_y),
            scenario.initial.radius,
            facecolor="#2e90fa",
            edgecolor="#84caff",
            alpha=0.5,
        )
    )
    scene_axis.add_patch(
        Circle(
            (scenario.goal.center_x, scenario.goal.center_y),
            scenario.goal.radius,
            facecolor="#12b76a",
            edgecolor="#6ce9a6",
            alpha=0.55,
        )
    )
    path_line, = scene_axis.plot([], [], color="#fdb022", linewidth=2.0)
    rover_point, = scene_axis.plot(
        [],
        [],
        marker="o",
        markersize=9,
        color="#ffffff",
        markeredgecolor="#fdb022",
    )
    status_text = scene_axis.text(
        0.02,
        0.98,
        "",
        transform=scene_axis.transAxes,
        va="top",
        ha="left",
        color="#ffffff",
        fontsize=9,
        bbox={
            "facecolor": "#101828",
            "edgecolor": "#475467",
            "alpha": 0.9,
            "boxstyle": "round,pad=0.45",
        },
    )

    times = [float(row["simulated_time_s"]) for row in rows]
    barrier_values = [
        float(row["certificate"]["barrier_value_W"]) for row in rows
    ]
    q_value = float(rows[0]["certificate"]["q"])
    lower = min(min(barrier_values), q_value, 0.0)
    upper = max(max(barrier_values), 1.0)
    padding = max(0.05, 0.08 * (upper - lower))
    value_axis.set_xlim(0.0, max(times[-1], CANONICAL_SYSTEM.dt_s))
    value_axis.set_ylim(lower - padding, upper + padding)
    value_axis.set_xlabel("simulated time [s]", color="#d7deea")
    value_axis.set_ylabel("certificate value", color="#d7deea")
    value_axis.set_title("Live obstacle-certificate telemetry", color="#f2f4f7")
    value_axis.axhline(
        q_value,
        color="#2e90fa",
        linestyle="--",
        linewidth=1.4,
        label=f"q={q_value:.6f}",
    )
    value_axis.axhline(
        1.0,
        color="#f04438",
        linestyle="--",
        linewidth=1.4,
        label="unsafe-set certificate level = 1",
    )
    value_line, = value_axis.plot([], [], color="#12b76a", linewidth=2.0)
    value_point, = value_axis.plot([], [], "o", color="#ffffff", markersize=5)
    legend = value_axis.legend(loc="upper right", framealpha=0.85)
    for text in legend.get_texts():
        text.set_color("#101828")

    x_values = [float(row["state"]["x_m"]) for row in rows]
    y_values = [float(row["state"]["y_m"]) for row in rows]
    indices = list(range(0, len(rows), display_stride))
    if indices[-1] != len(rows) - 1:
        indices.append(len(rows) - 1)

    def update(frame_index: int) -> tuple[Any, ...]:
        row = rows[frame_index]
        path_line.set_data(x_values[: frame_index + 1], y_values[: frame_index + 1])
        rover_point.set_data([x_values[frame_index]], [y_values[frame_index]])
        value_line.set_data(
            times[: frame_index + 1],
            barrier_values[: frame_index + 1],
        )
        value_point.set_data(
            [times[frame_index]],
            [barrier_values[frame_index]],
        )
        readiness = (
            "READY"
            if row["certificate"]["first_slice_ready"]
            else "FORMAL VALID / NOT_READY"
        )
        terminal = row["termination_reason"] or "running"
        status_text.set_text(
            f"t={row['simulated_time_s']:.2f} s   step={row['step_index']}\n"
            f"x=({x_values[frame_index]:.3f}, {y_values[frame_index]:.3f}) m\n"
            f"W(x)={barrier_values[frame_index]:.6f}   q={q_value:.6f}\n"
            f"{readiness}\nMONITOR_ONLY | {terminal}\n"
            f"cert={row['certificate']['content_sha256'][:12]}"
        )
        return path_line, rover_point, value_line, value_point, status_text

    movie = animation.FuncAnimation(
        figure,
        update,
        frames=indices,
        interval=1000.0 * CANONICAL_SYSTEM.dt_s * display_stride,
        blit=False,
    )
    fps = max(
        1,
        round(1.0 / (CANONICAL_SYSTEM.dt_s * display_stride)),
    )
    movie.save(
        gif_path,
        writer=animation.PillowWriter(fps=fps),
        dpi=85,
    )
    update(len(rows) - 1)
    figure.savefig(final_frame_path, dpi=140, bbox_inches="tight")
    plt.close(figure)


def simulate_exact_certificate_live(
    *,
    controller_path: str | Path,
    certificate_path: str | Path,
    output_dir: str | Path,
    seed: int,
    wall_clock: bool = True,
    emit_console: bool = True,
    render_gif: bool = True,
    display_stride: int = 4,
) -> Path:
    """Execute, record, render, and hash one exact live certificate run."""

    controller_file = Path(controller_path).resolve()
    certificate_file = Path(certificate_path).resolve()
    destination = Path(output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    controller = load_frozen_controller(controller_file)
    certificate = QSBCertificate.load(
        certificate_file,
        controller=controller,
    )
    rows, wall_elapsed_s = run_exact_certificate_episode(
        controller=controller,
        certificate=certificate,
        seed=seed,
        wall_clock=wall_clock,
        emit_console=emit_console,
    )
    trace_path = destination / "live_trace.jsonl"
    _write_trace(trace_path, rows)
    animation_path = destination / "certificate_live_1x.gif"
    final_frame_path = destination / "certificate_final_frame.png"
    if render_gif:
        render_exact_certificate_animation(
            rows,
            gif_path=animation_path,
            final_frame_path=final_frame_path,
            display_stride=display_stride,
        )

    final = rows[-1]
    termination_reason = str(final["termination_reason"])
    status = (
        "goal_reached"
        if termination_reason == "goal_reached"
        else "failed"
    )
    manifest: dict[str, Any] = {
        "schema_version": 1,
        "kind": "exact_q_sbc_live_run",
        "status": status,
        "claim_boundary": {
            "certificate": "formal_valid",
            "certificate_first_slice_ready": bool(
                certificate.validation.get("first_slice_ready", False)
            ),
            "trajectory": "one_exact_stochastic_sample",
            "gazebo_tracking_claim": "not_applicable",
        },
        "seed": int(seed),
        "step_count": len(rows) - 1,
        "simulated_duration_s": float(final["simulated_time_s"]),
        "wall_elapsed_s": wall_elapsed_s,
        "wall_clock_paced": bool(wall_clock),
        "monitor_only": True,
        "certificate_modified_action": False,
        "controller": {
            "path": str(controller_file),
            "file_sha256": file_sha256(controller_file),
            "content_sha256": controller.sha256,
        },
        "certificate": {
            "path": str(certificate_file),
            "file_sha256": file_sha256(certificate_file),
            "content_sha256": certificate.sha256,
            "certificate_id": certificate.certificate_id,
            "q": certificate.q,
            "epsbar": certificate.epsbar,
            "independent_validation_status": certificate.validation.get(
                "status"
            ),
            "first_slice_ready": bool(
                certificate.validation.get("first_slice_ready", False)
            ),
        },
        "telemetry": {
            "trace_file": trace_path.name,
            "trace_file_sha256": file_sha256(trace_path),
            "frame_count": len(rows),
            "minimum_barrier_value": min(
                float(row["certificate"]["barrier_value_W"])
                for row in rows
            ),
            "maximum_barrier_value": max(
                float(row["certificate"]["barrier_value_W"])
                for row in rows
            ),
            "final_barrier_value": float(
                final["certificate"]["barrier_value_W"]
            ),
            "minimum_unsafe_set_distance_m": min(
                float(row["state"]["minimum_unsafe_set_distance_m"])
                for row in rows
            ),
            "q_is_not_an_instantaneous_probability": True,
        },
        "render": (
            {
                "playback_rate": "1x_simulated_time",
                "display_stride": display_stride,
                "gif_file": animation_path.name,
                "gif_file_sha256": file_sha256(animation_path),
                "final_frame_file": final_frame_path.name,
                "final_frame_file_sha256": file_sha256(final_frame_path),
            }
            if render_gif
            else None
        ),
    }
    manifest["manifest_content_sha256"] = content_sha256(manifest)
    manifest_path = destination / "run_manifest.json"
    _write_json(manifest_path, manifest)
    return manifest_path
