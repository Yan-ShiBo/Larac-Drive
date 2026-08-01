"""Rendering for lifted-CBF differential-drive road traces."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

from cbf_rover.domain.dynamic_road import DynamicRoadScenario


def render_differential_road_animation(
    rows: list[Mapping[str, Any]],
    *,
    scenario: DynamicRoadScenario,
    gif_path: Path,
    final_frame_path: Path,
    dt_s: float,
    display_stride: int,
    title_backend: str = "DIFFERENTIAL DRIVE (EULER)",
    title_claim: str = "NOT q-SBC / NOT GAZEBO",
    footer_backend: str = "DIFFERENTIAL EULER / NOT GAZEBO",
) -> None:
    """Render a 1x GIF and final PNG for a differential-drive trace."""

    if not rows:
        raise ValueError("differential-road trace must not be empty")
    if dt_s <= 0.0 or display_stride <= 0:
        raise ValueError("dt and display stride must be positive")
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import animation
    from matplotlib.patches import Circle, Rectangle

    geometry = scenario.geometry
    lookahead_distance = float(rows[0]["state"]["lookahead_distance_m"])
    inflation = float(
        rows[0]["certificate"]["additional_body_inflation_m"]
    )
    figure = plt.figure(figsize=(15.2, 7.8), facecolor="#0b1118")
    grid = figure.add_gridspec(
        2,
        2,
        width_ratios=(1.65, 1.0),
        height_ratios=(1.0, 1.0),
        wspace=0.20,
        hspace=0.30,
    )
    scene_axis = figure.add_subplot(grid[:, 0])
    barrier_axis = figure.add_subplot(grid[0, 1])
    command_axis = figure.add_subplot(grid[1, 1])
    for axis in (scene_axis, barrier_axis, command_axis):
        axis.set_facecolor("#111b26")
        axis.tick_params(colors="#d0d5dd")
        for spine in axis.spines.values():
            spine.set_color("#475467")

    scene_axis.set_xlim(geometry.x_min_m, geometry.x_max_m)
    y_padding = 0.48
    scene_axis.set_ylim(
        geometry.physical_y_min_m - y_padding,
        geometry.physical_y_max_m + y_padding,
    )
    scene_axis.set_aspect("equal", adjustable="box")
    scene_axis.set_xlabel("road x [m]", color="#d0d5dd")
    scene_axis.set_ylabel("road y [m]", color="#d0d5dd")
    scene_axis.set_title(
        "Differential-drive body + lifted certificate point",
        color="#f2f4f7",
        fontsize=13,
    )
    scene_axis.add_patch(
        Rectangle(
            (
                geometry.x_min_m,
                geometry.physical_y_min_m,
            ),
            geometry.x_max_m - geometry.x_min_m,
            geometry.physical_y_max_m - geometry.physical_y_min_m,
            facecolor="#344054",
            edgecolor="#98a2b3",
            linewidth=1.5,
            zorder=0,
        )
    )
    scene_axis.axhline(
        0.0,
        color="#f2f4f7",
        linewidth=1.2,
        linestyle=(0, (7, 7)),
        alpha=0.75,
        zorder=1,
    )
    lifted_y_min = geometry.center_y_min_m + inflation
    lifted_y_max = geometry.center_y_max_m - inflation
    for y_value in (lifted_y_min, lifted_y_max):
        scene_axis.axhline(
            y_value,
            color="#7f56d9",
            linewidth=1.0,
            linestyle=":",
            alpha=0.95,
        )
    scene_axis.axvline(
        geometry.finish_line_x_m,
        color="#32d583",
        linewidth=2.5,
        label="finish line",
    )
    for lane_y in geometry.lane_centers_y_m:
        scene_axis.axhline(
            lane_y,
            color="#fdb022",
            linewidth=0.8,
            alpha=0.25,
        )

    obstacle_patches: dict[str, tuple[Any, Any, Any]] = {}
    for obstacle in scenario.obstacles:
        physical = Circle(
            obstacle.center.as_tuple(),
            obstacle.physical_radius_m,
            facecolor="#f04438",
            edgecolor="#fda29b",
            linewidth=1.4,
            alpha=0.92,
            visible=False,
            zorder=5,
        )
        certificate = Circle(
            obstacle.center.as_tuple(),
            obstacle.certificate_radius_m + inflation,
            facecolor="none",
            edgecolor="#fdb022",
            linewidth=1.8,
            linestyle="--",
            alpha=0.95,
            visible=False,
            zorder=4,
        )
        label = scene_axis.text(
            obstacle.center.x_m,
            (
                obstacle.center.y_m
                + obstacle.certificate_radius_m
                + inflation
                + 0.08
            ),
            obstacle.obstacle_id.replace("obstacle_", "O"),
            color="#ffffff",
            fontsize=8,
            ha="center",
            va="bottom",
            visible=False,
            zorder=6,
        )
        scene_axis.add_patch(certificate)
        scene_axis.add_patch(physical)
        obstacle_patches[obstacle.obstacle_id] = (
            physical,
            certificate,
            label,
        )

    center_path, = scene_axis.plot(
        [],
        [],
        color="#2e90fa",
        linewidth=2.2,
        zorder=3,
        label="body-center path",
    )
    point_path, = scene_axis.plot(
        [],
        [],
        color="#b692f6",
        linewidth=1.3,
        linestyle="--",
        zorder=3,
        label="lookahead-point path",
    )
    rover_patch = Circle(
        geometry.start.as_tuple(),
        geometry.rover_radius_m,
        facecolor="#ffffff",
        edgecolor="#2e90fa",
        linewidth=2.0,
        zorder=7,
    )
    scene_axis.add_patch(rover_patch)
    heading_line, = scene_axis.plot(
        [],
        [],
        color="#2e90fa",
        linewidth=2.4,
        zorder=8,
    )
    lookahead_marker, = scene_axis.plot(
        [],
        [],
        marker="o",
        markersize=6,
        markerfacecolor="#b692f6",
        markeredgecolor="#ffffff",
        zorder=9,
        label="CBF evaluation point",
    )
    target_lane_marker, = scene_axis.plot(
        [],
        [],
        marker="x",
        markersize=8,
        markeredgewidth=2,
        color="#fdb022",
        zorder=8,
    )
    status_text = scene_axis.text(
        0.015,
        0.985,
        "",
        transform=scene_axis.transAxes,
        va="top",
        ha="left",
        color="#ffffff",
        fontsize=8.7,
        bbox={
            "facecolor": "#101828",
            "edgecolor": "#667085",
            "alpha": 0.94,
            "boxstyle": "round,pad=0.50",
        },
        zorder=10,
    )
    scene_axis.legend(
        loc="lower left",
        framealpha=0.88,
        facecolor="#f2f4f7",
    )

    times = [float(row["simulated_time_s"]) for row in rows]
    x_values = [float(row["state"]["x_m"]) for row in rows]
    y_values = [float(row["state"]["y_m"]) for row in rows]
    headings = [float(row["state"]["heading_rad"]) for row in rows]
    point_x = [
        float(row["state"]["lookahead_point_m"][0]) for row in rows
    ]
    point_y = [
        float(row["state"]["lookahead_point_m"][1]) for row in rows
    ]
    barrier_values = [
        float(row["certificate"]["minimum_barrier_m"])
        for row in rows
    ]

    def optional_series(
        *keys: str,
        absolute: bool = False,
    ) -> list[float]:
        result: list[float] = []
        for row in rows:
            value: Any = row
            for key in keys:
                if value is None:
                    break
                value = value.get(key)
            numeric = float("nan") if value is None else float(value)
            result.append(abs(numeric) if absolute else numeric)
        return result

    contraction_values = optional_series(
        "certificate",
        "minimum_realized_contraction_residual_m",
    )
    linear_values = optional_series(
        "action",
        "differential_command",
        "linear_mps",
    )
    angular_values = optional_series(
        "action",
        "differential_command",
        "angular_radps",
        absolute=True,
    )
    intervention_values = optional_series(
        "action",
        "intervention_norm_mps",
    )

    time_end = max(times[-1], dt_s)
    barrier_low = min(0.0, min(barrier_values))
    barrier_high = max(max(barrier_values), 0.10)
    barrier_pad = max(0.04, 0.08 * (barrier_high - barrier_low))
    barrier_axis.set_xlim(0.0, time_end)
    barrier_axis.set_ylim(
        barrier_low - barrier_pad,
        barrier_high + barrier_pad,
    )
    barrier_axis.axhline(
        0.0,
        color="#f04438",
        linestyle="--",
        linewidth=1.4,
        label="lifted boundary h = 0",
    )
    barrier_line, = barrier_axis.plot(
        [],
        [],
        color="#32d583",
        linewidth=2.0,
        label="minimum lifted h",
    )
    barrier_point, = barrier_axis.plot(
        [],
        [],
        marker="o",
        color="#ffffff",
        markersize=5,
    )
    barrier_axis.set_title(
        "Lifted online obstacle-certificate margin",
        color="#f2f4f7",
    )
    barrier_axis.set_xlabel("simulated time [s]", color="#d0d5dd")
    barrier_axis.set_ylabel("minimum h [m]", color="#d0d5dd")
    barrier_axis.legend(loc="upper right", framealpha=0.88)

    finite_commands = [
        value
        for series in (
            linear_values,
            angular_values,
            intervention_values,
        )
        for value in series
        if value == value
    ]
    command_high = max(finite_commands, default=0.1)
    command_axis.set_xlim(0.0, time_end)
    command_axis.set_ylim(-0.05, max(0.2, 1.08 * command_high))
    linear_line, = command_axis.plot(
        [],
        [],
        color="#36bffa",
        linewidth=1.8,
        label="linear v [m/s]",
    )
    angular_line, = command_axis.plot(
        [],
        [],
        color="#b692f6",
        linewidth=1.8,
        label="|angular omega| [rad/s]",
    )
    intervention_line, = command_axis.plot(
        [],
        [],
        color="#fdb022",
        linewidth=1.8,
        label="CBF intervention [m/s]",
    )
    command_axis.set_title(
        "Reachable differential command telemetry",
        color="#f2f4f7",
    )
    command_axis.set_xlabel("simulated time [s]", color="#d0d5dd")
    command_axis.set_ylabel("command / intervention", color="#d0d5dd")
    command_axis.legend(loc="upper right", framealpha=0.88)

    figure.suptitle(
        f"LIFTED ONLINE CBF • {title_backend} • {title_claim}",
        color="#ffffff",
        fontsize=14,
        fontweight="bold",
        y=0.985,
    )
    indices = list(range(0, len(rows), display_stride))
    if indices[-1] != len(rows) - 1:
        indices.append(len(rows) - 1)

    def update(frame_index: int) -> tuple[Any, ...]:
        row = rows[frame_index]
        center_path.set_data(
            x_values[: frame_index + 1],
            y_values[: frame_index + 1],
        )
        point_path.set_data(
            point_x[: frame_index + 1],
            point_y[: frame_index + 1],
        )
        rover_patch.center = (
            x_values[frame_index],
            y_values[frame_index],
        )
        heading_line.set_data(
            [x_values[frame_index], point_x[frame_index]],
            [y_values[frame_index], point_y[frame_index]],
        )
        lookahead_marker.set_data(
            [point_x[frame_index]],
            [point_y[frame_index]],
        )
        planner = row["planner"] or {}
        target_y = float(
            planner.get("target_lane_y_m", y_values[frame_index])
        )
        target_lane_marker.set_data(
            [x_values[frame_index] + 0.45],
            [target_y],
        )
        active_ids = set(row["obstacles"]["active_ids"])
        barrier_by_id = {
            constraint["barrier_id"].split(":", 1)[1]: float(
                constraint["barrier_m"]
            )
            for constraint in row["certificate"]["constraints"]
            if constraint["barrier_kind"] == "obstacle"
        }
        for obstacle_id, patches in obstacle_patches.items():
            visible = obstacle_id in active_ids
            physical, certificate, label = patches
            physical.set_visible(visible)
            certificate.set_visible(visible)
            label.set_visible(visible)
            if visible:
                barrier = barrier_by_id[obstacle_id]
                certificate.set_edgecolor(
                    "#f04438" if barrier < 0.10 else "#fdb022"
                )

        barrier_line.set_data(
            times[: frame_index + 1],
            barrier_values[: frame_index + 1],
        )
        barrier_point.set_data(
            [times[frame_index]],
            [barrier_values[frame_index]],
        )
        linear_line.set_data(
            times[: frame_index + 1],
            linear_values[: frame_index + 1],
        )
        angular_line.set_data(
            times[: frame_index + 1],
            angular_values[: frame_index + 1],
        )
        intervention_line.set_data(
            times[: frame_index + 1],
            intervention_values[: frame_index + 1],
        )
        action = row["action"]
        if action is None:
            command_text = "v=n/a, omega=n/a"
            intervention = 0.0
            mapping_residual = 0.0
        else:
            command = action["differential_command"]
            command_text = (
                f"v={float(command['linear_mps']):.3f} m/s, "
                f"omega={float(command['angular_radps']):.3f} rad/s"
            )
            intervention = float(action["intervention_norm_mps"])
            mapping_residual = float(action["mapping_residual_mps"])
        contraction = contraction_values[frame_index]
        contraction_text = (
            "n/a"
            if contraction != contraction
            else f"{contraction:.5f} m"
        )
        newly_active = row["obstacles"]["newly_activated_ids"]
        activation_text = (
            f"NEW: {', '.join(newly_active)}"
            if newly_active
            else "new obstacle: none"
        )
        termination = row["termination_reason"] or "running"
        status_text.set_text(
            f"t={times[frame_index]:.2f} s  step={row['step_index']}\n"
            f"center=({x_values[frame_index]:.3f}, "
            f"{y_values[frame_index]:.3f}) m\n"
            f"heading={headings[frame_index]:.3f} rad\n"
            f"lookahead ell={lookahead_distance:.2f} m\n"
            f"active obstacles={len(active_ids)}/{len(scenario.obstacles)}\n"
            f"min lifted h={barrier_values[frame_index]:.4f} m\n"
            f"realized contraction={contraction_text}\n"
            f"{command_text}\n"
            f"filter intervention={intervention:.4f} m/s\n"
            f"mapping residual={mapping_residual:.2e} m/s\n"
            f"{activation_text}\n"
            f"{footer_backend} | {termination}"
        )
        return (
            center_path,
            point_path,
            rover_patch,
            heading_line,
            lookahead_marker,
            target_lane_marker,
            barrier_line,
            barrier_point,
            linear_line,
            angular_line,
            intervention_line,
            status_text,
        )

    movie = animation.FuncAnimation(
        figure,
        update,
        frames=indices,
        interval=1000.0 * dt_s * display_stride,
        blit=False,
    )
    fps = max(1, round(1.0 / (dt_s * display_stride)))
    movie.save(
        gif_path,
        writer=animation.PillowWriter(fps=fps),
        dpi=85,
    )
    update(len(rows) - 1)
    figure.savefig(final_frame_path, dpi=145, bbox_inches="tight")
    plt.close(figure)
