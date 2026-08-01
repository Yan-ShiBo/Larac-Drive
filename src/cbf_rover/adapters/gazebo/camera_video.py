"""Render timestamp-aligned Gazebo camera recordings as verified MP4 evidence."""

from __future__ import annotations

from bisect import bisect_left
from fractions import Fraction
from hashlib import sha256
import json
from math import cos, isfinite, pi, sin, sqrt, tan
from pathlib import Path
import re
import subprocess
from typing import Any, Mapping, Sequence

from PIL import Image, ImageDraw, ImageFont


_CAMERA_BOX = (32, 80, 1440, 872)
_TOP_DOWN_BOX = (1472, 80, 1888, 496)
_AMBER = (255, 176, 0)
_DANGER = (255, 92, 77)
_DANGER_THRESHOLD_M = 0.10


def _file_sha256(path: Path) -> str:
    digest = sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _canonical_sha256(payload: Mapping[str, Any]) -> str:
    encoded = json.dumps(
        payload,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")
    return sha256(encoded).hexdigest()


def _atomic_write_json(path: Path, payload: Mapping[str, Any]) -> None:
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


def _write_text(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def _load_object(path: Path, *, label: str) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"{label} must be a JSON object")
    return payload


def _mapping(value: Any, *, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must be a JSON object")
    return value


def _finite_float(value: Any, *, label: str) -> float:
    number = float(value)
    if not isfinite(number):
        raise ValueError(f"{label} must be finite")
    return number


def _positive_float(value: Any, *, label: str) -> float:
    number = _finite_float(value, label=label)
    if number <= 0.0:
        raise ValueError(f"{label} must be positive")
    return number


def _positive_int(value: Any, *, label: str) -> int:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be a positive integer")
    number = int(value)
    if number <= 0 or float(value) != number:
        raise ValueError(f"{label} must be a positive integer")
    return number


def _inside_root(root: Path, candidate: Path, *, label: str) -> Path:
    resolved = candidate.resolve()
    try:
        resolved.relative_to(root)
    except ValueError as error:
        raise ValueError(f"{label} must stay inside {root}") from error
    return resolved


def _source_path(root: Path, value: Any, *, label: str) -> Path:
    relative = Path(str(value))
    candidate = relative if relative.is_absolute() else root / relative
    return _inside_root(root, candidate, label=label)


def _output_path(root: Path, value: Any, *, label: str) -> Path:
    relative = Path(str(value))
    if relative.is_absolute():
        raise ValueError(f"{label} must be relative to the output directory")
    return _inside_root(root, root / relative, label=label)


def _load_index(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            if not line.strip():
                continue
            payload = json.loads(line)
            if not isinstance(payload, dict):
                raise ValueError(f"camera index line {line_number} is not an object")
            rows.append(payload)
    if not rows:
        raise ValueError("camera recording index is empty")
    return rows


def _nearest_source_index(
    capture_times_ns: Sequence[int],
    target_time_ns: int,
) -> int:
    """Return a nearest timestamp using exact integer-nanosecond distances."""

    position = bisect_left(capture_times_ns, target_time_ns)
    if position <= 0:
        return 0
    if position >= len(capture_times_ns):
        chosen_time_ns = capture_times_ns[-1]
    else:
        before_time_ns = capture_times_ns[position - 1]
        after_time_ns = capture_times_ns[position]
        chosen_time_ns = (
            before_time_ns
            if target_time_ns - before_time_ns
            <= after_time_ns - target_time_ns
            else after_time_ns
        )
    return bisect_left(capture_times_ns, chosen_time_ns)


def _font(size: int, *, bold: bool = False) -> ImageFont.ImageFont:
    filename = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    try:
        return ImageFont.truetype(filename, size)
    except OSError:
        return ImageFont.load_default()


def _draw_dashed_segment(
    draw: ImageDraw.ImageDraw,
    start: tuple[float, float],
    end: tuple[float, float],
    *,
    fill: tuple[int, int, int],
    width: int,
    dash_px: float = 18.0,
    gap_px: float = 11.0,
) -> None:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = sqrt(dx * dx + dy * dy)
    if length <= 1.0e-9:
        return
    ux = dx / length
    uy = dy / length
    offset = 0.0
    while offset < length:
        finish = min(offset + dash_px, length)
        draw.line(
            (
                start[0] + ux * offset,
                start[1] + uy * offset,
                start[0] + ux * finish,
                start[1] + uy * finish,
            ),
            fill=fill,
            width=width,
        )
        offset += dash_px + gap_px


def _draw_dashed_polyline(
    draw: ImageDraw.ImageDraw,
    points: Sequence[tuple[float, float]],
    *,
    fill: tuple[int, int, int],
    width: int,
    bounds: tuple[float, float, float, float] | None = None,
) -> bool:
    visible = False
    for start, end in zip(points, points[1:]):
        if bounds is None or not (
            max(start[0], end[0]) < bounds[0]
            or min(start[0], end[0]) > bounds[2]
            or max(start[1], end[1]) < bounds[1]
            or min(start[1], end[1]) > bounds[3]
        ):
            visible = True
        _draw_dashed_segment(draw, start, end, fill=fill, width=width)
    return visible


def _project_camera_point(
    *,
    camera: Mapping[str, Any],
    state: Mapping[str, Any],
    world_x_m: float,
    world_y_m: float,
    world_z_m: float,
) -> tuple[float, float] | None:
    heading = _finite_float(state["heading_rad"], label="state.heading_rad")
    dx = world_x_m - _finite_float(state["x_m"], label="state.x_m")
    dy = world_y_m - _finite_float(state["y_m"], label="state.y_m")
    heading_cos = cos(heading)
    heading_sin = sin(heading)
    base_forward = (
        heading_cos * dx
        + heading_sin * dy
        - _finite_float(camera["mount_x_m"], label="camera.mount_x_m")
    )
    left = (
        -heading_sin * dx
        + heading_cos * dy
        - _finite_float(camera["mount_y_m"], label="camera.mount_y_m")
    )
    base_up = world_z_m - _finite_float(
        camera["world_height_m"], label="camera.world_height_m"
    )
    pitch = _finite_float(camera["pitch_rad"], label="camera.pitch_rad")
    pitch_cos = cos(pitch)
    pitch_sin = sin(pitch)
    forward = pitch_cos * base_forward - pitch_sin * base_up
    near_clip = _positive_float(camera["near_clip_m"], label="camera.near_clip_m")
    if forward <= near_clip + 0.01:
        return None
    up = pitch_sin * base_forward + pitch_cos * base_up
    width = _positive_int(camera["image_width"], label="camera.image_width")
    height = _positive_int(camera["image_height"], label="camera.image_height")
    horizontal_fov = _positive_float(
        camera["horizontal_fov_rad"], label="camera.horizontal_fov_rad"
    )
    if horizontal_fov >= pi:
        raise ValueError("camera.horizontal_fov_rad must be smaller than pi")
    focal = width / (2.0 * tan(horizontal_fov / 2.0))
    return (
        width / 2.0 - focal * left / forward,
        height / 2.0 - focal * up / forward,
    )


def _draw_camera_certificates(
    image: Image.Image,
    *,
    camera: Mapping[str, Any],
    row: Mapping[str, Any],
    obstacles: Mapping[str, Mapping[str, Any]],
) -> set[str]:
    draw = ImageDraw.Draw(image)
    alignment = _mapping(row["alignment"], label="index.alignment")
    state = _mapping(alignment["state"], label="index.alignment.state")
    active_ids_value = alignment["active_obstacle_ids"]
    if not isinstance(active_ids_value, list) or not all(
        isinstance(item, str) for item in active_ids_value
    ):
        raise ValueError("alignment.active_obstacle_ids must be a string list")
    barriers = _mapping(
        alignment["obstacle_barrier_m"],
        label="index.alignment.obstacle_barrier_m",
    )
    inflation = _finite_float(
        alignment["additional_body_inflation_m"],
        label="alignment.additional_body_inflation_m",
    )
    if inflation < 0.0:
        raise ValueError("alignment.additional_body_inflation_m must be nonnegative")
    source_width = _positive_int(camera["image_width"], label="camera.image_width")
    source_height = _positive_int(camera["image_height"], label="camera.image_height")
    scale_x = image.width / source_width
    scale_y = image.height / source_height
    visible_ids: set[str] = set()
    for obstacle_id in active_ids_value:
        if obstacle_id not in obstacles:
            raise ValueError(f"active obstacle is absent from scenario: {obstacle_id}")
        obstacle = obstacles[obstacle_id]
        radius = _positive_float(
            obstacle["certificate_radius_m"],
            label=f"scenario.obstacles[{obstacle_id}].certificate_radius_m",
        ) + inflation
        projected: list[tuple[float, float]] = []
        segments: list[list[tuple[float, float]]] = []
        for sample in range(97):
            angle = 2.0 * pi * sample / 96.0
            point = _project_camera_point(
                camera=camera,
                state=state,
                world_x_m=_finite_float(obstacle["x_m"], label="obstacle.x_m")
                + radius * cos(angle),
                world_y_m=_finite_float(obstacle["y_m"], label="obstacle.y_m")
                + radius * sin(angle),
                world_z_m=0.018,
            )
            if point is None:
                if len(projected) >= 2:
                    segments.append(projected)
                projected = []
                continue
            projected.append((point[0] * scale_x, point[1] * scale_y))
        if len(projected) >= 2:
            segments.append(projected)
        barrier_value = barriers.get(obstacle_id)
        barrier = None if barrier_value is None else _finite_float(
            barrier_value,
            label=f"alignment.obstacle_barrier_m.{obstacle_id}",
        )
        danger = barrier is not None and barrier < _DANGER_THRESHOLD_M
        color = _DANGER if danger else _AMBER
        width = 8 if danger else 6
        visible = False
        for segment in segments:
            segment_visible = _draw_dashed_polyline(
                draw,
                segment,
                fill=color,
                width=width,
                bounds=(0.0, 0.0, float(image.width - 1), float(image.height - 1)),
            )
            visible = segment_visible or visible
        if visible:
            visible_ids.add(obstacle_id)
            label_point = _project_camera_point(
                camera=camera,
                state=state,
                world_x_m=_finite_float(
                    obstacle["x_m"], label="obstacle.x_m"
                ),
                world_y_m=_finite_float(
                    obstacle["y_m"], label="obstacle.y_m"
                ),
                world_z_m=0.58,
            )
            if label_point is not None:
                label = obstacle_id.replace("obstacle_", "O")
                label += "  h=" + (
                    "n/a" if barrier is None else f"{barrier:.3f}m"
                )
                font = _font(19, bold=True)
                text_box = draw.textbbox((0, 0), label, font=font)
                text_width = text_box[2] - text_box[0]
                text_height = text_box[3] - text_box[1]
                label_x = label_point[0] * scale_x - text_width / 2.0
                label_y = label_point[1] * scale_y - text_height - 12.0
                label_x = max(
                    8.0,
                    min(label_x, image.width - text_width - 20.0),
                )
                label_y = max(
                    8.0,
                    min(label_y, image.height - text_height - 20.0),
                )
                background = (
                    label_x - 7.0,
                    label_y - 5.0,
                    label_x + text_width + 7.0,
                    label_y + text_height + 7.0,
                )
                draw.rounded_rectangle(
                    background,
                    radius=5,
                    fill=(7, 10, 12),
                    outline=color,
                    width=2,
                )
                draw.text(
                    (label_x, label_y),
                    label,
                    fill=color,
                    font=font,
                )
    return visible_ids


def _world_transform(
    road: Mapping[str, Any],
    *,
    box: tuple[int, int, int, int],
) -> tuple[float, float, float, float, float]:
    x_min = _finite_float(road["x_min_m"], label="scenario.road.x_min_m")
    x_max = _finite_float(road["x_max_m"], label="scenario.road.x_max_m")
    y_min = _finite_float(
        road["physical_y_min_m"], label="scenario.road.physical_y_min_m"
    )
    y_max = _finite_float(
        road["physical_y_max_m"], label="scenario.road.physical_y_max_m"
    )
    if x_max <= x_min or y_max <= y_min:
        raise ValueError("scenario road bounds must be ordered")
    margin = 24.0
    available_width = box[2] - box[0] - 2.0 * margin
    available_height = box[3] - box[1] - 2.0 * margin
    scale = min(available_width / (x_max - x_min), available_height / (y_max - y_min))
    origin_x = box[0] + (box[2] - box[0] - scale * (x_max - x_min)) / 2.0
    origin_y = box[1] + (box[3] - box[1] + scale * (y_max - y_min)) / 2.0
    return x_min, y_min, origin_x, origin_y, scale


def _draw_top_down(
    dashboard: Image.Image,
    *,
    road: Mapping[str, Any],
    row: Mapping[str, Any],
    obstacles: Mapping[str, Mapping[str, Any]],
) -> set[str]:
    draw = ImageDraw.Draw(dashboard)
    box = _TOP_DOWN_BOX
    draw.rounded_rectangle(box, radius=14, fill=(18, 25, 29), outline=(60, 74, 79), width=2)
    x_min, y_min, origin_x, origin_y, scale = _world_transform(road, box=box)

    def point(x_m: float, y_m: float) -> tuple[float, float]:
        return origin_x + (x_m - x_min) * scale, origin_y - (y_m - y_min) * scale

    road_top_left = point(
        _finite_float(road["x_min_m"], label="road.x_min_m"),
        _finite_float(road["physical_y_max_m"], label="road.physical_y_max_m"),
    )
    road_bottom_right = point(
        _finite_float(road["x_max_m"], label="road.x_max_m"),
        _finite_float(road["physical_y_min_m"], label="road.physical_y_min_m"),
    )
    draw.rectangle(
        (road_top_left[0], road_top_left[1], road_bottom_right[0], road_bottom_right[1]),
        fill=(48, 55, 58),
        outline=(214, 220, 218),
        width=2,
    )
    lanes = road.get("lane_centers_y_m", [])
    if isinstance(lanes, Sequence) and not isinstance(lanes, (str, bytes)):
        for lane in lanes:
            start = point(_finite_float(road["x_min_m"], label="road.x_min_m"), float(lane))
            end = point(_finite_float(road["x_max_m"], label="road.x_max_m"), float(lane))
            _draw_dashed_segment(
                draw,
                start,
                end,
                fill=(135, 145, 145),
                width=1,
                dash_px=8.0,
                gap_px=7.0,
            )
    if "finish_line_x_m" in road:
        finish_x = _finite_float(road["finish_line_x_m"], label="road.finish_line_x_m")
        finish_start = point(
            finish_x,
            _finite_float(road["physical_y_min_m"], label="road.physical_y_min_m"),
        )
        finish_end = point(
            finish_x,
            _finite_float(road["physical_y_max_m"], label="road.physical_y_max_m"),
        )
        draw.line((*finish_start, *finish_end), fill=(224, 232, 230), width=2)

    alignment = _mapping(row["alignment"], label="index.alignment")
    state = _mapping(alignment["state"], label="index.alignment.state")
    active_ids = set(alignment["active_obstacle_ids"])
    barriers = _mapping(alignment["obstacle_barrier_m"], label="obstacle_barrier_m")
    inflation = _finite_float(
        alignment["additional_body_inflation_m"],
        label="additional_body_inflation_m",
    )
    visible_ids: set[str] = set()
    for obstacle_id, obstacle in obstacles.items():
        center = point(
            _finite_float(obstacle["x_m"], label="obstacle.x_m"),
            _finite_float(obstacle["y_m"], label="obstacle.y_m"),
        )
        physical_radius = _positive_float(
            obstacle["physical_radius_m"], label="obstacle.physical_radius_m"
        ) * scale
        draw.ellipse(
            (
                center[0] - physical_radius,
                center[1] - physical_radius,
                center[0] + physical_radius,
                center[1] + physical_radius,
            ),
            fill=(112, 76, 60),
            outline=(227, 183, 153),
            width=2,
        )
        if obstacle_id not in active_ids:
            continue
        certificate_radius = (
            _positive_float(
                obstacle["certificate_radius_m"],
                label="obstacle.certificate_radius_m",
            )
            + inflation
        ) * scale
        barrier_value = barriers.get(obstacle_id)
        barrier = None if barrier_value is None else _finite_float(
            barrier_value, label=f"obstacle_barrier_m.{obstacle_id}"
        )
        color = _DANGER if barrier is not None and barrier < _DANGER_THRESHOLD_M else _AMBER
        samples = [
            (
                center[0] + certificate_radius * cos(2.0 * pi * index / 64.0),
                center[1] + certificate_radius * sin(2.0 * pi * index / 64.0),
            )
            for index in range(65)
        ]
        _draw_dashed_polyline(draw, samples, fill=color, width=3)
        visible_ids.add(obstacle_id)

    rover_center = point(
        _finite_float(state["x_m"], label="state.x_m"),
        _finite_float(state["y_m"], label="state.y_m"),
    )
    rover_radius = _positive_float(road.get("rover_radius_m", 0.31), label="road.rover_radius_m") * scale
    draw.ellipse(
        (
            rover_center[0] - rover_radius,
            rover_center[1] - rover_radius,
            rover_center[0] + rover_radius,
            rover_center[1] + rover_radius,
        ),
        fill=(55, 164, 193),
        outline=(171, 232, 242),
        width=2,
    )
    heading = _finite_float(state["heading_rad"], label="state.heading_rad")
    nose = (
        rover_center[0] + max(rover_radius * 1.7, 6.0) * cos(heading),
        rover_center[1] - max(rover_radius * 1.7, 6.0) * sin(heading),
    )
    draw.line((*rover_center, *nose), fill=(236, 251, 255), width=3)
    return visible_ids


def _render_dashboard(
    source: Image.Image,
    *,
    width: int,
    height: int,
    camera: Mapping[str, Any],
    road: Mapping[str, Any],
    row: Mapping[str, Any],
    obstacles: Mapping[str, Mapping[str, Any]],
) -> tuple[Image.Image, set[str], set[str], bool]:
    if (width, height) != (1920, 1080):
        raise ValueError("camera certificate dashboard is frozen at 1920x1080")
    dashboard = Image.new("RGB", (width, height), (8, 12, 15))
    draw = ImageDraw.Draw(dashboard)
    terminal_status = row.get("terminal_status") is True
    termination = row.get("terminal_reason") if terminal_status else None
    draw.text(
        (32, 24),
        "REAL GAZEBO RGB / TIMESTAMP-ALIGNED OBSTACLE CERTIFICATE",
        fill=(225, 233, 231),
        font=_font(27, bold=True),
    )
    draw.text(
        (1888, 28),
        "COMPLETED" if terminal_status else "RUNNING",
        fill=(67, 199, 217) if terminal_status else (102, 199, 143),
        font=_font(22, bold=True),
        anchor="ra",
    )
    camera_width = _CAMERA_BOX[2] - _CAMERA_BOX[0]
    camera_height = _CAMERA_BOX[3] - _CAMERA_BOX[1]
    camera_view = source.convert("RGB").resize(
        (camera_width, camera_height),
        Image.Resampling.LANCZOS,
    )
    camera_visible_ids = _draw_camera_certificates(
        camera_view,
        camera=camera,
        row=row,
        obstacles=obstacles,
    )
    dashboard.paste(camera_view, (_CAMERA_BOX[0], _CAMERA_BOX[1]))
    draw.rectangle(_CAMERA_BOX, outline=(90, 109, 113), width=2)
    top_down_visible_ids = _draw_top_down(
        dashboard,
        road=road,
        row=row,
        obstacles=obstacles,
    )

    alignment = _mapping(row["alignment"], label="index.alignment")
    active_ids = alignment["active_obstacle_ids"]
    simulated_time = _finite_float(
        row.get("playback_simulated_time_s", row["capture_simulated_time_s"]),
        label="playback_simulated_time_s",
    )
    minimum_barrier_value = row.get("minimum_barrier_m")
    minimum_barrier = (
        "n/a"
        if minimum_barrier_value is None
        else f"{_finite_float(minimum_barrier_value, label='minimum_barrier_m'):.3f} m"
    )
    draw.text((1472, 526), "SIM TIME", fill=(128, 143, 146), font=_font(20, bold=True))
    draw.text((1472, 558), f"{simulated_time:8.2f} s", fill=(238, 244, 242), font=_font(34, bold=True))
    draw.text((1472, 632), "MIN h", fill=(128, 143, 146), font=_font(20, bold=True))
    barrier_color = (
        _DANGER
        if minimum_barrier_value is not None
        and float(minimum_barrier_value) < _DANGER_THRESHOLD_M
        else _AMBER
    )
    draw.text((1472, 664), minimum_barrier, fill=barrier_color, font=_font(34, bold=True))
    draw.text((1472, 738), "ACTIVE", fill=(128, 143, 146), font=_font(20, bold=True))
    draw.text((1472, 770), str(len(active_ids)), fill=(238, 244, 242), font=_font(42, bold=True))
    draw.text(
        (32, 914),
        "Amber dashed: active certificate     Red: h < 0.10 m",
        fill=(182, 193, 192),
        font=_font(21),
    )
    draw.text(
        (32, 964),
        f"frame {int(row['frame_counter'])}   capture-aligned error "
        f"{_finite_float(alignment['absolute_time_error_s'], label='absolute_time_error_s'):.4f} s",
        fill=(126, 142, 145),
        font=_font(19),
    )
    draw.text(
        (1472, 852),
        "EMPIRICAL ONLINE CBF",
        fill=(182, 193, 192),
        font=_font(18, bold=True),
    )
    draw.text(
        (1472, 884),
        "OBSERVER-ONLY OVERLAY",
        fill=(128, 143, 146),
        font=_font(17),
    )
    terminal_status_visible = False
    if terminal_status:
        terminal_time_s = _finite_float(
            row["terminal_simulated_time_s"],
            label="terminal_simulated_time_s",
        )
        if not isinstance(termination, str) or not termination.strip():
            raise ValueError("terminal dashboard frame requires a terminal reason")
        draw.rounded_rectangle(
            (1464, 906, 1892, 1012),
            radius=12,
            fill=(14, 25, 29),
            outline=(67, 199, 217),
            width=2,
        )
        draw.text(
            (1476, 922),
            f"END: {termination.strip()[:28]}",
            fill=(225, 233, 231),
            font=_font(18, bold=True),
        )
        draw.text(
            (1476, 958),
            f"TERMINAL t={terminal_time_s:.2f} s",
            fill=(67, 199, 217),
            font=_font(18, bold=True),
        )
        terminal_status_visible = True
    return (
        dashboard,
        camera_visible_ids | top_down_visible_ids,
        camera_visible_ids,
        terminal_status_visible,
    )


def _parse_discoverer(output: str) -> dict[str, float | int]:
    duration_match = re.search(
        r"Duration:\s*(\d+):(\d+):(\d+)(?:\.(\d+))?",
        output,
        flags=re.IGNORECASE,
    )
    width_match = re.search(r"Width:\s*(\d+)", output, flags=re.IGNORECASE)
    height_match = re.search(r"Height:\s*(\d+)", output, flags=re.IGNORECASE)
    fps_match = re.search(
        r"Frame\s*rate:\s*(\d+)\s*/\s*(\d+)",
        output,
        flags=re.IGNORECASE,
    )
    if not duration_match or not width_match or not height_match or not fps_match:
        raise ValueError("gst-discoverer output is missing duration, resolution, or frame rate")
    hours, minutes, seconds = (int(duration_match.group(index)) for index in range(1, 4))
    fractional = duration_match.group(4) or ""
    duration_s = hours * 3600.0 + minutes * 60.0 + seconds
    if fractional:
        duration_s += int(fractional) / (10 ** len(fractional))
    denominator = int(fps_match.group(2))
    if denominator <= 0:
        raise ValueError("gst-discoverer reported an invalid frame rate")
    return {
        "duration_s": duration_s,
        "width": int(width_match.group(1)),
        "height": int(height_match.group(1)),
        "fps": int(fps_match.group(1)) / denominator,
    }


def _command_log_payload(
    *,
    command: Sequence[str],
    return_code: int | None,
    stdout_path: Path,
    stderr_path: Path,
    root: Path,
) -> dict[str, Any]:
    return {
        "command": list(command),
        "return_code": return_code,
        "stdout_log": str(stdout_path.relative_to(root)),
        "stdout_log_sha256": _file_sha256(stdout_path),
        "stderr_log": str(stderr_path.relative_to(root)),
        "stderr_log_sha256": _file_sha256(stderr_path),
    }


def _run_logged(
    command: list[str],
    *,
    name: str,
    destination: Path,
) -> subprocess.CompletedProcess[str]:
    log_directory = destination / "logs"
    stdout_path = log_directory / f"{name}.stdout.log"
    stderr_path = log_directory / f"{name}.stderr.log"
    command_path = log_directory / f"{name}.command.json"
    _atomic_write_json(command_path, {"argv": command})
    try:
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            check=False,
        )
    except BaseException as error:
        _write_text(stdout_path, "")
        _write_text(stderr_path, f"{type(error).__name__}: {error}\n")
        raise
    _write_text(stdout_path, result.stdout or "")
    _write_text(stderr_path, result.stderr or "")
    return result


def _write_final_manifest(path: Path, manifest: dict[str, Any]) -> None:
    manifest.pop("manifest_content_sha256", None)
    manifest["manifest_content_sha256"] = _canonical_sha256(manifest)
    _atomic_write_json(path, manifest)


def render_camera_certificate_video(
    *,
    recording_manifest_path: str | Path,
    output_dir: str | Path,
    terminal_reason: str,
    terminal_simulated_time_s: float,
    gst_launch_binary: str = "gst-launch-1.0",
    gst_discoverer_binary: str = "gst-discoverer-1.0",
) -> Path:
    """Verify a camera recording, render a clean dashboard, and encode MP4.

    The destination must not already exist. Once created, it is retained on
    both success and failure; operational failures are represented by the
    returned ``camera_video_manifest.json`` rather than by deleting evidence.
    """

    source_manifest_path = Path(recording_manifest_path).resolve()
    destination = Path(output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    manifest_path = destination / "camera_video_manifest.json"
    rendered_directory = destination / "dashboard_frames"
    rendered_directory.mkdir()
    log_directory = destination / "logs"
    log_directory.mkdir()
    terminal_reason_value = (
        terminal_reason.strip() if isinstance(terminal_reason, str) else None
    )
    manifest: dict[str, Any] = {
        "schema_version": 1,
        "kind": "gazebo_timestamp_aligned_camera_certificate_video",
        "status": "failed",
        "observer_only": True,
        "source_frame_count": 0,
        "rendered_frame_count": 0,
        "visible_overlay_frame_count": 0,
        "visible_overlay_obstacle_ids": [],
        "video_file": None,
        "video_file_sha256": None,
        "encoded_duration_s": None,
        "terminal_reason": terminal_reason_value,
        "terminal_simulated_time_s": None,
        "terminal": {
            "reason": terminal_reason_value,
            "simulated_time_s": None,
            "final_render_index": None,
            "status_label": "COMPLETED",
            "visible": False,
        },
        "source": {
            "recording_manifest": str(source_manifest_path),
            "recording_manifest_file_sha256": None,
            "recording_kind": None,
            "recording_status": None,
            "index_file": None,
            "index_file_sha256_expected": None,
            "index_file_sha256_actual": None,
            "recording_run_simulated_duration_s": None,
            "rendered_frame_index_file": None,
            "rendered_frame_index_file_sha256": None,
        },
        "counts": {
            "recording_manifest_frame_count": 0,
            "index_frame_count": 0,
            "source_verified_frame_count": 0,
            "playback_frame_count": 0,
            "unique_source_counter_count": 0,
            "repeated_output_frame_count": 0,
            "skipped_source_callback_count": 0,
            "rendered_frame_count": 0,
            "encoded_frame_count": 0,
        },
        "overlays": {
            "danger_threshold_m": _DANGER_THRESHOLD_M,
            "visible_frame_count": 0,
            "camera_visible_frame_count": 0,
            "obstacle_ids": [],
            "camera_obstacle_ids": [],
            "source_activated_obstacle_ids": [],
            "camera_visible_output_frame_count_by_obstacle": {},
        },
        "timeline": {
            "mapping_strategy": (
                "fixed_1x_playback_grid_nearest_source_timestamp_"
                "ties_to_earlier_capture_terminal_target_exact"
            ),
            "mapping_time_basis": (
                "integer_nanoseconds_rounded_from_manifest_seconds"
            ),
            "tie_policy": "earlier_capture_then_earlier_source_counter",
            "run_simulated_duration_s": None,
            "playback_fps": None,
            "playback_frame_count": 0,
            "encoded_timeline_duration_s": None,
            "duration_quantization_error_s": None,
            "unique_source_counter_count": 0,
            "repeated_output_frame_count": 0,
            "skipped_source_callback_count": 0,
            "maximum_absolute_mapping_error_s": None,
            "mean_absolute_mapping_error_s": None,
            "source_mapping_monotonic": False,
        },
        "render": {
            "layout": "left_16x9_gazebo_rgb_right_compact_top_down_metrics_outside_track",
            "width": 1920,
            "height": 1080,
            "frame_directory": rendered_directory.name,
        },
        "encoder": {
            "backend": "gstreamer_x264",
            "launch": None,
            "discoverer": None,
            "decode_validation": None,
        },
        "artifact": {
            "video_file": None,
            "video_file_sha256": None,
            "video_file_bytes": None,
        },
        "video": {
            "duration_s": None,
            "expected_duration_s": None,
            "duration_error_s": None,
            "encoded_timeline_duration_s": None,
            "duration_quantization_error_s": None,
            "width": None,
            "height": None,
            "fps": None,
        },
        "gates": {
            "recording_manifest_kind_valid": False,
            "recording_status_completed": False,
            "terminal_arguments_valid": False,
            "recording_run_duration_matches_terminal": False,
            "index_file_sha256_valid": False,
            "index_frame_count_matches": False,
            "source_frame_hashes_valid": False,
            "source_frame_hash_closure": False,
            "source_frame_dimensions_valid": False,
            "source_capture_times_nondecreasing": False,
            "timeline_mapping_monotonic": False,
            "timeline_duration_quantization_within_half_frame": False,
            "rendered_frame_count_matches": False,
            "rendered_frame_hashes_recorded": False,
            "active_obstacle_projection_only": False,
            "minimum_overlay_visible_frame_count": False,
            "all_activated_obstacles_visible": False,
            "terminal_status_visible": False,
            "gst_launch_succeeded": False,
            "video_file_present": False,
            "gst_discoverer_succeeded": False,
            "full_decode_succeeded": False,
            "video_resolution_matches": False,
            "video_fps_matches": False,
            "video_duration_matches": False,
        },
        "error": None,
    }
    launch_command: list[str] | None = None
    discoverer_command: list[str] | None = None
    decode_command: list[str] | None = None
    try:
        if terminal_reason_value is None or not terminal_reason_value:
            raise ValueError("terminal_reason must be a nonempty string")
        terminal_time_s = _positive_float(
            terminal_simulated_time_s,
            label="terminal_simulated_time_s",
        )
        manifest["terminal_simulated_time_s"] = terminal_time_s
        manifest["terminal"]["simulated_time_s"] = terminal_time_s
        manifest["timeline"]["run_simulated_duration_s"] = terminal_time_s
        manifest["gates"]["terminal_arguments_valid"] = True
        recording = _load_object(source_manifest_path, label="recording manifest")
        manifest["source"]["recording_manifest_file_sha256"] = _file_sha256(
            source_manifest_path
        )
        manifest["source"]["recording_kind"] = recording.get("kind")
        manifest["source"]["recording_status"] = recording.get("status")
        manifest["gates"]["recording_manifest_kind_valid"] = (
            recording.get("kind") == "gazebo_timestamp_aligned_camera_recording"
        )
        manifest["gates"]["recording_status_completed"] = (
            recording.get("status") == "completed"
        )
        if not manifest["gates"]["recording_manifest_kind_valid"]:
            raise ValueError("unexpected camera recording manifest kind")
        if not manifest["gates"]["recording_status_completed"]:
            raise ValueError("camera recording manifest is not completed")
        recording_duration_s = _positive_float(
            recording["run_simulated_duration_s"],
            label="recording.run_simulated_duration_s",
        )
        manifest["source"]["recording_run_simulated_duration_s"] = (
            recording_duration_s
        )
        duration_identity_tolerance_s = max(1.0e-9, terminal_time_s * 1.0e-12)
        manifest["gates"]["recording_run_duration_matches_terminal"] = (
            abs(recording_duration_s - terminal_time_s)
            <= duration_identity_tolerance_s
        )
        if not manifest["gates"]["recording_run_duration_matches_terminal"]:
            raise ValueError(
                "camera recording duration does not match terminal trace time"
            )

        recording_root = source_manifest_path.parent.resolve()
        frame_count = _positive_int(recording["frame_count"], label="frame_count")
        manifest["counts"]["recording_manifest_frame_count"] = frame_count
        index_path = _source_path(
            recording_root,
            recording["index_file"],
            label="recording index",
        )
        manifest["source"]["index_file"] = str(index_path)
        expected_index_hash = str(recording["index_file_sha256"])
        actual_index_hash = _file_sha256(index_path)
        manifest["source"]["index_file_sha256_expected"] = expected_index_hash
        manifest["source"]["index_file_sha256_actual"] = actual_index_hash
        manifest["gates"]["index_file_sha256_valid"] = (
            actual_index_hash == expected_index_hash
        )
        if not manifest["gates"]["index_file_sha256_valid"]:
            raise ValueError("camera recording index SHA-256 mismatch")
        rows = _load_index(index_path)
        manifest["counts"]["index_frame_count"] = len(rows)
        manifest["gates"]["index_frame_count_matches"] = len(rows) == frame_count
        if not manifest["gates"]["index_frame_count_matches"]:
            raise ValueError("camera recording frame count does not match its index")

        camera = _mapping(recording["camera"], label="recording.camera")
        camera_width = _positive_int(camera["image_width"], label="camera.image_width")
        camera_height = _positive_int(camera["image_height"], label="camera.image_height")
        scenario = _mapping(recording["scenario"], label="recording.scenario")
        road = _mapping(scenario["road"], label="recording.scenario.road")
        obstacle_items = scenario["obstacles"]
        if not isinstance(obstacle_items, list):
            raise ValueError("recording.scenario.obstacles must be a list")
        obstacles: dict[str, Mapping[str, Any]] = {}
        for item in obstacle_items:
            obstacle = _mapping(item, label="recording.scenario.obstacle")
            obstacle_id = str(obstacle["id"])
            if not obstacle_id or obstacle_id in obstacles:
                raise ValueError(f"duplicate or empty scenario obstacle id: {obstacle_id}")
            obstacles[obstacle_id] = obstacle

        playback = _mapping(recording["playback"], label="recording.playback")
        fps = _positive_float(playback["fps"], label="playback.fps")
        playback_frame_count = int(round(terminal_time_s * fps))
        if playback_frame_count <= 0:
            raise ValueError(
                "terminal duration and playback fps must produce at least one frame"
            )
        encoded_timeline_duration_s = playback_frame_count / fps
        duration_quantization_error_s = abs(
            encoded_timeline_duration_s - terminal_time_s
        )
        manifest["counts"]["playback_frame_count"] = playback_frame_count
        manifest["timeline"].update(
            {
                "playback_fps": fps,
                "playback_frame_count": playback_frame_count,
                "encoded_timeline_duration_s": encoded_timeline_duration_s,
                "duration_quantization_error_s": (
                    duration_quantization_error_s
                ),
            }
        )
        manifest["video"]["encoded_timeline_duration_s"] = (
            encoded_timeline_duration_s
        )
        manifest["video"]["duration_quantization_error_s"] = (
            duration_quantization_error_s
        )
        manifest["gates"][
            "timeline_duration_quantization_within_half_frame"
        ] = duration_quantization_error_s <= 0.5 / fps + 1.0e-9
        width = _positive_int(playback["width"], label="playback.width")
        height = _positive_int(playback["height"], label="playback.height")
        if (width, height) != (1920, 1080):
            raise ValueError("playback dimensions must be 1920x1080")
        if playback.get("encoder_backend") != "gstreamer_x264":
            raise ValueError("playback encoder_backend must be gstreamer_x264")
        bitrate = _positive_int(playback["bitrate_kbps"], label="playback.bitrate_kbps")
        minimum_overlay_count = int(
            playback.get("minimum_overlay_visible_frame_count", 0)
        )
        if minimum_overlay_count < 0:
            raise ValueError(
                "playback.minimum_overlay_visible_frame_count must be nonnegative"
            )
        require_all_activated_visible = bool(
            playback.get("require_all_activated_obstacles_visible", False)
        )
        video_path = _output_path(
            destination,
            playback["video_file"],
            label="playback.video_file",
        )
        video_path.parent.mkdir(parents=True, exist_ok=True)
        manifest["artifact"]["video_file"] = str(video_path.relative_to(destination))
        manifest["video_file"] = str(video_path.relative_to(destination))
        manifest["video"]["expected_duration_s"] = terminal_time_s

        source_frames: list[tuple[dict[str, Any], Path]] = []
        source_capture_times: list[float] = []
        source_activated_obstacle_ids: set[str] = set()
        dimensions_valid = True
        for row_number, row in enumerate(rows, start=1):
            for required_key in (
                "frame_file",
                "frame_file_sha256",
                "frame_counter",
                "capture_simulated_time_s",
                "alignment",
                "minimum_barrier_m",
                "termination_reason",
            ):
                if required_key not in row:
                    raise ValueError(
                        f"camera index row {row_number} is missing {required_key}"
                    )
            source_frame = _source_path(
                recording_root,
                row["frame_file"],
                label=f"camera frame {row_number}",
            )
            actual_frame_hash = _file_sha256(source_frame)
            if actual_frame_hash != str(row["frame_file_sha256"]):
                raise ValueError(f"camera frame {row_number} SHA-256 mismatch")
            with Image.open(source_frame) as image:
                image.load()
                dimensions_valid = dimensions_valid and image.size == (
                    camera_width,
                    camera_height,
                )
            if not dimensions_valid:
                raise ValueError(
                    f"camera frame {row_number} dimensions do not match camera metadata"
                )
            capture_time_s = _finite_float(
                row["capture_simulated_time_s"],
                label=f"camera frame {row_number} capture_simulated_time_s",
            )
            if capture_time_s < 0.0:
                raise ValueError("camera capture simulated time must be nonnegative")
            alignment = _mapping(
                row["alignment"],
                label=f"camera frame {row_number} alignment",
            )
            active_values = alignment["active_obstacle_ids"]
            if not isinstance(active_values, list) or not all(
                isinstance(item, str) for item in active_values
            ):
                raise ValueError("alignment.active_obstacle_ids must be a string list")
            for obstacle_id in active_values:
                if obstacle_id not in obstacles:
                    raise ValueError(
                        f"active obstacle is absent from scenario: {obstacle_id}"
                    )
            source_capture_times.append(capture_time_s)
            source_activated_obstacle_ids.update(active_values)
            source_frames.append((row, source_frame))
            manifest["counts"]["source_verified_frame_count"] += 1
        manifest["gates"]["source_frame_hashes_valid"] = (
            manifest["counts"]["source_verified_frame_count"] == frame_count
        )
        manifest["gates"]["source_frame_dimensions_valid"] = dimensions_valid
        source_times_nondecreasing = all(
            later >= earlier
            for earlier, later in zip(
                source_capture_times,
                source_capture_times[1:],
            )
        )
        manifest["gates"]["source_capture_times_nondecreasing"] = (
            source_times_nondecreasing
        )
        if not source_times_nondecreasing:
            raise ValueError("camera source capture timestamps must be nondecreasing")
        manifest["gates"]["source_frame_hash_closure"] = manifest["gates"][
            "source_frame_hashes_valid"
        ]
        manifest["source_frame_count"] = manifest["counts"][
            "source_verified_frame_count"
        ]
        manifest["overlays"]["source_activated_obstacle_ids"] = sorted(
            source_activated_obstacle_ids
        )

        source_capture_times_ns = [
            int(round(value * 1_000_000_000.0))
            for value in source_capture_times
        ]
        playback_mapping: list[dict[str, int | float]] = []
        for render_index in range(playback_frame_count):
            playback_time_s = (
                terminal_time_s
                if render_index == playback_frame_count - 1
                else (render_index + 1) / fps
            )
            playback_time_ns = int(round(playback_time_s * 1_000_000_000.0))
            source_index = _nearest_source_index(
                source_capture_times_ns,
                playback_time_ns,
            )
            mapping_error_ns = abs(
                source_capture_times_ns[source_index] - playback_time_ns
            )
            playback_mapping.append(
                {
                    "source_index": source_index,
                    "playback_simulated_time_s": playback_time_s,
                    "playback_simulated_time_ns": playback_time_ns,
                    "absolute_mapping_error_s": mapping_error_ns * 1.0e-9,
                }
            )
        mapped_source_indices = [
            int(item["source_index"]) for item in playback_mapping
        ]
        mapped_source_counters = [
            int(source_frames[index][0]["frame_counter"])
            for index in mapped_source_indices
        ]
        unique_source_indices = set(mapped_source_indices)
        unique_source_counter_count = len(set(mapped_source_counters))
        repeated_output_frame_count = (
            playback_frame_count - unique_source_counter_count
        )
        skipped_source_callback_count = frame_count - len(unique_source_indices)
        source_mapping_monotonic = all(
            later >= earlier
            for earlier, later in zip(
                mapped_source_counters,
                mapped_source_counters[1:],
            )
        )
        mapping_errors = [
            float(item["absolute_mapping_error_s"])
            for item in playback_mapping
        ]
        maximum_mapping_error_s = max(mapping_errors)
        mean_mapping_error_s = sum(mapping_errors) / len(mapping_errors)
        manifest["counts"].update(
            {
                "unique_source_counter_count": unique_source_counter_count,
                "repeated_output_frame_count": repeated_output_frame_count,
                "skipped_source_callback_count": skipped_source_callback_count,
            }
        )
        manifest["timeline"].update(
            {
                "unique_source_counter_count": unique_source_counter_count,
                "repeated_output_frame_count": repeated_output_frame_count,
                "skipped_source_callback_count": skipped_source_callback_count,
                "maximum_absolute_mapping_error_s": maximum_mapping_error_s,
                "mean_absolute_mapping_error_s": mean_mapping_error_s,
                "source_mapping_monotonic": source_mapping_monotonic,
            }
        )
        manifest["gates"]["timeline_mapping_monotonic"] = (
            source_mapping_monotonic
        )

        rendered_index_rows: list[dict[str, Any]] = []
        visible_frame_count = 0
        camera_visible_frame_count = 0
        visible_ids: set[str] = set()
        camera_visible_ids: set[str] = set()
        camera_visible_count_by_obstacle = {
            obstacle_id: 0 for obstacle_id in source_activated_obstacle_ids
        }
        seen_source_indices: set[int] = set()
        terminal_status_visible = False
        for render_index, mapping in enumerate(playback_mapping):
            source_index = int(mapping["source_index"])
            row, source_frame = source_frames[source_index]
            source_capture_time_s = source_capture_times[source_index]
            terminal_frame = render_index == playback_frame_count - 1
            display_row = dict(row)
            display_row["playback_simulated_time_s"] = float(
                mapping["playback_simulated_time_s"]
            )
            display_row["terminal_status"] = terminal_frame
            display_row["terminal_reason"] = (
                terminal_reason_value if terminal_frame else None
            )
            display_row["terminal_simulated_time_s"] = (
                terminal_time_s if terminal_frame else None
            )
            with Image.open(source_frame) as image:
                image.load()
                (
                    dashboard,
                    frame_visible,
                    frame_camera_visible,
                    frame_terminal_visible,
                ) = _render_dashboard(
                    image,
                    width=width,
                    height=height,
                    camera=camera,
                    road=road,
                    row=display_row,
                    obstacles=obstacles,
                )
            rendered_path = rendered_directory / f"frame_{render_index:06d}.png"
            dashboard.save(rendered_path, format="PNG", compress_level=1)
            if frame_visible:
                visible_frame_count += 1
            if frame_camera_visible:
                camera_visible_frame_count += 1
            visible_ids.update(frame_visible)
            camera_visible_ids.update(frame_camera_visible)
            for obstacle_id in frame_camera_visible:
                camera_visible_count_by_obstacle[obstacle_id] = (
                    camera_visible_count_by_obstacle.get(obstacle_id, 0) + 1
                )
            source_reused = source_index in seen_source_indices
            seen_source_indices.add(source_index)
            terminal_status_visible = (
                terminal_status_visible or frame_terminal_visible
            )
            rendered_index_rows.append(
                {
                    "render_index": render_index,
                    "rendered_frame_file": str(rendered_path.relative_to(destination)),
                    "rendered_frame_file_sha256": _file_sha256(rendered_path),
                    "mapping_strategy": manifest["timeline"]["mapping_strategy"],
                    "source_index": source_index,
                    "source_frame_counter": int(row["frame_counter"]),
                    "source_frame_reused": source_reused,
                    "capture_simulated_time_s": source_capture_time_s,
                    "source_capture_simulated_time_s": source_capture_time_s,
                    "source_capture_simulated_time_ns": (
                        source_capture_times_ns[source_index]
                    ),
                    "playback_simulated_time_s": float(
                        mapping["playback_simulated_time_s"]
                    ),
                    "playback_simulated_time_ns": int(
                        mapping["playback_simulated_time_ns"]
                    ),
                    "absolute_mapping_error_s": float(
                        mapping["absolute_mapping_error_s"]
                    ),
                    "visible_overlay_obstacle_ids": sorted(frame_visible),
                    "camera_visible_overlay_obstacle_ids": sorted(
                        frame_camera_visible
                    ),
                    "terminal_status_visible": frame_terminal_visible,
                    "terminal_reason": (
                        terminal_reason_value if terminal_frame else None
                    ),
                    "terminal_simulated_time_s": (
                        terminal_time_s if terminal_frame else None
                    ),
                }
            )
            manifest["counts"]["rendered_frame_count"] += 1
        rendered_index_path = destination / "rendered_frame_index.jsonl"
        rendered_index_path.write_text(
            "".join(
                json.dumps(item, sort_keys=True, ensure_ascii=False) + "\n"
                for item in rendered_index_rows
            ),
            encoding="utf-8",
        )
        manifest["source"]["rendered_frame_index_file"] = rendered_index_path.name
        manifest["source"]["rendered_frame_index_file_sha256"] = _file_sha256(
            rendered_index_path
        )
        manifest["overlays"]["visible_frame_count"] = visible_frame_count
        manifest["overlays"]["camera_visible_frame_count"] = (
            camera_visible_frame_count
        )
        manifest["overlays"]["obstacle_ids"] = sorted(visible_ids)
        manifest["overlays"]["camera_obstacle_ids"] = sorted(camera_visible_ids)
        manifest["overlays"][
            "camera_visible_output_frame_count_by_obstacle"
        ] = dict(sorted(camera_visible_count_by_obstacle.items()))
        manifest["rendered_frame_count"] = manifest["counts"][
            "rendered_frame_count"
        ]
        manifest["visible_overlay_frame_count"] = camera_visible_frame_count
        manifest["visible_overlay_obstacle_ids"] = sorted(camera_visible_ids)
        manifest["gates"]["rendered_frame_count_matches"] = (
            manifest["counts"]["rendered_frame_count"]
            == playback_frame_count
        )
        manifest["gates"]["rendered_frame_hashes_recorded"] = (
            len(rendered_index_rows) == playback_frame_count
            and all(item["rendered_frame_file_sha256"] for item in rendered_index_rows)
        )
        manifest["gates"]["active_obstacle_projection_only"] = (
            visible_ids.issubset(source_activated_obstacle_ids)
            and camera_visible_ids.issubset(source_activated_obstacle_ids)
        )
        manifest["gates"]["minimum_overlay_visible_frame_count"] = (
            camera_visible_frame_count >= minimum_overlay_count
        )
        manifest["gates"]["all_activated_obstacles_visible"] = (
            not require_all_activated_visible
            or source_activated_obstacle_ids.issubset(camera_visible_ids)
        )
        manifest["terminal"].update(
            {
                "final_render_index": playback_frame_count - 1,
                "visible": terminal_status_visible,
            }
        )
        manifest["gates"]["terminal_status_visible"] = (
            terminal_status_visible
            and bool(rendered_index_rows)
            and rendered_index_rows[-1]["terminal_status_visible"] is True
        )

        fraction = Fraction(str(fps)).limit_denominator(1000)
        caps = f"image/png,framerate={fraction.numerator}/{fraction.denominator}"
        raw_caps = (
            f"video/x-raw,format=I420,width={width},height={height},"
            f"framerate={fraction.numerator}/{fraction.denominator}"
        )
        launch_command = [
            gst_launch_binary,
            "-e",
            "multifilesrc",
            f"location={(rendered_directory / 'frame_%06d.png').as_posix()}",
            "start-index=0",
            f"stop-index={playback_frame_count - 1}",
            f"caps={caps}",
            "!",
            "pngdec",
            "!",
            "videoconvert",
            "!",
            raw_caps,
            "!",
            "x264enc",
            "byte-stream=false",
            "speed-preset=ultrafast",
            "threads=2",
            f"bitrate={bitrate}",
            "!",
            "video/x-h264,stream-format=avc,alignment=au",
            "!",
            "mp4mux",
            "faststart=true",
            "!",
            "filesink",
            f"location={video_path.as_posix()}",
        ]
        launch_result = _run_logged(
            launch_command,
            name="gst_launch",
            destination=destination,
        )
        manifest["encoder"]["launch"] = _command_log_payload(
            command=launch_command,
            return_code=launch_result.returncode,
            stdout_path=log_directory / "gst_launch.stdout.log",
            stderr_path=log_directory / "gst_launch.stderr.log",
            root=destination,
        )
        manifest["gates"]["gst_launch_succeeded"] = launch_result.returncode == 0
        if launch_result.returncode != 0:
            raise RuntimeError(
                f"GStreamer encoding failed with exit code {launch_result.returncode}"
            )
        manifest["gates"]["video_file_present"] = (
            video_path.is_file() and video_path.stat().st_size > 0
        )
        if not manifest["gates"]["video_file_present"]:
            raise RuntimeError("GStreamer completed without producing a video file")
        manifest["counts"]["encoded_frame_count"] = playback_frame_count
        manifest["artifact"]["video_file_sha256"] = _file_sha256(video_path)
        manifest["artifact"]["video_file_bytes"] = video_path.stat().st_size
        manifest["video_file_sha256"] = manifest["artifact"][
            "video_file_sha256"
        ]

        discoverer_command = [gst_discoverer_binary, "-v", str(video_path)]
        discoverer_result = _run_logged(
            discoverer_command,
            name="gst_discoverer",
            destination=destination,
        )
        manifest["encoder"]["discoverer"] = _command_log_payload(
            command=discoverer_command,
            return_code=discoverer_result.returncode,
            stdout_path=log_directory / "gst_discoverer.stdout.log",
            stderr_path=log_directory / "gst_discoverer.stderr.log",
            root=destination,
        )
        manifest["gates"]["gst_discoverer_succeeded"] = (
            discoverer_result.returncode == 0
        )
        if discoverer_result.returncode != 0:
            raise RuntimeError(
                "gst-discoverer failed with exit code "
                f"{discoverer_result.returncode}"
            )
        video = _parse_discoverer(discoverer_result.stdout or "")
        manifest["video"].update(video)
        duration_error = abs(float(video["duration_s"]) - terminal_time_s)
        manifest["video"]["duration_error_s"] = duration_error
        manifest["encoded_duration_s"] = float(video["duration_s"])
        manifest["gates"]["video_resolution_matches"] = (
            int(video["width"]) == width and int(video["height"]) == height
        )
        manifest["gates"]["video_fps_matches"] = (
            abs(float(video["fps"]) - fps) <= 1.0e-6
        )
        manifest["gates"]["video_duration_matches"] = (
            duration_error <= 1.0 / fps + 1.0e-9
        )
        decode_command = [
            gst_launch_binary,
            "-q",
            "filesrc",
            f"location={video_path.as_posix()}",
            "!",
            "qtdemux",
            "!",
            "avdec_h264",
            "!",
            "videoconvert",
            "!",
            "fakesink",
            "sync=false",
        ]
        decode_result = _run_logged(
            decode_command,
            name="gst_decode_validation",
            destination=destination,
        )
        manifest["encoder"]["decode_validation"] = _command_log_payload(
            command=decode_command,
            return_code=decode_result.returncode,
            stdout_path=log_directory / "gst_decode_validation.stdout.log",
            stderr_path=log_directory / "gst_decode_validation.stderr.log",
            root=destination,
        )
        manifest["gates"]["full_decode_succeeded"] = (
            decode_result.returncode == 0
        )
        manifest["status"] = (
            "completed" if all(manifest["gates"].values()) else "failed"
        )
        if manifest["status"] != "completed":
            raise RuntimeError("camera video acceptance gates did not all pass")
    except BaseException as error:
        manifest["status"] = "failed"
        for stage, command in (
            ("launch", launch_command),
            ("discoverer", discoverer_command),
            ("decode_validation", decode_command),
        ):
            stdout_path = log_directory / f"gst_{stage}.stdout.log"
            stderr_path = log_directory / f"gst_{stage}.stderr.log"
            if (
                command is not None
                and manifest["encoder"][stage] is None
                and stdout_path.is_file()
                and stderr_path.is_file()
            ):
                manifest["encoder"][stage] = _command_log_payload(
                    command=command,
                    return_code=None,
                    stdout_path=stdout_path,
                    stderr_path=stderr_path,
                    root=destination,
                )
        error_log = log_directory / "failure.log"
        _write_text(error_log, f"{type(error).__name__}: {error}\n")
        manifest["error"] = {
            "type": type(error).__name__,
            "message": str(error)[:1000],
            "log_file": str(error_log.relative_to(destination)),
            "log_file_sha256": _file_sha256(error_log),
        }
    _write_final_manifest(manifest_path, manifest)
    return manifest_path
