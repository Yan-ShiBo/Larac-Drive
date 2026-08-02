from __future__ import annotations

from hashlib import sha256
import json
from pathlib import Path
import subprocess
from typing import Any

from PIL import Image
import pytest

from cbf_rover.adapters.gazebo.camera_video import (
    render_camera_certificate_video,
)


DISCOVERER_OUTPUT = """Analyzing file:///camera_certificate.mp4
Done discovering file:///camera_certificate.mp4

Properties:
  Duration: 0:00:00.400000000
  Seekable: yes
  container #0: Quicktime
    video #1: H.264
      Width: 1920
      Height: 1080
      Frame rate: 10/1
"""

TERMINAL_REASON = "goal_reached"
TERMINAL_SIMULATED_TIME_S = 0.4
ACTIVE_OBSTACLE_IDS = [f"obstacle_{index:02d}" for index in range(5)]


def _sha256(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


def _write_recording(
    directory: Path,
) -> Path:
    capture_times = [0.04, 0.10, 0.10, 0.30, 0.39]
    frame_count = len(capture_times)
    frames = directory / "recorded_frames"
    frames.mkdir(parents=True)
    rows: list[dict[str, Any]] = []
    for index in range(frame_count):
        frame = frames / f"frame_{index:06d}.png"
        Image.new("RGB", (960, 540), (18 + index, 24, 29)).save(frame)
        rows.append(
            {
                "frame_file": str(frame.relative_to(directory)),
                "frame_file_sha256": _sha256(frame),
                "frame_counter": 41 + index,
                "capture_simulated_time_s": capture_times[index],
                "alignment": {
                    "state": {
                        "x_m": 0.0,
                        "y_m": 0.0,
                        "heading_rad": 0.0,
                    },
                    "active_obstacle_ids": ACTIVE_OBSTACLE_IDS,
                    "obstacle_barrier_m": {
                        obstacle_id: 0.20 if index < 3 else 0.05
                        for obstacle_id in ACTIVE_OBSTACLE_IDS
                    },
                    "additional_body_inflation_m": 0.0,
                    "display_level": {
                        "symbol": "q_h",
                        "value_m": 0.30,
                        "level_set": "h_t(x) = q_h",
                        "safe_boundary": "h_t(x) = 0",
                        "semantics": (
                            "runtime_distance_cbf_level_not_qsbc_probability"
                        ),
                        "observer_only": True,
                    },
                    "absolute_time_error_s": 0.0,
                },
                "minimum_barrier_m": 0.20 if index < 3 else 0.05,
                "termination_reason": None,
            }
        )
    index_path = directory / "camera_frame_index.jsonl"
    index_path.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows),
        encoding="utf-8",
    )
    manifest = {
        "schema_version": 1,
        "kind": "gazebo_timestamp_aligned_camera_recording",
        "status": "completed",
        "frame_count": frame_count,
        "run_simulated_duration_s": TERMINAL_SIMULATED_TIME_S,
        "index_file": index_path.name,
        "index_file_sha256": _sha256(index_path),
        "camera": {
            "image_width": 960,
            "image_height": 540,
            "horizontal_fov_rad": 1.0471975511965976,
            "near_clip_m": 0.1,
            "mount_x_m": 0.0,
            "mount_y_m": 0.0,
            "pitch_rad": 0.14,
            "world_height_m": 0.8,
        },
        "scenario": {
            "road": {
                "x_min_m": 0.0,
                "x_max_m": 10.0,
                "physical_y_min_m": -2.5,
                "physical_y_max_m": 2.5,
                "lane_centers_y_m": [-1.0, 1.0],
                "finish_line_x_m": 9.5,
                "rover_radius_m": 0.31,
            },
            "obstacles": [
                *[
                    {
                        "id": obstacle_id,
                        "x_m": 4.0,
                        "y_m": -0.4 + index * 0.2,
                        "physical_radius_m": 0.12,
                        "certificate_radius_m": 0.25,
                    }
                    for index, obstacle_id in enumerate(ACTIVE_OBSTACLE_IDS)
                ],
                {
                    "id": "obstacle_inactive",
                    "x_m": 4.0,
                    "y_m": 1.4,
                    "physical_radius_m": 0.12,
                    "certificate_radius_m": 0.25,
                },
            ],
        },
        "playback": {
            "fps": 10.0,
            "width": 1920,
            "height": 1080,
            "video_file": "camera_certificate.mp4",
            "encoder_backend": "gstreamer_x264",
            "bitrate_kbps": 1800,
            "minimum_overlay_visible_frame_count": 4,
            "require_all_activated_obstacles_visible": True,
        },
    }
    manifest_path = directory / "camera_recording_manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return manifest_path


def test_render_camera_certificate_video_verifies_and_renders_active_overlay(
    tmp_path: Path,
    monkeypatch: Any,
) -> None:
    recording = _write_recording(tmp_path / "recording")
    output = tmp_path / "video"
    commands: list[list[str]] = []

    def fake_run(command: list[str], **kwargs: Any) -> subprocess.CompletedProcess[str]:
        commands.append(command)
        if command[0] == "gst-launch-test":
            (output / "camera_certificate.mp4").write_bytes(b"verified fake mp4")
            return subprocess.CompletedProcess(command, 0, "pipeline completed\n", "")
        return subprocess.CompletedProcess(command, 0, DISCOVERER_OUTPUT, "")

    monkeypatch.setattr(subprocess, "run", fake_run)
    manifest_path = render_camera_certificate_video(
        recording_manifest_path=recording,
        output_dir=output,
        terminal_reason=TERMINAL_REASON,
        terminal_simulated_time_s=TERMINAL_SIMULATED_TIME_S,
        gst_launch_binary="gst-launch-test",
        gst_discoverer_binary="gst-discoverer-test",
    )

    assert manifest_path == output / "camera_video_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["status"] == "completed"
    assert manifest["counts"] == {
        "recording_manifest_frame_count": 5,
        "index_frame_count": 5,
        "source_verified_frame_count": 5,
        "playback_frame_count": 4,
        "unique_source_counter_count": 3,
        "repeated_output_frame_count": 1,
        "skipped_source_callback_count": 2,
        "rendered_frame_count": 4,
        "encoded_frame_count": 4,
    }
    assert manifest["overlays"]["visible_frame_count"] == 4
    assert manifest["overlays"]["obstacle_ids"] == ACTIVE_OBSTACLE_IDS
    assert manifest["overlays"]["source_activated_obstacle_ids"] == (
        ACTIVE_OBSTACLE_IDS
    )
    assert manifest["overlays"][
        "camera_visible_output_frame_count_by_obstacle"
    ] == {obstacle_id: 4 for obstacle_id in ACTIVE_OBSTACLE_IDS}
    assert manifest["overlays"]["display_level_q_h_m"] == pytest.approx(0.30)
    assert manifest["overlays"]["preview_contour_semantics"] == (
        "distance_cbf_h_equals_q_h_not_qsbc_probability"
    )
    assert manifest["overlays"]["preview_contour_radius_rule"] == (
        "certificate_radius_m + additional_body_inflation_m + q_h_m"
    )
    assert manifest["observer_only"] is True
    assert manifest["source_frame_count"] == 5
    assert manifest["rendered_frame_count"] == 4
    assert manifest["visible_overlay_frame_count"] == 4
    assert manifest["visible_overlay_obstacle_ids"] == ACTIVE_OBSTACLE_IDS
    assert manifest["video_file"] == "camera_certificate.mp4"
    assert manifest["video_file_sha256"] == sha256(
        b"verified fake mp4"
    ).hexdigest()
    assert manifest["encoded_duration_s"] == 0.4
    assert manifest["terminal"] == {
        "reason": TERMINAL_REASON,
        "simulated_time_s": TERMINAL_SIMULATED_TIME_S,
        "final_render_index": 3,
        "status_label": "COMPLETED",
        "visible": True,
    }
    assert manifest["timeline"]["mapping_strategy"] == (
        "fixed_1x_playback_grid_nearest_source_timestamp_"
        "ties_to_earlier_capture_terminal_target_exact"
    )
    assert manifest["timeline"]["mapping_time_basis"] == (
        "integer_nanoseconds_rounded_from_manifest_seconds"
    )
    assert manifest["timeline"]["tie_policy"] == (
        "earlier_capture_then_earlier_source_counter"
    )
    assert manifest["timeline"]["run_simulated_duration_s"] == 0.4
    assert manifest["timeline"]["playback_frame_count"] == 4
    assert manifest["timeline"]["unique_source_counter_count"] == 3
    assert manifest["timeline"]["repeated_output_frame_count"] == 1
    assert manifest["timeline"]["skipped_source_callback_count"] == 2
    assert manifest["timeline"]["maximum_absolute_mapping_error_s"] == 0.1
    assert manifest["timeline"]["source_mapping_monotonic"] is True
    assert manifest["source"]["index_file_sha256_expected"] == _sha256(
        recording.parent / "camera_frame_index.jsonl"
    )
    assert manifest["source"]["index_file_sha256_actual"] == manifest[
        "source"
    ]["index_file_sha256_expected"]
    assert manifest["artifact"]["video_file_sha256"] == sha256(
        b"verified fake mp4"
    ).hexdigest()
    assert manifest["video"]["width"] == 1920
    assert manifest["video"]["height"] == 1080
    assert manifest["video"]["fps"] == 10.0
    assert manifest["video"]["duration_s"] == 0.4
    assert manifest["video"]["expected_duration_s"] == 0.4
    assert manifest["video"]["duration_error_s"] == 0.0
    assert manifest["gates"]["terminal_status_visible"] is True
    assert all(manifest["gates"].values())
    expected_manifest_hash = manifest.pop("manifest_content_sha256")
    assert expected_manifest_hash == sha256(
        json.dumps(
            manifest,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
            allow_nan=False,
        ).encode("utf-8")
    ).hexdigest()
    rendered_index = output / manifest["source"]["rendered_frame_index_file"]
    assert manifest["source"]["rendered_frame_index_file_sha256"] == _sha256(
        rendered_index
    )
    for line in rendered_index.read_text(encoding="utf-8").splitlines():
        rendered_record = json.loads(line)
        rendered_frame = output / rendered_record["rendered_frame_file"]
        assert rendered_record["rendered_frame_file_sha256"] == _sha256(
            rendered_frame
        )
    rendered_records = [
        json.loads(line)
        for line in rendered_index.read_text(encoding="utf-8").splitlines()
    ]
    assert [record["source_index"] for record in rendered_records] == [1, 1, 3, 4]
    assert [record["source_frame_counter"] for record in rendered_records] == [
        42,
        42,
        44,
        45,
    ]
    assert [
        record["source_capture_simulated_time_s"]
        for record in rendered_records
    ] == [0.10, 0.10, 0.30, 0.39]
    assert [
        record["playback_simulated_time_s"] for record in rendered_records
    ] == [0.1, 0.2, 0.3, 0.4]
    assert rendered_records[-1]["terminal_status_visible"] is True
    assert rendered_records[-1]["terminal_reason"] == TERMINAL_REASON
    assert rendered_records[-1]["terminal_simulated_time_s"] == 0.4
    assert {
        record["display_level_q_h_m"] for record in rendered_records
    } == {0.30}

    launch = " ".join(commands[0])
    assert "multifilesrc" in launch
    assert "pngdec" in launch
    assert "videoconvert" in launch
    assert "x264enc" in launch
    assert "byte-stream=false" in launch
    assert "video/x-h264,stream-format=avc,alignment=au" in launch
    assert "h264parse" not in launch
    assert "speed-preset=ultrafast" in launch
    assert "threads=2" in launch
    assert "mp4mux" in launch
    assert "faststart=true" in launch
    assert "stop-index=3" in launch

    first = Image.open(output / "dashboard_frames" / "frame_000000.png").convert(
        "RGB"
    )
    third = Image.open(output / "dashboard_frames" / "frame_000002.png").convert(
        "RGB"
    )
    amber_x = [
        x
        for y in range(80, 872)
        for x in range(32, 1440)
        if first.getpixel((x, y)) == (255, 176, 0)
    ]
    cyan_preview_pixels = [
        1
        for y in range(80, 872)
        for x in range(32, 1440)
        if first.getpixel((x, y)) == (67, 199, 217)
    ]
    red_pixels = [
        1
        for y in range(80, 872)
        for x in range(32, 1440)
        if third.getpixel((x, y)) == (255, 92, 77)
    ]
    assert amber_x
    assert min(amber_x) > 500
    assert cyan_preview_pixels
    assert red_pixels
    final = Image.open(
        output / "dashboard_frames" / "frame_000003.png"
    ).convert("RGB")
    completed_pixels = [
        1
        for y in range(20, 70)
        for x in range(1650, 1890)
        if final.getpixel((x, y)) == (67, 199, 217)
    ]
    assert completed_pixels


def test_render_camera_certificate_video_rejects_tampered_source_frame(
    tmp_path: Path,
    monkeypatch: Any,
) -> None:
    recording = _write_recording(tmp_path / "recording")
    tampered = recording.parent / "recorded_frames" / "frame_000001.png"
    tampered.write_bytes(tampered.read_bytes() + b"tampered")

    def unexpected_run(*args: Any, **kwargs: Any) -> subprocess.CompletedProcess[str]:
        raise AssertionError("GStreamer must not run after a source hash failure")

    monkeypatch.setattr(subprocess, "run", unexpected_run)
    output = tmp_path / "failed-video"
    manifest_path = render_camera_certificate_video(
        recording_manifest_path=recording,
        output_dir=output,
        terminal_reason=TERMINAL_REASON,
        terminal_simulated_time_s=TERMINAL_SIMULATED_TIME_S,
        gst_launch_binary="gst-launch-test",
        gst_discoverer_binary="gst-discoverer-test",
    )

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["status"] == "failed"
    assert manifest["error"]["type"] == "ValueError"
    assert "frame 2 SHA-256 mismatch" in manifest["error"]["message"]
    assert manifest["counts"]["recording_manifest_frame_count"] == 5
    assert manifest["counts"]["index_frame_count"] == 5
    assert manifest["counts"]["source_verified_frame_count"] == 1
    assert manifest["gates"]["index_file_sha256_valid"] is True
    assert manifest["gates"]["source_frame_hashes_valid"] is False
    assert manifest["artifact"]["video_file_sha256"] is None
    assert tampered.is_file()
    assert (output / manifest["error"]["log_file"]).is_file()


def test_render_camera_certificate_video_records_gstreamer_start_failure(
    tmp_path: Path,
    monkeypatch: Any,
) -> None:
    recording = _write_recording(tmp_path / "recording")

    def missing_binary(command: list[str], **kwargs: Any) -> subprocess.CompletedProcess[str]:
        raise FileNotFoundError(command[0])

    monkeypatch.setattr(subprocess, "run", missing_binary)
    output = tmp_path / "failed-encoder"
    manifest_path = render_camera_certificate_video(
        recording_manifest_path=recording,
        output_dir=output,
        terminal_reason=TERMINAL_REASON,
        terminal_simulated_time_s=TERMINAL_SIMULATED_TIME_S,
        gst_launch_binary="missing-gst-launch",
        gst_discoverer_binary="missing-gst-discoverer",
    )

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["status"] == "failed"
    assert manifest["error"]["type"] == "FileNotFoundError"
    assert manifest["counts"]["rendered_frame_count"] == 4
    assert manifest["encoder"]["launch"]["command"][0] == "missing-gst-launch"
    assert manifest["encoder"]["launch"]["return_code"] is None
    stdout_log = output / manifest["encoder"]["launch"]["stdout_log"]
    stderr_log = output / manifest["encoder"]["launch"]["stderr_log"]
    assert stdout_log.is_file()
    assert stderr_log.is_file()
    assert "FileNotFoundError" in stderr_log.read_text(encoding="utf-8")
    assert manifest["encoder"]["launch"]["stdout_log_sha256"] == _sha256(
        stdout_log
    )
    assert manifest["encoder"]["launch"]["stderr_log_sha256"] == _sha256(
        stderr_log
    )


def test_render_camera_certificate_video_never_overwrites_existing_output(
    tmp_path: Path,
) -> None:
    recording = _write_recording(tmp_path / "recording")
    output = tmp_path / "existing-output"
    output.mkdir()
    sentinel = output / "keep.txt"
    sentinel.write_text("keep me", encoding="utf-8")

    with pytest.raises(FileExistsError):
        render_camera_certificate_video(
            recording_manifest_path=recording,
            output_dir=output,
            terminal_reason=TERMINAL_REASON,
            terminal_simulated_time_s=TERMINAL_SIMULATED_TIME_S,
            gst_launch_binary="unused",
            gst_discoverer_binary="unused",
        )

    assert sentinel.read_text(encoding="utf-8") == "keep me"
    assert not (output / "camera_video_manifest.json").exists()
