from __future__ import annotations

import os
from pathlib import Path
from time import monotonic, sleep

import pytest

from cbf_rover.adapters.exact.road_contract import (
    generate_seeded_road_scenario,
)
from cbf_rover.adapters.gazebo.classic_bridge import GazeboClassicSession
from cbf_rover.adapters.gazebo.differential_road_contract import (
    load_gazebo_differential_road_contract,
)
from cbf_rover.adapters.gazebo.differential_road_world import (
    render_gazebo_classic_world,
)
from cbf_rover.adapters.gazebo.live_display import GazeboLiveDisplay


ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "configs/experiments/dynamic_road_gazebo_classic_v6.yaml"
CONFIG_V13 = (
    ROOT / "configs/experiments/dynamic_road_gazebo_classic_v13.yaml"
)
CONFIG_V14 = (
    ROOT / "configs/experiments/dynamic_road_gazebo_classic_v14.yaml"
)
CONFIG_V15 = (
    ROOT / "configs/experiments/dynamic_road_gazebo_classic_v15.yaml"
)


@pytest.mark.server_integration
def test_fixed_step_returns_timestamp_aligned_terminal_pose(
    tmp_path: Path,
) -> None:
    bridge_value = os.environ.get("CBF_ROVER_GAZEBO_BRIDGE_BINARY")
    if not bridge_value:
        pytest.skip("CBF_ROVER_GAZEBO_BRIDGE_BINARY is not configured")
    bridge_binary = Path(bridge_value).resolve()
    if not bridge_binary.is_file():
        raise FileNotFoundError(bridge_binary)

    contract = load_gazebo_differential_road_contract(CONFIG)
    scenario = generate_seeded_road_scenario(
        contract.base.base,
        seed=contract.default_seed,
    )
    world_path = tmp_path / "bridge_probe.world"
    world_path.write_text(
        render_gazebo_classic_world(
            contract=contract,
            scenario=scenario,
        ),
        encoding="utf-8",
    )
    session = GazeboClassicSession(
        runtime=contract.runtime,
        world_path=world_path,
        bridge_binary=bridge_binary,
        log_dir=tmp_path / "runtime_logs",
    )

    with session:
        initial = session.initial_state
        assert initial is not None
        assert initial.iterations == 1
        assert initial.sim_time_s == pytest.approx(0.001, abs=1.0e-12)
        successor = session.step(
            linear_mps=0.10,
            angular_radps=0.0,
        )

    assert successor.iterations - initial.iterations == 50
    assert successor.sim_time_s - initial.sim_time_s == pytest.approx(
        0.05,
        abs=1.0e-12,
    )


@pytest.mark.server_integration
def test_pose_iteration_sync_preserves_repeated_exact_steps(
    tmp_path: Path,
) -> None:
    bridge_value = os.environ.get("CBF_ROVER_GAZEBO_BRIDGE_BINARY")
    if not bridge_value:
        pytest.skip("CBF_ROVER_GAZEBO_BRIDGE_BINARY is not configured")
    bridge_binary = Path(bridge_value).resolve()
    if not bridge_binary.is_file():
        raise FileNotFoundError(bridge_binary)

    contract = load_gazebo_differential_road_contract(CONFIG_V13)
    scenario = generate_seeded_road_scenario(
        contract.base.base,
        seed=contract.default_seed,
    )
    world_path = tmp_path / "pose_iteration_probe.world"
    world_path.write_text(
        render_gazebo_classic_world(
            contract=contract,
            scenario=scenario,
        ),
        encoding="utf-8",
    )

    with GazeboClassicSession(
        runtime=contract.runtime,
        world_path=world_path,
        bridge_binary=bridge_binary,
        log_dir=tmp_path / "runtime_logs",
    ) as session:
        previous = session.initial_state
        assert previous is not None
        for _ in range(10):
            successor = session.step(
                linear_mps=0.10,
                angular_radps=0.0,
            )
            assert successor.iterations - previous.iterations == 50
            assert successor.sim_time_s - previous.sim_time_s == pytest.approx(
                0.05,
                abs=1.0e-12,
            )
            previous = successor


@pytest.mark.server_integration
def test_v14_front_camera_writes_atomic_bmp_frames(tmp_path: Path) -> None:
    bridge_value = os.environ.get("CBF_ROVER_GAZEBO_BRIDGE_BINARY")
    if not bridge_value:
        pytest.skip("CBF_ROVER_GAZEBO_BRIDGE_BINARY is not configured")
    bridge_binary = Path(bridge_value).resolve()
    contract = load_gazebo_differential_road_contract(CONFIG_V14)
    assert contract.front_camera is not None
    camera_plugin = bridge_binary.parent / contract.front_camera.plugin
    if not camera_plugin.is_file():
        raise FileNotFoundError(camera_plugin)
    scenario = generate_seeded_road_scenario(
        contract.base.base,
        seed=contract.default_seed,
    )
    camera_path = tmp_path / "camera" / contract.front_camera.frame_file
    camera_path.parent.mkdir()
    world_path = tmp_path / "front_camera_probe.world"
    world = render_gazebo_classic_world(
        contract=contract,
        scenario=scenario,
        camera_output_path=camera_path,
        camera_queue_directory=camera_path.parent / "ignored_queue",
    )
    assert "<queue_directory>" not in world
    world_path.write_text(world, encoding="utf-8")

    with GazeboClassicSession(
        runtime=contract.runtime,
        world_path=world_path,
        bridge_binary=bridge_binary,
        log_dir=tmp_path / "runtime_logs",
    ) as session:
        latest_state = session.initial_state
        assert latest_state is not None
        for _ in range(6):
            latest_state = session.step(linear_mps=0.05, angular_radps=0.0)
            sleep(contract.base.system.dt_s)
        deadline = monotonic() + 5.0
        metadata = None
        content = b""
        fresh_after_s = (
            latest_state.sim_time_s
            - 1.0 / contract.front_camera.update_rate_hz
            - 0.01
        )
        while monotonic() < deadline:
            if camera_path.is_file():
                content = camera_path.read_bytes()
                metadata = GazeboLiveDisplay._camera_bmp_metadata(
                    content,
                    camera=contract.front_camera,
                )
                if (
                    metadata is not None
                    and float(metadata["capture_simulated_time_s"])
                    >= fresh_after_s
                ):
                    break
            sleep(0.05)

    assert content[:2] == b"BM"
    assert int.from_bytes(content[18:22], "little", signed=True) == 960
    assert int.from_bytes(content[22:26], "little", signed=True) == 540
    assert int.from_bytes(content[28:30], "little") == 24
    assert metadata is not None
    assert metadata["frame_counter"] >= 1
    assert float(metadata["capture_simulated_time_s"]) <= (
        latest_state.sim_time_s + 1.0e-12
    )
    assert float(metadata["capture_simulated_time_s"]) >= fresh_after_s


@pytest.mark.server_integration
def test_v15_front_camera_queue_preserves_every_atomic_frame(
    tmp_path: Path,
) -> None:
    bridge_value = os.environ.get("CBF_ROVER_GAZEBO_BRIDGE_BINARY")
    if not bridge_value:
        pytest.skip("CBF_ROVER_GAZEBO_BRIDGE_BINARY is not configured")
    bridge_binary = Path(bridge_value).resolve()
    contract = load_gazebo_differential_road_contract(CONFIG_V15)
    assert contract.front_camera is not None
    assert contract.camera_recording is not None
    camera_plugin = bridge_binary.parent / contract.front_camera.plugin
    if not camera_plugin.is_file():
        raise FileNotFoundError(camera_plugin)
    scenario = generate_seeded_road_scenario(
        contract.base.base,
        seed=contract.default_seed,
    )
    camera_directory = tmp_path / "camera"
    camera_directory.mkdir()
    camera_path = camera_directory / contract.front_camera.frame_file
    queue_directory = camera_directory / "frame_queue"
    queue_directory.mkdir()
    world = render_gazebo_classic_world(
        contract=contract,
        scenario=scenario,
        camera_output_path=camera_path,
        camera_queue_directory=queue_directory,
    )
    assert f"<queue_directory>{queue_directory}</queue_directory>" in world
    world_path = tmp_path / "front_camera_queue_probe.world"
    world_path.write_text(world, encoding="utf-8")

    with GazeboClassicSession(
        runtime=contract.runtime,
        world_path=world_path,
        bridge_binary=bridge_binary,
        log_dir=tmp_path / "runtime_logs",
    ) as session:
        latest_state = session.initial_state
        assert latest_state is not None
        for _ in range(40):
            latest_state = session.step(linear_mps=0.05, angular_radps=0.0)
            sleep(contract.base.system.dt_s)
            if len(list(queue_directory.glob("frame_*.bmp"))) >= 6:
                break

    queue_frames = sorted(queue_directory.glob("frame_*.bmp"))
    assert len(queue_frames) >= 6
    assert not list(camera_directory.rglob("*.tmp"))
    counters: list[int] = []
    capture_times: list[float] = []
    for queue_frame in queue_frames:
        content = queue_frame.read_bytes()
        metadata = GazeboLiveDisplay._camera_bmp_metadata(
            content,
            camera=contract.front_camera,
        )
        assert metadata is not None
        counter = int(metadata["frame_counter"])
        counters.append(counter)
        capture_times.append(float(metadata["capture_simulated_time_s"]))
        assert queue_frame.name == f"frame_{counter:09d}.bmp"

    assert counters == list(range(1, len(counters) + 1))
    assert all(
        later >= earlier
        for earlier, later in zip(capture_times, capture_times[1:])
    )
    assert camera_path.is_file()
    assert camera_path.read_bytes() == queue_frames[-1].read_bytes()
    latest_metadata = GazeboLiveDisplay._camera_bmp_metadata(
        camera_path.read_bytes(),
        camera=contract.front_camera,
    )
    assert latest_metadata is not None
    assert int(latest_metadata["frame_counter"]) == counters[-1]
    assert float(latest_metadata["capture_simulated_time_s"]) <= (
        latest_state.sim_time_s + 1.0e-12
    )
