"""Process boundary for the standalone Gazebo Classic transport bridge."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
import os
from pathlib import Path
from queue import Empty, Queue
import shutil
import signal
import socket
import subprocess
from threading import Thread
from time import monotonic, sleep
from typing import IO
from urllib.parse import urlparse

from .differential_road_contract import GazeboClassicRuntimeSpec


def _gazebo_plugin_directories(
    *,
    bridge_binary: Path,
    inherited: str,
) -> tuple[str, ...]:
    candidates = [bridge_binary.parent]
    candidates.extend(
        Path(part)
        for part in inherited.split(os.pathsep)
        if part
    )
    for library_root in (Path("/usr/lib"), Path("/usr/local/lib")):
        candidates.extend(library_root.glob("gazebo-*/plugins"))
        candidates.extend(library_root.glob("*/gazebo-*/plugins"))

    directories: list[str] = []
    for candidate in candidates:
        resolved = str(candidate.resolve())
        if candidate.is_dir() and resolved not in directories:
            directories.append(resolved)
    return tuple(directories)


@dataclass(frozen=True, slots=True)
class GazeboClassicState:
    sim_time_s: float
    iterations: int
    x_m: float
    y_m: float
    z_m: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float

    def __post_init__(self) -> None:
        values = (
            self.sim_time_s,
            self.x_m,
            self.y_m,
            self.z_m,
            self.roll_rad,
            self.pitch_rad,
            self.yaw_rad,
        )
        if not all(isfinite(float(value)) for value in values):
            raise ValueError("Gazebo state values must be finite")
        if self.sim_time_s < 0.0 or self.iterations < 0:
            raise ValueError("Gazebo time and iterations must be nonnegative")


def parse_bridge_state(line: str) -> GazeboClassicState:
    fields = line.strip().split()
    if len(fields) != 9 or fields[0] != "STATE":
        raise ValueError(f"unexpected Gazebo bridge state line: {line!r}")
    return GazeboClassicState(
        sim_time_s=float(fields[1]),
        iterations=int(fields[2]),
        x_m=float(fields[3]),
        y_m=float(fields[4]),
        z_m=float(fields[5]),
        roll_rad=float(fields[6]),
        pitch_rad=float(fields[7]),
        yaw_rad=float(fields[8]),
    )


class GazeboClassicSession:
    """Own one isolated gzserver and one line-oriented bridge process."""

    def __init__(
        self,
        *,
        runtime: GazeboClassicRuntimeSpec,
        world_path: str | Path,
        bridge_binary: str | Path,
        log_dir: str | Path,
        gzserver_binary: str = "gzserver",
    ) -> None:
        self.runtime = runtime
        self.world_path = Path(world_path).resolve()
        self.bridge_binary = Path(bridge_binary).resolve()
        self.log_dir = Path(log_dir).resolve()
        self.gzserver_binary = str(gzserver_binary)
        self._gzserver: subprocess.Popen[str] | None = None
        self._bridge: subprocess.Popen[str] | None = None
        self._gzserver_log: IO[str] | None = None
        self._bridge_stderr: IO[str] | None = None
        self._lines: Queue[str | None] = Queue()
        self._reader: Thread | None = None
        self.initial_state: GazeboClassicState | None = None

    @property
    def environment(self) -> dict[str, str]:
        environment = dict(os.environ)
        environment["GAZEBO_MASTER_URI"] = self.runtime.master_uri
        environment["GAZEBO_IP"] = "127.0.0.1"
        environment["GAZEBO_MODEL_DATABASE_URI"] = ""
        environment["CBF_ROVER_GAZEBO_STARTUP_TIMEOUT_S"] = str(
            self.runtime.startup_timeout_s
        )
        inherited_plugin_path = environment.get("GAZEBO_PLUGIN_PATH", "")
        environment["GAZEBO_PLUGIN_PATH"] = os.pathsep.join(
            _gazebo_plugin_directories(
                bridge_binary=self.bridge_binary,
                inherited=inherited_plugin_path,
            )
        )
        return environment

    def _wait_for_master(self) -> None:
        parsed = urlparse(self.runtime.master_uri)
        assert parsed.hostname is not None
        assert parsed.port is not None
        deadline = monotonic() + self.runtime.startup_timeout_s
        while monotonic() < deadline:
            if self._gzserver is not None and self._gzserver.poll() is not None:
                raise RuntimeError(
                    "gzserver exited before opening its isolated master"
                )
            try:
                with socket.create_connection(
                    (parsed.hostname, parsed.port),
                    timeout=0.25,
                ):
                    return
            except OSError:
                sleep(0.05)
        raise TimeoutError("timed out waiting for isolated Gazebo master")

    def _read_bridge_stdout(self) -> None:
        assert self._bridge is not None
        assert self._bridge.stdout is not None
        try:
            for line in self._bridge.stdout:
                self._lines.put(line.rstrip("\r\n"))
        finally:
            self._lines.put(None)

    def _stderr_tail(self) -> str:
        path = self.log_dir / "bridge.stderr.log"
        if not path.is_file():
            return ""
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
        return "\n".join(lines[-20:])

    def _read_line(self, *, timeout_s: float) -> str:
        try:
            line = self._lines.get(timeout=timeout_s)
        except Empty as exc:
            raise TimeoutError(
                "timed out waiting for Gazebo bridge protocol output"
            ) from exc
        if line is None:
            return_code = None if self._bridge is None else self._bridge.poll()
            raise RuntimeError(
                "Gazebo bridge closed its protocol stream "
                f"(exit={return_code}): {self._stderr_tail()}"
            )
        return line

    def start(self) -> GazeboClassicState:
        if self._gzserver is not None or self._bridge is not None:
            raise RuntimeError("Gazebo session was already started")
        if not self.world_path.is_file():
            raise FileNotFoundError(self.world_path)
        if not self.bridge_binary.is_file():
            raise FileNotFoundError(self.bridge_binary)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self._gzserver_log = (self.log_dir / "gzserver.log").open(
            "w",
            encoding="utf-8",
        )
        self._bridge_stderr = (self.log_dir / "bridge.stderr.log").open(
            "w",
            encoding="utf-8",
        )
        environment = self.environment
        gzserver_command = [
            self.gzserver_binary,
            "--verbose",
            "-u",
            "-e",
            self.runtime.physics_engine,
            str(self.world_path),
        ]
        if self.runtime.headless_renderer == "xvfb":
            xvfb_run = shutil.which("xvfb-run")
            if xvfb_run is None:
                raise FileNotFoundError("xvfb-run is required for camera rendering")
            gzserver_command = [
                xvfb_run,
                "-a",
                "--server-args=-screen 0 1280x720x24 -ac "
                "+extension GLX +render -noreset",
                *gzserver_command,
            ]
        self._gzserver = subprocess.Popen(
            gzserver_command,
            stdin=subprocess.DEVNULL,
            stdout=self._gzserver_log,
            stderr=subprocess.STDOUT,
            text=True,
            env=environment,
            start_new_session=os.name == "posix",
        )
        self._wait_for_master()
        self._bridge = subprocess.Popen(
            [
                str(self.bridge_binary),
                "--world",
                self.runtime.world_name,
                "--model",
                self.runtime.model_name,
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=self._bridge_stderr,
            text=True,
            bufsize=1,
            env=environment,
        )
        self._reader = Thread(
            target=self._read_bridge_stdout,
            name="gazebo-classic-bridge-stdout",
            daemon=True,
        )
        self._reader.start()
        ready = self._read_line(timeout_s=self.runtime.startup_timeout_s)
        expected_ready = (
            f"READY GZCB1 {self.runtime.world_name} "
            f"{self.runtime.model_name}"
        )
        if ready != expected_ready:
            raise RuntimeError(
                f"unexpected Gazebo bridge handshake: {ready!r}"
            )
        self.initial_state = parse_bridge_state(
            self._read_line(timeout_s=self.runtime.state_timeout_s)
        )
        return self.initial_state

    def step(
        self,
        *,
        linear_mps: float,
        angular_radps: float,
    ) -> GazeboClassicState:
        if self._bridge is None or self._bridge.stdin is None:
            raise RuntimeError("Gazebo bridge is not running")
        if self._bridge.poll() is not None:
            raise RuntimeError(
                "Gazebo bridge exited before STEP: " + self._stderr_tail()
            )
        if not isfinite(linear_mps) or not isfinite(angular_radps):
            raise ValueError("Gazebo command must be finite")
        self._bridge.stdin.write(
            "STEP "
            f"{linear_mps:.17g} "
            f"{self.runtime.plugin_angular_command_sign * angular_radps:.17g} "
            f"{self.runtime.control_iterations_per_step}\n"
        )
        self._bridge.stdin.flush()
        return parse_bridge_state(
            self._read_line(timeout_s=self.runtime.state_timeout_s)
        )

    def close(self) -> None:
        if (
            self._bridge is not None
            and self._bridge.poll() is None
            and self._bridge.stdin is not None
        ):
            try:
                self._bridge.stdin.write("STOP\n")
                self._bridge.stdin.flush()
                self._bridge.wait(timeout=5.0)
            except (BrokenPipeError, subprocess.TimeoutExpired):
                self._bridge.terminate()
                try:
                    self._bridge.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    self._bridge.kill()
                    self._bridge.wait(timeout=5.0)
        if self._gzserver is not None and self._gzserver.poll() is None:
            self._signal_gzserver(signal.SIGTERM)
            try:
                self._gzserver.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                self._signal_gzserver(signal.SIGKILL)
                self._gzserver.wait(timeout=5.0)
        if self._bridge is not None and self._bridge.stdout is not None:
            self._bridge.stdout.close()
        if self._bridge is not None and self._bridge.stdin is not None:
            try:
                self._bridge.stdin.close()
            except BrokenPipeError:
                pass
        if self._gzserver_log is not None:
            self._gzserver_log.close()
        if self._bridge_stderr is not None:
            self._bridge_stderr.close()

    def _signal_gzserver(self, requested_signal: signal.Signals) -> None:
        process = self._gzserver
        if process is None or process.poll() is not None:
            return
        if os.name == "posix":
            try:
                os.killpg(process.pid, requested_signal)
            except ProcessLookupError:
                return
        elif requested_signal == signal.SIGKILL:
            process.kill()
        else:
            process.terminate()

    def __enter__(self) -> "GazeboClassicSession":
        try:
            self.start()
        except BaseException:
            self.close()
            raise
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
