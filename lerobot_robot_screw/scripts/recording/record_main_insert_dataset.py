#!/usr/bin/env python
# PROJECT FILE HEADER
# 文件：lerobot_robot_screw/scripts/recording/record_main_insert_dataset.py
# 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
# 用法：请从项目根目录按 README/项目说明.md 中的命令运行；修改参数前先确认单位、设备和输出路径。
# 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
# END PROJECT FILE HEADER

from __future__ import annotations

import argparse
import datetime as dt
import json
import shutil
import sys
import threading
import traceback
import time
from pathlib import Path
from typing import Any

import numpy as np

from lerobot.datasets import LeRobotDataset

from lerobot_robot_screw.dataset import ACTION, OBS_IMAGE, OBS_SCREW_STATE, OBS_STATE, dataset_features
from lerobot_robot_screw.processors import build_observation_state, build_screw_state, synthetic_front_image


REPO_ROOT = Path(__file__).resolve().parents[3]
SCREW_DEMO_DIR = REPO_ROOT / "apps" / "screw_demo"
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
if str(SCREW_DEMO_DIR) not in sys.path:
    sys.path.insert(0, str(SCREW_DEMO_DIR))

from main import (  # noqa: E402
    DEMO_APPROACH_POSE,
    DryRunRobot,
    DemoState,
    format_values,
    manual_align,
    move_joint_to_pose,
    move_linear,
    offset_in_tcp_frame,
    read_key,
    resolve_approach_pose,
    rotation_vector_to_matrix,
    validate_args,
    wait_for_enter_or_abort,
)
from screw_client import make_screw_client  # noqa: E402
from ur_driver import URDriver  # noqa: E402


PHASE_MANUAL_ALIGN = 1
PHASE_INSERT = 2
PHASE_POST_INSERT = 3

MM_TO_M = 0.001


def pose_xyz_mm_to_m(pose: list[float] | tuple[float, ...] | np.ndarray | None):
    if pose is None:
        return None
    converted = np.asarray(pose, dtype=float).copy()
    if converted.shape != (6,):
        raise ValueError(f"--approach-pose must contain 6 values, got {converted.shape}")
    converted[:3] *= MM_TO_M
    return converted.tolist()


def pose_with_xyz_mm(pose: list[float] | tuple[float, ...] | np.ndarray) -> np.ndarray:
    converted = np.asarray(pose, dtype=float).copy()
    converted[:3] /= MM_TO_M
    return converted


def format_pose_mm(pose: list[float] | tuple[float, ...] | np.ndarray) -> str:
    values = pose_with_xyz_mm(pose)
    return "[" + ", ".join(f"{value:.6f}" for value in values) + "]"


def convert_cli_mm_to_robot_m(args):
    args.approach_pose = pose_xyz_mm_to_m(args.approach_pose)
    for name in (
        "jog_step",
        "max_align_offset",
        "insert_depth",
        "max_insert_depth",
        "jog_speed",
        "insert_speed",
        "linear_acc",
        "warn_distance",
        "ik_position_error",
    ):
        setattr(args, name, getattr(args, name) * MM_TO_M)
    return args


class InsertDatasetRecorder:
    def __init__(self, robot, screw_client, screw_lock: threading.Lock, args):
        self.robot = robot
        self.screw_client = screw_client
        self.screw_lock = screw_lock
        self.args = args
        self.period = 1.0 / args.fps
        self.stop_event = threading.Event()
        self.ready_event = threading.Event()
        self.thread: threading.Thread | None = None
        self.error: BaseException | None = None
        self.frame_count = 0
        self.phase_id = PHASE_MANUAL_ALIGN
        self.screw_speed_rpm = 0.0
        self.previous_pose: np.ndarray | None = None
        self._lock = threading.Lock()
        self.raw_output_path: Path | None = args.raw_output_path
        self.raw_file = None

        if args.overwrite and args.dataset_root.exists():
            shutil.rmtree(args.dataset_root)
        if self.raw_output_path is not None:
            self.raw_output_path.parent.mkdir(parents=True, exist_ok=True)
            self.raw_file = self.raw_output_path.open("w", encoding="utf-8")
        self.dataset = LeRobotDataset.create(
            repo_id=args.repo_id,
            fps=args.fps,
            features=dataset_features(args.image_height, args.image_width, args.use_videos),
            root=args.dataset_root,
            robot_type="screw_robot",
            use_videos=args.use_videos,
            image_writer_processes=0,
            image_writer_threads=0,
        )

    def start(self) -> None:
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        if not self.ready_event.wait(timeout=2.0):
            raise RuntimeError("LeRobotDataset recorder did not start within 2 seconds")

    def stop(self, save: bool = True) -> None:
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=5.0)
        if save and self.frame_count > 0:
            self.dataset.save_episode()
            self.dataset.finalize()
        if self.raw_file is not None:
            self.raw_file.close()
            self.raw_file = None
        if self.error is not None:
            raise RuntimeError("LeRobotDataset recorder failed") from self.error

    def set_phase(self, phase_id: int) -> None:
        with self._lock:
            self.phase_id = phase_id

    def set_screw_speed(self, speed_rpm: float) -> None:
        with self._lock:
            self.screw_speed_rpm = speed_rpm

    def _snapshot_command_state(self) -> tuple[int, float]:
        with self._lock:
            return self.phase_id, self.screw_speed_rpm

    def _run(self) -> None:
        next_sample = time.monotonic()
        self.ready_event.set()
        try:
            while not self.stop_event.is_set():
                now = time.monotonic()
                if now < next_sample:
                    time.sleep(min(next_sample - now, 0.002))
                    continue
                next_sample += self.period
                self._record_frame()
        except BaseException as exc:
            self.error = exc
            traceback.print_exc()
            self.stop_event.set()

    def _record_frame(self) -> None:
        phase_id, screw_speed_rpm = self._snapshot_command_state()
        if screw_speed_rpm != 0.0 and hasattr(self.screw_client, "heartbeat"):
            with self.screw_lock:
                self.screw_client.heartbeat()

        tcp_pose = np.asarray(self.robot.get_tcp_pose(), dtype=np.float32)
        tcp_force = np.asarray(self.robot.get_tcp_force(), dtype=np.float32)
        with self.screw_lock:
            response = self.screw_client.status()
        screw_status = self._extract_screw_status(response)
        screw_state = build_screw_state(screw_status)

        if self.previous_pose is None:
            tcp_delta = np.zeros(6, dtype=np.float32)
        else:
            tcp_delta = (tcp_pose - self.previous_pose).astype(np.float32)
        self.previous_pose = tcp_pose.copy()

        action = np.concatenate(
            [tcp_delta, np.asarray([screw_speed_rpm], dtype=np.float32)]
        ).astype(np.float32)
        image = synthetic_front_image(
            self.args.image_height,
            self.args.image_width,
            self.frame_count,
            phase_id,
        )

        observation_state = build_observation_state(
            tcp_pose,
            tcp_force,
            phase_id=phase_id,
            hole_origin_pose=self.args.hole_origin_pose,
        )
        frame = {
            OBS_STATE: observation_state,
            OBS_SCREW_STATE: screw_state,
            OBS_IMAGE: image,
            ACTION: action,
            "phase_id": np.asarray([phase_id], dtype=np.float32),
            "hole_id": self.args.hole_id,
            "task": self.args.task,
        }

        if self.raw_file is not None:
            tcp_pose_mm = pose_with_xyz_mm(tcp_pose)
            tcp_delta_mm = pose_with_xyz_mm(tcp_delta)
            self.raw_file.write(
                json.dumps(
                    {
                        "frame_index": self.frame_count,
                        "timestamp": time.time(),
                        OBS_STATE: observation_state.astype(float).tolist(),
                        OBS_SCREW_STATE: screw_state.astype(float).tolist(),
                        ACTION: action.astype(float).tolist(),
                        "phase_id": phase_id,
                        "hole_id": self.args.hole_id,
                        "tcp_pose": tcp_pose.astype(float).tolist(),
                        "tcp_pose_mm": tcp_pose_mm.astype(float).tolist(),
                        "tcp_delta": tcp_delta.astype(float).tolist(),
                        "tcp_delta_mm": tcp_delta_mm.astype(float).tolist(),
                        "tcp_force": tcp_force.astype(float).tolist(),
                        "screw_status": screw_status,
                    },
                    ensure_ascii=False,
                )
                + "\n"
            )
            self.raw_file.flush()

        self.dataset.add_frame(frame)
        self.frame_count += 1

    @staticmethod
    def _extract_screw_status(response: dict[str, Any]) -> dict[str, Any]:
        if "ok" in response and not response.get("ok", False):
            raise RuntimeError(f"Screw-driver command failed: {response}")
        status = response.get("status", response)
        if not isinstance(status, dict):
            raise RuntimeError(f"Screw-driver response has no status dict: {response}")
        return status


class ScrewHeartbeat:
    def __init__(self, screw_client, screw_lock: threading.Lock, period_s: float = 0.1):
        self.screw_client = screw_client
        self.screw_lock = screw_lock
        self.period_s = period_s
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None
        self.error: BaseException | None = None

    def start(self) -> None:
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=2.0)
        if self.error is not None:
            raise RuntimeError("Screw heartbeat failed") from self.error

    def _run(self) -> None:
        try:
            while not self.stop_event.is_set():
                with self.screw_lock:
                    self.screw_client.heartbeat()
                time.sleep(self.period_s)
        except BaseException as exc:
            self.error = exc
            self.stop_event.set()


def parse_args(argv: list[str] | None = None):
    timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    parser = argparse.ArgumentParser(
        description="Run the main manual-align/insert flow and save one standard LeRobotDataset episode."
    )
    parser.add_argument("--robot-host", default="192.168.1.5")
    parser.add_argument("--enable-robot", action="store_true")
    parser.add_argument(
        "--approach-pose",
        nargs=6,
        type=float,
        metavar=("X", "Y", "Z", "RX", "RY", "RZ"),
        default=None,
        help="approach TCP pose; XYZ in millimeters, RX/RY/RZ in radians",
    )
    parser.add_argument("--use-hardcoded-pose", action="store_true")
    parser.add_argument("--jog-step", type=float, default=0.5, help="manual jog step in millimeters")
    parser.add_argument(
        "--max-align-offset",
        type=float,
        default=10.0,
        help="maximum manual XY offset radius in millimeters",
    )
    parser.add_argument("--insert-depth", type=float, default=2.0, help="insert depth in millimeters")
    parser.add_argument(
        "--max-insert-depth",
        type=float,
        default=400.0,
        help="maximum accepted insert depth in millimeters",
    )
    parser.add_argument("--insert-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--joint-speed", type=float, default=0.15)
    parser.add_argument("--joint-acc", type=float, default=0.15)
    parser.add_argument("--jog-speed", type=float, default=5.0, help="manual jog moveL speed in mm/s")
    parser.add_argument("--insert-speed", type=float, default=3.0, help="insert moveL speed in mm/s")
    parser.add_argument("--linear-acc", type=float, default=30.0, help="moveL acceleration in mm/s^2")
    parser.add_argument("--motion-timeout", type=float, default=20.0)
    parser.add_argument(
        "--warn-distance",
        type=float,
        default=50.0,
        help="ask for extra confirmation when TCP target is farther than this many millimeters",
    )
    parser.add_argument("--ik-position-error", type=float, default=0.1, help="IK position error in millimeters")
    parser.add_argument("--ik-orientation-error", type=float, default=1e-3)

    # Compatibility fields required by screw_demo.main.validate_args().
    parser.set_defaults(record_force=False)
    parser.add_argument("--force-log-hz", type=float, default=50.0)
    parser.add_argument("--force-log-start-timeout", type=float, default=2.0)
    parser.add_argument("--force-log-tail", type=float, default=0.5)
    parser.add_argument("--force-log-success-tail", type=float, default=0.0)
    parser.add_argument("--force-bias-samples", type=int, default=0)
    parser.add_argument("--force-bias-dt", type=float, default=0.02)

    parser.add_argument("--screw-host", default="127.0.0.1")
    parser.add_argument("--screw-port", type=int, default=5055)
    parser.add_argument("--screw-timeout-s", type=float, default=2.0)
    parser.add_argument("--screw-speed-rpm", type=float, default=60.0)
    parser.add_argument("--post-insert-record-s", type=float, default=1.0)

    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=None,
help="dataset output directory; defaults to a timestamped directory under artifacts/datasets",
    )
    parser.add_argument(
        "--repo-id",
        default=None,
        help="LeRobot repo id; defaults to local/real_screw_insert_<timestamp>",
    )
    parser.add_argument("--raw-output-path", type=Path, default=None)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--use-videos", action="store_true")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--image-height", type=int, default=64)
    parser.add_argument("--image-width", type=int, default=64)
    parser.add_argument("--hole-id", default="hole_000")
    parser.add_argument("--task", default="manual align then insert screw with screwdriver")
    args = parser.parse_args(argv)

    if args.dataset_root is None:
        args.dataset_root = REPO_ROOT / "artifacts" / "datasets" / f"real_insert_dataset_{timestamp}"
    if args.repo_id is None:
        args.repo_id = f"local/real_screw_insert_{timestamp}"
    args.hole_origin_pose = None
    if args.raw_output_path is None:
        args.raw_output_path = args.dataset_root.parent / f"{args.dataset_root.name}_raw.jsonl"
    if args.force_bias_dt is None:
        args.force_bias_dt = 1.0 / args.force_log_hz
    return convert_cli_mm_to_robot_m(args)


def run(args, robot=None, screw_client=None) -> DemoState:
    validate_args(args)
    approach_pose = resolve_approach_pose(args)
    if robot is None:
        robot = URDriver(args.robot_host) if args.enable_robot else DryRunRobot()
    if screw_client is None:
        screw_client = make_screw_client(
            host=args.screw_host,
            port=args.screw_port,
            timeout=args.screw_timeout_s,
            dry_run=not args.enable_robot,
        )
    screw_lock = threading.Lock()
    recorder: InsertDatasetRecorder | None = None

    try:
        robot_is_connected = getattr(robot, "connected", False)
        if args.enable_robot and robot_is_connected:
            print("Using preconnected robot...")
        elif args.enable_robot:
            print("Connecting to robot...")
        else:
            print("Starting dry-run robot...")
        if not robot_is_connected:
            if not robot.connect():
                raise RuntimeError("Robot connection failed")
        screw_client.connect()

        print("\n[INIT]")
        print("This script records a LeRobotDataset episode from manual align through insert.")
        print("Current pose [mm, rad]:", format_pose_mm(robot.get_tcp_pose()))
        print("Current joints:", format_values(robot.get_joint_positions()))
        print("Configured approach pose [mm, rad]:", format_pose_mm(approach_pose))
        if not wait_for_enter_or_abort("Confirm the workspace is clear."):
            return DemoState.ABORTED

        if not move_joint_to_pose(robot, approach_pose, args, "MOVE_APPROACH_MOVEJ"):
            return DemoState.ABORTED
        print("Approach pose reached by joint-space motion.")
        if not wait_for_enter_or_abort("Confirm the tool is near the intended hole."):
            return DemoState.ABORTED

        recorder = InsertDatasetRecorder(robot, screw_client, screw_lock, args)
        recorder.set_phase(PHASE_MANUAL_ALIGN)
        recorder.start()
        print(f"LeRobotDataset recording started at manual align: {args.dataset_root}")

        if not manual_align(robot, approach_pose, args):
            return DemoState.ABORTED

        aligned_pose = np.asarray(robot.get_tcp_pose(), dtype=float)
        args.hole_origin_pose = aligned_pose.astype(np.float32)
        local_insert = [0.0, 0.0, args.insert_sign * args.insert_depth]
        insert_target = offset_in_tcp_frame(aligned_pose, local_insert)
        axis = rotation_vector_to_matrix(aligned_pose[3:])[:, 2] * args.insert_sign

        print("\n[INSERT_WITH_SCREWDRIVER]")
        print(f"Insertion depth: {args.insert_depth * 1000.0:.1f} mm")
        print("Insertion direction in robot base frame:", format_values(axis))
        print("Insert target pose [mm, rad]:", format_pose_mm(insert_target))
        print(f"Screwdriver target speed: {args.screw_speed_rpm:.1f} rpm")
        print("Press P to start screwdriver and execute insert moveL, or Q to abort.")
        while True:
            key = read_key()
            if key == "q":
                return DemoState.ABORTED
            if key == "p":
                break

        insert_error: BaseException | None = None
        heartbeat: ScrewHeartbeat | None = None
        recorder.set_phase(PHASE_INSERT)
        recorder.set_screw_speed(args.screw_speed_rpm)
        try:
            with screw_lock:
                screw_client.set_speed(args.screw_speed_rpm)
            heartbeat = ScrewHeartbeat(screw_client, screw_lock)
            heartbeat.start()
            if not move_linear(robot, insert_target, args, speed=args.insert_speed, label="INSERT_MOVEL"):
                insert_error = RuntimeError("Insert moveL failed or timed out")
        except BaseException as exc:
            insert_error = exc
        finally:
            recorder.set_phase(PHASE_POST_INSERT)
            recorder.set_screw_speed(0.0)
            try:
                if heartbeat is not None:
                    heartbeat.stop()
                with screw_lock:
                    screw_client.hold()
            finally:
                if args.post_insert_record_s > 0.0:
                    time.sleep(args.post_insert_record_s)

        if insert_error is not None:
            raise insert_error

        print("\n[DONE]")
        print(f"Recorded {recorder.frame_count} frame(s).")
        print("Final pose [mm, rad]:", format_pose_mm(robot.get_tcp_pose()))
        print("Final joints:", format_values(robot.get_joint_positions()))
        return DemoState.DONE

    except KeyboardInterrupt:
        print("\nCtrl+C received. Stopping motion.")
        robot.stop_linear_motion()
        robot.stop_joint_motion()
        return DemoState.ABORTED
    except Exception as exc:
        print(f"\n[ERROR] {exc}")
        robot.stop_linear_motion()
        robot.stop_joint_motion()
        return DemoState.ERROR
    finally:
        if recorder is not None:
            try:
                recorder.stop(save=recorder.frame_count > 0)
                print(f"Saved LeRobotDataset: {args.dataset_root}")
            except Exception as exc:
                print(f"Recorder stop/save failed: {exc}")
        try:
            with screw_lock:
                screw_client.hold()
        except Exception:
            pass
        if hasattr(screw_client, "close"):
            screw_client.close()
        robot.close()


def main() -> None:
    result = run(parse_args())
    if result not in (DemoState.DONE, DemoState.ABORTED):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
