"""Local-only server for VLA Data Workbench.

It serves the static page and exports labeled raw Episode segments without
allowing arbitrary filesystem paths from the browser.
"""
from __future__ import annotations

import json
import re
import shutil
from bisect import bisect_left
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import imageio_ffmpeg
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
from PIL import Image

APP_ROOT = Path(__file__).resolve().parent
SOURCE_ROOT = Path(r"G:\codex-yufan\dataset\episodes").resolve()
OUTPUT_ROOT = Path(r"G:\codex-yufan\after data processing").resolve()
LEROBOT_ROOT = Path(r"G:\codex-yufan\LeRobot Dataset").resolve()
EPISODE_NAME = re.compile(r"^episode_\d+$")
RATE_HZ = 15.0


def read_jsonl(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def write_jsonl(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n" for row in rows), encoding="utf-8")


def copy_image(source: Path, target: Path, relative: str) -> None:
    candidate = (source / relative).resolve()
    if SOURCE_ROOT not in candidate.parents or not candidate.is_file():
        raise ValueError(f"image is unavailable: {relative}")
    destination = target / relative
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(candidate, destination)


def output_metadata(metadata: dict, source_name: str, process: dict, start_ns: int, end_ns: int,
                    camera_rows: list[dict], robot_rows: list[dict]) -> dict:
    result = dict(metadata)
    by_source = {source: sum(1 for row in camera_rows if row.get("source") == source) for source in ("external", "wrist")}
    result.update({
        "dataset_stage": "postprocessed_episode_segment.v1",
        "parent_episode": source_name,
        "task": process["title"],
        "prompt": process["title"],
        "segment_order": [process["title"]],
        "segments": [{"title": process["title"], "start_frame": process["start_frame"], "end_frame": process["end_frame"],
                      "tags": process.get("tags", [])}],
        "collection_monotonic_ns": {"start": start_ns, "end": end_ns},
        "duration_s": max(0.0, (end_ns - start_ns) / 1e9),
        "raw_collection": False,
        "training_view_generated": False,
        "horizon_windows_built": False,
        "sensor_frames": None,
        "control_steps": None,
        "raw_streams": {
            **(metadata.get("raw_streams") or {}),
            "camera_frames": min(by_source.values()),
            "robot_states": len(robot_rows),
            "camera_frames_by_source": by_source,
            "camera_total_frames": len(camera_rows),
            "camera_pairing": "deferred_to_postprocessing",
        },
    })
    return result


def export_segments(payload: dict) -> list[dict]:
    source_name = str(payload.get("source_episode") or "")
    processes = payload.get("processes")
    if not EPISODE_NAME.fullmatch(source_name):
        raise ValueError("invalid source episode name")
    if not isinstance(processes, list) or not processes:
        raise ValueError("at least one labeled process is required")
    source = (SOURCE_ROOT / source_name).resolve()
    if source.parent != SOURCE_ROOT or not source.is_dir():
        raise ValueError("source episode was not found in dataset/episodes")
    metadata = json.loads((source / "episode.json").read_text(encoding="utf-8"))
    files = metadata.get("files") or {}
    camera_rows = read_jsonl(source / files["raw_camera"])
    robot_rows = read_jsonl(source / files["raw_robot_state"])
    bounds = metadata.get("collection_monotonic_ns") or {}
    lower, upper = int(bounds["start"]), int(bounds["end"])
    camera_rows = [row for row in camera_rows if lower <= int(row["capture_monotonic_ns"]) <= upper]
    robot_rows = [row for row in robot_rows if lower <= int(row["feedback_monotonic_ns"]) <= upper]
    camera_starts = {source: min(int(row["capture_monotonic_ns"]) for row in camera_rows if row.get("source") == source)
                     for source in ("external", "wrist")}
    camera_ends = {source: max(int(row["capture_monotonic_ns"]) for row in camera_rows if row.get("source") == source)
                   for source in ("external", "wrist")}
    start_ns = max(*camera_starts.values(), int(robot_rows[0]["feedback_monotonic_ns"]))
    end_ns = min(*camera_ends.values(), int(robot_rows[-1]["feedback_monotonic_ns"]))
    period_ns = round(1e9 / RATE_HZ)
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    prepared = []
    for ordinal, process in enumerate(processes, 1):
        if not isinstance(process, dict) or not isinstance(process.get("title"), str):
            raise ValueError("each process needs a title")
        left, right = int(process.get("start_frame", -1)), int(process.get("end_frame", -1))
        if left < 0 or right < left:
            raise ValueError("invalid process frame range")
        segment_start, segment_end = start_ns + left * period_ns, start_ns + right * period_ns
        if segment_end > end_ns:
            raise ValueError("process range is outside the source episode")
        target = OUTPUT_ROOT / f"{source_name}.{ordinal}"
        if target.exists():
            raise FileExistsError(f"{target.name} already exists; it was not overwritten")
        selected_cameras = [row for row in camera_rows if segment_start <= int(row["capture_monotonic_ns"]) <= segment_end]
        selected_robot = [row for row in robot_rows if segment_start <= int(row["feedback_monotonic_ns"]) <= segment_end]
        if not selected_cameras or not selected_robot:
            raise ValueError(f"{target.name} has no raw data in its selected range")
        prepared.append((target, process, segment_start, segment_end, selected_cameras, selected_robot))
    created = []
    for target, process, segment_start, segment_end, selected_cameras, selected_robot in prepared:
        target.mkdir()
        interface = source / "raw" / "interface"
        if interface.is_dir():
            shutil.copytree(interface, target / "raw" / "interface")
        for row in selected_cameras:
            copy_image(source, target, str(row["image"]))
        write_jsonl(target / files["raw_camera"], selected_cameras)
        write_jsonl(target / files["raw_robot_state"], selected_robot)
        segment_metadata = output_metadata(metadata, source_name, process, segment_start, segment_end, selected_cameras, selected_robot)
        (target / "episode.json").write_text(json.dumps(segment_metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        created.append({"name": target.name, "path": str(target), "frames": [process["start_frame"], process["end_frame"]]})
    return created


def _nearest(rows: list[dict], times: list[int], target: int) -> tuple[dict, int]:
    index = bisect_left(times, target)
    choices = [candidate for candidate in (index - 1, index) if 0 <= candidate < len(times)]
    selected = min(choices, key=lambda candidate: abs(times[candidate] - target))
    return rows[selected], abs(times[selected] - target)


def _interpolate_robot(rows: list[dict], times: list[int], target: int) -> dict | None:
    right_index = bisect_left(times, target)
    if right_index <= 0 or right_index >= len(times):
        return None
    left, right = rows[right_index - 1], rows[right_index]
    left_time, right_time = times[right_index - 1], times[right_index]
    span = right_time - left_time
    if span <= 0:
        return None
    alpha = (target - left_time) / span

    def blend(key: str) -> list[float]:
        return [(1 - alpha) * float(value) + alpha * float(right.get(key, [])[i] if i < len(right.get(key, [])) else value)
                for i, value in enumerate(left.get(key, []))]

    return {
        "joint_position_rad": blend("joint_position_rad"),
        "gripper_opening_ratio": (1 - alpha) * float(left.get("gripper_opening_ratio", 0)) + alpha * float(right.get("gripper_opening_ratio", 0)),
        "left_time": left_time,
        "right_time": right_time,
    }


def aligned_rows(metadata: dict, camera_rows: list[dict], robot_rows: list[dict]) -> list[dict]:
    """Rebuild the same valid 15 Hz view shown by the browser, on the server."""
    bounds = metadata.get("collection_monotonic_ns") or {}
    lower, upper = int(bounds["start"]), int(bounds["end"])
    streams = {
        source: sorted((row for row in camera_rows if row.get("source") == source and lower <= int(row["capture_monotonic_ns"]) <= upper),
                       key=lambda row: int(row["capture_monotonic_ns"]))
        for source in ("external", "wrist")
    }
    robots = sorted((row for row in robot_rows if lower <= int(row["feedback_monotonic_ns"]) <= upper),
                    key=lambda row: int(row["feedback_monotonic_ns"]))
    if not streams["external"] or not streams["wrist"] or not robots:
        raise ValueError("source episode has incomplete camera or robot streams")
    camera_times = {source: [int(row["capture_monotonic_ns"]) for row in rows] for source, rows in streams.items()}
    robot_times = [int(row["feedback_monotonic_ns"]) for row in robots]
    start = max(camera_times["external"][0], camera_times["wrist"][0], robot_times[0])
    finish = min(camera_times["external"][-1], camera_times["wrist"][-1], robot_times[-1])
    period = round(1e9 / RATE_HZ)
    result = []
    for target in range(start, finish + 1, period):
        external, external_error = _nearest(streams["external"], camera_times["external"], target)
        wrist, wrist_error = _nearest(streams["wrist"], camera_times["wrist"], target)
        robot = _interpolate_robot(robots, robot_times, target)
        if (robot is None or max(external_error, wrist_error) > 30_000_000
                or max(target - robot["left_time"], robot["right_time"] - target) > 35_000_000):
            continue
        result.append({
            "target_ns": target,
            "external_image": external["image"],
            "wrist_image": wrist["image"],
            "state": [*robot["joint_position_rad"], robot["gripper_opening_ratio"]],
        })
    if not result:
        raise ValueError("source episode has no valid 15 Hz aligned rows")
    return result


def _write_video(path: Path, image_paths: list[Path]) -> None:
    if not image_paths:
        raise ValueError("a LeRobot video cannot be empty")
    with Image.open(image_paths[0]) as first:
        rgb = first.convert("RGB")
        size = rgb.size
    writer = imageio_ffmpeg.write_frames(str(path), size, fps=RATE_HZ, codec="libx264", pix_fmt_in="rgb24",
                                         output_params=["-pix_fmt", "yuv420p", "-crf", "23"])
    writer.send(None)
    try:
        for image_path in image_paths:
            with Image.open(image_path) as image:
                frame = np.asarray(image.convert("RGB"), dtype=np.uint8)
            if (frame.shape[1], frame.shape[0]) != size:
                raise ValueError(f"image size differs from first frame: {image_path.name}")
            writer.send(frame.tobytes())
    finally:
        writer.close()


def _feature_stats(values: list[list[float]]) -> dict:
    matrix = np.asarray(values, dtype=np.float32)
    return {"min": matrix.min(axis=0).tolist(), "max": matrix.max(axis=0).tolist(),
            "mean": matrix.mean(axis=0).tolist(), "std": matrix.std(axis=0).tolist()}


def export_lerobot_dataset(payload: dict) -> dict:
    source_name = str(payload.get("source_episode") or "")
    processes = payload.get("processes")
    if not EPISODE_NAME.fullmatch(source_name):
        raise ValueError("invalid source episode name")
    if not isinstance(processes, list) or not processes:
        raise ValueError("at least one labeled process is required")
    source = (SOURCE_ROOT / source_name).resolve()
    if source.parent != SOURCE_ROOT or not source.is_dir():
        raise ValueError("source episode was not found in dataset/episodes")
    metadata = json.loads((source / "episode.json").read_text(encoding="utf-8"))
    files = metadata.get("files") or {}
    aligned = aligned_rows(metadata, read_jsonl(source / files["raw_camera"]), read_jsonl(source / files["raw_robot_state"]))
    target = LEROBOT_ROOT / f"{source_name}_lerobot"
    if target.exists():
        raise FileExistsError(f"{target.name} already exists; it was not overwritten")
    prepared = []
    for episode_index, process in enumerate(processes):
        if not isinstance(process, dict) or not isinstance(process.get("title"), str) or not process["title"].strip():
            raise ValueError("each process needs a title")
        left, right = int(process.get("start_frame", -1)), int(process.get("end_frame", -1))
        if left < 0 or right < left or right >= len(aligned):
            raise ValueError("process range is outside the valid 15 Hz view")
        rows = aligned[left:right + 1]
        if len(rows) < 2:
            raise ValueError("each LeRobot episode needs at least two valid 15 Hz frames")
        prepared.append((episode_index, process, rows))
    # All validation happens before the new output folder is created.
    data_dir = target / "data" / "chunk-000"
    video_dir = target / "videos" / "chunk-000"
    data_dir.mkdir(parents=True)
    task_titles = list(dict.fromkeys(process["title"].strip() for _, process, _ in prepared))
    task_index = {title: index for index, title in enumerate(task_titles)}
    all_states, all_actions = [], []
    episode_records = []
    for episode_index, process, rows in prepared:
        title = process["title"].strip()
        states = [row["state"] for row in rows]
        actions = states[1:] + [states[-1]]  # next 15 Hz desired joint/gripper state; final action is held.
        all_states.extend(states)
        all_actions.extend(actions)
        rel_external = f"videos/chunk-000/observation.images.external/episode_{episode_index:06d}.mp4"
        rel_wrist = f"videos/chunk-000/observation.images.wrist/episode_{episode_index:06d}.mp4"
        external_path, wrist_path = target / rel_external, target / rel_wrist
        external_path.parent.mkdir(parents=True, exist_ok=True)
        wrist_path.parent.mkdir(parents=True, exist_ok=True)
        _write_video(external_path, [(source / row["external_image"]).resolve() for row in rows])
        _write_video(wrist_path, [(source / row["wrist_image"]).resolve() for row in rows])
        timestamps = [frame / RATE_HZ for frame in range(len(rows))]
        video_type = pa.struct([("path", pa.string()), ("timestamp", pa.float64())])
        table = pa.table({
            "observation.state": pa.array(states, type=pa.list_(pa.float32(), 8)),
            "action": pa.array(actions, type=pa.list_(pa.float32(), 8)),
            "observation.images.external": pa.array([{"path": rel_external, "timestamp": stamp} for stamp in timestamps], type=video_type),
            "observation.images.wrist": pa.array([{"path": rel_wrist, "timestamp": stamp} for stamp in timestamps], type=video_type),
            "timestamp": pa.array(timestamps, type=pa.float64()),
            "frame_index": pa.array(list(range(len(rows))), type=pa.int64()),
            "episode_index": pa.array([episode_index] * len(rows), type=pa.int64()),
            "index": pa.array(list(range(sum(len(item[2]) for item in prepared[:episode_index]), sum(len(item[2]) for item in prepared[:episode_index + 1]))), type=pa.int64()),
            "task_index": pa.array([task_index[title]] * len(rows), type=pa.int64()),
        })
        pq.write_table(table, data_dir / f"episode_{episode_index:06d}.parquet", compression="zstd")
        episode_records.append({"episode_index": episode_index, "tasks": [title], "length": len(rows)})
    features = {
        "observation.state": {"dtype": "float32", "shape": [8], "names": ["joint_1.pos", "joint_2.pos", "joint_3.pos", "joint_4.pos", "joint_5.pos", "joint_6.pos", "joint_7.pos", "gripper.opening_ratio"]},
        "action": {"dtype": "float32", "shape": [8], "names": ["joint_1.pos.next", "joint_2.pos.next", "joint_3.pos.next", "joint_4.pos.next", "joint_5.pos.next", "joint_6.pos.next", "joint_7.pos.next", "gripper.opening_ratio.next"]},
        "observation.images.external": {"dtype": "video", "shape": [480, 640, 3], "names": ["height", "width", "channel"]},
        "observation.images.wrist": {"dtype": "video", "shape": [480, 640, 3], "names": ["height", "width", "channel"]},
        "timestamp": {"dtype": "float64", "shape": [1], "names": None},
        "frame_index": {"dtype": "int64", "shape": [1], "names": None},
        "episode_index": {"dtype": "int64", "shape": [1], "names": None},
        "index": {"dtype": "int64", "shape": [1], "names": None},
        "task_index": {"dtype": "int64", "shape": [1], "names": None},
    }
    meta = target / "meta"
    meta.mkdir()
    (meta / "info.json").write_text(json.dumps({"codebase_version": "v2.1", "robot_type": "agilex_nero", "fps": RATE_HZ,
        "total_frames": len(all_states), "total_episodes": len(prepared), "total_tasks": len(task_titles), "total_videos": len(prepared) * 2,
        "splits": {"train": f"0:{len(prepared)}"}, "data_path": "data/chunk-{chunk_index:03d}/episode_{episode_index:06d}.parquet",
        "video_path": "videos/chunk-{chunk_index:03d}/{video_key}/episode_{episode_index:06d}.mp4", "features": features}, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    (meta / "tasks.jsonl").write_text("".join(json.dumps({"task_index": index, "task": title}, ensure_ascii=False) + "\n" for index, title in enumerate(task_titles)), encoding="utf-8")
    (meta / "episodes.jsonl").write_text("".join(json.dumps(record, ensure_ascii=False) + "\n" for record in episode_records), encoding="utf-8")
    (meta / "stats.json").write_text(json.dumps({"observation.state": _feature_stats(all_states), "action": _feature_stats(all_actions)}, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return {"name": target.name, "path": str(target), "episodes": len(prepared), "frames": len(all_states)}


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(APP_ROOT), **kwargs)

    def send_json(self, status: int, value: dict) -> None:
        payload = json.dumps(value, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def do_POST(self) -> None:
        if self.path not in {"/api/export-episodes", "/api/export-lerobot"}:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > 2_000_000:
                raise ValueError("invalid request body")
            payload = json.loads(self.rfile.read(length).decode("utf-8"))
            if self.path == "/api/export-episodes":
                self.send_json(HTTPStatus.OK, {"created": export_segments(payload)})
            else:
                self.send_json(HTTPStatus.OK, {"created": export_lerobot_dataset(payload)})
        except FileExistsError as error:
            self.send_json(HTTPStatus.CONFLICT, {"error": str(error)})
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self.send_json(HTTPStatus.BAD_REQUEST, {"error": str(error)})
        except Exception:
            self.send_json(HTTPStatus.INTERNAL_SERVER_ERROR, {"error": "local export failed; no existing Episode was overwritten"})


if __name__ == "__main__":
    ThreadingHTTPServer(("127.0.0.1", 8790), Handler).serve_forever()
