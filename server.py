"""Local-only server for VLA Data Workbench.

It serves the static page and exports labeled raw Episode segments without
allowing arbitrary filesystem paths from the browser.
"""
from __future__ import annotations

import json
import re
import shutil
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

APP_ROOT = Path(__file__).resolve().parent
SOURCE_ROOT = Path(r"G:\codex-yufan\dataset\episodes").resolve()
OUTPUT_ROOT = Path(r"G:\codex-yufan\after data processing").resolve()
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
        if self.path != "/api/export-episodes":
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > 2_000_000:
                raise ValueError("invalid request body")
            created = export_segments(json.loads(self.rfile.read(length).decode("utf-8")))
            self.send_json(HTTPStatus.OK, {"created": created})
        except FileExistsError as error:
            self.send_json(HTTPStatus.CONFLICT, {"error": str(error)})
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self.send_json(HTTPStatus.BAD_REQUEST, {"error": str(error)})
        except Exception:
            self.send_json(HTTPStatus.INTERNAL_SERVER_ERROR, {"error": "local export failed; no existing Episode was overwritten"})


if __name__ == "__main__":
    ThreadingHTTPServer(("127.0.0.1", 8790), Handler).serve_forever()
