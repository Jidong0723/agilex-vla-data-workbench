"""Batch FK worker run with the pinned NERO kinematics environment."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin


def finite_joint_vector(value: object) -> list[float]:
    if not isinstance(value, list) or len(value) != 7:
        raise ValueError("each joint vector must contain seven values")
    result = [float(item) for item in value]
    if not all(math.isfinite(item) for item in result):
        raise ValueError("joint vector contains a non-finite value")
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--tcp-offset-json", required=True)
    args = parser.parse_args()
    offset = json.loads(args.tcp_offset_json)
    if not isinstance(offset, list) or len(offset) != 3:
        raise ValueError("tcp offset must contain three values")

    model = pin.buildModelFromUrdf(str(args.urdf))
    joint_ids = [model.getJointId(f"joint{index}") for index in range(1, 8)]
    if any(joint_id == 0 for joint_id in joint_ids):
        raise ValueError("URDF is missing NERO joint1 through joint7")
    frame_id = model.addFrame(pin.Frame(
        "nero_tcp_vla_postprocess",
        joint_ids[-1],
        pin.SE3(pin.Quaternion.Identity(), np.asarray(offset, dtype=float)),
        pin.FrameType.OP_FRAME,
    ))
    data = model.createData()
    requests = json.loads(sys.stdin.read())
    if not isinstance(requests, list):
        raise ValueError("expected a list of joint vectors")

    result: list[dict[str, list[float]]] = []
    for joints in requests:
        q = finite_joint_vector(joints)
        configuration = np.zeros(model.nq)
        configuration[:7] = q
        pin.forwardKinematics(model, data, configuration)
        pin.updateFramePlacements(model, data)
        placement = data.oMf[frame_id]
        quaternion = pin.Quaternion(placement.rotation).coeffs().tolist()
        result.append({
            "position_m": placement.translation.tolist(),
            "orientation_xyzw": quaternion,
        })
    print(json.dumps(result, ensure_ascii=False, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
