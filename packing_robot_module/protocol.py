# 클라이언트 요청/응답 한 줄(JSON)의 인코딩·디코딩을 담당한다
from __future__ import annotations

import json
from dataclasses import dataclass


@dataclass
class Request:
    """`{"cmd": ..., "payload": {...}}` 한 줄을 해석한 결과."""

    cmd: str
    payload: dict


@dataclass
class SequenceStep:
    """시퀀스 한 step에서 모션에 실제로 필요한 값만 뽑아낸 결과."""

    step_no: int
    label: str
    pick_pose: list[float]
    release_pose: list[float]


@dataclass
class Sequence:
    """SUBMIT_SEQUENCE payload.sequence에서 모션에 필요한 값만 뽑아낸 결과."""

    job_id: str
    steps: list[SequenceStep]


def parse_request(line: str) -> Request:
    """요청 한 줄을 해석한다. 형식이 다르면 ValueError/KeyError/TypeError."""
    data = json.loads(line)
    cmd = data["cmd"]
    payload = data.get("payload", {})
    if not isinstance(cmd, str) or not isinstance(payload, dict):
        raise ValueError(f"잘못된 요청 형식: {line!r}")
    return Request(cmd=cmd, payload=payload)


def parse_submit_sequence_payload(payload: dict) -> Sequence:
    """SUBMIT_SEQUENCE의 payload(`{"sequence": {...}}`)를 해석한다.

    `status`가 `"ready"`가 아니면 ValueError. 그 외 형식이 다르면
    KeyError/TypeError.
    """
    sequence = payload["sequence"]
    status = sequence["status"]
    if status != "ready":
        raise ValueError(f"sequence.status가 ready가 아닙니다: {status!r} blocked_reasons={sequence.get('blocked_reasons')}")
    steps = []
    for step in sequence["steps"]:
        obj = step["object"]
        label = obj.get("product_id") or obj.get("segment_id") or ""
        steps.append(
            SequenceStep(
                step_no=int(step["step_no"]),
                label=str(label),
                pick_pose=[float(v) for v in step["pick"]["robot_pose"]],
                release_pose=[float(v) for v in step["destination"]["release_pose"]],
            )
        )
    steps.sort(key=lambda s: s.step_no)
    return Sequence(job_id=sequence["job_id"], steps=steps)


def parse_get_sequence_status_payload(payload: dict) -> str:
    """GET_SEQUENCE_STATUS의 payload(`{"job_id": ...}`)를 해석한다. 형식이 다르면 예외를 올린다."""
    job_id = payload["job_id"]
    if not isinstance(job_id, str):
        raise ValueError(f"job_id가 문자열이 아닙니다: {job_id!r}")
    return job_id


def build_response(cmd: str, ok: bool, payload: dict | None = None) -> str:
    """응답 한 줄(JSON)을 만든다: `{"cmd": ..., "status": "ACK"|"ERROR"[, "payload": {...}]}`."""
    data = {"cmd": cmd, "status": "ACK" if ok else "ERROR"}
    if payload is not None:
        data["payload"] = payload
    return json.dumps(data)
