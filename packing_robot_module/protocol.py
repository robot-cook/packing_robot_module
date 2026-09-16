# 클라이언트 요청/응답 한 줄(JSON)의 인코딩·디코딩을 담당한다
from __future__ import annotations

import json
from dataclasses import dataclass


@dataclass
class JobItem:
    """REQ_JOB payload.tasks에 담긴 상품 하나 (product_id + 6-DOF pose)."""

    product_id: str
    pose: list[float]


@dataclass
class Request:
    """`{"cmd": ..., "payload": {...}}` 한 줄을 해석한 결과."""

    cmd: str
    payload: dict


def parse_request(line: str) -> Request:
    """요청 한 줄을 해석한다. 형식이 다르면 ValueError/KeyError/TypeError."""
    data = json.loads(line)
    cmd = data["cmd"]
    payload = data.get("payload", {})
    if not isinstance(cmd, str) or not isinstance(payload, dict):
        raise ValueError(f"잘못된 요청 형식: {line!r}")
    return Request(cmd=cmd, payload=payload)


def parse_req_job_payload(payload: dict) -> list[JobItem]:
    """REQ_JOB의 payload(`{"tasks": [...]}`)를 해석한다. 형식이 다르면 예외를 올린다."""
    tasks = payload["tasks"]
    return [
        JobItem(product_id=task["product_id"], pose=[float(v) for v in task["pose"]])
        for task in tasks
    ]


def parse_set_pack_pose_payload(payload: dict) -> list[float]:
    """SET_PACK_POSE의 payload(`{"pose": [...]}`)를 해석한다. 형식이 다르면 예외를 올린다."""
    return [float(v) for v in payload["pose"]]


def build_response(cmd: str, ok: bool) -> str:
    """응답 한 줄(JSON)을 만든다: `{"cmd": ..., "status": "ACK"|"ERROR"}`."""
    return json.dumps({"cmd": cmd, "status": "ACK" if ok else "ERROR"})
