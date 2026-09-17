# SUBMIT_SEQUENCE를 큐에 적재해 워커 스레드가 순차 실행하고, GET_SEQUENCE_STATUS로
# 조회 가능한 job 상태를 관리한다
from __future__ import annotations

import logging
import queue
import threading
from dataclasses import dataclass
from typing import Callable

from .protocol import (
    Sequence,
    build_response,
    parse_get_sequence_status_payload,
    parse_request,
    parse_submit_sequence_payload,
)

log = logging.getLogger("packing_robot_module.job_handler")

SendFn = Callable[[str], None]


@dataclass
class _StepStatus:
    step_no: int
    status: str  # "pending" | "running" | "done" | "failed"


@dataclass
class _JobState:
    status: str  # "queued" | "running" | "succeeded" | "failed"
    total_steps: int
    step_statuses: list[_StepStatus]
    current_step: int | None = None
    completed_steps: int = 0
    failure_reason: str | None = None

    def to_payload(self, job_id: str) -> dict:
        return {
            "job_id": job_id,
            "status": self.status,
            "current_step": self.current_step,
            "completed_steps": self.completed_steps,
            "total_steps": self.total_steps,
            "failure_reason": self.failure_reason,
            "step_statuses": [{"step_no": s.step_no, "status": s.status} for s in self.step_statuses],
        }


class JobHandler:
    """서버가 받은 한 줄(JSON)을 해석해 로봇을 움직이고 ACK/ERROR 응답을 보낸다.

    `SUBMIT_SEQUENCE`는 즉시 queued ACK만 보내고 시퀀스를 큐에 넣는다. 실행은 생성 시
    시작되는 상시 워커 스레드가 큐에서 하나씩 꺼내 순차(로봇이 하나뿐이므로 직렬)로
    처리하며, `GET_SEQUENCE_STATUS`가 조회하는 job 상태를 갱신한다.
    """

    def __init__(self, cfg: dict, robot: object) -> None:
        self.cfg = cfg
        self.robot = robot
        self._lock = threading.Lock()
        self._jobs: dict[str, _JobState] = {}
        self._queue: queue.Queue[Sequence] = queue.Queue()
        self._worker = threading.Thread(target=self._worker_loop, name="job-worker", daemon=True)
        self._worker.start()

    def handle_line(self, line: str, send: SendFn) -> None:
        try:
            request = parse_request(line=line)
        except (ValueError, KeyError, TypeError) as exc:
            log.error("요청 해석 실패: %s | line=%r", exc, line)
            return
        if request.cmd == "SUBMIT_SEQUENCE":
            self._handle_submit_sequence(payload=request.payload, send=send)
        elif request.cmd == "GET_SEQUENCE_STATUS":
            self._handle_get_sequence_status(payload=request.payload, send=send)
        else:
            log.warning("알 수 없는 cmd: %r", request.cmd)

    def wait_idle(self) -> None:
        """큐에 적재된 모든 job이 처리될 때까지 대기한다 (테스트용)."""
        self._queue.join()

    def move_home(self) -> bool:
        """motion.home_joints_deg로 이동한다. 서버 시작 시와 시퀀스의 모든 step 처리가 끝난 뒤 호출된다."""
        motion = self.cfg["motion"]
        if not self.robot.movej(joints_deg=motion["home_joints_deg"], vel=motion["joint_vel"], acc=motion["joint_acc"]):
            return self._fail(what="movej(home)")
        log.info("home 이동 완료.")
        return True

    # ------------------------------------------------------------- SUBMIT_SEQUENCE

    def _handle_submit_sequence(self, payload: dict, send: SendFn) -> None:
        try:
            sequence = parse_submit_sequence_payload(payload=payload)
        except (ValueError, KeyError, TypeError) as exc:
            log.error("SUBMIT_SEQUENCE payload 해석 실패: %s | payload=%r", exc, payload)
            send(build_response(cmd="SUBMIT_SEQUENCE", ok=False))
            return
        with self._lock:
            self._jobs[sequence.job_id] = _JobState(
                status="queued",
                total_steps=len(sequence.steps),
                step_statuses=[_StepStatus(step_no=step.step_no, status="pending") for step in sequence.steps],
            )
        self._queue.put(sequence)
        log.info("SUBMIT_SEQUENCE 수신: job_id=%s, %d개 step 큐잉", sequence.job_id, len(sequence.steps))
        send(build_response(cmd="SUBMIT_SEQUENCE", ok=True, payload={"job_id": sequence.job_id, "status": "queued"}))

    def _worker_loop(self) -> None:
        while True:
            sequence = self._queue.get()
            try:
                self._run_sequence(sequence=sequence)
            except Exception:
                log.exception("job_id=%s 처리 중 예상치 못한 예외가 발생했습니다.", sequence.job_id)
                self._update_job(job_id=sequence.job_id, status="failed", failure_reason="예상치 못한 예외")
            finally:
                self._queue.task_done()

    def _run_sequence(self, sequence: Sequence) -> None:
        self._update_job(job_id=sequence.job_id, status="running")
        for step in sequence.steps:
            self._update_job(job_id=sequence.job_id, current_step=step.step_no)
            self._set_step_status(job_id=sequence.job_id, step_no=step.step_no, status="running")
            if not self._pick_and_place(pick_pose=step.pick_pose, release_pose=step.release_pose):
                log.error(
                    "job_id=%s step_no=%d(%s) 처리 실패, 남은 step 처리를 중단합니다.",
                    sequence.job_id, step.step_no, step.label,
                )
                self._set_step_status(job_id=sequence.job_id, step_no=step.step_no, status="failed")
                self._update_job(
                    job_id=sequence.job_id, status="failed", failure_reason=f"step_no={step.step_no} 모션 실패",
                )
                return
            self._mark_step_done(job_id=sequence.job_id, step_no=step.step_no)
        self._update_job(job_id=sequence.job_id, status="succeeded")
        log.info("job_id=%s 모든 step 완료", sequence.job_id)
        self.move_home()

    def _pick_and_place(self, pick_pose: list[float], release_pose: list[float]) -> bool:
        """approach1 -> approach2 -> pick_approach -> pick -> gripper on -> retreat -> release_pose -> gripper off."""
        motion = self.cfg["motion"]
        gripper_cfg = self.cfg["gripper"]
        joint_vel, joint_acc = motion["joint_vel"], motion["joint_acc"]
        line_vel, line_acc = motion["line_vel"], motion["line_acc"]

        # pick pose의 tool z축(회전 반영) 기준으로 계산하는 최종 접근점. approach1/2와 달리
        # base 축 offset이 아니라 pose 회전을 반영해야 해서 trans()를 쓴다.
        pick_approach = self.robot.offset_along_tool(pose6=pick_pose, delta6=motion["pick_approach_tool_base"])
        if pick_approach is None:
            return self._fail(what="trans(pick_approach)")

        temp_pick_approach1 = list(pick_approach)
        temp_pick_approach1[1] += motion["temp_pick_approach_y_offset_mm"]
        temp_pick_approach1[2] += motion["temp_pick_approach_z_offset_mm"]

        temp_pick_approach2 = list(pick_approach)
        temp_pick_approach2[2] += motion["temp_pick_approach_z_offset_mm"]

        log.info(f"Approach 1로 jx 이동: {temp_pick_approach1}")
        if not self.robot.movejx(pose6=temp_pick_approach1, vel=joint_vel, acc=joint_acc):
            return self._fail(what="movejx(approach1)")

        log.info(f"Approach 2로 linear 이동: {temp_pick_approach2}")
        if not self.robot.movel(pose6=temp_pick_approach2, vel=line_vel, acc=line_acc):
            return self._fail(what="movel(approach2)")

        log.info(f"Pick Approach로 linear 이동: {pick_approach}")
        if not self.robot.movel(pose6=pick_approach, vel=line_vel, acc=line_acc):
            return self._fail(what="movel(pick_approach)")

        log.info(f"Pick으로 linear 이동: {pick_pose}")
        if not self.robot.movel(pose6=pick_pose, vel=line_vel, acc=line_acc):
            return self._fail(what="movel(pick)")

        log.info(f"Gripper On")
        if not self.robot.gripper(on=True, io_index=gripper_cfg["io_index"], settle_sec=gripper_cfg["settle_sec"]):
            return self._fail(what="gripper(on)")

        log.info(f"Pick Approach로 linear 이동: {pick_approach}")
        if not self.robot.movel(pose6=pick_approach, vel=line_vel, acc=line_acc):
            return self._fail(what="movel(retreat pick_approach)")

        log.info(f"Approach 2로 linear 이동: {temp_pick_approach2}")
        if not self.robot.movel(pose6=temp_pick_approach2, vel=line_vel, acc=line_acc):
            return self._fail(what="movel(retreat approach2)")

        log.info(f"Approach 1로 linear 이동: {temp_pick_approach1}")
        if not self.robot.movel(pose6=temp_pick_approach1, vel=line_vel, acc=line_acc):
            return self._fail(what="movel(retreat approach1)")

        log.info(f"Release로 jx 이동: {release_pose}")
        if not self.robot.movejx(pose6=release_pose, vel=joint_vel, acc=joint_acc):
            return self._fail(what="movejx(release_pose)")

        log.info(f"Gripper Off")
        if not self.robot.gripper(on=False, io_index=gripper_cfg["io_index"], settle_sec=gripper_cfg["settle_sec"]):
            return self._fail(what="gripper(off)")
        return True

    def _fail(self, what: str) -> bool:
        log.error("%s 실패.", what)
        return False

    # ------------------------------------------------------------- GET_SEQUENCE_STATUS

    def _handle_get_sequence_status(self, payload: dict, send: SendFn) -> None:
        try:
            job_id = parse_get_sequence_status_payload(payload=payload)
        except (ValueError, KeyError, TypeError) as exc:
            log.error("GET_SEQUENCE_STATUS payload 해석 실패: %s | payload=%r", exc, payload)
            send(build_response(cmd="GET_SEQUENCE_STATUS", ok=False))
            return
        with self._lock:
            state = self._jobs.get(job_id)
            response_payload = state.to_payload(job_id=job_id) if state is not None else None
        if response_payload is None:
            log.warning("알 수 없는 job_id: %r", job_id)
            send(build_response(cmd="GET_SEQUENCE_STATUS", ok=False))
            return
        send(build_response(cmd="GET_SEQUENCE_STATUS", ok=True, payload=response_payload))

    # ------------------------------------------------------------- job 상태 갱신 (락으로 보호)

    def _update_job(self, job_id: str, **kwargs) -> None:
        with self._lock:
            state = self._jobs[job_id]
            for key, value in kwargs.items():
                setattr(state, key, value)

    def _set_step_status(self, job_id: str, step_no: int, status: str) -> None:
        with self._lock:
            for step_status in self._jobs[job_id].step_statuses:
                if step_status.step_no == step_no:
                    step_status.status = status
                    return

    def _mark_step_done(self, job_id: str, step_no: int) -> None:
        with self._lock:
            state = self._jobs[job_id]
            for step_status in state.step_statuses:
                if step_status.step_no == step_no:
                    step_status.status = "done"
                    break
            state.completed_steps += 1
