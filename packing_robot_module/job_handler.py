# REQ_JOB/SET_PACK_POSE 요청을 로봇 모션으로 옮기고 config.yaml의 default_packing_pose를 갱신한다
from __future__ import annotations

import logging
import re
from typing import Callable

from .protocol import build_response, parse_req_job_payload, parse_request, parse_set_pack_pose_payload

log = logging.getLogger("packing_robot_module.job_handler")

_PACK_POSE_LINE_RE = re.compile(r"^default_packing_pose:.*$", re.MULTILINE)

SendFn = Callable[[str], None]


class JobHandler:
    """서버가 받은 한 줄(JSON)을 해석해 로봇을 움직이고 ACK/ERROR 응답을 보낸다."""

    def __init__(self, cfg: dict, robot: object, config_path: str) -> None:
        self.cfg = cfg
        self.robot = robot
        self._config_path = config_path

    def handle_line(self, line: str, send: SendFn) -> None:
        try:
            request = parse_request(line=line)
        except (ValueError, KeyError, TypeError) as exc:
            log.error("요청 해석 실패: %s | line=%r", exc, line)
            return
        if request.cmd == "REQ_JOB":
            self._handle_req_job(payload=request.payload, send=send)
        elif request.cmd == "SET_PACK_POSE":
            self._handle_set_pack_pose(payload=request.payload, send=send)
        else:
            log.warning("알 수 없는 cmd: %r", request.cmd)

    # ------------------------------------------------------------- REQ_JOB

    def _handle_req_job(self, payload: dict, send: SendFn) -> None:
        """JSON 해석에 성공하면 즉시 ACK를 보내고, 이후 상품별 모션 실패는 로그로만 남긴다."""
        try:
            items = parse_req_job_payload(payload=payload)
        except (ValueError, KeyError, TypeError) as exc:
            log.error("REQ_JOB payload 해석 실패: %s | payload=%r", exc, payload)
            send(build_response(cmd="REQ_JOB", ok=False))
            return
        send(build_response(cmd="REQ_JOB", ok=True))
        log.info("REQ_JOB 수신: %d개 상품", len(items))
        for item in items:
            if not self._pick_and_place(pose=item.pose):
                log.error("상품 %s 처리 실패, 남은 상품 처리를 중단합니다.", item.product_id)
                return
        self.move_home()

    def move_home(self) -> bool:
        """motion.home_joints_deg로 이동한다. 서버 시작 시와 REQ_JOB의 모든 상품 처리가 끝난 뒤 호출된다."""
        motion = self.cfg["motion"]
        if not self.robot.movej(joints_deg=motion["home_joints_deg"], vel=motion["joint_vel"], acc=motion["joint_acc"]):
            return self._fail(what="movej(home)")
        log.info("home 이동 완료.")
        return True

    def _pick_and_place(self, pose: list[float]) -> bool:
        """approach1 -> approach2 -> pick -> gripper on -> retreat -> packing pose -> gripper off."""
        motion = self.cfg["motion"]
        gripper_cfg = self.cfg["gripper"]
        joint_vel, joint_acc = motion["joint_vel"], motion["joint_acc"]
        line_vel, line_acc = motion["line_vel"], motion["line_acc"]

        approach1 = list(pose)
        approach1[1] += motion["pick_approach1_y_offset_mm"]
        approach1[2] += motion["pick_approach1_z_offset_mm"]
        approach2 = list(pose)
        approach2[2] += motion["pick_approach2_z_offset_mm"]
        packing_pose = self.cfg["default_packing_pose"]

        if not self.robot.movejx(pose6=approach1, vel=joint_vel, acc=joint_acc):
            return self._fail(what="movejx(approach1)")
        if not self.robot.movel(pose6=approach2, vel=line_vel, acc=line_acc):
            return self._fail(what="movel(approach2)")
        if not self.robot.movel(pose6=pose, vel=line_vel, acc=line_acc):
            return self._fail(what="movel(pick)")
        if not self.robot.gripper(on=True, io_index=gripper_cfg["io_index"], settle_sec=gripper_cfg["settle_sec"]):
            return self._fail(what="gripper(on)")
        if not self.robot.movel(pose6=approach2, vel=line_vel, acc=line_acc):
            return self._fail(what="movel(retreat approach2)")
        if not self.robot.movel(pose6=approach1, vel=line_vel, acc=line_acc):
            return self._fail(what="movel(retreat approach1)")
        if not self.robot.movejx(pose6=packing_pose, vel=joint_vel, acc=joint_acc):
            return self._fail(what="movejx(packing_pose)")
        if not self.robot.gripper(on=False, io_index=gripper_cfg["io_index"], settle_sec=gripper_cfg["settle_sec"]):
            return self._fail(what="gripper(off)")
        return True

    def _fail(self, what: str) -> bool:
        log.error("%s 실패.", what)
        return False

    # ------------------------------------------------------------- SET_PACK_POSE

    def _handle_set_pack_pose(self, payload: dict, send: SendFn) -> None:
        """JSON 해석과 config.yaml 갱신이 모두 성공했을 때만 ACK를 보낸다."""
        try:
            pose = parse_set_pack_pose_payload(payload=payload)
        except (ValueError, KeyError, TypeError) as exc:
            log.error("SET_PACK_POSE payload 해석 실패: %s | payload=%r", exc, payload)
            send(build_response(cmd="SET_PACK_POSE", ok=False))
            return
        ok = self._save_default_packing_pose(pose=pose)
        send(build_response(cmd="SET_PACK_POSE", ok=ok))

    def _save_default_packing_pose(self, pose: list[float]) -> bool:
        """메모리와 config.yaml 파일 양쪽의 default_packing_pose를 갱신한다.

        주석이 남아 있는 config.yaml을 통째로 다시 쓰면 주석이 사라지므로,
        `default_packing_pose:` 줄만 정규식으로 찾아 치환한다.
        """
        try:
            with open(self._config_path, encoding="UTF-8") as f:
                text = f.read()
            new_text, count = _PACK_POSE_LINE_RE.subn(f"default_packing_pose: {pose}", text, count=1)
            if count == 0:
                log.error("config.yaml에서 default_packing_pose 줄을 찾지 못했습니다.")
                return False
            with open(self._config_path, "w", encoding="UTF-8") as f:
                f.write(new_text)
        except OSError as exc:
            log.error("config.yaml 저장 실패: %s", exc)
            return False
        self.cfg["default_packing_pose"] = pose
        log.info("default_packing_pose 갱신: %s", pose)
        return True
