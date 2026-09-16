# doosan-robot2 제어 래퍼. 두산 공식 Python API(DSR_ROBOT2)로 ROS2 서비스를 호출한다.
#
# DSR_ROBOT2는 import 시점에 DR_init 전역(node/id/model)을 읽으므로, 노드 생성과
# DR_init 세팅 "이후"에 import해야 한다. 그래서 top-level이 아니라 __init__ 안에서
# 지연 import한다.
from __future__ import annotations

import logging
import time

import rclpy
from dsr_msgs2.msg import RobotError

log = logging.getLogger("randpal.ros_bridge")

NOT_REACHABLE = 1206  # 두산 모션 알람: 목표 pose에 대한 해가 없음
ERROR_WAIT_SEC = 0.3


class RosBridge:
    """DSR_ROBOT2 API 래퍼. 모든 모션은 블로킹이며 성공 시 True, 실패 시 False를 반환한다."""

    def __init__(self, robot_id: str = "dsr01", robot_model: str = "h2017") -> None:
        import DR_init

        # 클래스 안에서 DR_init.__dsr__id처럼 쓰면 name mangling으로
        # _RosBridge__dsr__id에 저장되어 DSR_ROBOT2가 못 읽는다. setattr로 우회한다.
        setattr(DR_init, "__dsr__id", robot_id)
        setattr(DR_init, "__dsr__model", robot_model)
        rclpy.init()
        self.node = rclpy.create_node("randpal_ros_bridge", namespace=f"{robot_id}/dsr_controller2")
        setattr(DR_init, "__dsr__node", self.node)

        # DR_init 세팅 후에 import해야 두산 API가 이 노드에 바인딩된다.
        from DSR_ROBOT2 import (
            ROBOT_MODE_AUTONOMOUS,
            get_current_solution_space,
            get_last_alarm,
            movej,
            movejx,
            movel,
            posj,
            posx,
            set_digital_output,
            set_robot_mode,
        )

        self._movej = movej
        self._movejx = movejx
        self._movel = movel
        self._set_do = set_digital_output
        self._posj = posj
        self._posx = posx
        self._get_last_alarm = get_last_alarm
        self._get_current_solution_space = get_current_solution_space

        # NOT REACHABLE 등 모션 알람은 movejx의 성공 응답과 별개로 이 토픽에서만 온다.
        self._last_error_code: int | None = None
        self.node.create_subscription(
            msg_type=RobotError,
            topic=f"/{robot_id}/error",
            callback=self._on_error,
            qos_profile=10,
        )

        set_robot_mode(ROBOT_MODE_AUTONOMOUS)  # 모션 전 필수
        log.info("dsr ready (id=%s, model=%s)", robot_id, robot_model)

    def shutdown(self) -> None:
        self.node.destroy_node()
        rclpy.shutdown()

    # ------------------------------------------------------------- motions

    def movej(self, joints_deg: list[float], vel: float, acc: float) -> bool:
        try:
            ret = self._movej(self._posj(*joints_deg), vel=float(vel), acc=float(acc))
        except Exception as exc:
            return self._fail("movej", exc)
        return self._ok("movej", ret)

    def movel(self, pose6: list[float], vel: list[float], acc: list[float]) -> bool:
        """pose6: x, y, z (mm) + ZYZ Euler (deg). vel/acc: [linear, angular]."""
        try:
            ret = self._movel(
                self._posx(*pose6),
                vel=[float(v) for v in vel],
                acc=[float(v) for v in acc],
            )
        except Exception as exc:
            return self._fail("movel", exc)
        return self._ok("movel", ret)

    def movejx(self, pose6: list[float], vel: float, acc: float) -> bool:
        """pose6를 sol(해공간) 0~7로 바꿔가며 도달 가능한 해를 찾아 이동한다.

        현재 sol과 비트 차이(Hamming distance)가 작은 sol부터 시도해 관절 이동이
        더 작을 가능성을 높인다. 현재 sol을 못 가져오면 0~7 순서로 시도한다.
        NOT REACHABLE(1206) 알람이 안 오면 성공, 모든 sol이 도달 불가면 False.
        """
        pose = self._posx(*pose6)
        current_sol = self._get_current_solution_space()
        if isinstance(current_sol, int) and 0 <= current_sol <= 7:
            sol_order = sorted(range(8), key=lambda s: bin(s ^ current_sol).count("1"))
        else:
            log.warning("get_current_solution_space 실패(%r), sol 0~7 순서로 시도", current_sol)
            sol_order = list(range(8))

        for sol in sol_order:
            self._last_error_code = None
            try:
                ret = self._movejx(pose, vel=vel, acc=acc, sol=sol)
            except Exception as exc:
                return self._fail("movejx", exc)
            self._drain_errors(ERROR_WAIT_SEC)
            if self._last_error_code is None:
                return self._ok("movejx", ret)
            if self._last_error_code != NOT_REACHABLE:
                return self._fail("movejx", RuntimeError(f"error code={self._last_error_code}"))
            log.warning("movejx NOT REACHABLE (sol=%d), 다음 sol 시도", sol)
        log.error("movejx 실패: 모든 sol(0~7)에서 도달 불가 %s", pose)
        return False

    def gripper(self, on: bool, io_index: int, settle_sec: float) -> bool:
        try:
            ok = self._ok("gripper", self._set_do(int(io_index), 1 if on else 0))
        except Exception as exc:
            ok = self._fail("gripper", exc)
        time.sleep(settle_sec)
        return ok

    # ------------------------------------------------------------- helpers

    def _ok(self, what: str, ret: int) -> bool:
        """두산 API 반환값(0=성공, -1=실패)을 bool로 바꾼다."""
        if ret == 0:
            return True
        return self._fail(what, RuntimeError(f"{what} returned {ret} (0=success)"))

    def _fail(self, what: str, exc: Exception) -> bool:
        try:
            alarm = self._get_last_alarm()
        except Exception:
            alarm = None
        log.error("%s failed: %s | last_alarm=%s", what, exc, alarm)
        return False

    def _on_error(self, msg: RobotError) -> None:
        """컨트롤러 error 토픽 콜백: 마지막 알람 코드를 저장한다."""
        self._last_error_code = int(msg.code)
        log.warning(
            "robot error: level=%d group=%d code=%d msg1=%s",
            msg.level, msg.group, msg.code, msg.msg1,
        )

    def _drain_errors(self, wait_sec: float) -> None:
        """wait_sec 동안 노드를 spin하며 error 콜백을 기다린다. 알람이 오면 즉시 반환한다."""
        end = time.time() + wait_sec
        while time.time() < end and self._last_error_code is None:
            rclpy.spin_once(self.node, timeout_sec=0.05)
