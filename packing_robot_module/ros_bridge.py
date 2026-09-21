# doosan-robot2 제어 래퍼. 모션 실행은 두산 공식 Python API(DSR_ROBOT2, movej/movel/movejx)의
# sol-space 방식을 그대로 쓴다. MoveIt(move_group)은 실행에는 관여하지 않고, 딱 두 가지
# 서비스로만 쓰인다: (1) 정적 collision object 등록(apply_planning_scene), (2) 후보
# 관절각이 그 collision object와 충돌하는지 확인(check_state_validity).
#
# 이렇게 나눈 이유: MoveIt으로 계획+실행을 다 맡기면 OMPL의 샘플링 경로(직선보다 덜
# 직접적)와 ros2_control의 100Hz 고정 주기 실행 루프를 같이 떠안게 되는데, 이 환경
# (WSL2 + 가상 에뮬레이터)은 그 주기를 못 맞춰 실행이 무작위로 실패했다(CONTROL_FAILED).
# 반면 DSR_ROBOT2 네이티브 모션은 로봇 자체의 모션 제어기가 실행해서 안정적이었다.
# 그래서 "충돌 검사"만 MoveIt에서 빌려오고, 실제 이동/실행은 원래 안정적이던 경로로
# 되돌린다.
#
# DSR_ROBOT2는 import 시점에 DR_init 전역(node/id/model)을 읽으므로, 노드 생성과
# DR_init 세팅 "이후"에 import해야 한다. 그래서 top-level이 아니라 __init__ 안에서
# 지연 import한다.
from __future__ import annotations

import logging
import math
import time

import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneWorld
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

log = logging.getLogger("randpal.ros_bridge")

# 이 환경(WSL2)에서 DDS 디스커버리가 느릴 수 있어(이전 세션에서 최대 45초까지 확인됨)
# move_group 서비스 대기 타임아웃을 넉넉하게 잡는다.
APPLY_PLANNING_SCENE_WAIT_TIMEOUT_SEC = 60.0
CHECK_STATE_VALIDITY_WAIT_TIMEOUT_SEC = 60.0

# dsr_moveit_config_*/config/dsr.srdf.xacro 전 모델 공통 (h2017/m1013/a0912 등 확인됨).
PLANNING_GROUP = "manipulator"
BASE_LINK = "base_link"
JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]

# a0912 URDF <limit> 값 (dsr_description2/xacro/macro.a0912.*.xacro, check_reachability.py에서
# 이미 확인됨). 모델을 바꾸면 이 값도 같이 바꿔야 한다.
JOINT_LIMIT_DEG = (360.0, 360.0, 160.0, 360.0, 360.0, 360.0)

# ikin의 success 필드는 신뢰 불가(dsr_controller2.cpp가 항상 success=true 반환, 이전에
# 확인됨) - fkin round-trip으로 오차가 이 허용치 이내인지로 실제 해 존재 여부를 판정한다.
POS_TOL_MM = 1.0
ANG_TOL_DEG = 1.0

# 두 관절각/pose 사이를 몇 개 지점으로 샘플링해서 collision을 검사할지. 임의로 정한
# 기본값 - 실측 후 필요하면 조정.
COLLISION_CHECK_STEPS = 10


def _rot_zyz(a_deg: float, b_deg: float, c_deg: float) -> list[list[float]]:
    """두산 posx의 ZYZ Euler(a,b,c) 표기를 회전행렬로 변환한다. R = Rz(a)Ry(b)Rz(c)."""

    def rz(t: float) -> list[list[float]]:
        c, s = math.cos(t), math.sin(t)
        return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]

    def ry(t: float) -> list[list[float]]:
        c, s = math.cos(t), math.sin(t)
        return [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]

    def matmul(m1: list[list[float]], m2: list[list[float]]) -> list[list[float]]:
        return [[sum(m1[i][k] * m2[k][j] for k in range(3)) for j in range(3)] for i in range(3)]

    a, b, c = math.radians(a_deg), math.radians(b_deg), math.radians(c_deg)
    return matmul(matmul(rz(a), ry(b)), rz(c))


def _matrix_to_quat_xyzw(r: list[list[float]]) -> tuple[float, float, float, float]:
    """회전행렬 -> quaternion(x,y,z,w). Shepperd's method."""
    m00, m01, m02 = r[0]
    m10, m11, m12 = r[1]
    m20, m21, m22 = r[2]
    trace = m00 + m11 + m22
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m21 - m12) * s
        y = (m02 - m20) * s
        z = (m10 - m01) * s
    elif m00 > m11 and m00 > m22:
        s = 2.0 * math.sqrt(1.0 + m00 - m11 - m22)
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = 2.0 * math.sqrt(1.0 + m11 - m00 - m22)
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = 2.0 * math.sqrt(1.0 + m22 - m00 - m11)
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    return (x / norm, y / norm, z / norm, w / norm)


def _rotation_angle_diff_deg(r1: list[list[float]], r2: list[list[float]]) -> float:
    """R1^T @ R2의 회전각(deg). ZYZ 표현 중복성(축퇴)과 무관하게 방향 차이만 비교."""
    rel = [[sum(r1[k][i] * r2[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    trace = rel[0][0] + rel[1][1] + rel[2][2]
    cos_angle = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    return math.degrees(math.acos(cos_angle))


def _interpolate(a: list[float], b: list[float], steps: int) -> list[list[float]]:
    """a와 b(같은 길이) 사이를 steps개 지점(양 끝 포함)으로 선형 보간한다."""
    return [[av + (bv - av) * i / (steps - 1) for av, bv in zip(a, b)] for i in range(steps)]


def _shortest_angle_delta_deg(a_deg: float, b_deg: float) -> float:
    """b-a를 [-180,180]로 정규화한 최단 각도 차이 (예: 179 -> -179는 -2, 358이 아님)."""
    return ((b_deg - a_deg + 180.0) % 360.0) - 180.0


def _interpolate_pose6(a: list[float], b: list[float], steps: int) -> list[list[float]]:
    """pose6(x,y,z mm + a,b,c ZYZ deg) 보간. x,y,z는 선형, a,b,c는 wrap-around를 고려해
    최단 각도 경로로 보간한다. 단순 선형보간하면 179deg<->-179deg 사이를 358도 도는
    것으로 계산해 중간에 물리적으로 말이 안 되는 자세가 나올 수 있다."""
    xyz = _interpolate(a[:3], b[:3], steps)
    ang_deltas = [_shortest_angle_delta_deg(a[3 + i], b[3 + i]) for i in range(3)]
    result = []
    for i in range(steps):
        t = i / (steps - 1)
        angles = [a[3 + j] + ang_deltas[j] * t for j in range(3)]
        result.append(xyz[i] + angles)
    return result


class RosBridge:
    """모션은 DSR_ROBOT2 네이티브 API로 실행하고, 실행 전 MoveIt planning scene을
    빌려 충돌 여부만 확인하는 래퍼. 모든 모션은 블로킹이며 성공 시 True, 실패 시
    False를 반환한다.
    """

    def __init__(
        self,
        robot_id: str = "dsr01",
        robot_model: str = "h2017",
    ) -> None:
        import DR_init

        # 클래스 안에서 DR_init.__dsr__id처럼 쓰면 name mangling으로
        # _RosBridge__dsr__id에 저장되어 DSR_ROBOT2가 못 읽는다. setattr로 우회한다.
        setattr(DR_init, "__dsr__id", robot_id)
        setattr(DR_init, "__dsr__model", robot_model)
        rclpy.init()
        # DSR_ROBOT2의 서비스(system/set_robot_mode, ikin/fkin 등)는 실제로
        # /{robot_id}/dsr_controller2/...에 있고, move_group의 서비스는
        # /{robot_id}/...에 있다(start.launch.py에서 move_group의 namespace가 'name'
        # 인자 그대로라 dsr_controller2 세그먼트가 없음). 서로 namespace가 달라 노드를
        # 분리한다.
        self.node = rclpy.create_node("randpal_ros_bridge", namespace=f"{robot_id}/dsr_controller2")
        setattr(DR_init, "__dsr__node", self.node)
        self._moveit_node = rclpy.create_node("packing_robot_moveit_bridge", namespace=robot_id)

        # DR_init 세팅 후에 import해야 두산 API가 이 노드에 바인딩된다.
        from DSR_ROBOT2 import (
            DR_BASE,
            DR_TOOL,
            ROBOT_MODE_AUTONOMOUS,
            fkin,
            get_current_posj,
            get_current_posx,
            get_current_solution_space,
            get_last_alarm,
            ikin,
            movej,
            movejx,
            movel,
            posj,
            posx,
            set_digital_output,
            set_robot_mode,
            trans,
        )

        self._movej = movej
        self._movejx = movejx
        self._movel = movel
        self._ikin = ikin
        self._fkin = fkin
        self._get_current_posj = get_current_posj
        self._get_current_posx = get_current_posx
        self._get_current_solution_space = get_current_solution_space
        self._set_do = set_digital_output
        self._posj = posj
        self._posx = posx
        self._trans = trans
        self._DR_TOOL = DR_TOOL
        self._DR_BASE = DR_BASE
        self._get_last_alarm = get_last_alarm

        self._apply_planning_scene_client = self._moveit_node.create_client(
            ApplyPlanningScene, "apply_planning_scene"
        )
        self._check_state_validity_client = self._moveit_node.create_client(
            GetStateValidity, "check_state_validity"
        )

        set_robot_mode(ROBOT_MODE_AUTONOMOUS)  # 모션 전 필수
        log.info("dsr ready (id=%s, model=%s)", robot_id, robot_model)

    def shutdown(self) -> None:
        self.node.destroy_node()
        self._moveit_node.destroy_node()
        rclpy.shutdown()

    # ------------------------------------------------------------- planning scene

    def register_static_collision_box(
        self,
        object_id: str,
        size_mm: list[float],
        position_mm: list[float],
        rotation_deg: float = 0.0,
        frame_id: str = BASE_LINK,
    ) -> bool:
        """정적 collision box를 planning scene에 등록한다 (base_link 기준, z축 회전만 지원)."""
        size_m = [v / 1000.0 for v in size_mm]
        position_m = [v / 1000.0 for v in position_mm]
        quat_xyzw = _matrix_to_quat_xyzw(_rot_zyz(rotation_deg, 0.0, 0.0))

        collision_object = CollisionObject(
            header=Header(frame_id=frame_id),
            id=object_id,
            pose=Pose(
                position=Point(x=position_m[0], y=position_m[1], z=position_m[2]),
                orientation=Quaternion(x=quat_xyzw[0], y=quat_xyzw[1], z=quat_xyzw[2], w=quat_xyzw[3]),
            ),
            primitives=[SolidPrimitive(type=SolidPrimitive.BOX, dimensions=size_m)],
            primitive_poses=[Pose(orientation=Quaternion(w=1.0))],
            operation=CollisionObject.ADD,
        )
        ok = self._apply_planning_scene(collision_object)
        log.info(
            "collision box '%s' 등록 %s (size_mm=%s, position_mm=%s, frame=%s)",
            object_id, "성공" if ok else "실패", size_mm, position_mm, frame_id,
        )
        return ok

    def remove_static_collision_object(self, object_id: str) -> bool:
        collision_object = CollisionObject(id=object_id, operation=CollisionObject.REMOVE)
        return self._apply_planning_scene(collision_object)

    def _apply_planning_scene(self, collision_object: CollisionObject) -> bool:
        """apply_planning_scene 서비스로 collision object를 동기적으로 반영한다.

        (pymoveit2의 add_collision_box는 /collision_object를 절대 경로로 하드코딩해
        publish해서 namespace가 있는 이 launch 구성에서는 move_group에 도달하지
        못했다 - 그래서 pymoveit2를 거치지 않고 이 서비스를 직접 호출한다. 서비스는
        상대 이름이라 노드 namespace를 정상적으로 따르고, 응답으로 반영 여부도 확인
        가능하다.)
        """
        request = ApplyPlanningScene.Request()
        request.scene = PlanningScene(
            is_diff=True,
            world=PlanningSceneWorld(collision_objects=[collision_object]),
        )
        if not self._apply_planning_scene_client.wait_for_service(timeout_sec=APPLY_PLANNING_SCENE_WAIT_TIMEOUT_SEC):
            log.warning(
                "apply_planning_scene 서비스가 %.0f초 내에 준비되지 않았습니다",
                APPLY_PLANNING_SCENE_WAIT_TIMEOUT_SEC,
            )
            return False
        future = self._apply_planning_scene_client.call_async(request)
        rclpy.spin_until_future_complete(self._moveit_node, future, timeout_sec=10.0)
        result = future.result()
        if result is None:
            log.warning("apply_planning_scene 응답을 못 받았습니다 (타임아웃)")
            return False
        return bool(result.success)

    def _is_state_valid(self, joints_deg: list[float]) -> bool:
        """이 관절각 상태가 planning scene(collision object 포함)과 충돌하는지 확인한다.

        서비스를 못 부르거나 응답이 없으면 안전하지 않다고 간주한다(fail-closed) -
        충돌 여부를 확인 못 했는데 이동을 허용하는 건 이 기능의 목적에 반한다.
        """
        request = GetStateValidity.Request()
        request.robot_state.joint_state.name = JOINT_NAMES
        request.robot_state.joint_state.position = [math.radians(j) for j in joints_deg]
        request.group_name = PLANNING_GROUP
        if not self._check_state_validity_client.wait_for_service(
            timeout_sec=CHECK_STATE_VALIDITY_WAIT_TIMEOUT_SEC
        ):
            log.warning(
                "check_state_validity 서비스가 %.0f초 내에 준비되지 않았습니다 - 안전하지 않다고 간주",
                CHECK_STATE_VALIDITY_WAIT_TIMEOUT_SEC,
            )
            return False
        future = self._check_state_validity_client.call_async(request)
        rclpy.spin_until_future_complete(self._moveit_node, future, timeout_sec=10.0)
        result = future.result()
        if result is None:
            log.warning("check_state_validity 응답을 못 받았습니다 (타임아웃) - 안전하지 않다고 간주")
            return False
        return bool(result.valid)

    def _path_collision_free(self, from_joints_deg: list[float], to_joints_deg: list[float], context: str = "") -> bool:
        """관절 공간에서 두 자세 사이(양 끝 포함)를 선형 보간해 COLLISION_CHECK_STEPS개
        지점을 collision 검사한다. movej/movejx가 실제로 이렇게 움직인다는 가정."""
        waypoints = _interpolate(from_joints_deg, to_joints_deg, COLLISION_CHECK_STEPS)
        for i, joints in enumerate(waypoints):
            if not self._is_state_valid(joints):
                log.info(
                    "%s: 경로 중간 지점 %d/%d에서 충돌 감지 joints_deg=%s",
                    context, i, len(waypoints) - 1, joints,
                )
                return False
        return True

    def _ik_for_sol(self, pose6: list[float], sol: int) -> list[float] | None:
        """pose6를 sol로 풀어 실제로 유효한(관절한계 + fkin round-trip 검증) 관절각을
        반환한다. 유효하지 않으면 None.

        ikin의 success 필드는 신뢰 불가하므로(모듈 상단 설명) round-trip으로 검증한다.
        """
        joints = self._ikin(self._posx(*pose6), sol, ref=self._DR_BASE)
        if isinstance(joints, int):  # -1 = 서비스 호출 실패
            return None
        joints = list(joints)
        if len(joints) != 6 or any(not math.isfinite(j) for j in joints):
            return None
        if any(abs(j) > limit + 1e-3 for j, limit in zip(joints, JOINT_LIMIT_DEG)):
            return None

        recovered = self._fkin(self._posj(*joints), ref=self._DR_BASE)
        if isinstance(recovered, int):
            return None
        recovered = list(recovered)
        if len(recovered) != 6:
            return None

        pos_err = math.sqrt(sum((a - b) ** 2 for a, b in zip(pose6[:3], recovered[:3])))
        if pos_err > POS_TOL_MM:
            return None
        if _rotation_angle_diff_deg(_rot_zyz(*pose6[3:]), _rot_zyz(*recovered[3:])) > ANG_TOL_DEG:
            return None
        return joints

    # ------------------------------------------------------------- motions

    def movej(self, joints_deg: list[float], vel: float, acc: float) -> bool:
        current_joints = self._get_current_posj()
        if isinstance(current_joints, int):
            return self._fail("movej", RuntimeError("get_current_posj 실패"))
        if not self._path_collision_free(list(current_joints), joints_deg, context=f"movej({joints_deg})"):
            log.error("movej 실패: 목표 경로에 충돌 위험이 있어 이동하지 않습니다 %s", joints_deg)
            return False
        try:
            ret = self._movej(self._posj(*joints_deg), vel=float(vel), acc=float(acc))
        except Exception as exc:
            return self._fail("movej", exc)
        return self._ok("movej", ret)

    def movel(self, pose6: list[float], vel: list[float], acc: list[float]) -> bool:
        """pose6: x, y, z (mm) + ZYZ Euler (deg). vel/acc: [linear, angular]."""
        current_pose, current_sol = self._get_current_posx(ref=self._DR_BASE)
        if current_pose is None or not isinstance(current_sol, int) or not (0 <= current_sol <= 7):
            return self._fail("movel", RuntimeError("get_current_posx 실패 - 충돌 검사 불가"))

        waypoints = _interpolate_pose6(list(current_pose), pose6, COLLISION_CHECK_STEPS)
        for i, waypoint in enumerate(waypoints):
            joints = self._ik_for_sol(waypoint, current_sol)
            if joints is None:
                log.error(
                    "movel 실패: 경로 중간 지점 %d/%d에서 IK(sol=%d)가 안 풀림(도달 불가) waypoint=%s 목표=%s",
                    i, len(waypoints) - 1, current_sol, waypoint, pose6,
                )
                return False
            if not self._is_state_valid(joints):
                log.error(
                    "movel 실패: 경로 중간 지점 %d/%d에서 충돌 감지 waypoint=%s joints_deg=%s 목표=%s",
                    i, len(waypoints) - 1, waypoint, joints, pose6,
                )
                return False

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
        """pose6로 이동. 현재 sol과 가까운(비트 차이가 적은) sol부터 시도해, 도달
        가능하고 충돌도 없는 첫 sol로 실행한다. 8개 sol 다 안 되면 실패."""
        current_joints = self._get_current_posj()
        if isinstance(current_joints, int):
            return self._fail("movejx", RuntimeError("get_current_posj 실패"))
        current_joints = list(current_joints)

        current_sol = self._get_current_solution_space()
        if isinstance(current_sol, int) and 0 <= current_sol <= 7:
            sol_order = sorted(range(8), key=lambda s: bin(s ^ current_sol).count("1"))
        else:
            log.warning("get_current_solution_space 실패(%r), sol 0~7 순서로 시도", current_sol)
            sol_order = list(range(8))

        for sol in sol_order:
            joints = self._ik_for_sol(pose6, sol)
            if joints is None:
                log.info("movejx(%s): sol=%d 도달 불가(IK round-trip 실패), 다음 sol 시도", pose6, sol)
                continue
            if not self._path_collision_free(current_joints, joints, context=f"movejx({pose6}) sol={sol}"):
                continue  # 이 sol은 도달 가능하지만 경로에 충돌 위험 (구체적 지점은 _path_collision_free가 로그)
            try:
                ret = self._movejx(self._posx(*pose6), vel=vel, acc=acc, sol=sol)
            except Exception as exc:
                return self._fail("movejx", exc)
            return self._ok("movejx", ret)

        log.error("movejx 실패: 도달 가능하면서 충돌도 없는 sol을 못 찾음 %s", pose6)
        return False

    def offset_along_tool(self, pose6: list[float], delta6: list[float]) -> list[float] | None:
        """pose6의 회전을 반영해 tool 좌표계 기준 delta6만큼 이동한 pose를 base 좌표계로 계산한다."""
        try:
            ret = self._trans(self._posx(*pose6), self._posx(*delta6), ref=self._DR_TOOL, ref_out=self._DR_BASE)
        except Exception as exc:
            self._fail("trans", exc)
            return None
        if isinstance(ret, int):
            self._fail("trans", RuntimeError(f"trans returned {ret}"))
            return None
        return list(ret)

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
