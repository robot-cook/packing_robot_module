# doosan-robot2 제어 래퍼. 모션은 MoveIt2(move_group, collision-aware)로,
# gripper I/O는 두산 공식 Python API(DSR_ROBOT2)로 ROS2 서비스를 호출한다.
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
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneWorld
from moveit_msgs.srv import ApplyPlanningScene
from pymoveit2 import MoveIt2
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

log = logging.getLogger("randpal.ros_bridge")

# 이 환경(WSL2)에서 DDS 디스커버리가 느릴 수 있어(이전 세션에서 최대 45초까지 확인됨)
# move_action/apply_planning_scene 서버 대기 타임아웃을 넉넉하게 잡는다.
MOVE_ACTION_WAIT_TIMEOUT_SEC = 60.0
APPLY_PLANNING_SCENE_WAIT_TIMEOUT_SEC = 60.0

# dsr_moveit_config_*/config/dsr.srdf.xacro 전 모델 공통 (h2017/m1013/a0912 등 확인됨).
PLANNING_GROUP = "manipulator"
BASE_LINK = "base_link"
END_EFFECTOR_LINK = "link_6"
JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]


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


def _pose6_to_position_quat(pose6: list[float]) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """두산 pose6(x,y,z mm + a,b,c ZYZ deg) -> MoveIt pose(m, quat_xyzw)."""
    x_mm, y_mm, z_mm, a_deg, b_deg, c_deg = pose6
    position = (x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0)
    quat_xyzw = _matrix_to_quat_xyzw(_rot_zyz(a_deg, b_deg, c_deg))
    return position, quat_xyzw


class RosBridge:
    """모션은 MoveIt2(collision-aware plan+execute), gripper I/O는 DSR_ROBOT2로 처리하는 래퍼.

    모든 모션은 블로킹이며 성공 시 True, 실패 시 False를 반환한다.
    """

    def __init__(
        self,
        robot_id: str = "dsr01",
        robot_model: str = "a0912",
        moveit_velocity_scaling: float = 0.3,
        moveit_acceleration_scaling: float = 0.3,
    ) -> None:
        import DR_init

        # 클래스 안에서 DR_init.__dsr__id처럼 쓰면 name mangling으로
        # _RosBridge__dsr__id에 저장되어 DSR_ROBOT2가 못 읽는다. setattr로 우회한다.
        setattr(DR_init, "__dsr__id", robot_id)
        setattr(DR_init, "__dsr__model", robot_model)
        rclpy.init()
        # DSR_ROBOT2의 서비스(system/set_robot_mode 등)는 실제로 /{robot_id}/dsr_controller2/...에
        # 있고, move_group의 move_action은 /{robot_id}/move_action에 있다(start.launch.py에서
        # move_group의 namespace가 'name' 인자 그대로라 dsr_controller2 세그먼트가 없음). 서로
        # namespace가 달라 노드를 분리한다.
        self.node = rclpy.create_node("randpal_ros_bridge", namespace=f"{robot_id}/dsr_controller2")
        setattr(DR_init, "__dsr__node", self.node)
        self._moveit_node = rclpy.create_node("packing_robot_moveit_bridge", namespace=robot_id)

        # DR_init 세팅 후에 import해야 두산 API가 이 노드에 바인딩된다.
        from DSR_ROBOT2 import (
            DR_BASE,
            DR_TOOL,
            ROBOT_MODE_AUTONOMOUS,
            get_last_alarm,
            posx,
            set_digital_output,
            set_robot_mode,
            trans,
        )

        self._set_do = set_digital_output
        self._posx = posx
        self._trans = trans
        self._DR_TOOL = DR_TOOL
        self._DR_BASE = DR_BASE
        self._get_last_alarm = get_last_alarm

        self._moveit = MoveIt2(
            node=self._moveit_node,
            joint_names=JOINT_NAMES,
            base_link_name=BASE_LINK,
            end_effector_name=END_EFFECTOR_LINK,
            group_name=PLANNING_GROUP,
            use_move_group_action=True,
        )
        self._moveit.max_velocity = float(moveit_velocity_scaling)
        self._moveit.max_acceleration = float(moveit_acceleration_scaling)

        # pymoveit2는 move_action 서버 준비 여부를 한 번만 확인하고 재시도하지 않는다
        # (DSR_ROBOT2의 서비스 대기와 다름). 이 환경은 DDS 디스커버리가 느릴 수 있어
        # (이전에 확인됨), 노드 생성 직후 바로 모션을 호출하면 디스커버리가 안 끝나
        # "not yet available"로 실패할 수 있다. 그래서 여기서 명시적으로 기다린다.
        move_action_client = ActionClient(self._moveit_node, MoveGroup, "move_action")
        if not move_action_client.wait_for_server(timeout_sec=MOVE_ACTION_WAIT_TIMEOUT_SEC):
            log.warning(
                "move_action 액션 서버가 %.0f초 내에 준비되지 않았습니다 "
                "(move_group이 안 떠 있거나 namespace가 다를 수 있음)",
                MOVE_ACTION_WAIT_TIMEOUT_SEC,
            )
        move_action_client.destroy()

        # pymoveit2.add_collision_box/remove_collision_object는 "/collision_object"를
        # 절대 경로로 하드코딩해서 publish한다(pymoveit2/moveit2.py:115). move_group은
        # namespace(/dsr01) 기준 상대 이름 "collision_object"만 듣기 때문에, namespace가
        # 있는 이 launch 구성에서는 pymoveit2로 등록한 collision object가 move_group에
        # 전혀 도달하지 못한다(ros2 topic info로 publisher 0건 확인됨). 그래서 collision
        # object 등록/삭제는 pymoveit2를 거치지 않고 apply_planning_scene 서비스를 직접
        # 호출한다. 이 서비스 이름은 상대 이름이라 노드 namespace를 정상적으로 따른다.
        self._apply_planning_scene_client = self._moveit_node.create_client(
            ApplyPlanningScene, "apply_planning_scene"
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

        pymoveit2.add_collision_box/remove_collision_object는 "/collision_object"를
        절대 경로로 하드코딩해 publish하므로(pymoveit2/moveit2.py:115), namespace가
        있는 이 launch 구성(move_group이 /dsr01 하위)에서는 move_group에 전혀 도달하지
        않는다(ros2 topic info로 publisher 0건 확인). apply_planning_scene은 서비스라
        노드 namespace를 정상적으로 따르고, 응답으로 실제 반영 여부도 확인할 수 있다.
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

    # ------------------------------------------------------------- motions

    def movej(self, joints_deg: list[float], vel: float, acc: float) -> bool:
        """joint-space 목표로 collision-aware plan+execute (MoveIt).

        vel/acc(deg/s, deg/s^2)는 DRL 전용 단위라 MoveIt의 0~1 스케일 팩터와 호환되지
        않는다. 그래서 이 값들은 무시하고, 생성자에서 받은 moveit_velocity/acceleration_scaling을
        모든 MoveIt 모션에 균일하게 적용한다(호출 시그니처는 job_handler.py 호환을 위해 유지).
        """
        del vel, acc
        joints_rad = [math.radians(j) for j in joints_deg]
        self._moveit.move_to_configuration(joint_positions=joints_rad)
        return self._wait_moveit("movej")

    def movel(self, pose6: list[float], vel: list[float], acc: list[float]) -> bool:
        """pose6: x, y, z (mm) + ZYZ Euler (deg). 직선 경로(cartesian) plan+execute."""
        del vel, acc
        position, quat_xyzw = _pose6_to_position_quat(pose6)
        self._moveit.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=True)
        return self._wait_moveit("movel")

    def movejx(self, pose6: list[float], vel: float, acc: float) -> bool:
        """pose6 목표로 collision-aware plan+execute (IK + 자유 경로, MoveIt).

        기존 DSR_ROBOT2 movejx의 sol(0~7) 재시도 로직은 MoveIt의 IK 플러그인(KDL)이
        대체한다 - 더 이상 sol을 직접 순회하지 않는다.
        """
        del vel, acc
        position, quat_xyzw = _pose6_to_position_quat(pose6)
        self._moveit.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=False)
        return self._wait_moveit("movejx")

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

    def _wait_moveit(self, what: str) -> bool:
        ok = self._moveit.wait_until_executed()
        if not ok:
            err = self._moveit.get_last_execution_error_code()
            log.error("%s failed (MoveIt): error_code=%s", what, getattr(err, "val", err))
        return ok

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
