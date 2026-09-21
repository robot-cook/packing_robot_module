# packing_robot_module 서버 실행 진입점: config 로드, 로깅 설정, TCP 서버 구동
from __future__ import annotations

import argparse
import logging
import os
import sys
from logging.handlers import RotatingFileHandler

import yaml

from .job_handler import JobHandler
from .tcp_server import TcpServer

log = logging.getLogger("packing_robot_module.server_node")


def _setup_logging(cfg_log: dict) -> None:
    """콘솔 + (config에 file이 있으면) 회전 파일 핸들러로 로깅을 설정한다."""
    level = getattr(logging, str(cfg_log.get("level", "INFO")).upper(), logging.INFO)
    fmt = logging.Formatter("%(asctime)s %(name)s %(message)s")
    handlers: list[logging.Handler] = [logging.StreamHandler()]
    path = cfg_log.get("file")
    if path:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        handlers.append(
            RotatingFileHandler(
                path,
                maxBytes=int(cfg_log.get("max_bytes", 5 * 1024 * 1024)),
                backupCount=int(cfg_log.get("backup_count", 3)),
                encoding="UTF-8",
            )
        )
    for h in handlers:
        h.setFormatter(fmt)
    logging.basicConfig(level=level, handlers=handlers)


def _register_collision_objects(robot: object, cfg: dict) -> None:
    """config.yaml의 collision_objects를 planning scene에 등록한다.

    size_mm에 0 이하 값이 있으면 아직 치수가 확정되지 않은 것으로 보고 건너뛴다.
    """
    for object_id, obj_cfg in cfg.items():
        size_mm = obj_cfg.get("size_mm", [0.0, 0.0, 0.0])
        if any(v <= 0.0 for v in size_mm):
            log.warning("collision object '%s' size_mm이 미설정(<=0)이라 등록을 건너뜁니다: %s", object_id, size_mm)
            continue
        robot.register_static_collision_box(
            object_id=object_id,
            size_mm=size_mm,
            position_mm=obj_cfg.get("position_mm", [0.0, 0.0, 0.0]),
            rotation_deg=obj_cfg.get("rotation_deg", 0.0),
            frame_id=obj_cfg.get("frame_id", "base_link"),
        )


def main() -> None:
    # 테스트가 ROS 없이 이 모듈을 import할 수 있도록 rclpy/RosBridge는 여기서 지연 import한다.
    from rclpy.utilities import remove_ros_args

    from .ros_bridge import RosBridge

    parser = argparse.ArgumentParser(description="packing_robot_module SUBMIT_SEQUENCE/GET_SEQUENCE_STATUS TCP 서버")
    parser.add_argument("--config", default="src/packing_robot_module/config/config.yaml")
    argv = remove_ros_args(args=sys.argv)  # launch가 붙이는 --ros-args 제거
    args = parser.parse_args(argv[1:])
    with open(args.config, encoding="UTF-8") as f:
        cfg = yaml.safe_load(f)
    _setup_logging(cfg_log=cfg.get("logging", {}))

    robot_cfg = cfg["robot"]
    robot = RosBridge(
        robot_id=robot_cfg.get("robot_id", "dsr01"),
        robot_model=robot_cfg.get("model", "h2017"),
    )
    _register_collision_objects(robot=robot, cfg=cfg.get("collision_objects", {}))
    handler = JobHandler(cfg=cfg, robot=robot)
    handler.move_home()
    server_cfg = cfg["server"]
    server = TcpServer(host=server_cfg["host"], port=int(server_cfg["port"]), handler=handler)
    try:
        server.serve_forever()
    except (KeyboardInterrupt, SystemExit):
        log.info("종료합니다")
    finally:
        robot.shutdown()


if __name__ == "__main__":
    main()
