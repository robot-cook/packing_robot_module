# doosan-robot2 로컬 수정 내역

`src/doosan-robot2`는 `origin/jazzy`(DoosanRobotics/doosan-robot2, HEAD 816ecb5
"chore: release 20260423")를 그대로 clone한 저장소이며, 아래 6개 파일에 **커밋되지
않은 로컬 수정**이 있다. 다른 에이전트가 이 저장소를 새로 clone/reset하거나 upstream과
동기화할 경우 이 수정들이 사라지므로, 왜 고쳤는지와 다시 고치는 방법을 정리한다.

전체 diff는 `git -C src/doosan-robot2 diff`로 확인 가능하다 (작성 시점 스냅샷은 이
문서 하단 참고). 아래는 파일별 원인/수정 내용.

## 1. `dsr_common2/imp/DSR_ROBOT2.py` — 실제 버그 수정 (오타)

`dsr_msgs2/srv/`에는 `SetSingularHandlingForce.srv`가 존재하는데
(`SetSingularityHandlingForce`가 아님), `DSR_ROBOT2.py`의 전역 client 생성부와
`CDsrRobot.__init__`에서는 존재하지 않는 타입 `SetSingularityHandlingForce`와
속성명 `self.req_SetSingularityHandlingForce`를 사용하고 있었다. 반면 실제로
그 request를 쓰는 메서드(9154번 줄 근처, `set_singular_handling_force` 관련)는
이미 `self.req_SetSingularHandlingForce`(Singular, Handling 사이에 "ity" 없음)를
참조하고 있어서, import/생성 시점에 `NameError` 혹은 실행 시 `AttributeError`가 나는
상태였다.

- 101/6666번 줄 근처: `SetSingularityHandlingForce` → `SetSingularHandlingForce`로
  타입명 수정 (client 생성 시 사용하는 타입).
- 6669번 줄: `self.req_SetSingularityHandlingForce` →
  `self.req_SetSingularHandlingForce`로 속성명 수정 (기존에 이 속성을 참조하던
  코드와 이름을 맞춤).

**재현 방법:** `SetSingularityHandlingForce`를 grep해서 나오는 모든 위치를
`SetSingularHandlingForce`로 치환하면 된다 (import한 srv 모듈 실제 클래스명과
일치시키는 것이 핵심).

## 2. `dsr_controller2/src/dsr_controller2.cpp` — 실제 버그 수정 (미초기화 값 사용)

`trans_cb` 서비스 콜백(767번 줄 근처)에서 `target_pos`/`delta_pos`
(`std::array<float, NUM_TASK>`)를 선언만 하고 값을 채우지 않은 채 바로
`Drfl->trans(target_pos.data(), delta_pos.data(), ...)`를 호출하고 있었다. 즉
요청(`req->pos`, `req->delta`)이 무시되고 스택의 미초기화 값으로 좌표 변환을 수행하는
버그였다.

- 수정: `Drfl->trans(...)` 호출 전에
  ```cpp
  std::copy(req->pos.cbegin(), req->pos.cend(), target_pos.begin());
  std::copy(req->delta.cbegin(), req->delta.cend(), delta_pos.begin());
  ```
  를 추가해 요청 데이터를 실제로 복사하도록 함.

**재현 방법:** `trans_cb` 람다에서 `target_pos`/`delta_pos` 선언 직후,
`Drfl->trans` 호출 전에 위 두 줄의 `std::copy`를 삽입.

## 3. `dsr_moveit2/dsr_moveit_config_a0912/config/moveit_controllers.yaml` — WSL2 에뮬레이터 환경 대응

WSL2 + 가상(docker) DRCF 에뮬레이터 환경에서 `ros2_control`의 `write()` 지연이
사이클당 100ms~1000ms+까지 튀는 현상이 관찰됨 (목표 100Hz = 10ms 주기). 기존
`allowed_execution_duration_scaling: 1.2` / `allowed_goal_duration_margin: 0.5`는
이 환경에서는 정상적으로 끝난 실행조차 "너무 오래 걸린다"며 `CONTROL_FAILED`로
중단시켜, collision 회피 로직과 무관하게 planning 성공/실패가 무작위로 나오는
원인이 됐다.

- `allowed_execution_duration_scaling: 1.2` → `10.0`
- `allowed_goal_duration_margin: 0.5` → `5.0`

**주의:** 이 값은 실제 로봇(실기)이나 지연이 적은 환경에서는 과도하게 관대한 값이다.
WSL2 에뮬레이터가 아닌 환경에서 이 저장소를 쓸 때는 원래 값(1.2/0.5)으로 되돌리는 것을
고려해야 한다. (요청받지 않은 이상 기존 필드는 유지, 다른 필드는 손대지 않음.)

## 4. `dsr_moveit2/dsr_moveit_config_a0912/launch/start.launch.py` — RViz 절대 topic 경로 고정

`rviz_and_move_group_fn` 안에서 `.rviz` 템플릿의 `Move Group Namespace`만
`target_ns`로 치환하고 있었는데, RViz 프로세스 자체는 네임스페이스 없이 뜨기 때문에
`.rviz` 안의 상대 topic 이름(`monitored_planning_scene`, `display_planned_path`)은
root(`/monitored_planning_scene`)로 풀려 실제 move_group이 발행하는
`/{ns}/monitored_planning_scene`을 구독하지 못하는 문제가 있었다.

- `Move Group Namespace` 치환 로직 아래에 `Planning Scene Topic:` /
  `Trajectory Topic:` 정규식 치환을 추가해 두 topic도
  `{target_ns}/monitored_planning_scene`, `{target_ns}/display_planned_path`
  절대 경로로 고정.
- RViz 실행 `Node`의 `arguments`가 `rviz_full_config`(치환 전 원본 경로)를 쓰고
  있던 것을 `tmp_rviz_path`(치환 결과가 실제로 기록된 임시 파일 경로)로 수정. 즉
  치환된 내용이 애초에 RViz에 전달되지 않던 버그였다.
- `run_emulator_node`를 `nodes` 리스트에서 주석 처리 (`#run_emulator_node,`).
  로컬 환경에서는 별도로 에뮬레이터/실기 연결을 관리하고 있어 이 launch가 자체적으로
  에뮬레이터를 또 띄우는 것을 막기 위함으로 추정됨 — 실기 연결 시에는 이 줄이
  없어도 문제없지만, 다시 에뮬레이터로 테스트하려면 주석을 해제해야 한다.

## 5. `dsr_bringup2/launch/dsr_bringup2_rviz.launch.py` — 로컬 실행 편의 목적 (주석 해제)

원본에는 다음 항목들이 통째로 주석 처리되어 있었는데, 로컬 개발/테스트 편의를 위해
주석을 해제함:

- `gui = LaunchConfiguration("gui")` 선언 복원.
- RViz `Node`의 `condition=IfCondition(gui)` 복원 (즉 `gui:=false`로 RViz를 끌 수
  있게 됨. 주석 상태에서는 항상 RViz가 실행됐음).
- `joint_trajectory_controller_spawner` (`dsr_joint_trajectory` controller
  spawner) `Node` 정의 복원. **단, 이 노드는 `nodes` 리스트에는 추가되지 않아 아직
  실제로 실행되지는 않는다** (변수만 정의됨, 사용 안 함 — upstream 원본 상태와 동일).
- `nodes` 리스트에서 `run_emulator_node` 대신 `# run_emulator_node`로 주석 처리
  (start.launch.py와 동일한 이유로 추정, 자체 에뮬레이터 기동 방지).

## 6. `dsr_moveit2/dsr_moveit_config_a0912/launch/moveit.rviz` — 로컬 RViz 세션 저장분

로컬에서 RViz를 띄우고 `world`/`base` 링크 트리 항목, `Move Group Namespace`
(`""` → `/dsr01`), 창 크기/위치(`X`/`Y`), 패널 펼침 상태 등을 조정한 뒤 RViz가
자동으로 다시 저장한 결과물이다. 로직 변경이 아니라 **RViz UI 상태 저장 파일**이므로,
동작에 영향 없고 로컬 화면 배치에만 영향을 준다. 재현이 딱히 필요하지 않으면 이 파일은
무시해도 된다 (단, `start.launch.py`가 이 파일을 런타임에 읽어서 namespace/topic을
치환하므로 `Move Group Namespace` 필드 자체는 존재해야 함 — 이미 원본에도 있음).

## 재적용 방법

이 수정들은 `src/doosan-robot2`가 git working tree에 아직 커밋되지 않은 상태로
남아있다 (`git -C src/doosan-robot2 status`로 확인 가능). 저장소를 재클론하거나
`git checkout .` / `git reset --hard`로 되돌리면 사라진다. 되돌아온 경우:

1. 위 1, 2번(실제 버그 수정)은 항상 다시 적용해야 한다.
2. 3, 4, 5번(WSL2/로컬 환경 대응)은 실행 환경이 여전히 WSL2 + 에뮬레이터인지 먼저
   확인한 뒤 적용 여부를 판단한다.
3. 6번(moveit.rviz)은 굳이 재현할 필요 없음.

작성 시점 전체 diff는 아래 명령으로 재확인 가능:
```
git -C src/doosan-robot2 diff -- \
  dsr_bringup2/launch/dsr_bringup2_rviz.launch.py \
  dsr_common2/imp/DSR_ROBOT2.py \
  dsr_controller2/src/dsr_controller2.cpp \
  dsr_moveit2/dsr_moveit_config_a0912/config/moveit_controllers.yaml \
  dsr_moveit2/dsr_moveit_config_a0912/launch/moveit.rviz \
  dsr_moveit2/dsr_moveit_config_a0912/launch/start.launch.py
```
