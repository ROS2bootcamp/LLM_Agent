# 작업 컨텍스트 — LLM Agent

> 이 파일은 다른 환경에서 Claude Code 세션을 이어받을 때 참고용으로 사용합니다.  
> 최종 업데이트: 2026-06-06

---

## 프로젝트 개요

- **원격 레포**: https://github.com/ROS2bootcamp/LLM_Agent.git
- **목적**: CLI 자연어 명령 → YOLO 오브젝트 탐지 → MoveIt Pick & Place 자동화
- **환경**: ROS2 Humble + MoveIt2 + MoveIt Task Constructor (Panda/UR3 로봇)
- **설계 문서**: `DESIGN.md` (상세 설계 전체 포함)

---

## 현재 상태 (Sprint 1 완료 / Sprint 2 구현됨)

### 완성된 파일 목록

```
src/
├── llm_agent_msgs/          ← ROS2 커스텀 메시지 패키지 (C++, ament_cmake)
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── msg/TaskStatus.msg
│   └── srv/SetPhase.srv, AgentQuery.srv
│
└── llm_agent/               ← 메인 에이전트 패키지 (Python, ament_python)
    ├── package.xml
    ├── setup.py / setup.cfg
    ├── resource/llm_agent
    ├── llm_agent/
    │   ├── __init__.py
    │   ├── agent_node.py        ← 메인 노드 (Phase 루프 전체 구현)
    │   ├── phase_manager.py     ← Phase 상태머신 + retry counter
    │   ├── llm_client.py        ← Anthropic API (JSON 응답, no tool use)
    │   ├── yolo_subscriber.py   ← /obj_data 구독 + 프레임 버퍼
    │   ├── tf_transformer.py    ← camera_frame → world_frame TF 변환
    │   ├── moveit_client.py     ← /moveit_command 발행 + /moveit_status 수신
    │   ├── cli_reader.py        ← stdin 명령 읽기 (daemon thread)
    │   └── tools/__init__.py
    ├── config/
    │   ├── agent.yaml           ← 임계값, retry 횟수, moveit/tf 설정
    │   ├── scan_waypoints.yaml  ← P1 스캔 waypoint 목록
    │   └── targets.yaml         ← P4 target 절대좌표
    ├── launch/agent.launch.py
    └── test/test_phase_manager.py
```

---

## 4-Phase 아키텍처 요약

```
CLI stdin 입력
    │  LLM: target_class_name 추출
    ▼
[ P1 ] YOLO로 오브젝트 탐색 (30s 타임아웃 × 최대 3회 재시도)
    │  탐지 성공 → position_3d_camera_frame 획득
    ▼
[ P2 ] TF 변환(camera→world) + LLM 파라미터 생성 + PICK 실행
    │  YOLO grip 검증 성공 (3회 retry)
    ▼
[ P3 ] 10cm 수직 상승 + LLM이 YOLO로 pickup 성공 판단
    │  실패 → HOME → P1 재시작 (retry_count 공유)
    ▼
[ P4 ] target 절대좌표로 PLACE + RELEASE + HOME → 세션 종료
```

---

## 외부 인터페이스

| 방향 | Topic | 타입 | 비고 |
|------|-------|------|------|
| YOLO → Agent | `/obj_data` | `std_msgs/String` (JSON) | 상시 구독 |
| Agent → MoveIt | `/moveit_command` | `std_msgs/String` (JSON) | cmd: scan/pick/lift/place/release/home |
| MoveIt → Agent | `/moveit_status` | `std_msgs/String` (JSON) | `{success, error_message}` |

### YOLO JSON 포맷 (`/obj_data`)
```json
{
  "class_name": "red_cup",
  "confidence": 0.923,
  "center_2d": {"u": 320, "v": 240},
  "distance_m": 0.452,
  "position_3d_camera_frame": {"x": 0.120, "y": -0.045, "z": 0.452}
}
```

### MoveIt Command JSON 예시
```json
// SCAN
{"cmd": "scan", "waypoints": [[x,y,z,r,p,y], ...]}

// PICK
{"cmd": "pick", "arm_group_name": "panda_arm", "object": {"name": "target_object", "shape": "cylinder", "dimensions": [0.12, 0.025], "pose_world": [x,y,z,r,p,y]}, ...}

// LIFT
{"cmd": "lift", "direction": "z+", "distance_m": 0.1, "frame": "world"}

// PLACE
{"cmd": "place", "object_name": "target_object", "target_pose_world": [x,y,z,r,p,y]}

// RELEASE / HOME
{"cmd": "release", "hand_open_pose": "open"}
{"cmd": "home", "arm_home_pose": "ready"}
```

---

## LLM 호출 지점 (3곳, tool use 없음)

| 지점 | Phase | 입력 | 출력 |
|------|-------|------|------|
| `parse_command` | P1 진입 | 자연어 명령 | `{"target_class_name": "red_cup"}` |
| `generate_pick_params` | P2 | target + pose_world + moveit_config | PICK JSON 전체 |
| `verify_pickup` | P3 | YOLO 프레임 목록 + target | `{"pickup_success": true, "reason": "..."}` |

---

## 환경 설정

### 빌드
```bash
cd ~/WorkspaceLLMagent
source /opt/ros/humble/setup.bash
colcon build --packages-select llm_agent_msgs
source install/setup.bash
colcon build --packages-select llm_agent
source install/setup.bash
```

### 실행
```bash
export ANTHROPIC_API_KEY=<your_key>
source /opt/ros/humble/setup.bash
source ~/WorkspaceLLMagent/install/setup.bash
ros2 launch llm_agent agent.launch.py
```

### 테스트
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 -m pytest src/llm_agent/test/ -v -p no:anyio
```

---

## 미결 사항 (외부팀 확인 필요)

| # | 항목 | 현재 가정값 | 확인 대상 |
|---|------|-----------|---------|
| 1 | camera_frame 이름 | `camera_color_optical_frame` | YOLO 팀 |
| 2 | `/moveit_status` 발행 시점·형식 | 명령 완료 후 1회 | MoveIt 팀 |
| 3 | arm_group_name / hand_frame | `panda_arm` / `panda_link8` | MoveIt 팀 |
| 4 | 오브젝트 형상·크기 (LLM 추정 vs config) | LLM 추정 | 팀 논의 필요 |

> 위 값들은 `src/llm_agent/config/agent.yaml`에서 수정 가능

---

## 다음 작업 (Sprint 3)

- [ ] S3-1: P1 루프 — 실제 scan waypoint 명령 동작 검증 (mock MoveIt 사용)
- [ ] S3-2: P2 루프 — TF 변환 + LLM PICK 파라미터 품질 검증
- [ ] S3-3: P3 루프 — YOLO 프레임 버퍼 기반 LLM pickup 판단 검증
- [ ] S3-4: P4 루프 — PLACE + RELEASE + HOME 동작 검증
- [ ] S4-1: mock YOLO / mock MoveIt으로 E2E 시나리오 테스트

---

## 참고 레포

- PANDA_ENV (MoveIt 파라미터 분석 기반): https://github.com/ROS2bootcamp/PANDA_ENV.git  
  → 로컬 경로: `~/Workspace/src/moveit_task_constructor/`
- ur3_ws: `~/ur3_ws/` (UR3 Gazebo 환경)
