# 작업 컨텍스트 — LLM Agent

> 이 파일은 다른 환경에서 Claude Code 세션을 이어받을 때 참고용으로 사용합니다.  
> 최종 업데이트: 2026-06-07

---

## 프로젝트 개요

- **원격 레포**: https://github.com/ROS2bootcamp/LLM_Agent.git
- **목적**: CLI 자연어 명령 → YOLO 오브젝트 탐지 → MoveIt Pick & Place 자동화
- **환경**: ROS2 Humble + MoveIt2 + MoveIt Task Constructor
- **로봇**: **UR3 + Robotiq 2F-85** (Gazebo Ignition) — 확정
- **설계 문서**: `DESIGN.md` (상세 설계 전체 포함)
- **연계 레포**:
  - YOLO/카메라: https://github.com/ROS2bootcamp/ROBOT_VISION.git
  - MoveIt/MTC: https://github.com/ROS2bootcamp/PANDA_ENV.git (UR3)

---

## 현재 상태 (Sprint 1 완료 / Sprint 2 구현됨 — 인터페이스 갱신 반영 필요)

> ⚠️ 문답 결과 인터페이스가 변경됨. 아래 파일 중 `yolo_subscriber`/`tf_transformer`/`moveit_client`는
> 신규 계약(서비스·토픽·포맷)에 맞춰 **수정 필요**. `MoveItExecute.srv`와 `objects.yaml`은 **신규**.

### 파일 목록

```
src/
├── llm_agent_msgs/          ← ROS2 커스텀 메시지 패키지 (C++, ament_cmake)
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── srv/MoveItExecute.srv   ← 유일 인터페이스 (미사용 msg/srv G2로 제거)
│
└── llm_agent/               ← 메인 에이전트 패키지 (Python, ament_python)
    ├── package.xml
    ├── setup.py / setup.cfg
    ├── resource/llm_agent
    ├── llm_agent/
    │   ├── __init__.py
    │   ├── agent_node.py        ← 메인 노드 (Phase 루프 전체 구현)
    │   ├── phase_manager.py     ← Phase 상태머신 + retry counter
    │   ├── llm_client.py        ← Gemini API (google-genai, .env, JSON 응답, no tool use)
    │   ├── yolo_subscriber.py   ← /vision/detection_results 구독 (objects[] 파싱) ⚠️수정
    │   ├── tf_transformer.py    ← base_link → world 정적 변환 ⚠️수정(축소)
    │   ├── moveit_client.py     ← /moveit/execute 서비스 client ⚠️수정
    │   ├── cli_reader.py        ← stdin 명령 읽기 (daemon thread)
    │   └── tools/__init__.py
    ├── config/
    │   ├── agent.yaml           ← UR3 group/frame, yolo 토픽, moveit 서비스 설정
    │   ├── scan_waypoints.yaml  ← P1 스캔 waypoint 목록
    │   ├── objects.yaml         ← class_name → shape/dimensions (Object Spec) ★신규
    │   └── targets.yaml         ← P4 target 절대좌표
    ├── launch/agent.launch.py
    └── test/test_phase_manager.py
```

---

## 4-Phase 아키텍처 요약

```
CLI stdin 입력
    │  LLM: target_class_name 추출 (YOLOv8/COCO 라벨)
    ▼
[ P1 ] YOLO로 오브젝트 탐색 (30s 타임아웃 × 최대 3회 재시도)
    │  탐지 성공 → position_3d_base_frame 획득
    ▼
[ P2 ] base→world 정적변환 + config Object Spec 조회 + PICK 서비스 호출 (LLM 없음)
    │  YOLO grip 검증 성공 (3회 retry)
    ▼
[ P3 ] 10cm 수직 상승(LIFT 서비스) + LLM이 YOLO로 pickup 성공 판단
    │  실패 → RELEASE+HOME → P1 재시작 (retry_count 공유)
    ▼
[ P4 ] target 절대좌표로 PLACE + RELEASE + HOME 서비스 → 세션 종료
```

---

## 외부 인터페이스

| 방향 | 채널 | 타입 | 비고 |
|------|------|------|------|
| YOLO → Agent | `/vision/detection_results` (topic) | `std_msgs/String` (JSON) | 상시 구독, `objects[]` 배열 |
| Agent ↔ MoveIt | `/moveit/execute` (**service**) | `llm_agent_msgs/MoveItExecute` | agent=client, MoveIt=server |

### YOLO JSON 포맷 (`/vision/detection_results`)
```json
{
  "timestamp_ns": 1718001234567890,
  "num_detections": 1,
  "objects": [
    {
      "class_name": "cup",
      "confidence": 0.892,
      "center_2d": {"u": 320, "v": 240},
      "distance_m": 0.452,
      "position_3d_camera_frame": {"X": 0.120, "Y": -0.045, "Z": 0.452},
      "position_3d_base_frame":   {"X": 0.400, "Y": 0.100,  "Z": 0.060}
    }
  ]
}
```
> 타겟 좌표는 `position_3d_base_frame`(camera→base TF 완료) 사용. class_name은 YOLOv8/COCO 형식.

### MoveIt 서비스 계약 (`llm_agent_msgs/MoveItExecute`)
```
string cmd            # scan | pick | lift | place | release | home
string params_json    # 명령별 파라미터 (JSON)
---
bool   success
int32  error_code     # MoveItErrorCodes (0 = SUCCESS)
string error_message
```
```jsonc
// params_json 예시
// scan:    {"waypoints": [[x,y,z,r,p,y], ...]}
// pick:    {"arm_group_name":"ur_manipulator", "hand_frame":"robotiq_2f_85_tcp",
//           "object":{"name":"target_object","shape":"cylinder","dimensions":[0.12,0.025],"pose_world":[x,y,z,r,p,y]},
//           "grasp_frame_transform":[0,0,0.13,3.1416,0,0], ...}
// lift:    {"direction":"z+", "distance_m":0.1, "frame":"world"}
// place:   {"object_name":"target_object", "target_pose_world":[x,y,z,r,p,y]}
// release: {"hand_open_pose":"open"}
// home:    {"arm_home_pose":"home"}
```

---

## LLM 호출 지점 (2곳, tool use 없음)

| 지점 | Phase | 입력 | 출력 |
|------|-------|------|------|
| `parse_command` | P1 진입 | 자연어 명령 | `{"target_class_name": "cup"}` (YOLOv8/COCO 라벨) |
| `verify_pickup` | P3 | YOLO 프레임 목록 + target | `{"pickup_success": true, "reason": "..."}` |

> P2의 `generate_pick_params`는 **config-only 정책**으로 결정론적 조립으로 대체되어 LLM 호출에서 제외됨
> (shape/dimensions=config/objects.yaml, pose=YOLO base_frame+TF, grasp 파라미터=config).

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
# API 키는 .env 파일로 관리 (export 불필요)
cp .env.example .env && ${EDITOR:-nano} .env   # GEMINI_API_KEY=<your_key> 입력
source /opt/ros/humble/setup.bash
source ~/WorkspaceLLMagent/install/setup.bash
ros2 launch llm_agent agent.launch.py
```
> `.env` 는 실행 시 CWD/상위 디렉터리에서 자동 탐색(python-dotenv). 보통 워크스페이스 루트에 둠.

### 테스트
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 -m pytest src/llm_agent/test/ -v -p no:anyio
```

---

## 미결 사항 — 문답으로 해결 완료 (2026-06-06)

| # | 항목 | 결정/확인 결과 | 출처 |
|---|------|---------------|------|
| 1 | 로봇 플랫폼 | **UR3 + Robotiq 2F-85** | 사용자 확정 / PANDA_ENV |
| 2 | arm/hand group·frame·home | `ur_manipulator` / `gripper` / `robotiq_2f_85_tcp` / `home` | PANDA_ENV config |
| 3 | YOLO 토픽·포맷 | `/vision/detection_results`, `{timestamp_ns,num_detections,objects[]}` | ROBOT_VISION |
| 4 | camera_frame / TF | YOLO가 camera_link→base_link 변환 완료 → 에이전트는 base_link→world(정적)만 | ROBOT_VISION |
| 5 | 오브젝트 형상·크기 | **config 사전 정의만** (`config/objects.yaml`) | 사용자 확정 |
| 6 | MoveIt 연동 | **ROS2 서비스** `/moveit/execute` (agent가 계약 정의, client) | 사용자 확정 |
| 7 | 다중 탐지 선택 | **최고 confidence** 기본 (`scan.selection_policy`: highest_conf\|nearest) | 사용자 확정 2026-06-07 |
| 8 | LLM 응답 견고성 | structured output **스키마 강제** + required 키 재검증 (조용한 None 실패 차단) | 사용자 확정 2026-06-07 |
| 9 | Object Spec 정책 | box 지원(바닥→중심), grasp 전역 고정, 미등록 class `default_spec` 폴백 | 사용자 확정 2026-06-07 |

### 잔여 확인 항목 (구현 단계)
| △ | MoveIt 서비스 서버 | MoveIt팀이 `ur3_pick_place.py`를 `MoveItExecute` 계약 기반 서버로 개조 필요 |
| △ | base_link↔world 정적변환 실측값 | identity 가정 — UR3 URDF/TF로 확인 필요 |
| ✅ | G2: 미사용 인터페이스 | 제거 완료 — `TaskStatus.msg`/`SetPhase.srv`/`AgentQuery.srv` + action_msgs/builtin_interfaces 의존 삭제, `MoveItExecute.srv`만 유지 |
| △ | G3: P2 grip 검증 메트릭 | distance_m 잠정 유지. 메트릭은 **이슈 #2(카메라 장착) 해소 후 확정** — 후속 체크리스트는 이슈 #2 본문에 기록 |

> 값들은 `src/llm_agent/config/agent.yaml`, `config/objects.yaml`, `config/targets.yaml`에서 수정 가능

---

## 다음 작업

### Sprint 2 보완 (인터페이스 갱신) ✅ 구현 완료
- [x] S2-6a: `llm_agent_msgs/srv/MoveItExecute.srv` 정의 + CMakeLists 등록
- [x] S2-2: `yolo_subscriber.py` → `/vision/detection_results` + `objects[]`/대문자 키 파싱
- [x] S2-3: `tf_transformer.py` → `base_link`→`world` 정적 변환으로 축소
- [x] S2-4: `moveit_client.py` → `/moveit/execute` 서비스 client(`call_and_wait`)
- [x] S2-6b: P2 PICK 파라미터 결정론적 조립(`_build_pick_params`) + `objects.yaml` 로딩
- [x] agent_node 전 Phase 서비스 호출/Object Spec/ base_frame TF 반영
> 미빌드/미실행 상태(ROS2 환경 필요). 다음: colcon build 후 mock 서버로 S4-1 E2E

### Sprint 3 — Phase 루프
- [ ] S3-1: P1 루프 — scan 서비스 호출 + YOLO 탐지 + 타임아웃/retry
- [ ] S3-2: P2 루프 — base→world 변환 + Object Spec 조회 + PICK 서비스 + grip verify (LLM 없음)
- [ ] S3-3: P3 루프 — LIFT 서비스 + YOLO 수집 + LLM pickup 판단
- [ ] S3-4: P4 루프 — PLACE + RELEASE + HOME 서비스 + 세션 종료
- [x] S4-1 (1차): 오케스트레이션 흐름 mock 검증 — `test/test_flow_mock.py` 10 시나리오 통과(ROS 불필요)
- [x] G1 수정: P4 place 실패가 성공으로 보고되던 버그 → place 성공 검증 + 실패 시 안전 복귀
- [ ] S4-1 (2차): Linux 워크스페이스 실제 ROS2 E2E — `test/mock_moveit_server.py`+`mock_yolo_publisher.py` (가이드: `MOCK_E2E.md`)

---

## 참고 레포

- **PANDA_ENV** (UR3 MTC, MoveIt 파라미터 출처): https://github.com/ROS2bootcamp/PANDA_ENV.git
  - 핵심: `src/ur3_mtc_pick_place/config/ur3_mtc_config.yaml`, `scripts/ur3_pick_place.py`
  - → MoveIt팀이 이 스크립트를 `/moveit/execute` 서비스 서버로 개조 예정
- **ROBOT_VISION** (YOLO 노드): https://github.com/ROS2bootcamp/ROBOT_VISION.git
  - 핵심: `robot_vision/yolo_detector.py` (`/vision/detection_results` 발행, COCO/YOLOv8)
- ur3_ws: `~/ur3_ws/` (UR3 Gazebo 환경)
