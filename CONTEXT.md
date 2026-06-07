# 작업 컨텍스트 — LLM Agent

> 다른 환경에서 Claude Code 세션을 이어받을 때 참고용.  
> 최종 업데이트: 2026-06-07 (pull 반영)

---

## 프로젝트 개요

- **원격 레포**: https://github.com/ROS2bootcamp/LLM_Agent.git
- **목적**: CLI 자연어 명령 → YOLO 오브젝트 탐지 → MoveIt Pick & Place 자동화
- **환경**: ROS2 Humble + MoveIt2 + MoveIt Task Constructor
- **로봇**: **UR3 + Robotiq 2F-85** (Gazebo Ignition)
- **LLM**: **Google Gemini** (`gemini-2.5-flash`)
- **상세 설계**: `DESIGN.md` / 아키텍처 개요: `ARCHITECTURE.md`

### 연계 레포 (이 레포에서 구현하지 않음)

| 레포 | 역할 | 담당 |
|------|------|------|
| https://github.com/ROS2bootcamp/ROBOT_VISION.git | YOLO 노드, `/vision/detection_results` 발행 | 별도 팀 |
| https://github.com/ROS2bootcamp/PANDA_ENV.git | MoveIt Module (서비스 서버로 개조 예정) | 별도 팀 |

---

## 현재 상태 (Sprint 2 완료)

### 파일 구조

```
WorkspaceLLMagent/
├── .env.example                        ← API 키 예시 (cp .env.example .env 후 키 입력)
├── .gitignore
├── DESIGN.md                           ← 전체 설계 (인터페이스, Phase 로직, 미결 사항)
├── CONTEXT.md                          ← 이 파일
├── ARCHITECTURE.md                     ← 시스템 구성 개요
├── MOVEIT_INTERFACE.md                 ← MoveIt 서비스 계약 상세
├── MOCK_E2E.md                         ← E2E mock 테스트 가이드
└── src/
    ├── llm_agent_msgs/                 ← 커스텀 메시지 패키지 (C++, ament_cmake)
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── srv/MoveItExecute.srv       ← MoveIt 서비스 계약 (agent 정의, MoveIt팀이 구현)
    │
    └── llm_agent/                      ← 메인 에이전트 패키지 (Python, ament_python)
        ├── package.xml
        ├── setup.py / setup.cfg
        ├── resource/llm_agent
        ├── llm_agent/
        │   ├── agent_node.py           ← 메인 노드: P1→P4 Phase 루프
        │   ├── phase_manager.py        ← Phase 상태머신 + session_retry_count
        │   ├── llm_client.py           ← Gemini API (structured output, 2곳 호출)
        │   ├── yolo_subscriber.py      ← /vision/detection_results 구독 + 버퍼
        │   ├── tf_transformer.py       ← base_link → world TF 변환
        │   ├── moveit_client.py        ← /moveit/execute 서비스 클라이언트
        │   ├── cli_reader.py           ← stdin 읽기 (daemon thread)
        │   └── tools/__init__.py
        ├── config/
        │   ├── agent.yaml              ← 임계값, Gemini 설정, MoveIt/YOLO 설정
        │   ├── scan_waypoints.yaml     ← P1 스캔 waypoint
        │   ├── targets.yaml            ← P4 절대좌표
        │   └── objects.yaml            ← YOLO class → shape/dimensions 매핑
        ├── launch/agent.launch.py
        └── test/
            ├── test_phase_manager.py
            ├── mock_yolo_publisher.py  ← YOLO 노드 mock (E2E 테스트용)
            ├── mock_moveit_server.py   ← MoveIt 서비스 서버 mock
            └── test_flow_mock.py       ← E2E 시나리오 테스트
```

---

## 4-Phase 아키텍처

```
CLI stdin 입력
    │  [LLM] target_class_name 추출 (Gemini, parse_command)
    ▼
[ P1 ] YOLO로 target 탐색 (30s 타임아웃 × 최대 3회, class_name 매칭)
    │  탐지 → position_3d_base_frame 획득
    ▼
[ P2 ] base_link→world TF 변환 + config Object Spec 조합 → /moveit/execute PICK 호출
    │  YOLO grip 검증 (distance_m < 0.15m, 3회 retry)
    ▼
[ P3 ] /moveit/execute LIFT 10cm → YOLO 수집 → [LLM] pickup 성공 판단 (verify_pickup)
    │  실패 → RELEASE + HOME → P1 재시작 (session_retry_count 공유)
    ▼
[ P4 ] /moveit/execute PLACE → RELEASE → HOME → 세션 종료
```

**LLM 호출: 2곳만** (tool use 없음, structured JSON 응답)
- P1 진입: `parse_command` → `{"target_class_name": "cup"}`
- P3: `verify_pickup` → `{"pickup_success": true, "reason": "..."}`
- ~~P2 파라미터 생성~~: config `objects.yaml` 기반 결정론적 조립으로 대체

---

## 인터페이스 요약

### YOLO → Agent (`/vision/detection_results`, std_msgs/String JSON)

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
      "position_3d_camera_frame": {"X": 0.12, "Y": -0.04, "Z": 0.45},
      "position_3d_base_frame":   {"X": 0.40, "Y": 0.10,  "Z": 0.06}
    }
  ]
}
```
> - 좌표 키: **대문자 X/Y/Z**
> - 에이전트는 **`position_3d_base_frame`** 사용 (YOLO가 camera→base TF 완료)
> - 같은 frame에 다중 객체 가능 → `selection_policy: highest_conf` (기본)

### Agent → MoveIt (`/moveit/execute`, llm_agent_msgs/MoveItExecute)

```
# MoveItExecute.srv
string cmd           # "scan"|"pick"|"lift"|"place"|"release"|"home"
string params_json   # 명령별 파라미터 JSON
---
bool   success
int32  error_code    # 0=SUCCESS
string error_message
```

> **계약 원칙**: 에이전트가 .srv 정의, MoveIt 팀이 서버 구현

### cmd별 params_json 스키마 요약

| cmd | 핵심 파라미터 |
|-----|-------------|
| `scan` | `{"waypoints": [[x,y,z,r,p,y], ...]}` |
| `pick` | `{"arm_group_name", "object": {"name","shape","dimensions","pose_world"}, "grasp_frame_transform", ...}` |
| `lift` | `{"direction":"z+", "distance_m":0.1, "frame":"world"}` |
| `place` | `{"object_name", "target_pose_world":[x,y,z,r,p,y], "place_surface_offset":0.001}` |
| `release` | `{"hand_open_pose":"open"}` |
| `home` | `{"arm_home_pose":"home"}` |

---

## 주요 설정 (`config/agent.yaml` 발췌)

```yaml
llm:
  provider: "gemini"
  model: "gemini-2.5-flash"

yolo:
  detection_topic: "/vision/detection_results"

moveit:
  service_name: "/moveit/execute"
  arm_group_name: "ur_manipulator"
  hand_frame: "robotiq_2f_85_tcp"
  arm_home_pose: "home"

tf:
  base_frame: "base_link"
  world_frame: "world"
```

### Object Spec (`config/objects.yaml`)

```yaml
objects:
  cup:     {shape: cylinder, dimensions: [0.12, 0.025]}
  bottle:  {shape: cylinder, dimensions: [0.20, 0.033]}
default_spec: {shape: cylinder, dimensions: [0.10, 0.03]}  # 미등록 class 폴백
```

---

## 환경 설정

### 1. API 키 설정

```bash
cp .env.example .env
# .env 파일에 GEMINI_API_KEY=<your_key> 입력
```

### 2. 빌드

```bash
cd ~/WorkspaceLLMagent
source /opt/ros/humble/setup.bash
colcon build --packages-select llm_agent_msgs
source install/setup.bash
colcon build --packages-select llm_agent
source install/setup.bash
```

### 3. 실행

```bash
source /opt/ros/humble/setup.bash
source ~/WorkspaceLLMagent/install/setup.bash
ros2 launch llm_agent agent.launch.py
```

### 4. 테스트

```bash
# PhaseManager 단위 테스트
python3 -m pytest src/llm_agent/test/test_phase_manager.py -v -p no:anyio

# Mock E2E (MOCK_E2E.md 참고)
# 터미널 1: mock YOLO 발행
python3 src/llm_agent/test/mock_yolo_publisher.py

# 터미널 2: mock MoveIt 서버
python3 src/llm_agent/test/mock_moveit_server.py

# 터미널 3: E2E 시나리오
python3 -m pytest src/llm_agent/test/test_flow_mock.py -v -p no:anyio
```

---

## 이전 설계 대비 주요 변경 사항 (이번 pull)

| 항목 | 이전 | 현재 |
|------|------|------|
| LLM Provider | Anthropic Claude (`claude-sonnet-4-6`) | **Google Gemini** (`gemini-2.5-flash`) |
| API Key | `ANTHROPIC_API_KEY` | **`GEMINI_API_KEY`** (`.env` 파일) |
| YOLO 토픽 | `/obj_data` | **`/vision/detection_results`** |
| YOLO JSON 구조 | 단일 객체 | **`objects[]` 배열**, 좌표 키 대문자 |
| YOLO 좌표 | `position_3d_camera_frame` (에이전트가 TF 변환) | **`position_3d_base_frame`** (YOLO가 TF 완료) |
| MoveIt 인터페이스 | Topic 기반 | **ROS2 Service** (`/moveit/execute`) |
| MoveIt 서비스 타입 | - | **`llm_agent_msgs/MoveItExecute`** |
| 로봇 모델 | Panda | **UR3 + Robotiq 2F-85** |
| P2 파라미터 생성 | LLM 호출 | **config 결정론적 조립** (`objects.yaml`) |
| LLM 호출 지점 | 3곳 | **2곳** (parse_command + verify_pickup) |
| llm_agent_msgs | TaskStatus, SetPhase, AgentQuery | **MoveItExecute.srv 만** |

---

## 미결 사항

| # | 항목 | 현재 가정 | 확인 대상 |
|---|------|----------|---------|
| 1 | `/moveit/execute` 서버 구현 완료 여부 | 개조 예정 | MoveIt 팀 |
| 2 | `base_link`→`world` TF 관계 | identity (정적 변환) | MoveIt 팀 |
| 3 | UR3 arm_group_name / hand_frame | `ur_manipulator` / `robotiq_2f_85_tcp` | MoveIt 팀 |
| 4 | objects.yaml 실제 치수 | Gazebo SDF 기준으로 확정 필요 | 팀 전체 |

---

## 다음 작업 (Sprint 3)

- [ ] S3-1: Mock E2E 전체 시나리오 실행 (`test_flow_mock.py`)
- [ ] S3-2: Gemini API 실제 호출 검증 (parse_command, verify_pickup)
- [ ] S3-3: MoveIt 서비스 서버 구현 확인 후 실기 연동 테스트
- [ ] S3-4: Gazebo 시뮬레이션 통합 테스트 (UR3 + ROBOT_VISION + LLM Agent)
