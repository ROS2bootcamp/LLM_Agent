# LLM Agent — 설계 문서

> 원격 레포: https://github.com/ROS2bootcamp/LLM_Agent.git  
> 환경: ROS2 Humble + MoveIt2 + MoveIt Task Constructor  
> 작성일: 2026-06-06

---

## 1. 프로젝트 목적

CLI로 입력된 자연어 명령("빨간 컵 집어")을 받아, 로봇팔 카메라(YOLO)로 대상 오브젝트를 탐지하고, MoveIt 모듈을 통해 Pick & Place를 수행하는 **에이전트 노드**.

의사결정(탐지 성공 여부, pickup 검증, 파라미터 생성)을 LLM에 위임하고, 실제 모션 실행은 외부 MoveIt 모듈에 위임한다.

---

## 2. 용어 정의

| 용어 | 정의 |
|------|------|
| **Session** | CLI 명령 1회 입력부터 P4 완료(또는 최종 실패 보고)까지의 처리 단위 |
| **Phase** | Session 내 처리 단계. P1 → P2 → P3 → P4 순서로 진행 |
| **LLM** | 의사결정을 위임받는 언어 모델(Claude). 탐지 판단·파라미터 생성·검증에 사용 |
| **YOLO Node** | 로봇팔 카메라 영상을 분석해 `/obj_data` 토픽을 발행하는 외부 노드 |
| **MoveIt Module** | MoveIt Task Constructor 기반 모션 실행 노드. `/moveit_command` 수신, `/moveit_status` 발행 |
| **World Frame** | MoveIt 계획의 기준 좌표계 (`world`). 모든 3D 좌표는 이 프레임 기준 |
| **Camera Frame** | 로봇팔 끝에 장착된 카메라의 좌표계. YOLO가 발행하는 `position_3d_camera_frame`의 기준 |
| **TF Transform** | Camera Frame → World Frame 좌표 변환. ROS2 tf2 라이브러리로 수행 |
| **Scan Pattern** | P1에서 팔이 순회하는 사전 정의된 waypoint 목록 (config로 관리) |
| **Parameter Package** | LLM이 생성해 MoveIt Module에 전달하는 Pick 실행 파라미터 묶음 |
| **Grip Verify** | Pick 직후 YOLO 재구독으로 gripper 근처에 object가 감지되는지 확인하는 절차 |
| **Home Position** | 스캔 시작 전·실패 복귀 시 팔이 이동하는 기준 자세 (SRDF named state `ready`) |

---

## 3. 전체 시스템 구성

```
┌─────────────────────────────────────────────────────────────────┐
│                        외부 모듈                                  │
│                                                                   │
│  ┌──────────────┐   /obj_data (JSON String)                      │
│  │  YOLO Node   │ ─────────────────────────────────┐            │
│  │  (외부 팀)   │                                   │            │
│  └──────────────┘                                   ▼            │
│                                          ┌─────────────────────┐ │
│  stdin ──────────────────────────────── │    LLM Agent Node   │ │
│                                          │   (이 레포)          │ │
│                                          └──────────┬──────────┘ │
│                                                     │            │
│          /moveit_command (JSON String)              │            │
│          ◄────────────────────────────────────────  │            │
│          /moveit_status  (JSON String)              │            │
│          ─────────────────────────────────────────► │            │
│                                                     │            │
│  ┌──────────────────────────────────────┐          │            │
│  │         MoveIt Module (외부)          │          │            │
│  │  MoveIt Task Constructor 기반        │          │            │
│  └──────────────────────────────────────┘          │            │
└─────────────────────────────────────────────────────────────────┘
```

### 모듈별 책임 경계

| 모듈 | 책임 | 비책임 |
|------|------|--------|
| **LLM Agent** | 명령 파싱, Phase 관리, LLM 호출, 파라미터 생성, 의사결정, TF 변환 | 모션 실행, 영상 처리 |
| **YOLO Node** | 카메라 영상 분석, `/obj_data` 발행 | 좌표 변환, 의사결정 |
| **MoveIt Module** | 경로 계획, 모션 실행, gripper 제어 | 오브젝트 인식, 의사결정 |

---

## 4. ROS2 인터페이스 정의

### 4.1 LLM Agent가 Subscribe하는 토픽

| Topic | 타입 | 발행자 | 사용 Phase |
|-------|------|--------|-----------|
| `/obj_data` | `std_msgs/String` (JSON) | YOLO Node | P1, P2, P3 |
| `/moveit_status` | `std_msgs/String` (JSON) | MoveIt Module | P2, P3, P4 |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | ROS2 TF tree | P2 (좌표 변환) |

### 4.2 LLM Agent가 Publish하는 토픽

| Topic | 타입 | 수신자 | 사용 Phase |
|-------|------|--------|-----------|
| `/moveit_command` | `std_msgs/String` (JSON) | MoveIt Module | P1, P2, P3, P4 |
| `/llm_agent/phase` | `std_msgs/String` | 모니터링용 | 전 Phase |
| `/llm_agent/log` | `std_msgs/String` | 모니터링용 | 전 Phase |

### 4.3 YOLO JSON 메시지 형식 (`/obj_data`)

```json
{
  "class_name": "red_cup",
  "confidence": 0.923,
  "center_2d": {
    "u": 320,
    "v": 240
  },
  "distance_m": 0.452,
  "position_3d_camera_frame": {
    "x": 0.120,
    "y": -0.045,
    "z": 0.452
  }
}
```

### 4.4 MoveIt Command 메시지 형식 (`/moveit_command`)

Agent → MoveIt으로 전달하는 JSON 명령. `cmd` 필드로 종류 구분.

#### SCAN — P1 스캔 시작
```json
{
  "cmd": "scan",
  "waypoints": [
    [0.3, 0.0, 0.5, 0.0, 0.0, 0.0],
    [0.3, 0.3, 0.5, 0.0, 0.0, 0.0],
    [0.3, -0.3, 0.5, 0.0, 0.0, 0.0]
  ]
}
```
> `waypoints`: `[[x, y, z, roll, pitch, yaw], ...]` — world frame 기준

#### PICK — P2 Pick 실행
```json
{
  "cmd": "pick",
  "arm_group_name": "panda_arm",
  "hand_group_name": "hand",
  "hand_frame": "panda_link8",
  "hand_open_pose": "open",
  "hand_close_pose": "close",
  "object": {
    "name": "target_object",
    "shape": "cylinder",
    "dimensions": [0.12, 0.025],
    "pose_world": [0.4, 0.1, 0.06, 0.0, 0.0, 0.0]
  },
  "grasp_frame_transform": [0.0, 0.0, 0.1, 1.571, 0.785, 1.571],
  "approach_min_dist": 0.08,
  "approach_max_dist": 0.15,
  "lift_min_dist": 0.05,
  "lift_max_dist": 0.15,
  "max_solutions": 8
}
```

> `pose_world`: `[x, y, z, roll, pitch, yaw]` — world frame 기준  
> `grasp_frame_transform`: hand_frame 기준 TCP 오프셋 `[x,y,z,r,p,y]`  
> `object.pose_world.z`: 원통 중심 = 바닥 z + height/2

#### LIFT — P3 수직 상승
```json
{
  "cmd": "lift",
  "direction": "z+",
  "distance_m": 0.1,
  "frame": "world"
}
```

#### PLACE — P4 배치
```json
{
  "cmd": "place",
  "object_name": "target_object",
  "target_pose_world": [0.6, -0.2, 0.06, 0.0, 0.0, 0.0],
  "place_surface_offset": 0.001
}
```

#### RELEASE — P4 gripper 해제
```json
{
  "cmd": "release",
  "hand_open_pose": "open"
}
```

#### HOME — 홈 포지션 복귀
```json
{
  "cmd": "home",
  "arm_home_pose": "ready"
}
```

### 4.5 MoveIt Status 메시지 형식 (`/moveit_status`)

MoveIt → Agent로 전달하는 실행 결과 JSON.

```json
{
  "cmd_echo": "pick",
  "success": true,
  "phase": "P2",
  "error_code": 0,
  "error_message": "",
  "timestamp": 1748700000.123
}
```

> `error_code`: MoveIt ErrorCode (0 = SUCCESS)

---

## 5. Phase 상세 정의

### Phase 전환 규칙

```
[CLI 입력]
    │
    ▼
  [ P1 ]  탐지 성공 ──────────────────────────────► [ P2 ]
    │                                                   │
    │ 타임아웃(30s)×3회                          grip 성공(YOLO 확인)
    │                                                   │
    ▼                                                   ▼
 [실패 보고]                                         [ P3 ]
                                                        │
                                          ┌─ 성공 ──► [ P4 ] ──► [세션 종료]
                                          │
                                          └─ 실패 ──► HOME ──► [ P1 ]
                                                               (P1 재시도 카운터 공유)
```

---

### P1: 오브젝트 탐색

**목적**: 카메라가 달린 팔을 사전 정의된 waypoint 순서로 이동하며 CLI에서 지정한 오브젝트를 YOLO로 탐지한다.

**입력**: CLI 자연어 명령에서 LLM이 추출한 `target_class_name`

**처리 흐름**:

```
1. scan waypoints 로드 (config/scan_waypoints.yaml)
2. SCAN 명령 → MoveIt: 팔 움직임 시작
3. /obj_data 구독 시작
4. 루프 (최대 30초):
   a. YOLO 수신 프레임마다:
      - class_name == target_class_name AND confidence >= CONFIDENCE_THRESHOLD?
        → 탐지 성공, 해당 프레임의 position_3d_camera_frame 저장
        → P2 전환
   b. 30초 경과 → 타임아웃 처리
5. 타임아웃 시:
   - p1_retry_count += 1
   - p1_retry_count <= 3 → HOME 후 P1 재시작
   - p1_retry_count > 3  → 최종 실패 보고, 세션 종료
```

**LLM 역할**: 탐지 루프는 결정론적 코드로 처리. LLM은 P1 진입 시 CLI 명령에서 `target_class_name`을 추출하는 데 사용.

**파라미터**:
- `CONFIDENCE_THRESHOLD = 0.7` (config로 관리)
- `SCAN_TIMEOUT_SEC = 30`
- `P1_MAX_RETRY = 3`

**출력**: 탐지된 오브젝트의 `position_3d_camera_frame` (P2로 전달)

---

### P2: 파라미터 생성 및 Pick 실행

**목적**: YOLO에서 받은 카메라 프레임 좌표를 world 프레임으로 변환하고, LLM이 MoveIt 파라미터를 생성해 Pick을 실행한다.

**처리 흐름**:

```
1. TF 변환: position_3d_camera_frame → pose_world
   - tf2.lookup_transform(camera_frame, world_frame, Time())
   - TransformStamped 적용 → geometry_msgs/Pose (world frame)

2. /obj_data 재구독 (최신 프레임으로 좌표 갱신, 최대 1초 대기)

3. LLM 호출:
   Input:  target_class_name, pose_world, object 물리 속성
   Output: PICK 명령 JSON (파라미터 패키지 완성)

4. PICK 명령 → /moveit_command 발행

5. /moveit_status 대기 (성공/실패)

6. Grip 검증 (YOLO):
   - /obj_data 구독, 2초 내 수신 프레임 확인
   - class_name 매칭 AND position_3d_camera_frame.z < GRIP_DISTANCE_THRESHOLD?
     → grip 성공 → P3 전환
   - 조건 불만족 → grip 실패 처리

7. grip 실패 시:
   - p2_retry_count += 1
   - p2_retry_count <= 3 → 2번으로 돌아가 재시도
   - p2_retry_count > 3  → 실패 보고, 세션 종료
```

**LLM 역할**: `pose_world`와 물체 속성을 받아 PICK JSON 파라미터 패키지를 생성. 오브젝트 형상(CYLINDER/BOX), 크기 추정, `grasp_frame_transform` 결정에 개입.

**파라미터**:
- `GRIP_DISTANCE_THRESHOLD = 0.15` (m, gripper 근처 기준)
- `P2_MAX_RETRY = 3`
- `GRIP_VERIFY_WINDOW_SEC = 2.0`

**좌표 변환 상세**:
```
position_3d_camera_frame (x, y, z)
    │
    ▼  tf2.do_transform_point()
    │  transform: camera_link → world
    ▼
pose_world (x, y, z) in world frame
    │
    ▼  z 보정: pose_world.z += object_height / 2  (바닥 → 원통 중심)
    ▼
PICK 명령의 object.pose_world
```

**미결 사항**:
- `camera_frame` 이름: YOLO 노드 팀에게 확인 필요 (`camera_link`? `camera_color_optical_frame`?)
- 오브젝트 형상/크기: LLM 추정 vs config 사전 정의 중 결정 필요

---

### P3: Pickup 검증

**목적**: pick 직후 팔을 10cm 수직 상승시키고, YOLO가 gripper 근처에서 오브젝트를 여전히 감지하는지 LLM이 판단한다.

**처리 흐름**:

```
1. LIFT 명령 → /moveit_command 발행 (distance_m: 0.1, direction: z+)
2. /moveit_status 대기 (lift 완료)
3. /obj_data 구독, 2초간 수신 프레임 수집
4. LLM 호출:
   Input:  최근 N개 YOLO 프레임, target_class_name
   Output: {"pickup_success": bool, "reason": str}
5. pickup_success == true  → P4 전환
6. pickup_success == false:
   - RELEASE 명령 발행 (gripper 해제)
   - HOME 명령 발행 (홈 복귀)
   - p1_retry_count += 1  ← P1과 동일 카운터 사용
   - p1_retry_count <= 3 → P1 재시작
   - p1_retry_count > 3  → 최종 실패 보고
```

**LLM 판단 기준**:
- 수집된 YOLO 프레임 중 target_class_name이 `GRIP_VERIFY_WINDOW_SEC` 내에 감지됐는가
- `position_3d_camera_frame.z < GRIP_DISTANCE_THRESHOLD` (gripper 근처)
- confidence >= CONFIDENCE_THRESHOLD

**LLM 역할**: YOLO 프레임 목록을 분석해 pickup 성공 여부를 자연어 근거와 함께 판단.

---

### P4: 배치 및 세션 종료

**목적**: 사전 정의된 target 절대좌표로 이동해 오브젝트를 내려놓고, 홈으로 복귀 후 세션을 종료한다.

**처리 흐름**:

```
1. target_pose 로드 (config/targets.yaml 또는 하드코딩)
2. PLACE 명령 → /moveit_command 발행
3. /moveit_status 대기 (place 완료)
4. RELEASE 명령 → /moveit_command 발행
5. HOME 명령 → /moveit_command 발행
6. /moveit_status 대기 (home 완료)
7. CLI에 성공 보고 출력
8. 세션 종료
```

**LLM 역할**: P4는 결정론적 처리. LLM 호출 없음.

**파라미터 (config/targets.yaml)**:
```yaml
target_pose:
  x: 0.6
  y: -0.2
  z: 0.06
  roll: 0.0
  pitch: 0.0
  yaw: 0.0
```

---

## 6. LLM 에이전틱 루프 (Agentic Loop)

LLM 호출이 필요한 의사결정 지점은 3곳이며, 각각 단일 호출(single-shot)로 처리한다. multi-turn tool use 루프는 사용하지 않는다.

| 호출 지점 | Phase | Input | Output |
|----------|-------|-------|--------|
| **명령 파싱** | P1 진입 시 | 자연어 명령 | `target_class_name` |
| **파라미터 생성** | P2 | target_class_name + pose_world + 물체 속성 | PICK 명령 JSON |
| **Pickup 검증** | P3 | YOLO 프레임 목록 + target_class_name | `{pickup_success, reason}` |

```
LLM 호출 구조 (단일 호출):

system_prompt: [phase별 역할 정의]
user:          [structured context]
               ↓
assistant:     [JSON 응답]  ← tool use 아닌 JSON mode 또는 structured output
```

### Phase별 시스템 프롬프트

**명령 파싱 (P1 진입)**:
```
당신은 로봇 조작 시스템의 명령 파싱 에이전트입니다.
사용자의 자연어 명령에서 대상 오브젝트의 class_name을 추출하세요.
YOLO가 인식하는 label 형식(소문자, 언더스코어)으로 반환하세요.
응답 형식: {"target_class_name": "red_cup"}
```

**파라미터 생성 (P2)**:
```
당신은 로봇 Pick 파라미터를 생성하는 에이전트입니다.
주어진 오브젝트 정보와 world frame 좌표를 바탕으로
MoveIt PICK 명령 JSON을 완성하세요.
응답 형식: [PICK JSON 전체]
```

**Pickup 검증 (P3)**:
```
당신은 로봇 pickup 성공 여부를 판단하는 에이전트입니다.
제공된 YOLO 감지 데이터를 분석해 gripper가 오브젝트를 쥐고 있는지 판단하세요.
응답 형식: {"pickup_success": true/false, "reason": "..."}
```

---

## 7. 에러 처리 정책

| 상황 | 처리 |
|------|------|
| P1 탐지 타임아웃 | retry_count++ → 3회 초과 시 실패 보고 |
| P2 grip 실패 | retry_count++ → 3회 초과 시 실패 보고 |
| P3 pickup 실패 | RELEASE + HOME → P1 재시작 (retry_count 공유) |
| MoveIt status timeout | 10초 대기 후 실패로 처리, 해당 Phase retry |
| TF 변환 실패 | 최대 3초 retry → 실패 시 P2 진입 전 중단 |
| LLM API 오류 | 3회 retry (exponential backoff) → 실패 시 세션 종료 |

**공유 retry_count 정책**: P1 타임아웃, P3 pickup 실패 모두 동일한 `session_retry_count`를 증가시킨다. 최대 3이며 초과 시 세션 전체 종료.

---

## 8. 파일 구조

```
WorkspaceLLMagent/
├── src/
│   ├── llm_agent/
│   │   ├── llm_agent/
│   │   │   ├── __init__.py
│   │   │   ├── agent_node.py          # ROS2 Node, Phase 루프 관리
│   │   │   ├── phase_manager.py       # Phase 상태머신
│   │   │   ├── llm_client.py          # Anthropic API (단일 호출)
│   │   │   ├── context_builder.py     # LLM 입력 context 구성
│   │   │   ├── cli_reader.py          # stdin 명령 수신
│   │   │   ├── tf_transformer.py      # camera_frame → world_frame 변환
│   │   │   ├── yolo_subscriber.py     # /obj_data 구독 및 프레임 버퍼 관리
│   │   │   └── moveit_client.py       # /moveit_command 발행 + /moveit_status 수신
│   │   ├── config/
│   │   │   ├── agent.yaml             # 임계값, retry 횟수, 모델명
│   │   │   ├── scan_waypoints.yaml    # P1 스캔 waypoint 목록
│   │   │   └── targets.yaml           # P4 target 절대좌표
│   │   ├── launch/
│   │   │   └── agent.launch.py
│   │   ├── test/
│   │   │   ├── test_phase_manager.py
│   │   │   ├── test_llm_client.py
│   │   │   └── test_tf_transformer.py
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── setup.cfg
│   │
│   └── llm_agent_msgs/
│       ├── msg/
│       │   └── TaskStatus.msg
│       ├── srv/
│       │   ├── SetPhase.srv
│       │   └── AgentQuery.srv
│       ├── package.xml
│       └── CMakeLists.txt
│
└── DESIGN.md
```

> **변경**: 초기 설계의 `AgentTask.action` 및 action server 구조를 제거함.  
> CLI stdin 입력이므로 ROS2 action client가 없고, agent 자체가 루프를 직접 구동함.

---

## 9. 내부 컴포넌트 설계

### AgentNode (agent_node.py)
- `rclpy.Node` 상속
- `MultiThreadedExecutor` + `ReentrantCallbackGroup` 사용
- stdin 읽기는 별도 스레드, ROS2 callback과 분리
- Phase 루프는 명령 수신 시 `ThreadPoolExecutor`에서 실행 (spin loop 비차단)

### PhaseManager (phase_manager.py)
- Phase 전환 상태머신
- `session_retry_count` 관리
- Phase 별 시스템 프롬프트 반환

### YoloSubscriber (yolo_subscriber.py)
- `/obj_data` 구독 상시 유지
- 최근 N개 프레임을 deque 버퍼에 보관
- `wait_for_detection(class_name, timeout)` blocking API 제공

### TFTransformer (tf_transformer.py)
- `tf2_ros.Buffer` + `tf2_ros.TransformListener`
- `transform_to_world(point_camera, camera_frame)` → `geometry_msgs/Point` 반환

### MoveItClient (moveit_client.py)
- `/moveit_command` Publisher
- `/moveit_status` Subscriber
- `send_and_wait(command_dict, timeout)` → `{success, error_message}` 반환

### LLMClient (llm_client.py)
- Anthropic SDK, 동기 호출 (ThreadPoolExecutor 내에서 실행)
- tool use 없이 JSON 응답 파싱
- 3회 retry with exponential backoff

### CLIReader (cli_reader.py)
- 별도 daemon 스레드에서 `sys.stdin.readline()` 대기
- 명령 수신 시 `threading.Event` 또는 `queue.Queue`로 AgentNode에 전달

---

## 10. 설정 파일 스키마

### config/agent.yaml
```yaml
llm:
  model: "claude-sonnet-4-6"
  max_tokens: 1024
  retry_count: 3
  retry_backoff_sec: 1.0

scan:
  timeout_sec: 30
  confidence_threshold: 0.7

grip:
  verify_window_sec: 2.0
  distance_threshold_m: 0.15
  retry_count: 3

session:
  max_retry_count: 3

moveit:
  command_topic: "/moveit_command"
  status_topic: "/moveit_status"
  status_timeout_sec: 10.0
  arm_group_name: "panda_arm"
  hand_group_name: "hand"
  hand_frame: "panda_link8"
  hand_open_pose: "open"
  hand_close_pose: "close"
  arm_home_pose: "ready"
  grasp_frame_transform: [0.0, 0.0, 0.1, 1.571, 0.785, 1.571]

tf:
  camera_frame: "camera_color_optical_frame"  # YOLO팀 확인 필요
  world_frame: "world"
```

### config/scan_waypoints.yaml
```yaml
waypoints:
  - [0.3,  0.0,  0.5, 0.0, 0.785, 0.0]
  - [0.3,  0.25, 0.4, 0.0, 0.785, 0.3]
  - [0.3, -0.25, 0.4, 0.0, 0.785, -0.3]
  - [0.4,  0.0,  0.35, 0.0, 1.0, 0.0]
```

### config/targets.yaml
```yaml
place_target:
  x: 0.6
  y: -0.2
  z: 0.06
  roll: 0.0
  pitch: 0.0
  yaw: 0.0
```

---

## 11. 미결 사항 (YOLO팀 / MoveIt팀 확인 필요)

| # | 항목 | 현재 가정 | 확인 필요 대상 |
|---|------|----------|--------------|
| 1 | camera_frame 이름 | `camera_color_optical_frame` | YOLO 노드 팀 |
| 2 | TF를 YOLO 노드가 발행하는가, 별도 URDF 기반인가 | URDF + robot_state_publisher | MoveIt 팀 |
| 3 | `/moveit_status` 발행 시점 | 명령 완료 후 1회 발행 | MoveIt 팀 |
| 4 | MoveIt Module의 arm_group_name / hand_frame | `panda_arm` / `panda_link8` | MoveIt 팀 |
| 5 | 오브젝트 형상·크기 정보 | LLM이 추정 | 사전 config 정의 가능 여부 |
| 6 | `/obj_data` 발행 주기 | 30 FPS 가정 | YOLO 노드 팀 |

---

## 12. 세부 작업 단위 (Work Breakdown)

### Sprint 1 — 기반 구조 ✅ (완료)
- llm_agent_msgs 패키지 (msg/srv)
- llm_agent 패키지 스캐폴딩
- PhaseManager + 단위 테스트

### Sprint 2 — 핵심 컴포넌트
| ID | 작업 |
|----|------|
| S2-1 | `CLIReader`: stdin → queue 스레드 구현 |
| S2-2 | `YoloSubscriber`: /obj_data 구독 + 프레임 버퍼 + wait_for_detection() |
| S2-3 | `TFTransformer`: camera → world 좌표 변환 |
| S2-4 | `MoveItClient`: command 발행 + status 대기 (send_and_wait) |
| S2-5 | `LLMClient`: JSON 응답 파싱 + retry (tool use 없음) |

### Sprint 3 — Phase 루프 구현
| ID | 작업 |
|----|------|
| S3-1 | P1 루프: 스캔 명령 + YOLO 탐지 + 타임아웃/retry |
| S3-2 | P2 루프: TF 변환 + LLM 파라미터 생성 + PICK + grip verify |
| S3-3 | P3 루프: LIFT + YOLO 수집 + LLM pickup 검증 |
| S3-4 | P4 루프: PLACE + RELEASE + HOME + 세션 종료 |

### Sprint 4 — 통합 및 테스트
| ID | 작업 |
|----|------|
| S4-1 | AgentNode 통합 (CLIReader → PhaseManager → Phase 루프) |
| S4-2 | Mock YOLO / Mock MoveIt으로 E2E 시나리오 테스트 |
| S4-3 | 실패 케이스 테스트 (타임아웃, grip 실패, P3 실패) |
| S4-4 | Gazebo 시뮬레이션 연동 테스트 |
