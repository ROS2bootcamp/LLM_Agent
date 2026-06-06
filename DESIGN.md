# LLM Agent — 설계 문서

> 원격 레포: https://github.com/ROS2bootcamp/LLM_Agent.git
> 환경: ROS2 Humble + MoveIt2 + MoveIt Task Constructor
> 로봇: **UR3 + Robotiq 2F-85** (Gazebo Ignition)
> 작성일: 2026-06-06 / 최종 갱신: 2026-06-06 (미결 사항 문답 반영)

---

## 1. 프로젝트 목적

CLI로 입력된 자연어 명령("컵 집어")을 받아, 로봇팔 카메라(YOLO)로 대상 오브젝트를 탐지하고, MoveIt 모듈을 통해 Pick & Place를 수행하는 **에이전트 노드**.

의사결정(명령 파싱, pickup 검증)을 LLM에 위임하고, 실제 모션 실행은 외부 MoveIt 모듈(서비스 서버)에 위임한다. 영상 인식과 1차 좌표 변환은 외부 YOLO 노드가 담당한다.

> **연계 레포**
> - YOLO/카메라: https://github.com/ROS2bootcamp/ROBOT_VISION.git (`/vision/detection_results` 발행)
> - MoveIt/MTC: https://github.com/ROS2bootcamp/PANDA_ENV.git (UR3 `ur3_pick_place.py`, 현재 param 구동 → 서비스 서버로 개조 예정)

---

## 2. 용어 정의

| 용어 | 정의 |
|------|------|
| **Session** | CLI 명령 1회 입력부터 P4 완료(또는 최종 실패 보고)까지의 처리 단위 |
| **Phase** | Session 내 처리 단계. P1 → P2 → P3 → P4 순서로 진행 |
| **LLM** | 의사결정을 위임받는 언어 모델(Claude). 명령 파싱·pickup 검증에 사용 |
| **YOLO Node** | 로봇팔 카메라 영상을 분석해 `/vision/detection_results` 토픽을 발행하는 외부 노드 (ROBOT_VISION) |
| **MoveIt Module** | MoveIt Task Constructor 기반 모션 실행 노드. 본 에이전트가 정의한 **서비스 서버**를 구현 (PANDA_ENV 개조) |
| **World Frame** | MoveIt 계획의 기준 좌표계 (`world`) |
| **Base Frame** | UR3 베이스 좌표계 (`base_link`). YOLO가 변환해 주는 `position_3d_base_frame`의 기준 |
| **Camera Frame** | 카메라 좌표계 (`camera_link`). YOLO 내부에서만 사용, 에이전트는 직접 다루지 않음 |
| **Scan Pattern** | P1에서 팔이 순회하는 사전 정의된 waypoint 목록 (config로 관리) |
| **Object Spec** | class_name → shape/dimensions 매핑. **config에 사전 정의** (LLM 추정 안 함) |
| **Grip Verify** | Pick 직후 YOLO 재구독으로 gripper 근처에 object가 감지되는지 확인하는 절차 |
| **Home Position** | 스캔 시작 전·실패 복귀 시 팔이 이동하는 기준 자세 (UR3 SRDF named state `home`) |

---

## 3. 전체 시스템 구성

```
┌─────────────────────────────────────────────────────────────────┐
│                        외부 모듈                                  │
│                                                                   │
│  ┌──────────────┐  /vision/detection_results (JSON String)       │
│  │  YOLO Node   │ ─────────────────────────────────┐            │
│  │ (ROBOT_VISION)│  camera_link→base_link TF 포함    │            │
│  └──────────────┘                                   ▼            │
│                                          ┌─────────────────────┐ │
│  stdin ──────────────────────────────── │    LLM Agent Node   │ │
│                                          │   (이 레포)          │ │
│                                          └──────────┬──────────┘ │
│                                                     │            │
│          /moveit/execute (ROS2 Service)             │            │
│          요청: {cmd, params_json}                    │            │
│          ◄──── client ──────────────────────────────┤            │
│          응답: {success, error_code, error_message} │            │
│          ────────────────────────────────────────► │            │
│                                                     │            │
│  ┌──────────────────────────────────────┐          │            │
│  │     MoveIt Module (서비스 서버)        │          │            │
│  │  UR3 + Robotiq 2F-85, MTC 기반        │          │            │
│  └──────────────────────────────────────┘          │            │
└─────────────────────────────────────────────────────────────────┘
```

### 모듈별 책임 경계

| 모듈 | 책임 | 비책임 |
|------|------|--------|
| **LLM Agent** | 명령 파싱, Phase 관리, LLM 호출, 명령 패키지 조립, 의사결정 | 모션 실행, 영상 처리, 1차 좌표 변환 |
| **YOLO Node** | 카메라 영상 분석, camera→base TF, `/vision/detection_results` 발행 | 의사결정 |
| **MoveIt Module** | 경로 계획, 모션 실행, gripper 제어, 서비스 서버 구현 | 오브젝트 인식, 의사결정 |

---

## 4. ROS2 인터페이스 정의

### 4.1 LLM Agent가 Subscribe하는 토픽

| Topic | 타입 | 발행자 | 사용 Phase |
|-------|------|--------|-----------|
| `/vision/detection_results` | `std_msgs/String` (JSON) | YOLO Node | P1, P2, P3 |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | ROS2 TF tree | P2 (`base_link`→`world` 정적 변환만) |

### 4.2 LLM Agent가 Publish하는 토픽

| Topic | 타입 | 수신자 | 사용 Phase |
|-------|------|--------|-----------|
| `/llm_agent/phase` | `std_msgs/String` | 모니터링용 | 전 Phase |
| `/llm_agent/log` | `std_msgs/String` | 모니터링용 | 전 Phase |

### 4.3 LLM Agent가 호출하는 서비스 (Client)

| Service | 타입 | 서버 | 사용 Phase |
|---------|------|------|-----------|
| `/moveit/execute` | `llm_agent_msgs/MoveItExecute` | MoveIt Module | P1, P2, P3, P4 |

> 에이전트가 서비스 **계약(.srv)을 정의**하고 client로 호출한다. MoveIt 모듈이 이 계약에 맞춰 서버를 구현한다(현 `ur3_pick_place.py`를 서버로 개조).

### 4.4 YOLO JSON 메시지 형식 (`/vision/detection_results`)

> 실제 ROBOT_VISION 발행 포맷. 좌표 키는 **대문자 X/Y/Z**, 프레임당 다중 객체 배열.

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

> - **타겟 좌표는 `position_3d_base_frame` 사용** (YOLO가 `camera_link`→`base_link` TF 변환 완료한 값).
> - `class_name`은 YOLOv8 모델의 `model.names`(COCO 기본셋) 형식 — 공백 포함 가능(예: `"stop sign"`). 소문자_언더스코어 아님.
> - 깊이(거리) 추정 불가 프레임은 distance_m ≤ 0 → 좌표 무효.

### 4.5 MoveIt 서비스 계약 (`llm_agent_msgs/MoveItExecute`)

명령별 구조 차이를 흡수하기 위해 **단일 제네릭 서비스 + JSON 페이로드** 방식을 채택한다.

```
# srv/MoveItExecute.srv
string cmd            # "scan" | "pick" | "lift" | "place" | "release" | "home"
string params_json    # 명령별 파라미터 (JSON 직렬화)
---
bool   success
int32  error_code     # MoveIt MoveItErrorCodes (0 = SUCCESS)
string error_message
```

#### params_json 명령별 스키마

```jsonc
// SCAN — P1 스캔 (world frame 기준 [x,y,z,r,p,y] 목록)
{"waypoints": [[0.3,0.0,0.5,0.0,0.785,0.0], [0.3,0.25,0.4,0.0,0.785,0.3]]}

// PICK — P2 (object.shape/dimensions는 config에서, pose_world는 YOLO base_frame 기반)
{
  "arm_group_name": "ur_manipulator",
  "eef_name": "robotiq_2f_85",
  "hand_group_name": "gripper",
  "hand_frame": "robotiq_2f_85_tcp",
  "hand_open_pose": "open",
  "hand_close_pose": "close",
  "object": {
    "name": "target_object",
    "shape": "cylinder",
    "dimensions": [0.12, 0.025],
    "pose_world": [0.40, 0.10, 0.06, 0.0, 0.0, 0.0]
  },
  "grasp_frame_transform": [0.0, 0.0, 0.13, 3.1416, 0.0, 0.0],
  "approach_object_min_dist": 0.08,
  "approach_object_max_dist": 0.15,
  "lift_object_min_dist": 0.05,
  "lift_object_max_dist": 0.15,
  "max_solutions": 10
}

// LIFT — P3 수직 상승
{"direction": "z+", "distance_m": 0.1, "frame": "world"}

// PLACE — P4 배치
{"object_name": "target_object", "target_pose_world": [0.4,-0.2,0.06,0.0,0.0,0.0], "place_surface_offset": 0.001}

// RELEASE — P4
{"hand_open_pose": "open"}

// HOME — 복귀
{"arm_home_pose": "home"}
```

> - `pose_world`: `[x, y, z, roll, pitch, yaw]` — world frame 기준
> - `grasp_frame_transform`: hand_frame 기준 TCP 오프셋 (tool0→TCP z +0.13m)
> - `object.pose_world.z`: 원통 중심 = 바닥 z + height/2

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

**입력**: CLI 자연어 명령에서 LLM이 추출한 `target_class_name` (YOLO `model.names` 형식)

**처리 흐름**:

```
1. scan waypoints 로드 (config/scan_waypoints.yaml)
2. /moveit/execute(cmd="scan", waypoints) 호출 → 팔 움직임 시작
3. /vision/detection_results 구독 시작
4. 루프 (최대 30초):
   a. YOLO 수신 프레임의 objects[] 순회:
      - class_name == target_class_name AND confidence >= CONFIDENCE_THRESHOLD AND distance_m > 0?
        → 탐지 성공, 해당 객체의 position_3d_base_frame 저장
        → P2 전환
   b. 30초 경과 → 타임아웃 처리
5. 타임아웃 시:
   - session_retry_count += 1
   - <= 3 → HOME 후 P1 재시작
   - >  3 → 최종 실패 보고, 세션 종료
```

**LLM 역할**: 탐지 루프는 결정론적. LLM은 P1 진입 시 CLI 명령에서 `target_class_name` 추출에만 사용.

**파라미터**: `CONFIDENCE_THRESHOLD = 0.7`, `SCAN_TIMEOUT_SEC = 30`, `MAX_RETRY = 3`

**출력**: 탐지된 오브젝트의 `position_3d_base_frame` (P2로 전달)

---

### P2: 파라미터 조립 및 Pick 실행

**목적**: YOLO에서 받은 `base_link` 좌표를 `world` 프레임으로 정적 변환하고, **config의 Object Spec**과 결합해 PICK 명령을 조립·실행한다.

**처리 흐름**:

```
1. 좌표 변환: position_3d_base_frame → pose_world
   - tf2.lookup_transform("world", "base_link", Time())  ← 정적 변환(보통 identity)
   - do_transform_point() 적용 → world frame 좌표
   - z 보정: pose_world.z = 바닥 z + object_height / 2

2. /vision/detection_results 재구독 (최신 프레임으로 좌표 갱신, 최대 1초)

3. Object Spec 조회: config.objects[target_class_name] → {shape, dimensions}
   - 미등록 class → 실패 보고, 세션 종료 (LLM 추정 안 함)

4. PICK params_json 조립 (pose_world + config Object Spec + config grasp 파라미터)

5. /moveit/execute(cmd="pick", params_json) 호출 → 응답 대기

6. Grip 검증 (YOLO):
   - /vision/detection_results 구독, GRIP_VERIFY_WINDOW_SEC 내 프레임 확인
   - class_name 매칭 AND position_3d_base_frame 기준 gripper 근접(z < GRIP_DISTANCE_THRESHOLD)?
     → grip 성공 → P3 전환
   - 불만족 → grip 실패

7. grip 실패 시:
   - session_retry_count += 1
   - <= 3 → 2번부터 재시도 / > 3 → 실패 보고, 세션 종료
```

**LLM 역할**: **없음(결정론적).** shape/dimensions는 config, pose는 YOLO+TF, grasp 파라미터는 config에서 가져와 조립만 한다. (기존 설계의 LLM PICK 파라미터 생성은 config-only 정책에 따라 제거.)

**파라미터**: `GRIP_DISTANCE_THRESHOLD = 0.15` (m), `MAX_RETRY = 3`, `GRIP_VERIFY_WINDOW_SEC = 2.0`

**좌표 변환 상세**:
```
position_3d_base_frame (X, Y, Z)   ← YOLO가 camera→base 변환 완료
    │
    ▼  tf2.do_transform_point()  transform: base_link → world (정적)
    ▼
pose_world (x, y, z)
    │
    ▼  z 보정: += object_height / 2  (바닥 → 원통 중심)
    ▼
PICK params_json 의 object.pose_world
```

---

### P3: Pickup 검증

**목적**: pick 직후 팔을 10cm 수직 상승시키고, YOLO가 gripper 근처에서 오브젝트를 여전히 감지하는지 LLM이 판단한다.

**처리 흐름**:

```
1. /moveit/execute(cmd="lift", {distance_m:0.1, direction:"z+"}) 호출 → 응답 대기
2. /vision/detection_results 구독, GRIP_VERIFY_WINDOW_SEC 간 프레임 수집
3. LLM 호출:
   Input:  최근 N개 YOLO 프레임(objects[] 포함), target_class_name
   Output: {"pickup_success": bool, "reason": str}
4. pickup_success == true  → P4 전환
5. pickup_success == false:
   - /moveit/execute(cmd="release") → /moveit/execute(cmd="home")
   - session_retry_count += 1  (P1과 공유)
   - <= 3 → P1 재시작 / > 3 → 최종 실패 보고
```

**LLM 판단 기준**: target_class_name이 윈도우 내 감지됐는가 / position_3d_base_frame 기준 gripper 근접 / confidence >= 임계값.

**LLM 역할**: YOLO 프레임 목록을 분석해 pickup 성공 여부를 자연어 근거와 함께 판단.

---

### P4: 배치 및 세션 종료

**목적**: 사전 정의된 target 절대좌표로 이동해 오브젝트를 내려놓고, 홈 복귀 후 세션을 종료한다.

**처리 흐름**:

```
1. target_pose 로드 (config/targets.yaml)
2. /moveit/execute(cmd="place", {target_pose_world, ...}) → 응답 대기
3. /moveit/execute(cmd="release") → 응답 대기
4. /moveit/execute(cmd="home", {arm_home_pose:"home"}) → 응답 대기
5. CLI에 성공 보고 출력
6. 세션 종료
```

**LLM 역할**: 없음(결정론적).

---

## 6. LLM 호출 지점 (Agentic Loop)

LLM 호출이 필요한 의사결정 지점은 **2곳**이며, 각각 단일 호출(single-shot)로 처리한다. multi-turn tool use 루프는 사용하지 않는다.

| 호출 지점 | Phase | Input | Output |
|----------|-------|-------|--------|
| **명령 파싱** | P1 진입 시 | 자연어 명령 | `{"target_class_name": "cup"}` |
| **Pickup 검증** | P3 | YOLO 프레임 목록 + target_class_name | `{"pickup_success": bool, "reason": str}` |

> P2의 PICK 파라미터 생성은 **config-only 정책**으로 결정론적 조립으로 대체되어 LLM 호출에서 제외됨.

### Phase별 시스템 프롬프트

**명령 파싱 (P1 진입)**:
```
당신은 로봇 조작 시스템의 명령 파싱 에이전트입니다.
사용자의 자연어 명령에서 대상 오브젝트의 class_name을 추출하세요.
YOLO(YOLOv8/COCO) 모델의 라벨 형식 그대로 반환하세요(예: "cup", "bottle", "stop sign").
응답 형식: {"target_class_name": "cup"}
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
| P1 탐지 타임아웃 | session_retry_count++ → 3회 초과 시 실패 보고 |
| P2 미등록 class (Object Spec 없음) | 즉시 실패 보고, 세션 종료 |
| P2 grip 실패 | session_retry_count++ → 3회 초과 시 실패 보고 |
| P3 pickup 실패 | RELEASE + HOME → P1 재시작 (retry_count 공유) |
| MoveIt 서비스 timeout | `service_timeout_sec`(10s) 내 응답 없음 → 실패 처리, 해당 Phase retry |
| 서비스 미가용 (서버 미연결) | 대기/재시도 후 실패 보고 |
| base→world TF 변환 실패 | 최대 3초 retry → 실패 시 P2 진입 전 중단 |
| LLM API 오류 | 3회 retry (exponential backoff) → 실패 시 세션 종료 |

**공유 retry_count 정책**: P1 타임아웃, P2 grip 실패, P3 pickup 실패 모두 동일한 `session_retry_count`를 증가시킨다. 최대 3이며 초과 시 세션 전체 종료.

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
│   │   │   ├── cli_reader.py          # stdin 명령 수신
│   │   │   ├── tf_transformer.py      # base_link → world 정적 변환 (축소)
│   │   │   ├── yolo_subscriber.py     # /vision/detection_results 구독 + 프레임 버퍼
│   │   │   └── moveit_client.py       # /moveit/execute 서비스 client (call_and_wait)
│   │   ├── config/
│   │   │   ├── agent.yaml             # 임계값, retry, UR3 group/frame, 서비스명
│   │   │   ├── scan_waypoints.yaml    # P1 스캔 waypoint 목록
│   │   │   ├── targets.yaml           # P4 target 절대좌표
│   │   │   └── objects.yaml           # ★ class_name → shape/dimensions (Object Spec)
│   │   ├── launch/agent.launch.py
│   │   ├── test/
│   │   │   ├── test_phase_manager.py
│   │   │   └── test_tf_transformer.py
│   │   ├── package.xml / setup.py / setup.cfg
│   │
│   └── llm_agent_msgs/
│       ├── msg/TaskStatus.msg
│       ├── srv/
│       │   ├── SetPhase.srv
│       │   ├── AgentQuery.srv
│       │   └── MoveItExecute.srv     # ★ 신규: MoveIt 명령 서비스 계약
│       ├── package.xml / CMakeLists.txt
│
└── DESIGN.md
```

> `moveit_client.py`는 토픽 publisher/subscriber 대신 **서비스 client**(`/moveit/execute`)로 변경. `call_and_wait(cmd, params)` → `{success, error_code, error_message}` 반환.

---

## 9. 내부 컴포넌트 설계

### AgentNode (agent_node.py)
- `rclpy.Node` 상속, `MultiThreadedExecutor` + `ReentrantCallbackGroup`
- stdin 읽기는 별도 스레드, ROS2 callback과 분리
- Phase 루프는 명령 수신 시 워커에서 실행 (spin loop 비차단)

### PhaseManager (phase_manager.py)
- Phase 전환 상태머신, `session_retry_count` 관리, Phase별 프롬프트 반환

### YoloSubscriber (yolo_subscriber.py)
- `/vision/detection_results` 상시 구독, 최근 N개 프레임 deque 버퍼
- JSON 래퍼(`objects[]`) 파싱, `position_3d_base_frame` 추출
- `wait_for_detection(class_name, timeout)` blocking API 제공

### TFTransformer (tf_transformer.py)
- `tf2_ros.Buffer` + `TransformListener`
- `base_to_world(point_base)` → world `Point` 반환 (정적 변환, 대개 identity)

### MoveItClient (moveit_client.py)
- `/moveit/execute` **서비스 client**
- `call_and_wait(cmd, params_dict, timeout)` → `{success, error_code, error_message}`

### LLMClient (llm_client.py)
- Anthropic SDK 동기 호출, tool use 없이 JSON 응답 파싱, 3회 retry(backoff)

### CLIReader (cli_reader.py)
- daemon 스레드에서 `sys.stdin.readline()` 대기 → queue로 AgentNode 전달

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

yolo:
  detection_topic: "/vision/detection_results"

moveit:
  service_name: "/moveit/execute"          # ROS2 서비스 (agent=client)
  service_timeout_sec: 10.0
  arm_group_name: "ur_manipulator"
  eef_name: "robotiq_2f_85"
  hand_group_name: "gripper"
  hand_frame: "robotiq_2f_85_tcp"
  hand_open_pose: "open"
  hand_close_pose: "close"
  arm_home_pose: "home"
  grasp_frame_transform: [0.0, 0.0, 0.13, 3.1416, 0.0, 0.0]
  approach_object_min_dist: 0.08
  approach_object_max_dist: 0.15
  lift_object_min_dist: 0.05
  lift_object_max_dist: 0.15
  max_solutions: 10

tf:
  base_frame: "base_link"     # YOLO position_3d_base_frame 기준
  world_frame: "world"        # MoveIt 계획 기준
```

### config/objects.yaml (★ Object Spec — config 사전 정의 전용)
```yaml
# YOLO class_name → MTC 충돌물체 shape/dimensions
# dimensions: cylinder=[height_m, radius_m], box=[x,y,z]
objects:
  cup:
    shape: cylinder
    dimensions: [0.12, 0.025]
  bottle:
    shape: cylinder
    dimensions: [0.20, 0.033]
# 미등록 class는 P2에서 실패 처리 (LLM 추정 안 함)
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
  x: 0.4
  y: -0.2
  z: 0.06
  roll: 0.0
  pitch: 0.0
  yaw: 0.0
```

---

## 11. 미결 사항 / 확인 완료 현황

| # | 항목 | 결정/확인 결과 | 출처 |
|---|------|---------------|------|
| 1 | 로봇 플랫폼 | **UR3 + Robotiq 2F-85** 확정 | 사용자 확정 / PANDA_ENV |
| 2 | arm/hand group, frame, home | `ur_manipulator` / `gripper` / `robotiq_2f_85_tcp` / `home` | PANDA_ENV config |
| 3 | YOLO 토픽·포맷 | `/vision/detection_results`, `{timestamp_ns,num_detections,objects[]}` | ROBOT_VISION |
| 4 | camera_frame / TF | YOLO가 `camera_link→base_link` 변환 완료 → 에이전트는 `base_link→world`(정적)만 | ROBOT_VISION |
| 5 | 오브젝트 형상·크기 | **config 사전 정의만** (`objects.yaml`), LLM 추정 제거 | 사용자 확정 |
| 6 | MoveIt 연동 방식 | **ROS2 서비스** `/moveit/execute` (agent가 계약 정의, client) | 사용자 확정 |
| 7 | class_name 형식 | YOLOv8 `model.names`(COCO, 공백 가능) | ROBOT_VISION |
| △ | MoveIt 서비스 서버 구현 | **미완** — MoveIt팀이 `ur3_pick_place.py`를 본 계약 기반 서버로 개조 필요 | — |
| △ | `base_link`↔`world` 정적 변환 실측값 | identity 가정 — UR3 URDF/TF로 확인 필요 | — |

---

## 12. 세부 작업 단위 (Work Breakdown)

### Sprint 1 — 기반 구조 ✅
- llm_agent_msgs (msg/srv), llm_agent 스캐폴딩, PhaseManager + 단위 테스트

### Sprint 2 — 핵심 컴포넌트 (인터페이스 갱신 반영 필요)
| ID | 작업 | 상태 |
|----|------|------|
| S2-1 | `CLIReader`: stdin → queue 스레드 | 구현됨 |
| S2-2 | `YoloSubscriber`: **`/vision/detection_results` + objects[] 파싱** 으로 갱신 | 수정 필요 |
| S2-3 | `TFTransformer`: **`base_link`→`world` 정적 변환** 으로 축소 | 수정 필요 |
| S2-4 | `MoveItClient`: **토픽 → `/moveit/execute` 서비스 client** 로 변경 | 수정 필요 |
| S2-5 | `LLMClient`: JSON 응답 파싱 + retry | 구현됨 |
| S2-6 | `MoveItExecute.srv` 정의 + `objects.yaml` 추가 | 신규 |

### Sprint 3 — Phase 루프 구현
| ID | 작업 |
|----|------|
| S3-1 | P1 루프: scan 서비스 호출 + YOLO 탐지 + 타임아웃/retry |
| S3-2 | P2 루프: base→world 변환 + Object Spec 조회 + PICK 서비스 + grip verify |
| S3-3 | P3 루프: LIFT 서비스 + YOLO 수집 + LLM pickup 검증 |
| S3-4 | P4 루프: PLACE + RELEASE + HOME 서비스 + 세션 종료 |

### Sprint 4 — 통합 및 테스트
| ID | 작업 |
|----|------|
| S4-1 | AgentNode 통합 (CLIReader → PhaseManager → Phase 루프) |
| S4-2 | Mock YOLO / Mock MoveIt 서비스 서버로 E2E 시나리오 테스트 |
| S4-3 | 실패 케이스 테스트 (타임아웃, 미등록 class, grip 실패, P3 실패) |
| S4-4 | Gazebo(UR3) 시뮬레이션 연동 테스트 |
