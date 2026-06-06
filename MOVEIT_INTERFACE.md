# MoveIt 측 인터페이스 명세 / 확인 사항

> 목적: 미결 사항 문답으로 확정된 결정을 기반으로, **MoveIt 모듈(서비스 서버) 코드 생성 시 참고**할 명세서.
> 대상 독자: MoveIt/MTC 담당(PANDA_ENV `ur3_pick_place.py` 개조 담당).
> 기준 문서: `DESIGN.md`, `CONTEXT.md` / 작성일: 2026-06-06
>
> **핵심 전제**: LLM Agent가 서비스 **계약(.srv)을 정의**하고 **client**로 호출한다.
> MoveIt 모듈은 이 계약에 맞춰 **서버를 구현**한다. 현재 `ur3_pick_place.py`는
> config 파라미터를 읽어 pick+place 전체를 1회 실행하는 **독립 스크립트**이므로,
> 아래 명세에 맞춰 **단계별 명령을 받는 서비스 서버로 개조**가 필요하다.

---

## 0. 확정된 환경 (변경 금지 기준값)

| 항목 | 값 | 출처 |
|------|-----|------|
| 로봇 | UR3 + Robotiq 2F-85 | 확정 |
| arm_group_name | `ur_manipulator` | PANDA_ENV ur3_mtc_config.yaml |
| eef_name | `robotiq_2f_85` | 〃 |
| hand_group_name | `gripper` | 〃 |
| hand_frame | `robotiq_2f_85_tcp` | 〃 |
| hand_open_pose / close_pose | `open` / `close` | SRDF group_state |
| arm_home_pose | `home` | UR3 SRDF |
| world_frame | `world` | 〃 |
| grasp_frame_transform | `[0.0, 0.0, 0.13, 3.1416, 0.0, 0.0]` | 〃 (tool0→TCP z+0.13m) |

> 위 값은 에이전트가 PICK 요청에 함께 실어 보내지만, **서버 측 SRDF/URDF 실제 정의와 일치해야** 한다.
> 불일치 시 [§5 확인 사항](#5-moveit-팀-확인필요-항목) 참고.

---

## 1. 제공해야 할 서비스

| 항목 | 값 |
|------|-----|
| 서비스 이름 | `/moveit/execute` |
| 서비스 타입 | `llm_agent_msgs/srv/MoveItExecute` |
| 역할 | **서버** (에이전트가 client) |
| 동시성 | 단일 세션 직렬 호출. 한 번에 하나의 명령만 처리(blocking) |

### 1.1 서비스 계약 (`llm_agent_msgs/srv/MoveItExecute.srv`)

```
# Request
string cmd            # "scan" | "pick" | "lift" | "place" | "release" | "home"
string params_json    # 명령별 파라미터 (JSON 직렬화 문자열)
---
# Response
bool   success        # 모션 계획+실행 최종 성공 여부
int32  error_code     # moveit_msgs/MoveItErrorCodes 값 (0 = SUCCESS)
string error_message  # 실패 사유(사람이 읽는 문자열). 성공 시 ""
```

> - `params_json`은 `json.loads()`로 파싱. 명령별 스키마는 [§2](#2-명령별-파라미터-스키마-params_json) 참조.
> - 서버는 **계획→실행 완료 후** 응답을 반환한다(동기). 실행까지 끝나야 `success=true`.
> - `error_code`는 가능한 한 `MoveItErrorCodes`(PLANNING_FAILED=-1, ... SUCCESS=1 등 MoveIt 정의)에 맞춘다.
>   단, 본 계약에서는 **성공=0 규칙**으로 통일하므로, MoveIt 원시 코드와 매핑 표를 서버가 관리하거나
>   `0=SUCCESS, 음수=실패`로 정규화하여 반환한다. (아래 §4 참고)

---

## 2. 명령별 파라미터 스키마 (`params_json`)

에이전트는 Phase 진행에 따라 아래 6개 명령을 **개별 호출**한다.
현재 `ur3_pick_place.py`는 이 모든 단계를 한 번에 수행하므로, **단계 분리**가 핵심 개조 포인트다.

### 2.1 `scan` — P1 스캔 (팔을 waypoint 순서로 이동)
```json
{ "waypoints": [
    [0.3, 0.0, 0.5, 0.0, 0.785, 0.0],
    [0.3, 0.25, 0.4, 0.0, 0.785, 0.3]
] }
```
- `waypoints`: `[[x,y,z,roll,pitch,yaw], ...]` — **world frame** 기준 EEF 목표 자세 목록.
- 서버는 각 waypoint로 순차 이동. (인식은 에이전트/YOLO가 담당 → 서버는 모션만)
- **확인 필요**: 스캔 중 비동기 중단(에이전트가 탐지 성공 시 조기 종료) 지원 여부 → [§5-4](#5-moveit-팀-확인필요-항목)

### 2.2 `pick` — P2 Pick 실행
```json
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
```
- `object.dimensions`: **cylinder=[height_m, radius_m]**, box=[x,y,z]. (에이전트 config에서 옴)
- `object.pose_world`: 원통 **중심** 좌표 = 바닥 z + height/2 (에이전트가 보정해서 전달).
- 나머지 파라미터는 PANDA_ENV `ur3_mtc_config.yaml`과 1:1 대응 → [§3 매핑표](#3-기존-ur3_mtc_configyaml-↔-서비스-파라미터-매핑).
- 서버: 충돌물체 spawn → grasp 자세 생성 → approach → close → (lift는 별도 명령) 까지.
  - **확인 필요**: pick의 lift를 여기서 할지, P3 `lift`로 분리할지 → [§5-2](#5-moveit-팀-확인필요-항목)

### 2.3 `lift` — P3 수직 상승 (pickup 검증용)
```json
{ "direction": "z+", "distance_m": 0.1, "frame": "world" }
```
- 현재 잡은 상태에서 EEF를 `frame` 기준 `direction`으로 `distance_m` 만큼 직선 이동.

### 2.4 `place` — P4 배치
```json
{ "object_name": "target_object",
  "target_pose_world": [0.4, -0.2, 0.06, 0.0, 0.0, 0.0],
  "place_surface_offset": 0.001 }
```
- `target_pose_world`: world frame 절대 배치 좌표(에이전트 config `targets.yaml`).

### 2.5 `release` — gripper 해제
```json
{ "hand_open_pose": "open" }
```

### 2.6 `home` — 홈 자세 복귀
```json
{ "arm_home_pose": "home" }
```

---

## 3. 기존 `ur3_mtc_config.yaml` ↔ 서비스 파라미터 매핑

> 현 스크립트는 모든 값을 config에서 읽지만, 서비스화 후에는 **요청마다 에이전트가 동적으로 전달**한다.
> 단, group/frame/named-pose 등 **로봇 고정값은 서버 기본값으로 두고 요청값으로 override** 권장.

| ur3_mtc_config.yaml 키 | 서비스 전달 위치 | 비고 |
|---|---|---|
| `arm_group_name` | pick.arm_group_name | 고정값(서버 기본값 권장) |
| `eef_name` | pick.eef_name | 고정값 |
| `hand_group_name` | pick.hand_group_name | 고정값 |
| `hand_frame` | pick.hand_frame | 고정값 |
| `hand_open_pose`/`hand_close_pose` | pick/release | 고정값 |
| `arm_home_pose` | home.arm_home_pose | 고정값(`home`) |
| `world_frame` | (암묵적 world) | 고정값 |
| `object_name` | pick.object.name / place.object_name | 동적 |
| `object_dimensions` | pick.object.dimensions | **동적**(에이전트 config/objects.yaml) |
| `object_pose` | pick.object.pose_world | **동적**(YOLO+TF 기반) |
| `grasp_frame_transform` | pick.grasp_frame_transform | 고정값 |
| `place_pose` | place.target_pose_world | **동적**(targets.yaml) |
| `place_surface_offset` | place.place_surface_offset | 동적 |
| `approach_object_min/max_dist` | pick.approach_object_* | 동적 |
| `lift_object_min/max_dist` | pick.lift_object_* | 동적 |
| `max_solutions` | pick.max_solutions | 동적 |
| `spawn_table`, `table_*`, `surface_link` | (서버 내부 유지) | 씬 구성. 에이전트는 관여 안 함 |

---

## 4. 응답(Response) 규약

- `success`: **계획+실행 완료** 기준. 계획만 성공/실행 실패면 `false`.
- `error_code`: MoveIt 원시 코드를 그대로 줄 경우 에이전트는 `0`을 성공으로 간주하므로,
  **`0 = SUCCESS`로 정규화**해서 반환할 것(또는 서버가 매핑표 문서화).
- `error_message`: 실패 시 원인(예: `"IK solution not found for grasp pose"`, `"collision in approach"`).
- 타임아웃: 에이전트는 기본 `10초`(`agent.yaml: moveit.service_timeout_sec`) 대기.
  **장시간 모션은 서버가 이 시간 내 응답하거나, 에이전트 측 타임아웃 값 조정을 협의**할 것.

---

## 5. MoveIt 팀 확인/결정 필요 항목

| # | 항목 | 배경 | 필요한 결정 |
|---|------|------|------------|
| 1 | **base_link ↔ world 정적 변환** | YOLO는 `base_link` 좌표를 주고, 에이전트가 `world`로 변환해 `pose_world`를 보냄. | UR3 URDF/TF 상 `world→base_link`가 identity인지, 오프셋이 있는지 실측 확인. 오프셋 있으면 값 공유. |
| 2 | **pick의 lift 분리 여부** | 에이전트는 `pick`(파지) 후 별도 `lift`(10cm 상승, 검증용)를 호출한다. | `pick`에서 lift를 하지 않고 파지 완료까지만 수행하도록 단계 분리 가능한지. (MTC stage 재구성 필요) |
| 3 | **단계별 상태 유지** | 서비스가 stateless면 `lift`/`place`가 직전 `pick`의 잡은 오브젝트/플래닝 씬을 알아야 함. | 서버가 세션 동안 PlanningScene/attached object 상태를 유지하는지, 아니면 매 요청에 컨텍스트를 실어야 하는지. |
| 4 | **scan 비동기 중단** | 에이전트가 탐지 성공 시 스캔을 중간에 멈추고 P2로 넘어감. | `scan`을 blocking으로 끝까지 돌릴지, 중단(preempt) 가능하게 할지. (필요 시 액션 전환 검토) |
| 5 | **object shape 지원 범위** | 에이전트 config는 우선 `cylinder`(`[h, r]`)만 정의. `box` 가능성 있음. | 서버가 cylinder/box 외 형상을 받을 가능성과 처리 방식. |
| 6 | **error_code 규약** | §4 참고. | MoveItErrorCodes 원시값 전달 vs `0=SUCCESS` 정규화 중 택일 + 문서화. |
| 7 | **gripper 제어 경로** | Robotiq 2F-85 open/close가 named-pose(`open`/`close`)로 충분한지. | 폭(width) 기반 제어 필요 시 파라미터 추가 협의. |
| 8 | **좌표/단위/회전 표기** | `pose_world = [x,y,z,roll,pitch,yaw]`, 단위 m / rad, RPY 순서. | 서버 내부 표기와 일치 확인(특히 RPY vs quaternion, intrinsic/extrinsic). |

---

## 6. 개조 가이드 요약 (ur3_pick_place.py → 서비스 서버)

1. `main()`의 one-shot 실행 구조를, `/moveit/execute` **서비스 콜백 기반**으로 전환.
2. 기존 `build_pick_place_task()`의 단일 task를 **명령별 sub-task**로 분해:
   - `scan` → 다중 waypoint MoveTo
   - `pick` → spawn object + GenerateGraspPose + approach + close (lift 제외, [§5-2] 결정에 따름)
   - `lift` → MoveRelative(z+, distance)
   - `place` → GeneratePlacePose + place + (detach)
   - `release` → gripper open
   - `home` → MoveTo named state `home`
3. 로봇 고정값(group/frame/named-pose)은 **서버 파라미터 기본값**으로 보유, 요청값으로 override.
4. PlanningScene/attached object **세션 상태 유지** ([§5-3]).
5. 응답을 `MoveItExecute.Response`로 매핑(§4 규약 준수).

---

## 7. 참고

- 에이전트 측 계약/설정: `src/llm_agent_msgs/srv/MoveItExecute.srv`(신규 정의 예정), `src/llm_agent/config/agent.yaml`(`moveit:` 블록), `config/objects.yaml`, `config/targets.yaml`
- 현 MoveIt 구현: PANDA_ENV `src/ur3_mtc_pick_place/scripts/ur3_pick_place.py`, `config/ur3_mtc_config.yaml`
- 전체 설계: `DESIGN.md` §4(인터페이스), §5(Phase)
