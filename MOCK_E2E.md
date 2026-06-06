# Mock 검증 가이드

에이전트 전체 Pick & Place 흐름(P1→P2→P3→P4)을 실제 로봇/YOLO/MoveIt 없이 검증하는 두 가지 방법.

---

## 1. 오케스트레이션 로직 검증 (ROS 불필요, 어디서나)

ROS 모듈을 stub 으로 주입하고 협력자(YOLO/MoveIt/LLM/TF)를 mock 으로 교체해
`AgentNode` 의 Phase 루프·retry·폴백·에러 처리 로직만 빠르게 검증한다.

```bash
python src/llm_agent/test/test_flow_mock.py      # pytest 불필요, standalone
# 또는 ROS 환경에서:  pytest src/llm_agent/test/test_flow_mock.py -v
```

검증 시나리오 (10):
- happy path (P1→P2→P3→P4, 서비스 호출 순서 scan→pick→lift→place→release→home)
- P1 탐지 타임아웃 소진 / P2 grip 반복 실패 / P3 1회 실패 후 성공 / P3 소진
- **P4 place 실패가 성공으로 보고되지 않음(G1 회귀)**
- 미등록 class → default_spec 폴백 / 폴백 비활성 시 실패 / 명령 파싱 실패
- cylinder·box z 보정(바닥→중심)

---

## 2. 실제 ROS2 E2E (Linux 워크스페이스)

mock YOLO 발행기 + mock MoveIt 서버를 띄우고 **실제 에이전트 노드**를 구동한다.
(P3 의 LLM pickup 검증은 실제 Gemini 호출 → `.env` 에 `GEMINI_API_KEY` 필요)

### 빌드
```bash
cd ~/WorkspaceLLMagent
source /opt/ros/humble/setup.bash
colcon build --packages-select llm_agent_msgs && source install/setup.bash
colcon build --packages-select llm_agent     && source install/setup.bash
```

### 실행 (터미널 3개, 각각 source 후)
```bash
# T1 — mock MoveIt 서버
python3 src/llm_agent/test/mock_moveit_server.py
#   특정 명령 실패 주입 예:  ... mock_moveit_server.py --ros-args -p fail_cmds:="['place']"

# T2 — mock YOLO 발행기 (distance_m 기본 0.1 → P1 탐지·P2 grip 검증 통과)
python3 src/llm_agent/test/mock_yolo_publisher.py
#   다른 대상:  ... mock_yolo_publisher.py --ros-args -p class_name:=bottle

# T3 — 실제 에이전트
ros2 launch llm_agent agent.launch.py
#   프롬프트에 명령 입력:  컵 집어
```

### 기대 결과
- T3 로그: `P1: detected cup` → `P2: grip confirmed` → `P3: pickup_success=True` → `세션 완료: Pick & Place 성공.`
- T1 로그: `scan → pick → lift → place → release → home` 순으로 cmd 수신
- `fail_cmds:="['place']"` 주입 시: `P4 실패: place에 실패해 ...` 보고, release 미호출, home 으로 안전 복귀

### 모니터링(선택)
```bash
ros2 topic echo /llm_agent/phase    # 현재 Phase
ros2 topic echo /llm_agent/log      # 진행 로그
```
