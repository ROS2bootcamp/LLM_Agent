# LLM Agent 아키텍처 그래프

> 갱신된 설계(UR3 + Gemini + 서비스 기반 MoveIt) 기준.
> 관련 문서: `DESIGN.md`(상세), `MOVEIT_INTERFACE.md`(MoveIt 계약), `CONTEXT.md`(현황)

---

## 1. 주변 노드와의 인터페이스 구조

```mermaid
flowchart LR
    CLI["CLI<br/>(stdin)"]
    GEMINI["Gemini API<br/>google-genai · .env<br/>GEMINI_API_KEY"]
    YOLO["YOLO Node<br/>(ROBOT_VISION)"]
    TF["TF tree<br/>/tf · /tf_static"]
    MOVEIT["MoveIt Module<br/>(service server)<br/>UR3 + Robotiq 2F-85"]
    MON["모니터링<br/>(RViz/rqt 등)"]

    subgraph AGENT["LLM Agent Node (이 레포)"]
        direction TB
        CR["CLIReader"]
        ORCH["AgentNode<br/>(Phase 오케스트레이터)"]
        PM["PhaseManager<br/>(상태머신·retry)"]
        LLM["LLMClient"]
        YS["YoloSubscriber<br/>(deque 버퍼)"]
        TFX["TFTransformer<br/>(base→world)"]
        MC["MoveItClient<br/>(service client)"]
        CR --> ORCH
        ORCH --- PM
        ORCH --- LLM
        ORCH --- YS
        ORCH --- TFX
        ORCH --- MC
    end

    CLI -->|"자연어 명령 (stdin readline)"| CR
    LLM -->|"parse_command / verify_pickup<br/>system+user prompt"| GEMINI
    GEMINI -->|"JSON 응답<br/>(application/json)"| LLM

    YOLO -->|"/vision/detection_results<br/>std_msgs/String (JSON, objects[])"| YS
    TF -->|"base_link → world (정적)"| TFX

    MC -->|"/moveit/execute (service)<br/>MoveItExecute.Request<br/>cmd + params_json"| MOVEIT
    MOVEIT -->|"Response<br/>success · error_code · error_message"| MC

    ORCH -->|"/llm_agent/phase<br/>/llm_agent/log"| MON

    classDef ext fill:#eef,stroke:#557,color:#000;
    classDef cloud fill:#fef0e6,stroke:#c80,color:#000;
    class CLI,YOLO,TF,MOVEIT,MON ext;
    class GEMINI cloud;
```

| 채널 | 방향 | 타입 | 비고 |
|------|------|------|------|
| stdin | CLI → Agent | text | 자연어 명령 |
| `/vision/detection_results` | YOLO → Agent | `std_msgs/String`(JSON) | `objects[]`, `position_3d_base_frame` |
| `/tf`,`/tf_static` | TF → Agent | `tf2_msgs/TFMessage` | `base_link`→`world` 정적변환 |
| `/moveit/execute` | Agent ↔ MoveIt | `llm_agent_msgs/MoveItExecute` (**service**) | agent=client |
| Gemini API | Agent ↔ Google | HTTPS(google-genai) | `.env` GEMINI_API_KEY |
| `/llm_agent/phase`,`/llm_agent/log` | Agent → 모니터 | `std_msgs/String` | 상태 발행 |

---

## 2. Phase별 내부 데이터 플로우

```mermaid
flowchart TD
    START(["CLI 명령 수신"]) --> PARSE

    PARSE["parse_command (Gemini)"]
    PARSE -->|"target_class_name"| P1

    %% ---------- P1 ----------
    subgraph P1G["P1 · 오브젝트 탐색"]
        P1["scan 서비스 호출<br/>(waypoints)"]
        P1 --> P1WAIT{"YOLO 탐지?<br/>class·conf·distance"}
        P1WAIT -->|"timeout 30s"| P1RT["session_retry++"]
        P1RT --> P1CHK{"retry > 3?"}
        P1CHK -->|"yes"| FAIL(["실패 보고·종료"])
        P1CHK -->|"no"| P1HOME["home 서비스"] --> P1
    end
    P1WAIT -->|"검출 성공<br/>position_3d_base_frame"| P2

    %% ---------- P2 ----------
    subgraph P2G["P2 · 파라미터 조립 & Pick"]
        P2["Object Spec 조회<br/>(config/objects.yaml)"]
        P2 --> P2CHK{"등록된 class?"}
        P2CHK -->|"no"| FAIL
        P2CHK -->|"yes"| P2TF["base→world TF<br/>+ z 보정(h/2)"]
        P2TF --> P2BUILD["_build_pick_params<br/>(shape·dims·grasp = config)"]
        P2BUILD --> P2PICK["pick 서비스 호출"]
        P2PICK --> P2GRIP{"grip 검증 (YOLO)<br/>distance ≤ 0.15m"}
        P2GRIP -->|"실패 (≤3회)"| P2TF
        P2GRIP -->|"3회 초과"| FAIL
    end
    P2GRIP -->|"grip 성공"| P3

    %% ---------- P3 ----------
    subgraph P3G["P3 · Pickup 검증"]
        P3["lift 서비스 (z+ 0.1m)"]
        P3 --> P3COL["YOLO 프레임 수집<br/>(verify_window)"]
        P3COL --> P3LLM["verify_pickup (Gemini)<br/>{pickup_success, reason}"]
    end
    P3LLM -->|"success=false"| P3FAIL["release + home 서비스<br/>session_retry++"]
    P3FAIL --> P1CHK
    P3LLM -->|"success=true"| P4

    %% ---------- P4 ----------
    subgraph P4G["P4 · 배치 & 종료"]
        P4["place 서비스<br/>(targets.yaml)"]
        P4 --> P4REL["release 서비스"]
        P4REL --> P4HOME["home 서비스"]
    end
    P4HOME --> DONE(["세션 완료·성공 보고"])

    classDef llm fill:#fef0e6,stroke:#c80,color:#000;
    classDef svc fill:#e8f0fe,stroke:#37c,color:#000;
    classDef cfg fill:#eafbe7,stroke:#3a3,color:#000;
    class PARSE,P3LLM llm;
    class P1,P1HOME,P2PICK,P3,P4,P4REL,P4HOME,P3FAIL svc;
    class P2,P2BUILD cfg;
```

### 데이터 산출물 요약
| 단계 | 입력 | 처리 | 출력 |
|------|------|------|------|
| parse | 자연어 명령 | Gemini | `target_class_name` |
| P1 | target | scan 서비스 + YOLO 매칭 | `position_3d_base_frame` |
| P2 | base 좌표 + class | TF(base→world) + config Object Spec + pick 서비스 | grip 성공/실패 |
| P3 | — | lift 서비스 + YOLO 수집 + Gemini 판단 | `pickup_success` |
| P4 | place target | place/release/home 서비스 | 세션 완료 |

> **공유 retry 카운터**: P1 타임아웃 · P2 grip 실패 · P3 pickup 실패가 동일 `session_retry_count`를
> 증가시키며 3 초과 시 세션 전체 종료. (`PhaseManager`)
>
> **LLM 호출은 2곳**: `parse_command`(진입), `verify_pickup`(P3). P2 PICK 파라미터는 config 기반 결정론 조립.
