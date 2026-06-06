"""Phase state machine for the 4-phase Pick & Place agent."""

VALID_PHASES = {'P1', 'P2', 'P3', 'P4', 'IDLE', 'FAILED'}


class PhaseTransitionError(Exception):
    pass


class PhaseManager:
    """
    Manages session phase (P1 → P2 → P3 → P4) and shared retry counter.

    session_retry_count is shared between P1 timeout and P3 pickup failure.
    Maximum SESSION_MAX_RETRY retries before final failure report.
    """

    SESSION_MAX_RETRY = 3

    def __init__(self):
        self._phase = 'IDLE'
        self._session_retry_count = 0
        self._history: list[dict] = []

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def current_phase(self) -> str:
        return self._phase

    @property
    def session_retry_count(self) -> int:
        return self._session_retry_count

    @property
    def retry_exceeded(self) -> bool:
        return self._session_retry_count >= self.SESSION_MAX_RETRY

    def transition(self, target: str, reason: str = '') -> None:
        if target not in VALID_PHASES:
            raise PhaseTransitionError(f"Unknown phase: '{target}'")
        self._history.append({'from': self._phase, 'to': target, 'reason': reason})
        self._phase = target

    def increment_retry(self, reason: str = '') -> None:
        self._session_retry_count += 1
        self._history.append({
            'event': 'retry',
            'count': self._session_retry_count,
            'reason': reason,
        })

    def reset_session(self) -> None:
        self._phase = 'IDLE'
        self._session_retry_count = 0
        self._history.clear()

    def recent_history(self, n: int = 20) -> list[dict]:
        return self._history[-n:]

    def system_prompt(self, call_type: str) -> str:
        """Return the system prompt for the given LLM call type."""
        return _SYSTEM_PROMPTS.get(call_type, '')


# Phase별 LLM 시스템 프롬프트 (2개 호출 지점)
# P2 PICK 파라미터 생성은 config-only 정책으로 결정론적 조립으로 대체되어 LLM 호출 제외.
_SYSTEM_PROMPTS = {
    'parse_command': (
        "당신은 로봇 조작 시스템의 명령 파싱 에이전트입니다.\n"
        "사용자의 자연어 명령에서 대상 오브젝트의 class_name을 추출하세요.\n"
        "YOLO(YOLOv8/COCO) 모델의 라벨 형식 그대로 반환하세요"
        '(예: "cup", "bottle", "stop sign").\n'
        '응답은 반드시 JSON 형식으로만: {"target_class_name": "cup"}'
    ),
    'verify_pickup': (
        "당신은 로봇 pickup 성공 여부를 판단하는 에이전트입니다.\n"
        "제공된 YOLO 감지 데이터(objects 배열)를 분석해 gripper가 오브젝트를\n"
        "쥐고 있는지 판단하세요. 대상이 계속 가까이 감지되면 성공으로 봅니다.\n"
        '응답은 반드시 JSON 형식으로만: {"pickup_success": true, "reason": "..."}'
    ),
}
