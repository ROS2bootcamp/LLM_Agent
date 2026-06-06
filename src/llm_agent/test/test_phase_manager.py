import pytest
from llm_agent.phase_manager import PhaseManager, PhaseTransitionError


@pytest.fixture
def pm():
    return PhaseManager()


class TestInitialState:
    def test_starts_idle(self, pm):
        assert pm.current_phase == 'IDLE'

    def test_retry_zero(self, pm):
        assert pm.session_retry_count == 0

    def test_not_exceeded(self, pm):
        assert not pm.retry_exceeded


class TestTransitions:
    def test_p1_transition(self, pm):
        pm.transition('P1', reason='test')
        assert pm.current_phase == 'P1'

    def test_full_happy_path(self, pm):
        for phase in ['P1', 'P2', 'P3', 'P4']:
            pm.transition(phase)
        assert pm.current_phase == 'P4'

    def test_invalid_phase_raises(self, pm):
        with pytest.raises(PhaseTransitionError):
            pm.transition('INVALID')


class TestRetry:
    def test_increment_retry(self, pm):
        pm.increment_retry('timeout')
        assert pm.session_retry_count == 1

    def test_retry_exceeded_at_3(self, pm):
        for _ in range(3):
            pm.increment_retry()
        assert pm.retry_exceeded

    def test_not_exceeded_at_2(self, pm):
        for _ in range(2):
            pm.increment_retry()
        assert not pm.retry_exceeded


class TestReset:
    def test_reset_clears_all(self, pm):
        pm.transition('P2')
        pm.increment_retry()
        pm.reset_session()
        assert pm.current_phase == 'IDLE'
        assert pm.session_retry_count == 0
        assert pm.recent_history() == []


class TestSystemPrompts:
    def test_all_prompts_nonempty(self, pm):
        for key in ('parse_command', 'verify_pickup'):
            assert pm.system_prompt(key).strip() != ''

    def test_pick_params_prompt_removed(self, pm):
        # P2 PICK 파라미터 생성은 config-only 결정론 조립으로 대체됨
        assert pm.system_prompt('generate_pick_params') == ''
