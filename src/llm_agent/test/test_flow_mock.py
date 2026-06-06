"""End-to-end orchestration flow verification with mocked collaborators.

ROS2(rclpy) 없이 AgentNode 의 Phase 루프(_run_session, P1~P4)를 검증한다.
rclpy/std_msgs/tf2/llm_agent_msgs 등 ROS 모듈을 sys.modules 에 stub 으로
주입한 뒤, AgentNode 를 __new__ 로 만들어 협력자(yolo/moveit/llm/tf)만 mock 으로
교체하고 실제 오케스트레이션 코드를 그대로 실행한다.

실행:
    python src/llm_agent/test/test_flow_mock.py     # standalone (pytest 불필요)
    pytest src/llm_agent/test/test_flow_mock.py     # ROS 환경에서도 동작
"""

import io
import os
import sys
import types
from unittest.mock import MagicMock

# ---------------------------------------------------------------------------
# 1) ROS 모듈 stub 주입 (import 가능하게만; 서브클래스/인스턴스화되는 것만 실제 클래스)
# ---------------------------------------------------------------------------

def _mod(name: str) -> types.ModuleType:
    m = types.ModuleType(name)
    sys.modules[name] = m
    return m


def _install_ros_stubs() -> None:
    if 'rclpy' in sys.modules and not isinstance(sys.modules['rclpy'], types.ModuleType):
        return

    rclpy = _mod('rclpy')
    rclpy.init = lambda *a, **k: None
    rclpy.shutdown = lambda *a, **k: None
    t = _mod('rclpy.time'); t.Time = lambda *a, **k: None
    rclpy.time = t
    nd = _mod('rclpy.node')
    nd.Node = type('Node', (object,), {'__init__': lambda self, *a, **k: None})
    cbg = _mod('rclpy.callback_groups')
    cbg.ReentrantCallbackGroup = type('ReentrantCallbackGroup', (object,), {})
    ex = _mod('rclpy.executors')
    ex.MultiThreadedExecutor = type('MultiThreadedExecutor', (object,), {})
    du = _mod('rclpy.duration'); du.Duration = lambda *a, **k: None

    sm = _mod('std_msgs'); smm = _mod('std_msgs.msg')
    smm.String = type('String', (object,), {'__init__': lambda self: setattr(self, 'data', '')})
    sm.msg = smm

    gm = _mod('geometry_msgs'); gmm = _mod('geometry_msgs.msg')
    gmm.PointStamped = type('PointStamped', (object,), {})
    gm.msg = gmm

    sys.modules['tf2_ros'] = MagicMock()
    sys.modules['tf2_geometry_msgs'] = MagicMock()

    lam = _mod('llm_agent_msgs'); lamsrv = _mod('llm_agent_msgs.srv')
    lamsrv.MoveItExecute = MagicMock()
    lam.srv = lamsrv

    aip = _mod('ament_index_python'); aipp = _mod('ament_index_python.packages')
    aipp.get_package_share_directory = lambda pkg: '.'
    aip.packages = aipp


_install_ros_stubs()

# llm_agent 패키지 경로 (src/llm_agent) 추가
_PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

import llm_agent.agent_node as an  # noqa: E402
from llm_agent.agent_node import AgentNode  # noqa: E402
from llm_agent.llm_client import PARSE_COMMAND_SCHEMA  # noqa: E402
from llm_agent.phase_manager import PhaseManager  # noqa: E402

an.time.sleep = lambda *a, **k: None  # 테스트 중 sleep 제거

_CONFIG_DIR = os.path.join(_PKG_ROOT, 'config')

OK = {'success': True, 'error_code': 0, 'error_message': ''}
FAIL = {'success': False, 'error_code': -1, 'error_message': 'mock failure'}


def _load_cfg() -> dict:
    import yaml
    cfg = yaml.safe_load(io.open(os.path.join(_CONFIG_DIR, 'agent.yaml'), encoding='utf-8'))
    cfg['scan_waypoints'] = yaml.safe_load(
        io.open(os.path.join(_CONFIG_DIR, 'scan_waypoints.yaml'), encoding='utf-8')).get('waypoints', [])
    cfg['target_pose'] = yaml.safe_load(
        io.open(os.path.join(_CONFIG_DIR, 'targets.yaml'), encoding='utf-8')).get('place_target', {})
    od = yaml.safe_load(io.open(os.path.join(_CONFIG_DIR, 'objects.yaml'), encoding='utf-8')) or {}
    cfg['objects'] = od.get('objects', {})
    cfg['default_object_spec'] = od.get('default_spec')
    return cfg


def _detection(cls='cup', conf=0.92, dist=0.4, x=0.4, y=0.1, z=0.06) -> dict:
    return {
        'class_name': cls, 'confidence': conf, 'distance_m': dist,
        'position_3d_base_frame': {'X': x, 'Y': y, 'Z': z}, 'timestamp_ns': 1,
    }


def _make_node(cfg):
    """AgentNode 를 __init__ 우회로 생성하고 협력자를 mock 으로 교체."""
    node = AgentNode.__new__(AgentNode)
    node._cfg = cfg
    node._pm = PhaseManager()
    node._busy = False
    node._yolo = MagicMock()
    node._moveit = MagicMock()
    node._llm = MagicMock()
    node._tf = MagicMock()

    cap = {'logs': [], 'reports': [], 'phases': [], 'moveit': []}
    node._log = lambda t: cap['logs'].append(t)
    node._report = lambda t: cap['reports'].append(t)
    node._pub_phase = lambda: cap['phases'].append(node._pm.current_phase)

    node._tf.to_world = lambda x, y, z, timeout_sec=3.0: (x, y, z)
    return node, cap


def _wire(node, cap, *, scan, grip_verify, verify_seq, moveit_resp, target='cup'):
    """공통 mock 동작 주입.

    scan: P1/P2 재탐지 wait_for_detection 결과(None=미탐지)
    grip_verify: max_distance_m 지정 호출(P2 grip 검증) 결과(None=실패)
    verify_seq: P3 LLM pickup_success 시퀀스 (list[bool])
    moveit_resp: {cmd: response} (없으면 OK)
    """
    def yolo_wait(**kw):
        if kw.get('max_distance_m') is not None:
            return grip_verify
        return scan
    node._yolo.wait_for_detection = lambda **kw: yolo_wait(**kw)
    node._yolo.recent = lambda n=10: [{'objects': [_detection(target)], 'timestamp_ns': 1}]
    node._yolo.clear = lambda: None

    seq = list(verify_seq)

    def llm_call(system_prompt, user_content, response_schema=None):
        if response_schema is PARSE_COMMAND_SCHEMA:
            return {'target_class_name': target}
        ok = seq.pop(0) if seq else False
        return {'pickup_success': ok, 'reason': 'mock'}
    node._llm.call = llm_call

    def mv(cmd, params=None, timeout_sec=10.0):
        cap['moveit'].append(cmd)
        return moveit_resp.get(cmd, OK)
    node._moveit.call_and_wait = mv


# ---------------------------------------------------------------------------
# 시나리오
# ---------------------------------------------------------------------------

def test_happy_path():
    cfg = _load_cfg()
    node, cap = _make_node(cfg)
    _wire(node, cap, scan=_detection(), grip_verify=_detection(dist=0.05),
          verify_seq=[True], moveit_resp={})
    node._run_session('컵 집어')

    assert cap['reports'] == ['세션 완료: Pick & Place 성공.'], cap['reports']
    assert cap['phases'] == ['P1', 'P2', 'P3', 'P4'], cap['phases']
    assert cap['moveit'] == ['scan', 'pick', 'lift', 'place', 'release', 'home'], cap['moveit']
    assert node._busy is False


def test_p1_timeout_exhausted():
    cfg = _load_cfg()
    node, cap = _make_node(cfg)
    _wire(node, cap, scan=None, grip_verify=None, verify_seq=[], moveit_resp={})
    node._run_session('컵 집어')

    # NOTE: _run_session finally 가 retry_count 를 0 으로 리셋하므로 호출 횟수로 검증.
    assert any('P1 최종 실패' in r for r in cap['reports']), cap['reports']
    assert cap['moveit'].count('scan') == PhaseManager.SESSION_MAX_RETRY, cap['moveit']
    assert 'pick' not in cap['moveit']  # P2 도달 안 함


def test_p2_grip_fail():
    cfg = _load_cfg()
    node, cap = _make_node(cfg)
    # 탐지는 되지만 grip 검증(max_distance_m)은 계속 실패
    _wire(node, cap, scan=_detection(), grip_verify=None, verify_seq=[], moveit_resp={})
    node._run_session('컵 집어')

    assert any('P2 최종 실패' in r for r in cap['reports']), cap['reports']
    # grip retry_count 만큼 pick 시도
    assert cap['moveit'].count('pick') == cfg['grip']['retry_count'], cap['moveit']


def test_p3_fail_then_succeed():
    cfg = _load_cfg()
    node, cap = _make_node(cfg)
    _wire(node, cap, scan=_detection(), grip_verify=_detection(dist=0.05),
          verify_seq=[False, True], moveit_resp={})
    node._run_session('컵 집어')

    assert cap['reports'] == ['세션 완료: Pick & Place 성공.'], cap['reports']
    # P3 1회 실패 → release+home 후 재시작 → 2번째 성공
    assert cap['moveit'].count('scan') == 2  # P1 두 번 진입
    assert cap['phases'] == ['P1', 'P2', 'P3', 'P1', 'P2', 'P3', 'P4'], cap['phases']


def test_p3_fail_exhausted():
    cfg = _load_cfg()
    node, cap = _make_node(cfg)
    _wire(node, cap, scan=_detection(), grip_verify=_detection(dist=0.05),
          verify_seq=[False, False, False, False], moveit_resp={})
    node._run_session('컵 집어')

    assert any('P3 최종 실패' in r for r in cap['reports']), cap['reports']
    # P3 가 SESSION_MAX_RETRY 회 실패 → P1 그만큼 재진입
    assert cap['moveit'].count('scan') == PhaseManager.SESSION_MAX_RETRY, cap['moveit']
    assert cap['phases'].count('P3') == PhaseManager.SESSION_MAX_RETRY, cap['phases']


def test_p4_place_fail_reports_failure():
    """G1 회귀: place 실패가 성공으로 보고되지 않고, release 도 호출되지 않아야 한다."""
    cfg = _load_cfg()
    node, cap = _make_node(cfg)
    _wire(node, cap, scan=_detection(), grip_verify=_detection(dist=0.05),
          verify_seq=[True], moveit_resp={'place': FAIL})
    node._run_session('컵 집어')

    assert any('P4 실패' in r for r in cap['reports']), cap['reports']
    assert '세션 완료: Pick & Place 성공.' not in cap['reports']
    assert 'release' not in cap['moveit'], cap['moveit']   # 낙하 방지
    assert cap['moveit'][-1] == 'home'                      # 안전 복귀


def test_unregistered_class_uses_default_spec():
    cfg = _load_cfg()
    assert cfg['default_object_spec'] is not None
    node, cap = _make_node(cfg)
    _wire(node, cap, scan=_detection('banana'), grip_verify=_detection('banana', dist=0.05),
          verify_seq=[True], moveit_resp={}, target='banana')
    node._run_session('바나나 집어')

    assert cap['reports'] == ['세션 완료: Pick & Place 성공.'], cap['reports']
    assert any('기본 spec 폴백' in l for l in cap['logs']), cap['logs']


def test_unregistered_class_no_fallback_fails():
    cfg = _load_cfg()
    cfg['default_object_spec'] = None  # 폴백 비활성
    node, cap = _make_node(cfg)
    _wire(node, cap, scan=_detection('banana'), grip_verify=None,
          verify_seq=[], moveit_resp={}, target='banana')
    node._run_session('바나나 집어')

    assert any('P2 최종 실패' in r for r in cap['reports']), cap['reports']
    assert 'pick' not in cap['moveit']


def test_parse_fail_aborts():
    cfg = _load_cfg()
    node, cap = _make_node(cfg)
    _wire(node, cap, scan=_detection(), grip_verify=None, verify_seq=[], moveit_resp={})
    # parse 결과 빈 target
    node._llm.call = lambda system_prompt, user_content, response_schema=None: {'target_class_name': ''}
    node._run_session('알 수 없는 명령')

    assert any('명령 파싱 실패' in r for r in cap['reports']), cap['reports']
    assert cap['moveit'] == []


def test_build_pick_params_z_correction():
    """cylinder/box z 보정(바닥→중심) 검증."""
    cfg = _load_cfg()
    node, _ = _make_node(cfg)
    mc = cfg['moveit']
    cyl = node._build_pick_params({'shape': 'cylinder', 'dimensions': [0.12, 0.025]},
                                  0.4, 0.1, 0.0, mc)
    assert abs(cyl['object']['pose_world'][2] - 0.06) < 1e-9, cyl['object']['pose_world']
    box = node._build_pick_params({'shape': 'box', 'dimensions': [0.1, 0.1, 0.2]},
                                  0.4, 0.1, 0.0, mc)
    assert abs(box['object']['pose_world'][2] - 0.10) < 1e-9, box['object']['pose_world']


# ---------------------------------------------------------------------------
# standalone 러너 (pytest 없이 실행)
# ---------------------------------------------------------------------------

def _run_all():
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_') and callable(v)]
    passed = failed = 0
    for t in tests:
        try:
            t()
            print(f'  PASS  {t.__name__}')
            passed += 1
        except AssertionError as e:
            print(f'  FAIL  {t.__name__}: {e}')
            failed += 1
        except Exception as e:  # noqa: BLE001
            print(f'  ERROR {t.__name__}: {type(e).__name__}: {e}')
            failed += 1
    print(f'\n{passed} passed, {failed} failed (total {len(tests)})')
    return failed


if __name__ == '__main__':
    print('=== AgentNode flow mock verification ===')
    sys.exit(1 if _run_all() else 0)
