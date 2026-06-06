"""
LLM Agent Node — orchestrates P1→P2→P3→P4 Pick & Place session.

Phase loop runs in a ThreadPoolExecutor so the ROS2 spin thread
(MultiThreadedExecutor) is never blocked.
"""

import json
import time
from concurrent.futures import ThreadPoolExecutor

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from llm_agent.cli_reader import CLIReader
from llm_agent.llm_client import (
    LLMClient,
    PARSE_COMMAND_SCHEMA,
    VERIFY_PICKUP_SCHEMA,
)
from llm_agent.moveit_client import MoveItClient
from llm_agent.phase_manager import PhaseManager
from llm_agent.tf_transformer import TFTransformer
from llm_agent.yolo_subscriber import YoloSubscriber


class AgentNode(Node):

    def __init__(self):
        super().__init__('llm_agent')
        self._declare_params()

        cfg = self._load_config()
        self._cfg = cfg

        cb = ReentrantCallbackGroup()
        self._phase_pub = self.create_publisher(String, '/llm_agent/phase', 10)
        self._log_pub  = self.create_publisher(String, '/llm_agent/log',   10)

        self._yolo  = YoloSubscriber(
            self,
            topic=cfg['yolo']['detection_topic'],
            buffer_size=60,
        )
        self._tf    = TFTransformer(
            self,
            base_frame=cfg['tf']['base_frame'],
            world_frame=cfg['tf']['world_frame'],
        )
        self._moveit = MoveItClient(self, service_name=cfg['moveit']['service_name'])
        self._llm    = LLMClient(
            model=cfg['llm']['model'],
            max_tokens=cfg['llm']['max_tokens'],
            max_retry=cfg['llm']['retry_count'],
            retry_backoff_sec=cfg['llm']['retry_backoff_sec'],
        )
        self._pm     = PhaseManager()
        self._cli    = CLIReader()
        self._pool   = ThreadPoolExecutor(max_workers=1)
        self._busy   = False

        self._cli.start()
        self.create_timer(0.1, self._poll_cli, callback_group=cb)
        self.get_logger().info('AgentNode ready. Waiting for CLI input...')

    # ------------------------------------------------------------------
    # CLI polling
    # ------------------------------------------------------------------

    def _poll_cli(self) -> None:
        if self._busy:
            return
        cmd = self._cli.get(timeout=0.0)
        if cmd:
            self._busy = True
            self._pm.reset_session()
            self._pool.submit(self._run_session, cmd)

    # ------------------------------------------------------------------
    # Session orchestration
    # ------------------------------------------------------------------

    def _run_session(self, raw_command: str) -> None:
        self._log(f'Session start: "{raw_command}"')
        cfg = self._cfg
        try:
            # ── Parse command ──────────────────────────────────────────
            target = self._parse_command(raw_command)
            if not target:
                self._report('명령 파싱 실패: 대상 오브젝트를 인식하지 못했습니다.')
                return
            self._log(f'Target class: {target}')

            # ── Phase loop ────────────────────────────────────────────
            while True:
                # P1
                detection = self._run_p1(target, cfg)
                if detection is None:
                    self._report(f'P1 최종 실패: {target} 를 찾지 못했습니다.')
                    return

                # P2
                pick_ok = self._run_p2(target, detection, cfg)
                if not pick_ok:
                    self._report('P2 최종 실패: grip에 반복 실패했습니다.')
                    return

                # P3
                pickup_ok = self._run_p3(target, cfg)
                if pickup_ok:
                    break  # → P4

                # P3 실패 → retry counter 증가 후 P1 재시작
                self._pm.increment_retry(reason='P3 pickup failed')
                if self._pm.retry_exceeded:
                    self._report('P3 최종 실패: pickup 검증에 반복 실패했습니다.')
                    return
                self._log(f'P3 실패 — P1 재시작 (retry {self._pm.session_retry_count})')
                self._moveit.call_and_wait(
                    'release', {'hand_open_pose': cfg['moveit']['hand_open_pose']},
                    timeout_sec=cfg['moveit']['service_timeout_sec'],
                )
                self._moveit.call_and_wait(
                    'home', {'arm_home_pose': cfg['moveit']['arm_home_pose']},
                    timeout_sec=cfg['moveit']['service_timeout_sec'],
                )
                time.sleep(1.0)

            # P4
            if self._run_p4(cfg):
                self._report('세션 완료: Pick & Place 성공.')
            else:
                self._report('P4 실패: place에 실패해 오브젝트를 놓지 못했습니다.')

        except Exception as e:
            self._report(f'예외 발생으로 세션 종료: {e}')
            self.get_logger().error(str(e))
        finally:
            self._busy = False
            self._pm.reset_session()

    # ------------------------------------------------------------------
    # P1 — Object search
    # ------------------------------------------------------------------

    def _run_p1(self, target: str, cfg: dict) -> dict | None:
        self._pm.transition('P1', reason='search start')
        self._pub_phase()
        moveit_cfg = cfg['moveit']
        scan_cfg   = cfg['scan']

        waypoints = cfg.get('scan_waypoints', [])
        self._yolo.clear()
        self._moveit.call_and_wait(
            'scan', {'waypoints': waypoints},
            timeout_sec=scan_cfg['timeout_sec'] + moveit_cfg['service_timeout_sec'],
        )

        self._log(f'P1: scanning for "{target}" (timeout {scan_cfg["timeout_sec"]}s)')
        result = self._yolo.wait_for_detection(
            class_name=target,
            confidence_threshold=scan_cfg['confidence_threshold'],
            timeout_sec=scan_cfg['timeout_sec'],
            selection_policy=scan_cfg.get('selection_policy', 'highest_conf'),
        )

        if result is not None:
            self._log(f'P1: detected {target} conf={result["confidence"]}')
            return result

        # timeout
        self._pm.increment_retry(reason='P1 scan timeout')
        if self._pm.retry_exceeded:
            return None
        self._log(f'P1: timeout — retry {self._pm.session_retry_count}')
        self._moveit.call_and_wait(
            'home', {'arm_home_pose': moveit_cfg['arm_home_pose']},
            timeout_sec=moveit_cfg['service_timeout_sec'],
        )
        time.sleep(1.0)
        return self._run_p1(target, cfg)  # recursive retry

    # ------------------------------------------------------------------
    # P2 — Parameter generation and Pick
    # ------------------------------------------------------------------

    def _run_p2(self, target: str, detection: dict, cfg: dict) -> bool:
        self._pm.transition('P2', reason='detection confirmed')
        self._pub_phase()

        moveit_cfg = cfg['moveit']
        grip_cfg   = cfg['grip']

        # Object Spec 조회 (config 사전 정의만, LLM 추정 없음)
        spec = cfg.get('objects', {}).get(target)
        if spec is None:
            spec = cfg.get('default_object_spec')
            if spec is None:
                self._log(f'P2: 미등록 class "{target}" — Object Spec 없음 (config/objects.yaml)')
                return False
            self._log(f'P2: 미등록 class "{target}" — 기본 spec 폴백 사용 {spec}')

        for attempt in range(grip_cfg['retry_count']):
            # 최신 프레임으로 base_frame 좌표 갱신 (최대 1초)
            fresh = self._yolo.wait_for_detection(
                class_name=target,
                confidence_threshold=cfg['scan']['confidence_threshold'],
                timeout_sec=1.0,
                selection_policy=cfg['scan'].get('selection_policy', 'highest_conf'),
            ) or detection
            base = fresh['position_3d_base_frame']

            # base_link → world 정적 변환
            try:
                wx, wy, wz = self._tf.to_world(base['X'], base['Y'], base['Z'])
            except Exception as e:
                self._log(f'P2: TF transform failed: {e}')
                return False

            # PICK 파라미터 결정론적 조립 (config Object Spec + pose_world)
            pick_params = self._build_pick_params(spec, wx, wy, wz, moveit_cfg)
            self._log(f'P2: sending PICK (attempt {attempt+1})')

            status = self._moveit.call_and_wait(
                'pick', pick_params, timeout_sec=moveit_cfg['service_timeout_sec']
            )
            if not status.get('success', False):
                self._log(f'P2: MoveIt pick failed: {status.get("error_message")}')
                continue

            # Grip 검증 via YOLO (gripper/camera 근접 = distance_m 임계값 이내)
            time.sleep(0.5)
            verify = self._yolo.wait_for_detection(
                class_name=target,
                confidence_threshold=cfg['scan']['confidence_threshold'],
                timeout_sec=grip_cfg['verify_window_sec'],
                max_distance_m=grip_cfg['distance_threshold_m'],
            )
            if verify is not None:
                self._log(f'P2: grip confirmed (distance={verify["distance_m"]:.3f}m)')
                return True
            self._log('P2: grip verify — object not detected near gripper')

        return False

    # ------------------------------------------------------------------
    # P3 — Pickup verification
    # ------------------------------------------------------------------

    def _run_p3(self, target: str, cfg: dict) -> bool:
        self._pm.transition('P3', reason='pick complete')
        self._pub_phase()

        # Lift 10 cm
        lift_status = self._moveit.call_and_wait(
            'lift', {'direction': 'z+', 'distance_m': 0.1, 'frame': 'world'},
            timeout_sec=cfg['moveit']['service_timeout_sec'],
        )
        if not lift_status.get('success', False):
            self._log('P3: lift failed')
            return False

        # YOLO 수집
        time.sleep(cfg['grip']['verify_window_sec'])
        frames = self._yolo.recent(n=10)

        # LLM 판단
        result = self._llm.call(
            system_prompt=self._pm.system_prompt('verify_pickup'),
            user_content=(
                f'target_class_name: {target}\n'
                f'YOLO frames (recent {len(frames)}):\n'
                + json.dumps(frames, ensure_ascii=False)
            ),
            response_schema=VERIFY_PICKUP_SCHEMA,
        )
        success = result.get('pickup_success', False)
        self._log(f'P3: pickup_success={success} reason={result.get("reason")}')
        return success

    # ------------------------------------------------------------------
    # P4 — Place and finish
    # ------------------------------------------------------------------

    def _run_p4(self, cfg: dict) -> bool:
        self._pm.transition('P4', reason='pickup verified')
        self._pub_phase()

        target_pose = cfg['target_pose']
        object_name = 'target_object'
        moveit_cfg  = cfg['moveit']
        timeout     = moveit_cfg['service_timeout_sec']

        place_status = self._moveit.call_and_wait(
            'place',
            {
                'object_name': object_name,
                'target_pose_world': [
                    target_pose['x'], target_pose['y'], target_pose['z'],
                    target_pose['roll'], target_pose['pitch'], target_pose['yaw'],
                ],
                'place_surface_offset': 0.001,
            },
            timeout_sec=timeout,
        )
        if not place_status.get('success', False):
            # place 실패: 오브젝트를 쥔 채 release 하면 임의 위치 낙하 → release 생략, HOME 만.
            self._log(f'P4: place failed: {place_status.get("error_message")}')
            self._moveit.call_and_wait(
                'home', {'arm_home_pose': moveit_cfg['arm_home_pose']}, timeout_sec=timeout
            )
            return False
        self._log(f'P4: place status={place_status.get("success")}')

        self._moveit.call_and_wait(
            'release', {'hand_open_pose': moveit_cfg['hand_open_pose']}, timeout_sec=timeout
        )
        self._moveit.call_and_wait(
            'home', {'arm_home_pose': moveit_cfg['arm_home_pose']}, timeout_sec=timeout
        )
        return True

    # ------------------------------------------------------------------
    # LLM calls
    # ------------------------------------------------------------------

    def _parse_command(self, raw: str) -> str | None:
        result = self._llm.call(
            system_prompt=self._pm.system_prompt('parse_command'),
            user_content=f'명령: {raw}',
            response_schema=PARSE_COMMAND_SCHEMA,
        )
        return result.get('target_class_name')

    def _build_pick_params(
        self, spec: dict, wx: float, wy: float, wz: float, moveit_cfg: dict
    ) -> dict:
        """PICK params_json 결정론적 조립 (config Object Spec + world pose).

        shape/dimensions=config, pose=YOLO base_frame+TF, grasp 파라미터=config.
        YOLO position 을 바닥으로 보고 중심 z 로 보정
        (cylinder +height/2, box +z/2).
        """
        shape = spec['shape']
        dims = list(spec['dimensions'])
        z = wz
        if shape == 'cylinder':
            z = wz + dims[0] / 2.0  # 바닥 → 원통 중심
        elif shape == 'box':
            z = wz + dims[2] / 2.0  # 바닥 → 박스 중심

        return {
            'arm_group_name':  moveit_cfg['arm_group_name'],
            'eef_name':        moveit_cfg['eef_name'],
            'hand_group_name': moveit_cfg['hand_group_name'],
            'hand_frame':      moveit_cfg['hand_frame'],
            'hand_open_pose':  moveit_cfg['hand_open_pose'],
            'hand_close_pose': moveit_cfg['hand_close_pose'],
            'object': {
                'name': 'target_object',
                'shape': shape,
                'dimensions': dims,
                'pose_world': [wx, wy, z, 0.0, 0.0, 0.0],
            },
            'grasp_frame_transform':    moveit_cfg['grasp_frame_transform'],
            'approach_object_min_dist': moveit_cfg['approach_object_min_dist'],
            'approach_object_max_dist': moveit_cfg['approach_object_max_dist'],
            'lift_object_min_dist':     moveit_cfg['lift_object_min_dist'],
            'lift_object_max_dist':     moveit_cfg['lift_object_max_dist'],
            'max_solutions':            moveit_cfg['max_solutions'],
        }

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _pub_phase(self) -> None:
        msg = String()
        msg.data = self._pm.current_phase
        self._phase_pub.publish(msg)

    def _log(self, text: str) -> None:
        self.get_logger().info(text)
        msg = String()
        msg.data = text
        self._log_pub.publish(msg)

    def _report(self, text: str) -> None:
        print(f'\n[Agent] {text}\n')
        self._log(text)

    def _declare_params(self) -> None:
        import os
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('llm_agent')
        self.declare_parameter(
            'config_path',
            os.path.join(share, 'config', 'agent.yaml'),
        )
        self.declare_parameter(
            'scan_waypoints_path',
            os.path.join(share, 'config', 'scan_waypoints.yaml'),
        )
        self.declare_parameter(
            'targets_path',
            os.path.join(share, 'config', 'targets.yaml'),
        )
        self.declare_parameter(
            'objects_path',
            os.path.join(share, 'config', 'objects.yaml'),
        )

    def _load_config(self) -> dict:
        import yaml
        paths = {
            'config':    self.get_parameter('config_path').value,
            'waypoints': self.get_parameter('scan_waypoints_path').value,
            'targets':   self.get_parameter('targets_path').value,
            'objects':   self.get_parameter('objects_path').value,
        }
        with open(paths['config']) as f:
            cfg = yaml.safe_load(f)
        with open(paths['waypoints']) as f:
            cfg['scan_waypoints'] = yaml.safe_load(f).get('waypoints', [])
        with open(paths['targets']) as f:
            cfg['target_pose'] = yaml.safe_load(f).get('place_target', {})
        with open(paths['objects']) as f:
            objects_data = yaml.safe_load(f) or {}
        cfg['objects'] = objects_data.get('objects', {})
        cfg['default_object_spec'] = objects_data.get('default_spec')
        return cfg


def main(args=None):
    rclpy.init(args=args)
    node = AgentNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
