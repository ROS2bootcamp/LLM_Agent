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
from llm_agent.llm_client import LLMClient
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

        self._yolo  = YoloSubscriber(self, buffer_size=60)
        self._tf    = TFTransformer(
            self,
            camera_frame=cfg['tf']['camera_frame'],
            world_frame=cfg['tf']['world_frame'],
        )
        self._moveit = MoveItClient(self)
        self._llm    = LLMClient(
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
                self._moveit.send({'cmd': 'home', 'arm_home_pose': cfg['moveit']['arm_home_pose']})
                time.sleep(2.0)

            # P4
            self._run_p4(cfg)
            self._report('세션 완료: Pick & Place 성공.')

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
        self._moveit.send({'cmd': 'scan', 'waypoints': waypoints})

        self._log(f'P1: scanning for "{target}" (timeout {scan_cfg["timeout_sec"]}s)')
        result = self._yolo.wait_for_detection(
            class_name=target,
            confidence_threshold=scan_cfg['confidence_threshold'],
            timeout_sec=scan_cfg['timeout_sec'],
        )

        if result is not None:
            self._log(f'P1: detected {target} conf={result["confidence"]}')
            return result

        # timeout
        self._pm.increment_retry(reason='P1 scan timeout')
        if self._pm.retry_exceeded:
            return None
        self._log(f'P1: timeout — retry {self._pm.session_retry_count}')
        self._moveit.send({'cmd': 'home', 'arm_home_pose': moveit_cfg['arm_home_pose']})
        time.sleep(1.5)
        return self._run_p1(target, cfg)  # recursive retry

    # ------------------------------------------------------------------
    # P2 — Parameter generation and Pick
    # ------------------------------------------------------------------

    def _run_p2(self, target: str, detection: dict, cfg: dict) -> bool:
        self._pm.transition('P2', reason='detection confirmed')
        self._pub_phase()

        moveit_cfg = cfg['moveit']
        grip_cfg   = cfg['grip']

        for attempt in range(grip_cfg['retry_count']):
            # TF 변환: camera → world
            cam = detection['position_3d_camera_frame']
            try:
                wx, wy, wz = self._tf.to_world(cam['x'], cam['y'], cam['z'])
            except Exception as e:
                self._log(f'P2: TF transform failed: {e}')
                return False

            # LLM → PICK 파라미터 생성
            pick_cmd = self._generate_pick_params(target, wx, wy, wz, moveit_cfg)
            self._log(f'P2: sending PICK (attempt {attempt+1})')

            status = self._moveit.send_and_wait(
                pick_cmd, timeout_sec=cfg['moveit']['status_timeout_sec']
            )
            if not status.get('success', False):
                self._log(f'P2: MoveIt pick failed: {status.get("error_message")}')
                continue

            # Grip 검증 via YOLO
            time.sleep(0.5)
            verify = self._yolo.wait_for_detection(
                class_name=target,
                confidence_threshold=cfg['scan']['confidence_threshold'],
                timeout_sec=grip_cfg['verify_window_sec'],
            )
            if verify is None:
                self._log('P2: grip verify — object not detected near gripper')
                continue

            z_cam = verify['position_3d_camera_frame']['z']
            if z_cam <= grip_cfg['distance_threshold_m']:
                self._log(f'P2: grip confirmed (z_cam={z_cam:.3f}m)')
                return True
            self._log(f'P2: grip verify — object too far (z_cam={z_cam:.3f}m)')

        return False

    # ------------------------------------------------------------------
    # P3 — Pickup verification
    # ------------------------------------------------------------------

    def _run_p3(self, target: str, cfg: dict) -> bool:
        self._pm.transition('P3', reason='pick complete')
        self._pub_phase()

        # Lift 10 cm
        lift_status = self._moveit.send_and_wait(
            {'cmd': 'lift', 'direction': 'z+', 'distance_m': 0.1, 'frame': 'world'},
            timeout_sec=cfg['moveit']['status_timeout_sec'],
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
        )
        success = result.get('pickup_success', False)
        self._log(f'P3: pickup_success={success} reason={result.get("reason")}')
        return success

    # ------------------------------------------------------------------
    # P4 — Place and finish
    # ------------------------------------------------------------------

    def _run_p4(self, cfg: dict) -> None:
        self._pm.transition('P4', reason='pickup verified')
        self._pub_phase()

        target_pose = cfg['target_pose']
        object_name = 'target_object'
        moveit_cfg  = cfg['moveit']

        place_status = self._moveit.send_and_wait(
            {
                'cmd': 'place',
                'object_name': object_name,
                'target_pose_world': [
                    target_pose['x'], target_pose['y'], target_pose['z'],
                    target_pose['roll'], target_pose['pitch'], target_pose['yaw'],
                ],
                'place_surface_offset': 0.001,
            },
            timeout_sec=cfg['moveit']['status_timeout_sec'],
        )
        self._log(f'P4: place status={place_status.get("success")}')

        self._moveit.send({'cmd': 'release', 'hand_open_pose': moveit_cfg['hand_open_pose']})
        time.sleep(1.0)
        self._moveit.send({'cmd': 'home', 'arm_home_pose': moveit_cfg['arm_home_pose']})

    # ------------------------------------------------------------------
    # LLM calls
    # ------------------------------------------------------------------

    def _parse_command(self, raw: str) -> str | None:
        result = self._llm.call(
            system_prompt=self._pm.system_prompt('parse_command'),
            user_content=f'명령: {raw}',
        )
        return result.get('target_class_name')

    def _generate_pick_params(
        self, target: str, wx: float, wy: float, wz: float, moveit_cfg: dict
    ) -> dict:
        user_content = (
            f'target_class_name: {target}\n'
            f'pose_world: [{wx:.4f}, {wy:.4f}, {wz:.4f}, 0.0, 0.0, 0.0]\n'
            f'moveit_config: {json.dumps(moveit_cfg, ensure_ascii=False)}\n'
            'object.dimensions (cylinder): [0.12, 0.025]  # [height, radius]\n'
            'object.pose_world.z 보정: wz + height/2 적용할 것'
        )
        result = self._llm.call(
            system_prompt=self._pm.system_prompt('generate_pick_params'),
            user_content=user_content,
        )
        # LLM이 PICK 명령 전체를 JSON으로 반환
        result.setdefault('cmd', 'pick')
        return result

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

    def _load_config(self) -> dict:
        import yaml
        paths = {
            'config':    self.get_parameter('config_path').value,
            'waypoints': self.get_parameter('scan_waypoints_path').value,
            'targets':   self.get_parameter('targets_path').value,
        }
        with open(paths['config']) as f:
            cfg = yaml.safe_load(f)
        with open(paths['waypoints']) as f:
            cfg['scan_waypoints'] = yaml.safe_load(f).get('waypoints', [])
        with open(paths['targets']) as f:
            cfg['target_pose'] = yaml.safe_load(f).get('place_target', {})
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
