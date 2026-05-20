# rack_transport_action v3.403 2026-01-25
# [이번 버전에서 수정된 사항]
# - (기능추가) /robot_action(ActionServer: RobotMove)에 cancel_callback 추가 + cancel 시 즉시 stop 시도
# - (기능추가) RackTransportAction에도 "취소/긴급정지 플래그" 도입: 실행 시작/중간에 즉시 중단 처리
# - (개선) EMERGENCY/CANCEL 시: hard stop 시도 → (가능하면) home 복귀 시도 → 실패 시 경고 로그
# - (유지) v3.402의 /bio_emergency 구독, TubeTransportNode cancel 즉시 stop, posj(home), vel/acc 400 통일, 기존 시퀀스 유지
#
# [중요]
# - 기존 코드 변경은 전부 "주석처리" 후, 바로 아래에 대체 코드를 추가했습니다.
# - 변경 구간은 [MOD v3.403] START/END로 감쌌습니다.

from __future__ import annotations

import re
from typing import Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from std_msgs.msg import Bool
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from bio_transport_interfaces.action import TubeTransport

import DR_init

try:
    from bio_transport_interfaces.action import RobotMove
except ImportError:
    # 타입/구문 체크용 더미
    class RobotMove:  # pragma: no cover
        class Goal:
            command = ""
        class Result:
            def __init__(self, success=False, message=""):
                self.success = success
                self.message = message
        class Feedback:
            status = ""


# ==========================================================
# ROBOT 상수 (사용자 규칙: 파람/상수 정의 바로 뒤 DR_init 세팅 1회)
# ==========================================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


# ==========================================================
# 기본 모션 파라미터
# ==========================================================
VELOCITY = 400
ACC = 400

# [Dispose(WASTE) 전용 파라미터]
VELOCITY_DISPOSE = 300
ACC_DISPOSE = 400
DISPOSE_J5_ROTATE_DEG = -70.0
DISPOSE_J2_ROTATE_DEG = 15.0
DISPOSE_OPEN_WAIT_SEC = 1.0

# [Pick 관련 파라미터]
DEFAULT_PICK_PRE_TOOL_MM = 18.0   # 잡기 전 Tool Z축 상승
DEFAULT_PICK_POST_BASE_MM = 30.0  # 잡은 후 Base Z축 상승

# ===== [MOD v3.401] START =====
# (원본) V_J=400, A_J=200 이라서 movej(home)에서 acc가 200으로만 적용되었습니다.
# V_J = 400
# A_J = 200
#
# (수정) "vel/acc 400 통일" 요구에 맞춰 Joint acc도 400으로 맞춥니다.
V_J = 400
A_J = 400
# ===== [MOD v3.401] END =====

V_L = 400.0
A_L = 400.0

V_L_SLOW = 300
A_L_SLOW = 250.0

# 홈(조인트)
HOME_J_DEG = (0.0, 0.0, 90.0, 0.0, 90.0, 0.0)

# MOVE(Transport) 관련 상수
MOVE_PICK_APP_DY = -100.0   # 1. 픽 접근 시 Y -100mm
MOVE_RETRACT_DY = -300.0    # 2. 픽 후퇴 시 Y -300mm
MOVE_PLACE_APP_DZ = 60.0

# IN/OUT 기본값
IN_WB_APP_DY = -50.0
IN_RACK_APP_DY = -100.0
IN_BASE_LIFT_Z = 250.0

OUT_RACK_APP_DY = -100.0
OUT_WB_APP_DZ = 200.0
OUT_WB_POST_X_MM = 200.0

GRIP_WAIT_SEC = 1.0

# =========================
# QoS (latched)
# =========================
qos_latched = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


# ===== [MOD v3.402] START =====
def _hard_stop_motion(node: Node, dr, reason: str) -> bool:
    """
    EMERGENCY/CANCEL 즉시 모션 정지 시도.
    - 환경별 stop/pause 함수명이 다를 수 있어 hasattr로 방어적으로 호출합니다.
    """
    try:
        node.get_logger().error(f"[EMG-STOP] {reason}")

        # stop 계열 우선
        if hasattr(dr, "stop"):
            if hasattr(dr, "DR_SSTOP"):
                dr.stop(dr.DR_SSTOP)
            elif hasattr(dr, "DR_QSTOP"):
                dr.stop(dr.DR_QSTOP)
            else:
                dr.stop()
            return True

        # pause 계열
        if hasattr(dr, "pause"):
            dr.pause()
            return True

        node.get_logger().warn("[EMG-STOP] no stop/pause API found in dr")
        return False

    except Exception as e:
        node.get_logger().warn(f"[EMG-STOP] failed: {repr(e)}")
        return False
# ===== [MOD v3.402] END =====


# ===== [MOD v3.403] START =====
def _try_home_recover(node: Node, dr, reason: str) -> bool:
    """
    stop 직후 home 복귀를 '시도'합니다.
    - 안전정지 상태면 movej 자체가 실패할 수 있으므로 예외는 삼키고 False 반환합니다.
    """
    try:
        node.get_logger().warn(f"[RECOVER] try HOME after {reason}")
        home_j = dr.posj(*HOME_J_DEG)
        dr.movej(home_j, vel=float(V_J), acc=float(A_J))
        node.get_logger().info("[RECOVER] HOME reached")
        return True
    except Exception as e:
        node.get_logger().error(f"[RECOVER] HOME failed: {repr(e)}")
        return False


def _request_stop_and_recover(node: Node, dr, reason: str) -> Tuple[bool, bool]:
    """
    공통 복구 흐름:
    1) hard stop 시도
    2) 가능하면 home 복귀 시도
    """
    stopped = _hard_stop_motion(node, dr, reason)
    recovered = False
    if stopped:
        recovered = _try_home_recover(node, dr, reason)
    return stopped, recovered
# ===== [MOD v3.403] END =====


def _norm(s: Optional[str]) -> Optional[str]:
    if s is None:
        return None
    t = str(s).strip()
    if t == "" or t.upper() == "NONE":
        return None
    return t


def _normalize_rack_key(raw: Optional[str]) -> str:
    """A-1 / a_1 / A1 같은 입력을 A-1로 정규화"""
    if raw is None:
        return ""
    s = str(raw).strip().upper()
    s = s.replace("_", "-")
    s = re.sub(r"\s+", "", s)

    m = re.match(r"^([A-Z])\-([0-9]+)$", s)
    if m:
        return f"{m.group(1)}-{m.group(2)}"
    m = re.match(r"^([A-Z])([0-9]+)$", s)
    if m:
        return f"{m.group(1)}-{m.group(2)}"
    return s


def _apply_offset(dr, pose: Sequence[float], dx=0.0, dy=0.0, dz=0.0):
    """posx(x,y,z,rx,ry,rz) 형태에 오프셋 적용"""
    return dr.posx(
        float(pose[0]) + float(dx),
        float(pose[1]) + float(dy),
        float(pose[2]) + float(dz),
        float(pose[3]),
        float(pose[4]),
        float(pose[5]),
    )


def _import_dsr():
    """
    DR_init.__dsr__node 주입 이후에만 호출되어야 합니다.
    - movel/posx는 dr.movel/dr.posx로만 사용
    """
    import DSR_ROBOT2 as dr
    from DSR_ROBOT2 import (
        set_tool, set_tcp, set_robot_mode, ROBOT_MODE_AUTONOMOUS,
        set_ref_coord,
    )
    DR_BASE = getattr(dr, "DR_BASE", None)
    return {
        "dr": dr,
        "set_tool": set_tool,
        "set_tcp": set_tcp,
        "set_robot_mode": set_robot_mode,
        "ROBOT_MODE_AUTONOMOUS": ROBOT_MODE_AUTONOMOUS,
        "set_ref_coord": set_ref_coord,
        "DR_BASE": DR_BASE,
    }


def initialize_robot(node: Node):
    """사용자 규칙: main()에서 노드 생성 후 1회만 호출."""
    # ✅ DSR_ROBOT2 import 위치는 여기(함수 내부)로 고정
    import DSR_ROBOT2 as dr

    node.get_logger().info("#" * 50)
    node.get_logger().info("Initializing robot with the following settings:")
    node.get_logger().info(f"ROBOT_ID: {ROBOT_ID}")
    node.get_logger().info(f"ROBOT_MODEL: {ROBOT_MODEL}")
    node.get_logger().info(f"ROBOT_TCP: {ROBOT_TCP}")
    node.get_logger().info(f"ROBOT_TOOL: {ROBOT_TOOL}")
    node.get_logger().info("#" * 50)

    # (필요 시) 모드 설정
    try:
        dr.set_robot_mode(dr.ROBOT_MODE_AUTONOMOUS)
    except Exception:
        pass

    # tool/tcp 1회 설정
    dr.set_tool(ROBOT_TOOL)
    dr.set_tcp(ROBOT_TCP)

    return dr


class RackTransportAction(Node):
    def __init__(self):
        super().__init__("rack_transport_action", namespace=ROBOT_ID)

        # main()에서 주입
        self.dr = None

        # dry_run=True면 로봇 이동 없이 성공만 반환
        self.declare_parameter("dry_run", False)

        # station builders / targets
        from .rack_stations import (
            build_rack_stations,
            build_workbench_station_dy,
            build_workbench_station_top,
            RACK_TARGETS,
        )
        self.RACK_TARGETS = RACK_TARGETS
        self.build_rack_stations = build_rack_stations
        self.build_wb_dy = build_workbench_station_dy
        self.build_wb_top = build_workbench_station_top

        # IO helpers
        from .gripper_io import grip_open, grip_close, grip_init_open
        self.grip_open = grip_open
        self.grip_close = grip_close
        self.grip_init_open = grip_init_open

        # rel move helpers (IN/OUT에 사용)
        from .rel_move import rel_movel_tool, rel_movel_base
        self.rel_movel_tool = rel_movel_tool
        self.rel_movel_base = rel_movel_base

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ===== [MOD v3.403] START =====
        # 내부 플래그: EMERGENCY / CANCEL 요청을 실행 흐름에서 즉시 감지하기 위함입니다.
        self._emergency = False
        self._cancel_requested = False
        # ===== [MOD v3.403] END =====

        # ✅ ACTION 이름(/robot_action)은 절대 변경 금지
        # ===== [MOD v3.403] START =====
        # (원본 v3.402) cancel_callback이 없어서, UI/오케스트라 cancel이 와도 stop을 못 걸 수 있습니다.
        # self._server = ActionServer(
        #     self,
        #     RobotMove,
        #     "/robot_action",
        #     execute_callback=self.execute_callback,
        #     callback_group=ReentrantCallbackGroup(),
        #     goal_service_qos_profile=qos,
        #     result_service_qos_profile=qos,
        #     cancel_service_qos_profile=qos,
        #     feedback_pub_qos_profile=qos,
        #     status_pub_qos_profile=qos,
        # )
        #
        # (수정 v3.403) cancel_callback 추가 + cancel 시 즉시 stop/recover 시도합니다.
        self._server = ActionServer(
            self,
            RobotMove,
            "/robot_action",
            execute_callback=self.execute_callback,
            cancel_callback=self._on_cancel_robot_action,
            callback_group=ReentrantCallbackGroup(),
            goal_service_qos_profile=qos,
            result_service_qos_profile=qos,
            cancel_service_qos_profile=qos,
            feedback_pub_qos_profile=qos,
            status_pub_qos_profile=qos,
        )
        # ===== [MOD v3.403] END =====

        # ===== [MOD v3.402] START =====
        # EMERGENCY 토픽 구독(랙/튜브 공통)
        # (v3.403에서는 _emergency 플래그는 이미 위에서 선언했습니다)
        self._sub_emg = self.create_subscription(
            Bool, "/bio_emergency", self._on_emergency, 10
        )
        # ===== [MOD v3.402] END =====

        self.get_logger().info("✅ [v3.403] /robot_action ready(cancel+emg) + /bio_emergency subscribed")

    def set_dr(self, dr):
        self.dr = dr

    def _home(self):
        home_j = self.dr.posj(*HOME_J_DEG)
        self.dr.movej(home_j, vel=V_J, acc=A_J)

    def _valid_keys(self):
        keys = list(self.RACK_TARGETS.keys())
        keys.sort()
        return keys

    # ===== [MOD v3.403] START =====
    def _abort_check(self, where: str) -> bool:
        """
        Rack 시퀀스 중간중간 호출해서 즉시 중단 여부를 판단합니다.
        - 블로킹 모션 "중간"을 쪼개지는 못하지만, 다음 스텝 진입을 막습니다.
        """
        if self._emergency:
            self.get_logger().error(f"[ABORT] EMERGENCY active at {where}")
            return True
        if self._cancel_requested:
            self.get_logger().warn(f"[ABORT] CANCEL active at {where}")
            return True
        return False

    def _on_cancel_robot_action(self, goal_handle):
        """
        /robot_action 취소 콜백:
        - 즉시 stop 시도(다른 콜백 스레드에서 동작)
        - 가능하면 home 복귀 시도
        """
        self.get_logger().warn("[CANCEL] /robot_action cancel requested")
        self._cancel_requested = True

        if self.dr is not None:
            stopped, recovered = _request_stop_and_recover(self, self.dr, "RACK_ACTION_CANCEL")
            if not stopped:
                self.get_logger().error("[CANCEL] stop failed (no API or exception)")
            if stopped and not recovered:
                self.get_logger().error("[CANCEL] stop OK but HOME recover failed (maybe safety stop)")
        return CancelResponse.ACCEPT
    # ===== [MOD v3.403] END =====

    # ===== [MOD v3.402] START =====
    def _on_emergency(self, msg: Bool):
        if not bool(msg.data):
            return
        self._emergency = True
        self.get_logger().error("[EMG] received in RackTransportAction")
        if self.dr is not None:
            # ===== [MOD v3.403] START =====
            # (원본 v3.402) stop만 시도했습니다.
            # _hard_stop_motion(self, self.dr, "RACK_ACTION_EMERGENCY")
            #
            # (수정 v3.403) stop → 가능하면 home 복귀까지 시도합니다.
            stopped, recovered = _request_stop_and_recover(self, self.dr, "RACK_ACTION_EMERGENCY")
            if not stopped:
                self.get_logger().error("[EMG] stop failed (no API or exception)")
            if stopped and not recovered:
                self.get_logger().error("[EMG] stop OK but HOME recover failed (maybe safety stop)")
            # ===== [MOD v3.403] END =====
    # ===== [MOD v3.402] END =====

    async def execute_callback(self, goal_handle):
        cmd = (goal_handle.request.command or "").strip()
        self.get_logger().info(f"📥 EXEC: {cmd}")

        # ===== [MOD v3.403] START =====
        # 새 goal이 들어오면 cancel 플래그는 리셋합니다(긴급정지는 리셋하지 않습니다).
        self._cancel_requested = False
        # ===== [MOD v3.403] END =====

        # ===== [MOD v3.402] START =====
        # EMERGENCY가 이미 활성화되어 있으면 즉시 거절합니다.
        if self._emergency:
            goal_handle.abort()
            return RobotMove.Result(success=False, message="EMERGENCY ACTIVE")
        # ===== [MOD v3.402] END =====

        # ==========================================================
        # [핵심 수정 1] 매 요청마다 로봇 모드를 강제로 리셋 (2번째 동작 멈춤 해결)
        # ==========================================================
        try:
            if self.dr:
                self.dr.set_robot_mode(self.dr.ROBOT_MODE_AUTONOMOUS)
        except Exception as e:
            self.get_logger().warn(f"Mode set warning: {e}")
        # ==========================================================

        if bool(self.get_parameter("dry_run").value):
            goal_handle.succeed()
            return RobotMove.Result(success=True, message="Dry Run Success")

        if self.dr is None:
            goal_handle.abort()
            return RobotMove.Result(success=False, message="Robot not initialized (dr is None)")

        parts = [p.strip() for p in cmd.split(",") if p.strip() != ""]
        op = parts[0].upper() if parts else ""
        a1 = _norm(parts[1]) if len(parts) > 1 else None
        a2 = _norm(parts[2]) if len(parts) > 2 else None

        src = _normalize_rack_key(a1) if a1 else None
        dst = _normalize_rack_key(a2) if a2 else None

        try:
            # ===== [MOD v3.403] START =====
            if self._abort_check("EXEC_BEGIN"):
                raise RuntimeError("ABORTED (EMERGENCY/CANCEL)")
            # ===== [MOD v3.403] END =====

            if op == "MOVE":
                if not src or not dst:
                    raise ValueError("MOVE requires src & dest")
                ok, msg = self._do_transport(src, dst)

            elif op == "IN":
                if not dst:
                    raise ValueError("IN requires dest")
                ok, msg = self._do_inbound(dst)

            elif op == "OUT":
                if not src:
                    raise ValueError("OUT requires src")
                ok, msg = self._do_outbound(src)
            else:
                ok, msg = False, f"Unknown op: {op}"

        except Exception as e:
            ok, msg = False, f"Error: {e}"

        if ok:
            goal_handle.succeed()
            self.get_logger().info(f"📥 sub_end: {cmd}")
        else:
            goal_handle.abort()

        return RobotMove.Result(success=ok, message=msg)

    # ==========================================================
    # MOVE (Transport)
    # ==========================================================
    def _do_transport(self, src: str, dest: str) -> Tuple[bool, str]:
        valid = self._valid_keys()
        if src not in valid or dest not in valid:
            return False, f"Invalid keys: {src}->{dest}"
        if src == dest:
            return False, "src == dest"

        self.get_logger().info(f"[MOVE] {src} -> {dest}")

        rack_pick = self.build_rack_stations(self.dr, approach_dy=MOVE_PICK_APP_DY)
        rack_place = self.build_rack_stations(self.dr, approach_dy=MOVE_PICK_APP_DY)

        st_src = rack_pick[src]
        st_dst = rack_place[dest]

        # ===== [MOD v3.403] START =====
        if self._abort_check("MOVE:PRE_HOME"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====

        self._home()
        self.grip_init_open(self.dr, wait_sec=0.2)

        # PICK
        # ===== [MOD v3.403] START =====
        if self._abort_check("MOVE:PICK_APPROACH"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self.dr.movel(st_src["approach"], vel=V_L, acc=A_L)

        # ===== [MOD v3.403] START =====
        if self._abort_check("MOVE:PICK_TARGET"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self.dr.movel(st_src["target"], vel=V_L, acc=A_L)

        if DEFAULT_PICK_PRE_TOOL_MM > 0:
            # ===== [MOD v3.403] START =====
            if self._abort_check("MOVE:PICK_PRE_TOOL"):
                return False, "ABORTED (EMERGENCY/CANCEL)"
            # ===== [MOD v3.403] END =====
            self.rel_movel_tool(self.dr, 0, 0, DEFAULT_PICK_PRE_TOOL_MM, 0, 0, 0, V_L_SLOW)

        # ===== [MOD v3.403] START =====
        if self._abort_check("MOVE:GRIP_CLOSE"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self.grip_close(self.dr, wait_sec=GRIP_WAIT_SEC)

        if DEFAULT_PICK_POST_BASE_MM > 0:
            # ===== [MOD v3.403] START =====
            if self._abort_check("MOVE:PICK_POST_BASE"):
                return False, "ABORTED (EMERGENCY/CANCEL)"
            # ===== [MOD v3.403] END =====
            self.rel_movel_base(self.dr, 0, 0, DEFAULT_PICK_POST_BASE_MM, 0, 0, 0, V_L_SLOW)

        # ===== [MOD v3.403] START =====
        if self._abort_check("MOVE:RETRACT"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self.rel_movel_base(self.dr, 0, MOVE_RETRACT_DY, 0, 0, 0, 0, V_L)

        # LATERAL X MOVE
        current_pos = self.dr.get_current_posx()[0]
        target_x = st_dst["target"][0]

        lateral_pos = self.dr.posx(
            target_x,
            current_pos[1],
            current_pos[2],
            current_pos[3],
            current_pos[4],
            current_pos[5],
        )
        self.get_logger().info(f"[MOVE] Sliding X to {dest}...")
        # ===== [MOD v3.403] START =====
        if self._abort_check("MOVE:SLIDE_X"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self.dr.movel(lateral_pos, vel=V_L, acc=A_L)

        # PLACE (ID-specific Y offset)
        y_offset = 0.0
        if dest == "A-2":
            y_offset = 23.0
        elif dest in "A-3":
            y_offset = 19.0
        elif dest == "B-1":
            y_offset = 14.0
        elif dest == "B-2":
            y_offset = 12.0

        final_target = _apply_offset(self.dr, st_dst["target"], dy=y_offset)
        self.get_logger().info(f"[MOVE] Place logic: Dest={dest}, Added Y={y_offset}mm")

        place_app = _apply_offset(self.dr, final_target, dz=MOVE_PLACE_APP_DZ)
        # ===== [MOD v3.403] START =====
        if self._abort_check("MOVE:PLACE_APP"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self.dr.movel(place_app, vel=V_L, acc=A_L)

        # ===== [MOD v3.403] START =====
        if self._abort_check("MOVE:PLACE_TARGET"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self.dr.movel(final_target, vel=V_L, acc=A_L)

        # ===== [MOD v3.403] START =====
        if self._abort_check("MOVE:GRIP_OPEN"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self.grip_open(self.dr, wait_sec=GRIP_WAIT_SEC)

        # ===== [MOD v3.403] START =====
        if self._abort_check("MOVE:PLACE_RETRACT"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self.rel_movel_base(self.dr, 0, MOVE_RETRACT_DY, 0, 0, 0, 0, V_L)

        # ===== [MOD v3.403] START =====
        if self._abort_check("MOVE:POST_HOME"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self._home()
        return True, "Transport Done"

    # ==========================================================
    # INBOUND : WORKBENCH -> RACK
    # ==========================================================
    def _do_inbound(self, dest: str) -> Tuple[bool, str]:
        valid = self._valid_keys()
        if dest not in valid:
            return False, f"Invalid rack key: {dest}"

        rack_place = self.build_rack_stations(self.dr, approach_dy=MOVE_PICK_APP_DY)
        st_dst = rack_place[dest]

        y_offset = 0.0
        if dest == "A-2":
            y_offset = 15.0
        elif dest in "A-3":
            y_offset = 13.0
        elif dest in "B-1":
            y_offset = 9.0
        elif dest == "B-2":
            y_offset = 10.0

        self.get_logger().info(f"[IN] WB -> {dest} (Offset: {y_offset}mm)")

        wb = self.build_wb_dy(self.dr, approach_dy=IN_WB_APP_DY)
        rack = self.build_rack_stations(self.dr, approach_dy=IN_RACK_APP_DY)
        st = rack[dest]

        # ===== [MOD v3.403] START =====
        if self._abort_check("IN:PRE_HOME"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====

        self._home()
        self.grip_init_open(self.dr, wait_sec=0.2)

        # WB Pick Sequence (기존 유지)
        # ===== [MOD v3.403] START =====
        if self._abort_check("IN:WB_BACK_1"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self.rel_movel_base(self.dr, 0, -180.0, 0, 0, 0, 0, 400.0)

        cur_pos, _ = self.dr.get_current_posx(self.dr.DR_BASE)
        x, y, z, rx, ry, rz = [float(v) for v in cur_pos]
        back_mm = 150
        target = self.dr.posx(x, y - back_mm, z, rx, ry, rz)
        # ===== [MOD v3.403] START =====
        if self._abort_check("IN:WB_BACK_2"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self.dr.movel(target, vel=V_L, acc=A_L)

        # ===== [MOD v3.403] START =====
        if self._abort_check("IN:WB_APPROACH"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self.dr.movel(wb["approach"], vel=V_L, acc=A_L)

        # ===== [MOD v3.403] START =====
        if self._abort_check("IN:WB_GRIP_CLOSE_1"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self.grip_close(self.dr, wait_sec=GRIP_WAIT_SEC)

        # ===== [MOD v3.403] START =====
        if self._abort_check("IN:WB_TOOL_UP_1"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self.rel_movel_tool(self.dr, 0, 0, 8.0, 0, 0, 0, 400.0)

        # ===== [MOD v3.403] START =====
        if self._abort_check("IN:WB_GRIP_OPEN_INIT"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self.grip_init_open(self.dr, wait_sec=0.2)

        # ===== [MOD v3.403] START =====
        if self._abort_check("IN:WB_TARGET"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self.dr.movel(wb["target"], vel=V_L, acc=A_L)

        # ===== [MOD v3.403] START =====
        if self._abort_check("IN:WB_GRIP_CLOSE_2"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self.grip_close(self.dr, wait_sec=GRIP_WAIT_SEC)

        # ===== [MOD v3.403] START =====
        if self._abort_check("IN:LIFT_BASE"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self.rel_movel_base(self.dr, 0, 0, IN_BASE_LIFT_Z, 0, 0, 0, 400.0)

        # Rack Approach Sequence (X -> Z -> Y)
        final_target = _apply_offset(self.dr, st_dst["target"], dy=y_offset)
        place_app = _apply_offset(self.dr, final_target, dz=MOVE_PLACE_APP_DZ)

        self.get_logger().info("[MOVE] Rack Approach: X(Align) -> Z(Lower) -> Y(Enter)")

        cur_pos_lifted = self.dr.get_current_posx()[0]

        waypoint_x = self.dr.posx(
            place_app[0],
            cur_pos_lifted[1],
            cur_pos_lifted[2],
            cur_pos_lifted[3], cur_pos_lifted[4], cur_pos_lifted[5]
        )
        self.get_logger().info("[INBOUND] Step 1: X Align")
        # ===== [MOD v3.403] START =====
        if self._abort_check("IN:ALIGN_X"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self.dr.movel(waypoint_x, vel=V_L, acc=A_L)

        waypoint_z = self.dr.posx(
            place_app[0],
            cur_pos_lifted[1],
            place_app[2],
            cur_pos_lifted[3], cur_pos_lifted[4], cur_pos_lifted[5]
        )
        self.get_logger().info("[INBOUND] Step 2: Z Lower (Pre-Leveling)")
        # ===== [MOD v3.403] START =====
        if self._abort_check("IN:ALIGN_Z"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self.dr.movel(waypoint_z, vel=V_L, acc=A_L)

        self.get_logger().info("[INBOUND] Step 3: Y Enter (Approach)")
        # ===== [MOD v3.403] START =====
        if self._abort_check("IN:ENTER_Y"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self.dr.movel(place_app, vel=V_L, acc=A_L)

        self.get_logger().info("[INBOUND] Step 4: Final Inbound")
        # ===== [MOD v3.403] START =====
        if self._abort_check("IN:FINAL"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self.dr.movel(final_target, vel=V_L, acc=A_L)

        # ===== [MOD v3.403] START =====
        if self._abort_check("IN:GRIP_OPEN"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self.grip_open(self.dr, wait_sec=GRIP_WAIT_SEC)

        ret = _apply_offset(self.dr, st["target"], dy=-150.0)
        # ===== [MOD v3.403] START =====
        if self._abort_check("IN:RETRACT"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self.dr.movel(ret, vel=V_L_SLOW, acc=A_L_SLOW)

        # ===== [MOD v3.403] START =====
        if self._abort_check("IN:POST_HOME"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self._home()
        return True, "Inbound Done"

    # ==========================================================
    # OUTBOUND : RACK -> WORKBENCH
    # ==========================================================
    def _do_outbound(self, src: str) -> Tuple[bool, str]:
        valid = self._valid_keys()
        if src not in valid:
            return False, f"Invalid rack key: {src}"

        self.get_logger().info(f"[OUT] {src} -> WB")

        rack = self.build_rack_stations(self.dr, approach_dy=OUT_RACK_APP_DY)
        wb = self.build_wb_top(self.dr, approach_dz=OUT_WB_APP_DZ)
        st = rack[src]

        # ===== [MOD v3.403] START =====
        if self._abort_check("OUT:PRE_HOME"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====

        self._home()
        self.grip_init_open(self.dr, wait_sec=0.2)

        # ===== [MOD v3.403] START =====
        if self._abort_check("OUT:BASE_BACK"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self.rel_movel_base(self.dr, 0, -100.0, 0, 0, 0, 0, 300.0)

        # ===== [MOD v3.403] START =====
        if self._abort_check("OUT:RACK_APPROACH"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self.dr.movel(st["approach"], vel=V_L, acc=A_L)

        # ===== [MOD v3.403] START =====
        if self._abort_check("OUT:RACK_TARGET"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self.dr.movel(st["target"], vel=V_L, acc=A_L)

        if DEFAULT_PICK_PRE_TOOL_MM > 0:
            # ===== [MOD v3.403] START =====
            if self._abort_check("OUT:PRE_TOOL"):
                return False, "ABORTED (EMERGENCY/CANCEL)"
            # ===== [MOD v3.403] END =====
            self.rel_movel_tool(self.dr, 0, 0, DEFAULT_PICK_PRE_TOOL_MM, 0, 0, 0, 300.0)

        # ===== [MOD v3.403] START =====
        if self._abort_check("OUT:GRIP_CLOSE"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self.grip_close(self.dr, wait_sec=GRIP_WAIT_SEC)

        if DEFAULT_PICK_POST_BASE_MM > 0:
            # ===== [MOD v3.403] START =====
            if self._abort_check("OUT:POST_BASE"):
                return False, "ABORTED (EMERGENCY/CANCEL)"
            # ===== [MOD v3.403] END =====
            self.rel_movel_base(self.dr, 0, 0, DEFAULT_PICK_POST_BASE_MM, 0, 0, 0, 300.0)

        # ===== [MOD v3.403] START =====
        if self._abort_check("OUT:RETRACT_Y"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self.rel_movel_base(self.dr, 0, -250.0, 0, 0, 0, 0, 300.0)

        # ===== [MOD v3.403] START =====
        if self._abort_check("OUT:WB_APPROACH"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self.dr.movel(wb["approach"], vel=V_L, acc=A_L)

        # ===== [MOD v3.403] START =====
        if self._abort_check("OUT:WB_TARGET"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self.dr.movel(wb["target"], vel=V_L, acc=A_L)

        # ===== [MOD v3.403] START =====
        if self._abort_check("OUT:GRIP_OPEN"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self.grip_open(self.dr, wait_sec=GRIP_WAIT_SEC)

        # ===== [MOD v3.403] START =====
        if self._abort_check("OUT:WB_RETREAT_1"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self.rel_movel_base(self.dr, 0, -50.0, 0, 0, 0, 0, 300.0)

        if OUT_WB_POST_X_MM > 0:
            # ===== [MOD v3.403] START =====
            if self._abort_check("OUT:POST_X"):
                return False, "ABORTED (EMERGENCY/CANCEL)"
            # ===== [MOD v3.403] END =====
            self.rel_movel_base(self.dr, OUT_WB_POST_X_MM, 0, 0, 0, 0, 0, 300.0)

        # ===== [MOD v3.403] START =====
        if self._abort_check("OUT:WB_LIFT"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self.rel_movel_base(self.dr, 0, 0, 50.0, 0, 0, 0, 300.0)

        # ===== [MOD v3.403] START =====
        if self._abort_check("OUT:POST_HOME"):
            return False, "ABORTED (EMERGENCY/CANCEL)"
        # ===== [MOD v3.403] END =====
        self._home()
        return True, "Outbound Done"


def _set_ref_base(dsr, node: Node):
    """
    기준 좌표계 BASE로 고정(상대이동/안전성 위해).
    """
    try:
        if dsr["DR_BASE"] is not None:
            dsr["set_ref_coord"](dsr["DR_BASE"])
            node.get_logger().info("set_ref_coord: DR_BASE")
        else:
            node.get_logger().info("set_ref_coord: DR_BASE not found (skip)")
    except Exception as e:
        node.get_logger().warn(f"set_ref_coord failed: {repr(e)}")


def _posx_from_list(dr, arr6):
    return dr.posx(
        float(arr6[0]), float(arr6[1]), float(arr6[2]),
        float(arr6[3]), float(arr6[4]), float(arr6[5]),
    )


class TubeTransportNode(Node):
    def __init__(self):
        super().__init__("tube_transport_node", namespace=ROBOT_ID)

        self.pub_done = self.create_publisher(Bool, "tube_transport_done", qos_latched)

        # ===== [MOD v3.402] START =====
        # EMERGENCY 토픽 구독(랙/튜브 공통)
        self._emergency = False
        self._sub_emg = self.create_subscription(
            Bool, "/bio_emergency", self._on_emergency, 10
        )
        # ===== [MOD v3.402] END =====

        self._as = ActionServer(
            self,
            TubeTransport,
            "/tube_transport",
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            execute_callback=self._on_execute,
        )

        self.get_logger().info("TubeTransportNode ready (ActionServer: /tube_transport)")

    # ===== [MOD v3.402] START =====
    def _on_emergency(self, msg: Bool):
        if not bool(msg.data):
            return
        self._emergency = True
        self.get_logger().error("[EMG] received in TubeTransportNode")
        try:
            dsr = _import_dsr()
            dr = dsr["dr"]
            # ===== [MOD v3.403] START =====
            # (원본 v3.402) stop만 시도했습니다.
            # _hard_stop_motion(self, dr, "TUBE_ACTION_EMERGENCY")
            #
            # (수정 v3.403) stop → 가능하면 home 복귀까지 시도합니다.
            stopped, recovered = _request_stop_and_recover(self, dr, "TUBE_ACTION_EMERGENCY")
            if not stopped:
                self.get_logger().error("[EMG] stop failed (no API or exception)")
            if stopped and not recovered:
                self.get_logger().error("[EMG] stop OK but HOME recover failed (maybe safety stop)")
            # ===== [MOD v3.403] END =====
        except Exception as e:
            self.get_logger().warn(f"[EMG] stop attempt failed: {repr(e)}")
    # ===== [MOD v3.402] END =====

    def initialize_robot(self):
        dsr = _import_dsr()

        self.get_logger().info("[INIT] set_tool")
        dsr["set_tool"](ROBOT_TOOL)

        self.get_logger().info("[INIT] set_tcp")
        dsr["set_tcp"](ROBOT_TCP)

        self.get_logger().info("[INIT] set_robot_mode")
        dsr["set_robot_mode"](dsr["ROBOT_MODE_AUTONOMOUS"])

        _set_ref_base(dsr, self)

        self.get_logger().info("#" * 50)
        self.get_logger().info("Robot initialized")
        self.get_logger().info(f"ROBOT_ID={ROBOT_ID}, MODEL={ROBOT_MODEL}, TCP={ROBOT_TCP}, TOOL={ROBOT_TOOL}")
        self.get_logger().info(f"VELOCITY={VELOCITY}, ACC={ACC}")
        self.get_logger().info("#" * 50)

    def _on_goal(self, goal_request: TubeTransport.Goal):
        if not hasattr(goal_request, "job_id"):
            return GoalResponse.REJECT
        if len(goal_request.pick_posx) != 6 or len(goal_request.place_posx) != 6:
            self.get_logger().error("Rejected goal: pick_posx/place_posx must be length 6")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        # ===== [MOD v3.402] START =====
        # cancel 들어오는 즉시 stop 시도
        self.get_logger().warn("Cancel requested. -> try hard stop now")
        try:
            dsr = _import_dsr()
            dr = dsr["dr"]
            # ===== [MOD v3.403] START =====
            # (원본 v3.402) stop만 시도했습니다.
            # _hard_stop_motion(self, dr, "TUBE_ACTION_CANCEL")
            #
            # (수정 v3.403) stop → 가능하면 home 복귀까지 시도합니다.
            stopped, recovered = _request_stop_and_recover(self, dr, "TUBE_ACTION_CANCEL")
            if not stopped:
                self.get_logger().error("[CANCEL] stop failed (no API or exception)")
            if stopped and not recovered:
                self.get_logger().error("[CANCEL] stop OK but HOME recover failed (maybe safety stop)")
            # ===== [MOD v3.403] END =====
        except Exception as e:
            self.get_logger().warn(f"Cancel stop attempt failed: {repr(e)}")
        # ===== [MOD v3.402] END =====
        return CancelResponse.ACCEPT

    def _fb(self, goal_handle, stage: str, progress: float, detail: str):
        fb = TubeTransport.Feedback()
        fb.stage = stage
        fb.progress = float(progress)
        fb.detail = detail
        goal_handle.publish_feedback(fb)

    def _cancel_check(self, goal_handle, where: str) -> bool:
        # ===== [MOD v3.402] START =====
        if self._emergency:
            self.get_logger().error(f"[EMG] active at {where}")
            return True
        # ===== [MOD v3.402] END =====
        if goal_handle.is_cancel_requested:
            self.get_logger().warn(f"[CANCEL] requested at {where}")
            return True
        return False

    def _ret_ok(self, ret, where: str) -> bool:
        if ret is None:
            return True
        try:
            r = float(ret)
            if r < 0:
                self.get_logger().error(f"[MOTION] {where} rejected ret={ret}")
                return False
            return True
        except Exception:
            self.get_logger().error(f"[MOTION] {where} ret(not-numeric)={ret}")
            return False

    def _movel_abs(self, dr, target_posx6, where: str):
        pos = _posx_from_list(dr, target_posx6)
        kwargs = {"vel": float(VELOCITY), "acc": float(ACC)}

        if hasattr(dr, "DR_BASE"):
            kwargs["ref"] = dr.DR_BASE
        if hasattr(dr, "DR_MV_MOD_ABS"):
            kwargs["mod"] = dr.DR_MV_MOD_ABS

        self.get_logger().info(f"[MOTION] {where} movel -> {target_posx6} kwargs={kwargs}")
        ret = dr.movel(pos, **kwargs)
        self.get_logger().info(f"[MOTION] {where} movel done ret={ret}")
        return ret

    def _on_execute(self, goal_handle):
        goal = goal_handle.request

        result = TubeTransport.Result()
        result.success = False
        result.error_code = ""
        result.message = ""

        job_id = goal.job_id
        job_u = str(job_id).upper().strip()
        is_waste = any(k in job_u for k in ("WASTE", "DISPOSE"))
        pick_posx_6 = list(goal.pick_posx)
        place_posx_6 = list(goal.place_posx)

        dsr = _import_dsr()
        dr = dsr["dr"]

        from .gripper_io import grip_open, grip_close
        from .rel_move import rel_movel_base

        PICK_DOWN_MM = 50.0
        PICK_UP_MM = 132.0

        PLACE_DOWN_MM = 110.0
        PLACE_OPEN_WAIT = 1.2
        PLACE_UP_MM = 90.0

        try:
            self.get_logger().info(f"[EXEC] start job_id={job_id}")
            self._fb(goal_handle, "INIT", 0.01, f"Job start job_id={job_id}")

            # ===== [MOD v3.402] START =====
            if self._emergency:
                result.error_code = "EMERGENCY"
                result.message = "Emergency active before execute"
                self._fb(goal_handle, "FAIL", 1.0, result.message)
                self.pub_done.publish(Bool(data=False))
                goal_handle.abort()
                return result
            # ===== [MOD v3.402] END =====

            self._fb(goal_handle, "PICK_MOVE", 0.10, "Move to pick_posx")
            if self._cancel_check(goal_handle, "PICK_MOVE"):
                result.error_code = "CANCELED"
                result.message = "Canceled before pick move"
                goal_handle.abort()
                return result

            ret = self._movel_abs(dr, pick_posx_6, "PICK->pick_posx")
            if not self._ret_ok(ret, "PICK movel->pick_posx"):
                result.error_code = "PICK_MOVE_REJECTED"
                result.message = f"Pick movel rejected ret={ret}"
                self._fb(goal_handle, "FAIL", 1.0, result.message)
                self.pub_done.publish(Bool(data=False))
                goal_handle.abort()
                return result

            _set_ref_base(dsr, self)

            self._fb(goal_handle, "PICK_SEQ", 0.30, "OPEN -> down 50 -> CLOSE -> up 132")
            if self._cancel_check(goal_handle, "PICK_SEQ"):
                result.error_code = "CANCELED"
                result.message = "Canceled during pick seq"
                goal_handle.abort()
                return result

            self.get_logger().info("[GRIP] grip_open()")
            grip_open(dr)

            self.get_logger().info(f"[PICK] down {PICK_DOWN_MM}mm")
            rel_movel_base(dr, 0, 0, -PICK_DOWN_MM, 0, 0, 0, vel=VELOCITY)

            self.get_logger().info("[GRIP] grip_close()")
            grip_close(dr)

            self.get_logger().info(f"[PICK] up {PICK_UP_MM}mm")
            rel_movel_base(dr, 0, 0, +PICK_UP_MM, 0, 0, 0, vel=VELOCITY)

            self._fb(goal_handle, "PLACE_MOVE", 0.70, "Move to place_posx")
            if self._cancel_check(goal_handle, "PLACE_MOVE"):
                result.error_code = "CANCELED"
                result.message = "Canceled before place move"
                goal_handle.abort()
                return result

            ret = self._movel_abs(dr, place_posx_6, "PLACE->place_posx")
            if not self._ret_ok(ret, "PLACE movel->place_posx"):
                result.error_code = "PLACE_MOVE_REJECTED"
                result.message = f"Place movel rejected ret={ret}"
                self._fb(goal_handle, "FAIL", 1.0, result.message)
                self.pub_done.publish(Bool(data=False))
                goal_handle.abort()
                return result

            _set_ref_base(dsr, self)

            if is_waste:
                self._fb(goal_handle, "DISPOSE_SEQ", 0.85, "rotate(J5/J2) -> OPEN -> JReady")
                if self._cancel_check(goal_handle, "DISPOSE_SEQ"):
                    result.error_code = "CANCELED"
                    result.message = "Canceled during dispose seq"
                    goal_handle.abort()
                    return result

                # J5 rotate
                try:
                    cur_j = [float(v) for v in dr.get_current_posj()]
                    tgt = cur_j[:]
                    tgt[4] += float(DISPOSE_J5_ROTATE_DEG)
                    self.get_logger().info(f"[DISPOSE] movej J5 += {DISPOSE_J5_ROTATE_DEG}deg")
                    retj = dr.movej(tgt, vel=float(VELOCITY_DISPOSE), acc=float(ACC_DISPOSE))
                    if not self._ret_ok(retj, "DISPOSE movej(J5)"):
                        raise RuntimeError(f"DISPOSE movej(J5) rejected ret={retj}")
                except Exception as e:
                    self.get_logger().warn(f"[DISPOSE] J5 rotate skipped/failed: {repr(e)}")

                # J2 rotate
                try:
                    cur_j = [float(v) for v in dr.get_current_posj()]
                    tgt = cur_j[:]
                    tgt[1] += float(DISPOSE_J2_ROTATE_DEG)
                    self.get_logger().info(f"[DISPOSE] movej J2 += {DISPOSE_J2_ROTATE_DEG}deg")
                    retj = dr.movej(tgt, vel=float(VELOCITY_DISPOSE), acc=float(ACC_DISPOSE))
                    if not self._ret_ok(retj, "DISPOSE movej(J2)"):
                        raise RuntimeError(f"DISPOSE movej(J2) rejected ret={retj}")
                except Exception as e:
                    self.get_logger().warn(f"[DISPOSE] J2 rotate skipped/failed: {repr(e)}")

                self.get_logger().info(f"[DISPOSE] grip_open(wait={DISPOSE_OPEN_WAIT_SEC})")
                grip_open(dr, wait_sec=float(DISPOSE_OPEN_WAIT_SEC))
                try:
                    dr.wait(0.2)
                except Exception:
                    pass

                try:
                    self.get_logger().info("[DISPOSE] grip_close()")
                    grip_close(dr)
                    dr.wait(0.2)
                except Exception:
                    pass

                # ===== [MOD v3.401] START =====
                # (원본) 리스트를 그대로 movej에 넣음 → 일부 환경에서 vel/v 해석이 0으로 떨어져 에러가 나기 쉬움
                # try:
                #     self.get_logger().info("[DISPOSE] return HOME_J_DEG")
                #     dr.movej(list(HOME_J_DEG), vel=float(VELOCITY_DISPOSE), acc=float(ACC_DISPOSE))
                # except Exception as e:
                #     self.get_logger().warn(f"[DISPOSE] movej HOME_J_DEG failed: {repr(e)}")
                #
                # (수정) posj로 만들어서 넣으면 vel/acc가 안정적으로 적용됩니다.
                try:
                    self.get_logger().info("[DISPOSE] return HOME_J_DEG (posj)")
                    home_j = dr.posj(*HOME_J_DEG)
                    dr.movej(home_j, vel=float(VELOCITY_DISPOSE), acc=float(ACC_DISPOSE))
                except Exception as e:
                    self.get_logger().warn(f"[DISPOSE] movej HOME_J_DEG failed: {repr(e)}")
                # ===== [MOD v3.401] END =====

            else:
                self._fb(goal_handle, "PLACE_SEQ", 0.85, "down -> OPEN(wait) -> up")
                if self._cancel_check(goal_handle, "PLACE_SEQ"):
                    result.error_code = "CANCELED"
                    result.message = "Canceled during place seq"
                    goal_handle.abort()
                    return result

                if "IN" not in job_id:
                    self.get_logger().info(f"[PLACE] down {PLACE_DOWN_MM}mm")
                    rel_movel_base(dr, 0, 0, -PLACE_DOWN_MM, 0, 0, 0, vel=VELOCITY)

                    self.get_logger().info(f"[GRIP] grip_open(wait={PLACE_OPEN_WAIT})")
                    grip_open(dr, wait_sec=PLACE_OPEN_WAIT)

                    self.get_logger().info(f"[PLACE] up {PLACE_UP_MM}mm")
                    rel_movel_base(dr, 0, 0, +PLACE_UP_MM, 0, 0, 0, vel=VELOCITY)
                else:
                    temp = PLACE_DOWN_MM + 45
                    self.get_logger().info(f"[PLACE] down {temp}mm")
                    rel_movel_base(dr, 0, 0, -temp, 0, 0, 0, vel=VELOCITY)

                    self.get_logger().info(f"[GRIP] grip_open(wait={PLACE_OPEN_WAIT})")
                    grip_open(dr, wait_sec=PLACE_OPEN_WAIT)

                    self.get_logger().info(f"[PLACE] up {temp}mm")
                    rel_movel_base(dr, 0, 0, +temp, 0, 0, 0, vel=VELOCITY)

            result.success = True
            result.error_code = "OK"
            result.message = "Dispose sequence done" if is_waste else "Simple pick&place done"

            self.get_logger().info(f"[EXEC] done job_id={job_id}")
            self._fb(goal_handle, "DONE", 1.0, result.message)

            self.pub_done.publish(Bool(data=True))
            goal_handle.succeed()
            return result

        except Exception as e:
            self.get_logger().error(f"[EXEC] Exception: {repr(e)}")
            self.pub_done.publish(Bool(data=False))

            result.success = False
            result.error_code = "EXCEPTION"
            result.message = repr(e)
            self._fb(goal_handle, "FAIL", 1.0, result.message)

            goal_handle.abort()
            return result


def main(args=None):
    rclpy.init(args=args)

    action_node = RackTransportAction()
    tube_node = TubeTransportNode()

    # =========================================================
    # [핵심 수정 2] 로봇 통신(DSR)만을 위한 '전용 노드' 생성
    # =========================================================
    dsr_node = rclpy.create_node("dsr_internal_worker", namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node

    dr = initialize_robot(action_node)
    action_node.set_dr(dr)

    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(action_node)
    executor.add_node(dsr_node)
    executor.add_node(tube_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            action_node.destroy_node()
            dsr_node.destroy_node()
            tube_node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
