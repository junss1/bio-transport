# main_integrated v2.303 2026-01-25
# ✅ FIX (2026-01-24)
# - [버그수정] TUBE 성공인데도 항상 실패로 떨어지던 인덴트/흐름 오류 수정
# - [개선] call_robot / call_tube_transport에서 Action status(정수)까지 확인하여 "진짜 성공"만 success=True 처리
#          (GoalStatus import 없이 숫자 상수로 처리: 4=SUCCEEDED)
#
# ✅ MOD v2.302 (2026-01-25)
# - (기능추가) /bio_emergency(Bool) 퍼블리셔 추가: EMERGENCY 시 cancel보다 먼저 "즉시 정지 트리거" 브로드캐스트
# - (버그수정) call_robot / call_tube_transport에서 goal 거절(accepted=False) 시 active goal handle 정리 누락 수정
# - (유지) 기존 EMERGENCY 문자열 포맷/즉시 처리(락 대기 없이) 유지
#
# ✅ MOD v2.303 (2026-01-25)
# - (버그방지) ActionClient 이름을 절대 경로로 통일:
#   - "robot_action" -> "/robot_action"
#   - "tube_transport" -> "/tube_transport"
#   (namespace가 바뀌어도 절대 꼬이지 않도록)
#
# [중요]
# - 기존 코드 변경은 전부 "주석처리" 후, 바로 아래에 대체 코드를 추가했습니다.
# - 변경 구간은 [MOD v2.303] START/END로 감쌌습니다.

from __future__ import annotations

import asyncio

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Bool  # ✅ /bio_emergency publish용

try:
    from bio_transport_interfaces.action import BioCommand, RobotMove, TubeTransport
except ImportError:
    class BioCommand:  # pragma: no cover
        class Goal: command = ""
        class Result:
            def __init__(self, success=True, message=""):
                self.success = success
                self.message = message
        class Feedback:
            def __init__(self, status=""):
                self.status = status

    class RobotMove:  # pragma: no cover
        class Goal: command = ""
        class Result:
            success = True
            message = ""
        class Feedback:
            status = ""

    class TubeTransport:  # pragma: no cover
        class Goal:
            job_id = ""
            pick_posx = [0.0] * 6
            place_posx = [0.0] * 6
        class Result:
            success = True
            error_code = ""
            message = ""
        class Feedback:
            stage = ""
            progress = 0.0
            detail = ""


ACTION_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

# ✅ FIX: GoalStatus import 없이도 status 비교 가능하도록 상수로 정의
# action_msgs/msg/GoalStatus.STATUS_SUCCEEDED == 4
STATUS_SUCCEEDED = 4
STATUS_ABORTED = 6
STATUS_CANCELED = 5

# =========================
# [ADDED: EMERGENCY format] START
# =========================
# UI -> main: "EMERGENCY,STOP,NONE,NONE"
EMERGENCY_CMD0 = "EMERGENCY"
EMERGENCY_STOP = "STOP"
# =========================
# [ADDED: EMERGENCY format] END
# =========================


# =========================
# TUBE 명령 파싱
# =========================
def parse_command(cmd: str):
    """
    UI -> main으로 들어오는 1줄 명령을 절대좌표(pick_posx/place_posx)로 변환한다.

    입력 예)
    - "TUBE,IN,NONE,A-2-1"
    - "TUBE,OUT,A-2-1,NONE"
    - "TUBE,MOVE,A-2-1,A-2-3"
    - "TUBE,WASTE,A-2-1,NONE"

    반환)
    - (cmd_type, mode, pick_pose6, place_pose6)
    """

    ORIGIN_POINT = [367.32, 6.58, 422.710, 103.18, 179.97, 103.14]  # (참고용)

    A_OUT_1 = [300.11, -24.86, 421.12, 120.22, -179.78, 120.22]
    A_OUT_2 = [300.98, 13.85, 420.48, 156.15, -179.77, 155.93]
    A_OUT_3 = [302.63, 49.61, 419.08, 9.89, 179.71, 9.69]
    A_OUT_4 = [301.87, 85.68, 418.39, 20.96, 179.69, 20.63]

    B_OUT_1 = [299.76, -30.52, 416.24, 159.74, -179.66, 159.87]
    B_OUT_2 = [301.22, 3.92, 417.98, 2.79, 179.42, 3.06]
    B_OUT_3 = [299.42, 40.17, 418.31, 18.42, 179.13, 18.74]
    B_OUT_4 = [300.03, 80.63, 417.88, 16.66, 179.08, 17.21]

    OUT_1 = [627.11, -154.34, 414.82, 116.42, 180.0, 116.05]
    OUT_2 = [632.19, -116.61, 411.86, 169.15, 179.67, 168.46]
    OUT_3 = [634.42, -75.46, 411.88, 173.08, 179.62, 172.65]
    OUT_4 = [634.45, -39.53, 403.94, 165.87, -179.97, 165.84]

    A_IN_1 = [300, -24.86, 540, 120, 180, 120]
    A_IN_2 = [300, 13.85, 540, 156, 180, 156]
    A_IN_3 = [300, 51.61, 540, 10, 180, 10]
    A_IN_4 = [300, 87.68, 540, 21, 180, 21]

    B_IN_1 = [300, -30.52, 540, 160, 180, 160]
    B_IN_2 = [300, 3.92, 540, 3, 180, 3]
    B_IN_3 = [300, 40.17, 540, 18, 180, 18]
    B_IN_4 = [300, 80.63, 540, 17, 180, 17]

    IN_1 = [624.18, -154.70, 359.04, 2.33, 178.99, 2.90]
    IN_2 = [626.52, -116.78, 358.43, 5.68, 179.09, 6.08]
    IN_3 = [628.10, -81.45, 355.87, 12.00, 179.23, 12.20]
    IN_4 = [629.05, -42.82, 351.11, 18.24, 179.32, 18.48]

    DISPOSE_POSX = [640.0, -160.0, 410.0, 11.8, 180.0, 105.0]

    RACK_OUT_POINTS = {
        "A": {1: A_OUT_1, 2: A_OUT_2, 3: A_OUT_3, 4: A_OUT_4},
        "B": {1: B_OUT_1, 2: B_OUT_2, 3: B_OUT_3, 4: B_OUT_4},
    }
    RACK_IN_POINTS = {
        "A": {1: A_IN_1, 2: A_IN_2, 3: A_IN_3, 4: A_IN_4},
        "B": {1: B_IN_1, 2: B_IN_2, 3: B_IN_3, 4: B_IN_4},
    }
    OUT_POINTS = {1: OUT_1, 2: OUT_2, 3: OUT_3, 4: OUT_4}
    IN_POINTS = {1: IN_1, 2: IN_2, 3: IN_3, 4: IN_4}

    def _parse_loc(loc_str: str):
        if not loc_str or str(loc_str).upper() == "NONE":
            raise ValueError("Location is NONE. Expected a rack location like A-2-1")

        loc = str(loc_str).strip().replace("_", "-")
        toks = [t for t in loc.split("-") if t]
        if len(toks) != 3:
            raise ValueError(f"Invalid location format: {loc_str} (expected like A-2-1)")

        rack_letter = toks[0].upper()
        rack_no = toks[1].strip()
        try:
            slot = int(toks[2])
        except Exception:
            raise ValueError(f"Invalid slot in location: {loc_str}")

        if rack_letter not in ("A", "B"):
            raise ValueError("Rack letter must be A or B")
        if slot not in (1, 2, 3, 4):
            raise ValueError("Slot must be 1~4")

        return rack_letter, rack_no, slot

    parts = [p.strip() for p in str(cmd).split(",")]
    if len(parts) < 4:
        raise ValueError("Invalid command format (need at least 4 comma-separated fields)")

    cmd_type = parts[0].upper()
    if cmd_type != "TUBE":
        raise ValueError(f"parse_command only supports TUBE. got: {cmd_type}")

    mode_str = parts[1].upper()
    if mode_str in ("IN", "입고"):
        mode = "IN"
    elif mode_str in ("OUT", "출고"):
        mode = "OUT"
    elif mode_str in ("MOVE", "이동"):
        mode = "MOVE"
    elif mode_str in ("WASTE", "DISPOSE", "폐기"):
        mode = "WASTE"
    else:
        raise ValueError(f"Unknown mode: {mode_str} (expected IN/OUT/MOVE/WASTE)")

    src_str = parts[2].strip()
    dst_str = parts[3].strip()

    if mode == "IN":
        rack_letter, rack_no, slot = _parse_loc(dst_str)
        pick_pose = IN_POINTS[1]
        place_pose = RACK_IN_POINTS[rack_letter][slot]
        return cmd_type, mode, pick_pose, place_pose

    if mode == "OUT":
        rack_letter, rack_no, slot = _parse_loc(src_str)
        pick_pose = RACK_OUT_POINTS[rack_letter][slot]
        place_pose = OUT_POINTS[1]
        return cmd_type, mode, pick_pose, place_pose

    if mode == "WASTE":
        rack_letter, rack_no, slot = _parse_loc(src_str)
        pick_pose = RACK_OUT_POINTS[rack_letter][slot]
        place_pose = DISPOSE_POSX
        return cmd_type, mode, pick_pose, place_pose

    src_letter, src_no, src_slot = _parse_loc(src_str)
    dst_letter, dst_no, dst_slot = _parse_loc(dst_str)

    if (src_letter != dst_letter) or (str(src_no) != str(dst_no)):
        raise ValueError(
            f"MOVE는 같은 rack_id 내에서만 지원합니다. (src={src_str}, dst={dst_str})"
        )

    pick_pose = RACK_OUT_POINTS[src_letter][src_slot]
    place_pose = RACK_IN_POINTS[dst_letter][dst_slot]
    return cmd_type, mode, pick_pose, place_pose


class MainIntegrated(Node):
    def __init__(self):
        super().__init__("main_orchestrator")
        self.callback_group = ReentrantCallbackGroup()
        self._robot_lock = asyncio.Lock()

        # ===== [MOD v2.302] START =====
        # EMERGENCY 즉시 정지 트리거 토픽(/bio_emergency) 퍼블리셔
        self._pub_emg = self.create_publisher(Bool, "/bio_emergency", 10)
        # ===== [MOD v2.302] END =====

        # Rack server
        self._rack_server = ActionServer(
            self,
            BioCommand,
            "/bio_main_control",
            execute_callback=self.handle_rack_command,
            callback_group=self.callback_group,
            goal_callback=self.rack_goal_callback,
            cancel_callback=self.rack_cancel_callback,
            goal_service_qos_profile=ACTION_QOS,
            result_service_qos_profile=ACTION_QOS,
            cancel_service_qos_profile=ACTION_QOS,
            feedback_pub_qos_profile=ACTION_QOS,
            status_pub_qos_profile=ACTION_QOS,
        )

        # Rack client -> /robot_action
        # ===== [MOD v2.303] START =====
        # (원본) 상대 이름이라 main namespace가 붙으면 /robot_action 서버를 못 찾을 수 있습니다.
        # self.robot_client = ActionClient(self, RobotMove, "robot_action", ...)
        #
        # (수정) 절대 이름으로 통일합니다.
        self.robot_client = ActionClient(
            self,
            RobotMove,
            "/robot_action",
            callback_group=self.callback_group,
            goal_service_qos_profile=ACTION_QOS,
            result_service_qos_profile=ACTION_QOS,
            cancel_service_qos_profile=ACTION_QOS,
            feedback_sub_qos_profile=ACTION_QOS,
            status_sub_qos_profile=ACTION_QOS,
        )
        # ===== [MOD v2.303] END =====

        # Tube server (UI -> main)
        self._tube_server = ActionServer(
            self,
            BioCommand,
            "/tube_main_control",
            execute_callback=self.handle_ui_command,
            callback_group=self.callback_group,
            goal_callback=self.tube_goal_callback,
            cancel_callback=self.tube_cancel_callback,
            goal_service_qos_profile=ACTION_QOS,
            result_service_qos_profile=ACTION_QOS,
            cancel_service_qos_profile=ACTION_QOS,
            feedback_pub_qos_profile=ACTION_QOS,
            status_pub_qos_profile=ACTION_QOS,
        )

        # Tube client (main -> robot)
        # ===== [MOD v2.303] START =====
        # (원본) 상대 이름이라 main namespace가 붙으면 /tube_transport 서버를 못 찾을 수 있습니다.
        # self.tube_client = ActionClient(self, TubeTransport, "tube_transport", ...)
        #
        # (수정) 절대 이름으로 통일합니다.
        self.tube_client = ActionClient(
            self,
            TubeTransport,
            "/tube_transport",
            callback_group=self.callback_group,
            goal_service_qos_profile=ACTION_QOS,
            result_service_qos_profile=ACTION_QOS,
            cancel_service_qos_profile=ACTION_QOS,
            feedback_sub_qos_profile=ACTION_QOS,
            status_sub_qos_profile=ACTION_QOS,
        )
        # ===== [MOD v2.303] END =====

        # =========================
        # [ADDED: active goal handles for emergency cancel] START
        # =========================
        self._active_robot_goal_handle = None  # RobotMove goal handle
        self._active_tube_goal_handle = None   # TubeTransport goal handle
        self._active_gh_lock = asyncio.Lock()  # handle 보호용
        # =========================
        # [ADDED: active goal handles for emergency cancel] END
        # =========================

        self.get_logger().info("🧠 [Integrated] main_integrated ready (Rack+Tube).")

    # ---------------- Rack ----------------
    def rack_goal_callback(self, goal_request: BioCommand.Goal):
        self.get_logger().info(f"📩 [Rack] Goal: {getattr(goal_request, 'command', '')}")
        return GoalResponse.ACCEPT

    def _make_rack_pull_return_cmd(self, raw_cmd: str):
        parts = [p.strip() for p in str(raw_cmd).split(",") if p.strip()]
        if len(parts) < 4:
            raise ValueError(f"Invalid TUBE cmd (need 4 fields): {raw_cmd}")

        mode = parts[1].upper()
        src = parts[2].upper()
        dst = parts[3].upper()

        loc = dst if mode == "IN" else src
        if not loc or loc == "NONE":
            return "", ""

        loc = loc.replace("_", "-")
        toks = [t for t in loc.split("-") if t]
        if len(toks) < 2:
            raise ValueError(f"Invalid slot format for rack resolve: {loc}")

        rack_id = f"{toks[0].upper()}-{toks[1]}"
        pull_cmd = f"OUT,{rack_id},NONE"
        return_cmd = f"IN,NONE,{rack_id}"
        return pull_cmd, return_cmd

    def rack_cancel_callback(self, goal_handle):
        self.get_logger().warn("🛑 [Rack] Cancel")
        return CancelResponse.ACCEPT

    async def handle_rack_command(self, goal_handle):
        raw_cmd = goal_handle.request.command

        # EMERGENCY 즉시 처리
        try:
            raw_s = str(raw_cmd).strip()
            _parts0 = [p.strip() for p in raw_s.split(",")]
            _cmd0 = _parts0[0].upper() if _parts0 else ""
        except Exception:
            _cmd0 = ""

        if _cmd0 == EMERGENCY_CMD0:
            ok, msg = await self._emergency_stop_and_home(ui_goal_handle=goal_handle)
            if ok:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return BioCommand.Result(success=ok, message=str(msg))

        try:
            parts = [p.strip() for p in str(raw_cmd).split(",")]
            sub_cmd = ",".join(parts[1:]) if len(parts) >= 2 else str(raw_cmd)
        except Exception:
            sub_cmd = str(raw_cmd)

        if self._robot_lock.locked():
            try:
                goal_handle.publish_feedback(BioCommand.Feedback(status=f"대기열: 다른 작업 실행 중 ({sub_cmd})"))
            except Exception:
                pass

        async with self._robot_lock:
            try:
                goal_handle.publish_feedback(BioCommand.Feedback(status=f"실행 중: {sub_cmd}"))
            except Exception:
                pass
            success, msg = await self.call_robot(sub_cmd)

        goal_handle.succeed() if success else goal_handle.abort()
        return BioCommand.Result(success=success, message=msg)

    # ---------------- Tube ----------------
    def tube_goal_callback(self, goal_request: BioCommand.Goal):
        self.get_logger().info(f"📩 [Tube] Goal: {getattr(goal_request, 'command', '')}")
        return GoalResponse.ACCEPT

    def tube_cancel_callback(self, goal_handle):
        self.get_logger().warn("🛑 [Tube] Cancel")
        return CancelResponse.ACCEPT

    async def call_robot(self, cmd_str: str):
        cmd_str = str(cmd_str).strip()
        if not cmd_str:
            return False, "robot_action으로 보낼 cmd가 비어 있습니다."

        if not self.robot_client.wait_for_server(timeout_sec=2.0):
            return False, "하위 로봇 Action(/robot_action) 서버 연결 실패"

        goal = RobotMove.Goal()
        goal.command = cmd_str

        gh = await self.robot_client.send_goal_async(goal)

        try:
            async with self._active_gh_lock:
                self._active_robot_goal_handle = gh
        except Exception:
            pass

        # goal 거절 시 active handle 정리
        if not gh.accepted:
            try:
                async with self._active_gh_lock:
                    if self._active_robot_goal_handle is gh:
                        self._active_robot_goal_handle = None
            except Exception:
                pass
            return False, "하위 로봇 Action Goal 거절됨"

        res = await gh.get_result_async()

        try:
            async with self._active_gh_lock:
                if self._active_robot_goal_handle is gh:
                    self._active_robot_goal_handle = None
        except Exception:
            pass

        status = int(getattr(res, "status", -1))
        child = getattr(res, "result", None)

        child_success = bool(getattr(child, "success", False)) if child is not None else False
        child_msg = str(getattr(child, "message", "")) if child is not None else ""

        ok = (status == STATUS_SUCCEEDED) and child_success

        self.get_logger().info(
            f"[call_robot] cmd='{cmd_str}' status={status} child_success={child_success} -> ok={ok} msg='{child_msg}'"
        )

        return ok, child_msg

    async def handle_ui_command(self, goal_handle):
        raw_cmd = str(goal_handle.request.command).strip()
        self.get_logger().info(f"📥 명령 실행 시작: {raw_cmd}")

        # EMERGENCY 즉시 처리 (락 대기 없이)
        try:
            _parts0 = [p.strip() for p in raw_cmd.split(",")]
            _cmd0 = _parts0[0].upper() if _parts0 else ""
        except Exception:
            _cmd0 = ""

        if _cmd0 == EMERGENCY_CMD0:
            ok, msg = await self._emergency_stop_and_home(ui_goal_handle=goal_handle)
            if ok:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return BioCommand.Result(success=ok, message=str(msg))

        if self._robot_lock.locked():
            try:
                goal_handle.publish_feedback(BioCommand.Feedback(status=f"대기열: 다른 작업 실행 중 ({raw_cmd})"))
            except Exception:
                pass

        async with self._robot_lock:
            parts = [p.strip() for p in raw_cmd.split(",")]
            cmd_type = parts[0].upper() if parts else ""

            success = False
            msg = ""

            if cmd_type == "TUBE":
                try:
                    _, mode, pick_pose, place_pose = parse_command(raw_cmd)
                except Exception as e:
                    msg = f"명령 파싱 실패(TUBE): {e}"
                    self.get_logger().error(msg)
                    goal_handle.abort()
                    return BioCommand.Result(success=False, message=msg)

                try:
                    goal_handle.publish_feedback(BioCommand.Feedback(status=f"실행 중: TUBE({mode}) (RACK->TUBE->RACK)"))
                except Exception:
                    pass

                pull_cmd, return_cmd = self._make_rack_pull_return_cmd(raw_cmd)

                ok_pull, pull_msg = (True, "skip")
                if pull_cmd:
                    ok_pull, pull_msg = await self.call_robot(pull_cmd)

                if not ok_pull:
                    msg = f"랙 빼기 실패: {pull_msg}"
                    self.get_logger().error(msg)
                    goal_handle.abort()
                    return BioCommand.Result(success=False, message=msg)

                ok_tube, err_code, tube_msg = await self.call_tube_transport(
                    mode, pick_pose, place_pose, ui_goal_handle=goal_handle
                )

                if not ok_tube:
                    self.get_logger().error(f"튜브 이송 실패: {tube_msg} (error_code={err_code})")

                    ok_ret, ret_msg = (True, "skip")
                    if return_cmd:
                        ok_ret, ret_msg = await self.call_robot(return_cmd)

                    if not ok_ret:
                        msg = f"튜브 이송 실패({err_code}): {tube_msg} / 랙 복귀도 실패: {ret_msg}"
                        self.get_logger().error(msg)
                        goal_handle.abort()
                        return BioCommand.Result(success=False, message=msg)

                    msg = f"튜브 이송 실패({err_code}): {tube_msg} (랙은 복귀 완료)"
                    goal_handle.abort()
                    return BioCommand.Result(success=False, message=msg)

                ok_ret, ret_msg = (True, "skip")
                if return_cmd:
                    ok_ret, ret_msg = await self.call_robot(return_cmd)

                if not ok_ret:
                    msg = f"랙 원위치 실패: {ret_msg}"
                    self.get_logger().error(msg)
                    goal_handle.abort()
                    return BioCommand.Result(success=False, message=msg)

                success = True
                msg = "TUBE 작업 완료"

            elif cmd_type == "RACK":
                try:
                    sub_cmd = ",".join(parts[1:]) if len(parts) >= 2 else raw_cmd
                except Exception:
                    sub_cmd = raw_cmd

                try:
                    goal_handle.publish_feedback(BioCommand.Feedback(status="실행 중: RACK Move"))
                except Exception:
                    pass

                success, msg = await self.call_robot(sub_cmd)

            else:
                msg = f"지원하지 않는 cmd_type: '{cmd_type}' (TUBE 또는 RACK만 처리)"
                self.get_logger().warn(msg)
                goal_handle.abort()
                return BioCommand.Result(success=False, message=msg)

        goal_handle.succeed() if success else goal_handle.abort()
        return BioCommand.Result(success=success, message=str(msg))

    def _make_tube_feedback_callback(self, ui_goal_handle):
        if ui_goal_handle is None:
            return None

        def _cb(feedback_msg):
            try:
                fb = getattr(feedback_msg, "feedback", feedback_msg)
                stage = str(getattr(fb, "stage", ""))
                progress = float(getattr(fb, "progress", 0.0))
                detail = str(getattr(fb, "detail", ""))

                pct = progress * 100.0 if progress <= 1.0 else progress
                pct = max(0.0, min(100.0, pct))

                msg = f"🟡 [TubeFeedback] {stage} ({pct:.0f}%) {detail}".strip()
                ui_goal_handle.publish_feedback(BioCommand.Feedback(status=msg))
            except Exception:
                pass

        return _cb

    async def call_tube_transport(self, mode: str, pick_pose, place_pose, ui_goal_handle=None):
        if not self.tube_client.wait_for_server(timeout_sec=2.0):
            return False, "NO_SERVER", "하위 튜브 Action(/tube_transport) 서버 연결 실패"

        goal = TubeTransport.Goal()

        mode_u = str(mode).upper().strip()
        if hasattr(goal, "job_id"):
            goal.job_id = f"TUBE_{mode_u}"

        goal.pick_posx = [float(x) for x in pick_pose]
        goal.place_posx = [float(x) for x in place_pose]

        fb_cb = self._make_tube_feedback_callback(ui_goal_handle)

        gh = await self.tube_client.send_goal_async(goal, feedback_callback=fb_cb)

        try:
            async with self._active_gh_lock:
                self._active_tube_goal_handle = gh
        except Exception:
            pass

        if not gh.accepted:
            try:
                async with self._active_gh_lock:
                    if self._active_tube_goal_handle is gh:
                        self._active_tube_goal_handle = None
            except Exception:
                pass
            return False, "GOAL_REJECTED", "하위 튜브 Action Goal 거절됨"

        res = await gh.get_result_async()

        try:
            async with self._active_gh_lock:
                if self._active_tube_goal_handle is gh:
                    self._active_tube_goal_handle = None
        except Exception:
            pass

        status = int(getattr(res, "status", -1))
        child = getattr(res, "result", None)

        child_success = bool(getattr(child, "success", False)) if child is not None else False
        child_err = str(getattr(child, "error_code", "")) if child is not None else ""
        child_msg = str(getattr(child, "message", "")) if child is not None else ""

        ok = (status == STATUS_SUCCEEDED) and child_success

        self.get_logger().info(
            f"[call_tube_transport] mode={mode_u} status={status} child_success={child_success} -> ok={ok} err='{child_err}' msg='{child_msg}'"
        )

        return ok, child_err, child_msg

    async def _emergency_stop_and_home(self, ui_goal_handle=None):
        def _ui_fb(text: str):
            if ui_goal_handle is None:
                return
            try:
                ui_goal_handle.publish_feedback(BioCommand.Feedback(status=text))
            except Exception:
                pass

        _ui_fb("🛑 EMERGENCY: broadcasting /bio_emergency ...")
        try:
            self._pub_emg.publish(Bool(data=True))
        except Exception:
            pass

        _ui_fb("🛑 EMERGENCY: canceling active goals...")

        async with self._active_gh_lock:
            robot_gh = self._active_robot_goal_handle
            tube_gh = self._active_tube_goal_handle

        try:
            if tube_gh is not None:
                await tube_gh.cancel_goal_async()
        except Exception:
            pass

        try:
            if robot_gh is not None:
                await robot_gh.cancel_goal_async()
        except Exception:
            pass

        _ui_fb("🛑 EMERGENCY: canceled. requesting HOME...")

        ok_home, msg_home = await self.call_robot("HOME")
        if ok_home:
            _ui_fb("✅ EMERGENCY: HOME done")
            return True, "EMERGENCY: broadcast + cancel + HOME done"
        else:
            _ui_fb(f"⚠️ EMERGENCY: HOME failed ({msg_home})")
            return False, f"EMERGENCY: broadcast + cancel, but HOME failed: {msg_home}"


def main():
    rclpy.init()
    node = MainIntegrated()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
