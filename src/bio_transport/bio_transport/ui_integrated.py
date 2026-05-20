# ui_integrated v2.711 2026-01-25
# [수정 사항]
# 1. 초기 재고 커스텀 설정
#    - A-2 렉: 1, 3번 튜브 보유
#    - B-1 렉: 1, 2번 튜브 보유
# 2. 버튼 클릭 시 재고 상태 경고창, 작업 중 점등(Blinking), 동기화 로직 유지
#
# =========================
# [ADDED: EMERGENCY STOP / HOME] START (2026-01-25)
# - UI에 "긴급정지(EMERGENCY STOP)" 버튼 추가
# - UI에 "홈 복귀(HOME)" 버튼 추가
# - 전송 포맷(최소 수정):
#   - "EMERGENCY,STOP,NONE,NONE"
#   - "HOME,NONE,NONE,NONE"
# - 전송 경로:
#   - /tube_main_control (BioCommand)로 전송 (기존 send_tube_command_line 재사용)
# - BusyPopup(기존 팝업)를 재사용해 "긴급정지 요청/홈 복귀 요청" 즉시 표시
# =========================
# [ADDED: EMERGENCY STOP / HOME] END (2026-01-25)


import sys
import os
import re
import subprocess
from PySide6.QtWidgets import (
   QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
   QTabWidget, QScrollArea, QGroupBox, QFrame, QGridLayout,
   QLabel, QDialog, QToolButton, QPushButton, QRadioButton, QLineEdit,
   QFormLayout, QTextEdit, QSizePolicy, QButtonGroup, QMessageBox
)
from PySide6.QtCore import Qt, QTimer


import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


try:
   from bio_transport_interfaces.action import BioCommand
except ImportError:  # pragma: no cover
   class BioCommand:
       class Goal:
           command = ""
       class Result:
           def __init__(self, success=True, message=""):
               self.success = success
               self.message = message
       class Feedback:
           def __init__(self, status=""):
               self.status = status




# =========================
# QoS
# =========================
ACTION_QOS = QoSProfile(
   reliability=ReliabilityPolicy.RELIABLE,
   durability=DurabilityPolicy.VOLATILE,
   history=HistoryPolicy.KEEP_LAST,
   depth=5,
)


# ========================================================
# [스타일시트]
# ========================================================
STYLE_SHEET = """QWidget { font-family: "Segoe UI", "Malgun Gothic", sans-serif; color: #000000; }
QMainWindow { background-color: #F1F5F9; }


QRadioButton { font-size: 14px; font-weight: bold; color: #333333; padding: 4px; }
QTabWidget::pane { border: 1px solid #CBD5E1; background: #FFFFFF; border-radius: 6px; }
QTabBar::tab { background: #E2E8F0; color: #64748B; padding: 10px 25px; margin-right: 2px; font-weight: bold; }
QTabBar::tab:selected { background: #FFFFFF; color: #2563EB; border-top: 3px solid #2563EB; }


QGroupBox {
   font-weight: bold; font-size: 20px;
   border: 2px solid #334155; border-radius: 8px; margin-top: 35px;
   background-color: #FFFFFF; color: #FFFFFF;
}
QGroupBox::title {
   subcontrol-origin: margin; left: 10px; padding: 5px 15px;
   background-color: #334155; border-radius: 6px;
}


QFrame.RackFrame { background-color: #334155; border-radius: 6px; border: 1px solid #1E293B; }
QLineEdit { border: 1px solid #CBD5E1; border-radius: 4px; padding: 6px; background: #F8FAFC; color: #000000; }
QLineEdit:focus { border: 1px solid #2563EB; background: #FFFFFF; }
QTextEdit { background-color: #1E293B; color: #00FF00; font-family: "Consolas", monospace; font-size: 12px; border-radius: 4px; border: 1px solid #334155; }


QPushButton {
   background-color: #FFFFFF;
   border: 1px solid #CBD5E1;
   color: #333333;
   font-weight: bold;
   border-radius: 4px;
   padding: 8px;
   min-height: 35px;
}
QPushButton:pressed { background-color: #E2E8F0; padding-top: 10px; padding-bottom: 6px; }


QPushButton#btnConfirm {
   background-color: #2563EB;
   color: #000000;
   border: 1px solid #1D4ED8;
   border-bottom: 3px solid #1D4ED8;
   font-weight: bold;
   border-radius: 4px;
}
QPushButton#btnConfirm:hover { background-color: #000000; color: #000000; }
QPushButton#btnConfirm:pressed {
   background-color: #FFFFFF;
   color: #FFFFFF;
   border-bottom: 0px solid;
   border-top: 3px solid transparent;
   padding-top: 10px; padding-bottom: 6px;
}


QToolButton.TubeBtn { background-color: #F8FAFC; border: 2px solid #94A3B8; border-radius: 13px; width: 52px; height: 52px; margin: 4px; }
QToolButton.TubeBtn:checked { background-color: #F59E0B; border-color: #D97706; }


QToolButton.TubeBtnOccupied {
   background-color: #FECACA; border: 2px solid #EF4444;
   border-radius: 13px; width: 52px; height: 52px; margin: 4px;
}
QToolButton.TubeBtnOccupied:checked { background-color: #F59E0B; border-color: #D97706; }


QToolButton.TubeBtnBlocked {
   background-color: #FECACA; border: 2px solid #EF4444;
   border-radius: 13px; width: 52px; height: 52px; margin: 4px;
}


/* 튜브 점등 스타일 */
QToolButton.TubeBtnBlinking {
   background-color: #2563EB; border: 2px solid #1D4ED8;
   border-radius: 13px; width: 52px; height: 52px; margin: 4px;
}


QPushButton.RackSelectBtn {
   background-color: #475569; color: #FFFFFF;
   border: 1px solid #64748B; border-radius: 4px;
   font-size: 18px; font-weight: bold; min-height: 30px;
}
QPushButton.RackSelectBtn:checked { background-color: #F59E0B; border-color: #D97706; color: #FFFFFF; }


QPushButton.RackSelectBtnOccupied {
   background-color: #FECACA; color: #B91C1C;
   border: 2px solid #EF4444; border-radius: 4px;
   font-size: 18px; font-weight: bold; min-height: 30px;
}
QPushButton.RackSelectBtnOccupied:checked { background-color: #F59E0B; border-color: #D97706; color: #FFFFFF; }


QPushButton.RackSelectBtnBlocked {
   background-color: #FECACA; color: #B91C1C;
   border: 2px solid #EF4444; border-radius: 4px;
   font-size: 18px; font-weight: bold; min-height: 30px;
}


/* 렉 점등 스타일 */
QPushButton.RackSelectBtnBlinking {
   background-color: #2563EB; color: #FFFFFF;
   border: 2px solid #1D4ED8; border-radius: 4px;
   font-size: 18px; font-weight: bold; min-height: 30px;
}
"""




class UiActionClientNode(Node):
   """Qt 이벤트 루프와 rclpy를 함께 돌리기 위한 ActionClient 노드."""


   def __init__(self, ui):
       super().__init__("ui_integrated_client")
       self.ui = ui


       self.qos = ACTION_QOS
       cbg = ReentrantCallbackGroup()


       self.client = ActionClient(
           self, BioCommand, "/bio_main_control",
           callback_group=cbg,
           goal_service_qos_profile=self.qos,
           result_service_qos_profile=self.qos,
           cancel_service_qos_profile=self.qos,
           feedback_sub_qos_profile=self.qos,
           status_sub_qos_profile=self.qos,
       )


       self.tube_client = ActionClient(
           self, BioCommand, "/tube_main_control",
           callback_group=cbg,
           goal_service_qos_profile=self.qos,
           result_service_qos_profile=self.qos,
           cancel_service_qos_profile=self.qos,
           feedback_sub_qos_profile=self.qos,
           status_sub_qos_profile=self.qos,
       )


   # -------------------------
   # Rack (BioCommand)
   # -------------------------
   def send_rack_command(self, cmd_type: str, src: str, dest: str) -> bool:
       cmd_type = (cmd_type or "").strip().upper()
       src = (src or "NONE").strip()
       dest = (dest or "NONE").strip()
       final_cmd = f"RACK,{cmd_type},{src},{dest}"


       if not self.client.wait_for_server(timeout_sec=5.0):
           self.ui.log_t2("❌ [Action] /bio_main_control 서버 연결 실패")
           return False


       goal = BioCommand.Goal()
       goal.command = final_cmd


       self.ui.log_t2(f"📤 [Action] 전송: {final_cmd}")
       fut = self.client.send_goal_async(goal, feedback_callback=self._on_rack_feedback)
       fut.add_done_callback(self._on_rack_goal_response)
       return True


   def _on_rack_feedback(self, feedback_msg):
       try:
           st = getattr(feedback_msg.feedback, "status", "")
           if st:
               self.ui.log_t2(f"🟡 [Feedback] {st}")
       except Exception:
           pass


   def _on_rack_goal_response(self, future):
       try:
           goal_handle = future.result()
       except Exception as e:
           self.ui.on_rack_action_result(False, f"Goal exception: {e}")
           return


       if not goal_handle.accepted:
           self.ui.on_rack_action_result(False, "Goal rejected")
           return


       res_future = goal_handle.get_result_async()
       res_future.add_done_callback(self._on_rack_result)


   def _on_rack_result(self, future):
       try:
           res = future.result().result
           ok = bool(getattr(res, "success", False))
           msg = str(getattr(res, "message", ""))
       except Exception as e:
           ok = False
           msg = f"Result exception: {e}"


       self.ui.on_rack_action_result(ok, msg)


   # -------------------------
   # Tube (BioCommand)
   # -------------------------
   def send_tube_command_line(self, line: str) -> bool:
       if not self.tube_client.wait_for_server(timeout_sec=5.0):
           self.ui.log_t1("❌ [Tube] /tube_main_control 서버 연결 실패")
           return False


       goal = BioCommand.Goal()
       goal.command = str(line)


       self.ui.log_t1(f"📤 [Tube] 전송: {goal.command}")
       fut = self.tube_client.send_goal_async(goal, feedback_callback=self._on_tube_feedback)
       fut.add_done_callback(self._on_tube_goal_response)
       return True


   def _on_tube_feedback(self, feedback_msg):
       try:
           st = getattr(feedback_msg.feedback, "status", "")
           if st:
               self.ui.log_t1(f"🟡 [TubeFeedback] {st}")

               # =========================
               # [ADDED: EMERGENCY STOP / HOME] START (2026-01-25)
               # - 긴급정지/홈 복귀 관련 피드백이 오면 같은 팝업창 텍스트를 갱신
               # =========================
               try:
                   s_u = str(st).upper()
                   if ("EMERGENCY" in s_u) or (s_u.startswith("HOME")) or ("HOME" in s_u):
                       # BusyPopup 재사용(요청대로)
                       self.ui.show_busy_popup(str(st))
               except Exception:
                   pass
               # =========================
               # [ADDED: EMERGENCY STOP / HOME] END (2026-01-25)

       except Exception:
           pass


   def _on_tube_goal_response(self, future):
       try:
           goal_handle = future.result()
       except Exception as e:
           self.ui.on_tube_action_result(False, "GOAL_EXCEPTION", str(e))
           return


       if not goal_handle.accepted:
           self.ui.on_tube_action_result(False, "GOAL_REJECTED", "Goal rejected")
           return


       res_future = goal_handle.get_result_async()
       res_future.add_done_callback(self._on_tube_result)


   def _on_tube_result(self, future):
       try:
           res = future.result().result
           ok = bool(getattr(res, "success", False))
           msg = str(getattr(res, "message", ""))
           err = "" if ok else "FAIL"
       except Exception as e:
           ok = False
           err = "RESULT_EXCEPTION"
           msg = str(e)


       self.ui.on_tube_action_result(ok, err, msg)




class BusyPopup(QDialog):
   def __init__(self, parent=None):
       super().__init__(parent)
       self.setWindowTitle("작동 중")
       self.setModal(True)
       self.setFixedSize(360, 180)
       v = QVBoxLayout(self)
       v.setContentsMargins(16, 16, 16, 16)
       v.setSpacing(12)
       self.lbl = QLabel("작동 중입니다.", self)
       self.lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
       self.lbl.setWordWrap(True)
       self.lbl.setStyleSheet("font-size: 16px; font-weight: bold; color: #d32f2f;")
       v.addWidget(self.lbl, stretch=1)
       btn = QPushButton("닫기", self)
       btn.setCursor(Qt.CursorShape.PointingHandCursor)
       btn.clicked.connect(self.close)
       v.addWidget(btn, alignment=Qt.AlignmentFlag.AlignCenter)


   def set_message(self, text: str):
       self.lbl.setText(str(text))




class BioBankApp(QMainWindow):
   def __init__(self):
       super().__init__()
       self.setWindowTitle("BioBank System")
       self.resize(1300, 850)
       self.setStyleSheet(STYLE_SHEET)


       self.t1_mode_group = QButtonGroup(self)
       self.t2_mode_group = QButtonGroup(self)


       self.t1_selected_items = set()
       self.t1_dest_items = set()
       self.t1_active_buttons = set()


       self.t2_selected_items = set()
       self.t2_dest_items = set()
       self.t2_active_buttons = set()


       self.blocked_specific = []
       self.blocked_prefix = []


       # ====================================================
       # [재고(Inventory) 초기 설정]
       # ====================================================
       self.inventory = set()


       # 1. A-2 렉: 1번, 3번 튜브
       self.inventory.add("A-2")      # 렉 존재
       self.inventory.add("A-2-1")    # 튜브 1
       self.inventory.add("A-2-3")    # 튜브 3
       # A-2-2, A-2-4는 추가 안 함 (비어있음)


       # 2. B-1 렉: 1번, 2번 튜브
       self.inventory.add("B-1")      # 렉 존재
       self.inventory.add("B-1-1")    # 튜브 1
       self.inventory.add("B-1-2")    # 튜브 2
       # B-1-3, B-1-4는 추가 안 함 (비어있음)


       # 3. (옵션) C, D열은 가득 찬 상태로 유지 (화면 구성용)
       for prefix in ["C", "D"]:
           for r in range(1, 4):
               rack = f"{prefix}-{r}"
               self.inventory.add(rack)
               for t in range(1, 5):
                   self.inventory.add(f"{rack}-{t}")
       # A-1, B-3만 등록 + 각 rack의 1~4 슬롯도 등록
       for rack in ["A-1", "B-3"]:
           self.inventory.add(rack)
           for t in range(1, 5):
               self.inventory.add(f"{rack}-{t}")


       # ====================================================


       self.widget_map = {}
      
       # 점등 관련
       self.blinking_items = set()
       self.blink_timer = QTimer()
       self.blink_timer.timeout.connect(self._handle_blink)
       self.blink_state_on = True


       self._pending_rack_change = None
       self._pending_tube_change = None
       self._tube_job_queue = []
       self._tube_job_running = False


       self.ros_node = None


       central_widget = QWidget()
       self.setCentralWidget(central_widget)
       main_layout = QVBoxLayout(central_widget)


       self.tabs = QTabWidget()
       main_layout.addWidget(self.tabs)


       self.setup_tab1()
       self.setup_tab2()


       # Busy Popup
       self._rack_job_running = False
       self._busy_reason = ""
       self._busy_popup = BusyPopup(self)

       # =========================
       # [ADDED: EMERGENCY STOP / HOME] START (2026-01-25)
       # - UI 커맨드 상수(요청 포맷)
       # =========================
       self.CMD_EMERGENCY_STOP = "EMERGENCY,STOP,NONE,NONE"
       self.CMD_HOME = "HOME,NONE,NONE,NONE"
       # =========================
       # [ADDED: EMERGENCY STOP / HOME] END (2026-01-25)


   # --- Blinking Logic ---
   def _handle_blink(self):
       self.blink_state_on = not self.blink_state_on
       for item_id in self.blinking_items:
           if item_id in self.widget_map:
               btn, mode = self.widget_map[item_id]
               if self.blink_state_on:
                   # 점등 색상 (파란색)
                   cls = "TubeBtnBlinking" if mode == "tube" else "RackSelectBtnBlinking"
               else:
                   # 꺼짐 상태
                   cls = self._get_normal_style_class(item_id, mode)
              
               btn.setProperty("class", cls)
               btn.style().unpolish(btn)
               btn.style().polish(btn)


   def _get_normal_style_class(self, item_id, mode):
       if self.is_item_blocked(item_id):
           return "TubeBtnBlocked" if mode == "tube" else "RackSelectBtnBlocked"
       elif item_id in self.inventory:
           return "TubeBtnOccupied" if mode == "tube" else "RackSelectBtnOccupied"
       else:
           return "TubeBtn" if mode == "tube" else "RackSelectBtn"


   def start_blinking(self, items):
       for item in items:
           self.blinking_items.add(item)
       if not self.blink_timer.isActive():
           self.blink_timer.start(500) # 0.5초 간격


   def stop_blinking(self):
       items_to_reset = list(self.blinking_items)
       self.blinking_items.clear()
       self.blink_timer.stop()
       for item in items_to_reset:
           self.update_button_style(item)


   # ---------------------


   # ---------------------


   def set_ros_node(self, ros_node):
       self.ros_node = ros_node


   def _is_busy_global(self) -> bool:
       return bool(self._rack_job_running or self._tube_job_running or self._tube_job_queue)


   def _set_busy_reason(self, reason: str):
       self._busy_reason = str(reason or "").strip()


   def show_busy_popup(self, extra: str = ""):
       # 팝업을 띄우되, 뒷배경(버튼)이 보이도록 모달리스로 하거나
       # 깜빡임을 확인하기 위해 팝업을 안 띄울 수도 있음.
       detail = self._busy_reason or "알 수 없음"
       msg = "작동 중입니다.\n\n현재 작업: " + detail
       if extra:
           msg += "\n\n" + str(extra)
       self._busy_popup.set_message(msg)
       try:
           self._busy_popup.show()
           self._busy_popup.raise_()
       except Exception:
           pass


   def _auto_close_busy_popup_if_idle(self):
       if self._is_busy_global():
           return
       try:
           if self._busy_popup.isVisible():
               self._busy_popup.close()
       except Exception:
           pass


   def on_rack_action_result(self, success: bool, message: str):
       self.stop_blinking() # 점등 종료


       if success:
           self.log_t2(f"✅ [Result] 성공: {message}")
           if self._pending_rack_change is not None:
               mode, sel_list, dest_list = self._pending_rack_change
               self.process_inventory_change(mode, sel_list, dest_list)
       else:
           self.log_t2(f"❌ [Result] 실패: {message}")


       self._rack_job_running = False
       self._pending_rack_change = None


       if not self._tube_job_running and not self._tube_job_queue:
           self._set_busy_reason("")


       self._auto_close_busy_popup_if_idle()


   def on_tube_action_result(self, success: bool, error_code: str, message: str):
       # 튜브 작업 완료 시 점등 종료
       if success:
           self.log_t1(f"✅ [Result] 성공: {message}")
           if self._pending_tube_change is not None:
               mode, sel_list, dest_list = self._pending_tube_change
               self.process_inventory_change(mode, sel_list, dest_list)
       else:
           self.log_t1(f"❌ [Result] 실패({error_code}): {message}")


       self._pending_tube_change = None
       self._tube_job_running = False
      
       if not self._tube_job_queue:
           self.stop_blinking() # 모든 큐 작업 종료 시 점등 끔
           if not self._rack_job_running:
               self._set_busy_reason("")
      
       self._start_next_tube_job()
       self._auto_close_busy_popup_if_idle()


   def _start_next_tube_job(self):
       if self._tube_job_running:
           return
       if not self._tube_job_queue:
           self.log_t1("✅ [Tube] 모든 작업 완료")
           return


       mode_id, sel_list, dest_list, line = self._tube_job_queue.pop(0)
       self._tube_job_running = True
       self._pending_tube_change = (mode_id, sel_list, dest_list)
       self._set_busy_reason(f"TUBE: {line}")


       # 작업 대상 점등 시작
       targets = set(sel_list) | set(dest_list)
       self.start_blinking(targets)


       self.log_t1(f"📤 [Tube] 전송: {line}")
       if self.ros_node is None:
           self.log_t1("❌ [Tube] ROS 노드 미연동")
           self._tube_job_running = False
           self.stop_blinking()
           return


       ok = self.ros_node.send_tube_command_line(line)
       if not ok:
           self._tube_job_running = False
           self.stop_blinking()
           self.log_t1("❌ [Tube] tube_main_control 서버 연결 실패")


   def log_t1(self, msg):
       self.txt_log_t1.append(str(msg))


   def log_t2(self, msg):
       self.txt_log_t2.append(str(msg))


   def is_item_blocked(self, item_id):
       for bad in self.blocked_specific:
           if bad in item_id:
               return True
       for prefix in self.blocked_prefix:
           if item_id.startswith(prefix):
               return True
       return False


   def update_button_style(self, item_id):
       if item_id not in self.widget_map:
           return
       btn, mode = self.widget_map[item_id]
      
       # 블링크 중이면 스타일 업데이트 건너뜀 (타이머가 제어함)
       if item_id in self.blinking_items:
           return


       cls = self._get_normal_style_class(item_id, mode)
       btn.setProperty("class", cls)
       btn.style().unpolish(btn)
       btn.style().polish(btn)


   def process_inventory_change(self, mode_id, src_list, dest_list):
       # [수정] 렉 이동/출고 시 내부 튜브 동기화 로직 추가
      
       # 1. 튜브/렉 자체 인벤토리 처리
       if mode_id == 1:  # 입고
           for item in dest_list:
               self.inventory.add(item)
               # [추가] 렉 입고 시 내부 튜브도 채워진 상태로 반영 (튜브 탭 동기화)
               if len(item.split('-')) == 2:  # 예: "A-1"
                   for i in range(1, 5):
                       tube_id = f"{item}-{i}"
                       self.inventory.add(tube_id)
                       if tube_id in self.widget_map:
                           self.update_button_style(tube_id)


       elif mode_id in (2, 4):  # 출고/폐기
           for item in src_list:
               self.inventory.discard(item)
               # [동기화] 렉 출고 시 내부 튜브 제거
               if len(item.split('-')) == 2: # 렉인 경우 (예: A-1)
                   for i in range(1, 5):
                       tube_id = f"{item}-{i}"
                       self.inventory.discard(tube_id)
                       if tube_id in self.widget_map:
                           self.update_button_style(tube_id)


       elif mode_id == 3:  # 이동
           src_item = src_list[0]
           dst_item = dest_list[0]
          
           self.inventory.discard(src_item)
           self.inventory.add(dst_item)
          
           # [동기화] 렉 이동 시 내부 튜브 이동
           if len(src_item.split('-')) == 2: # 렉인 경우
               for i in range(1, 5):
                   old_tube = f"{src_item}-{i}"
                   new_tube = f"{dst_item}-{i}"
                  
                   if old_tube in self.inventory:
                       self.inventory.discard(old_tube)
                       self.inventory.add(new_tube)
                  
                   # 스타일 업데이트
                   if old_tube in self.widget_map: self.update_button_style(old_tube)
                   if new_tube in self.widget_map: self.update_button_style(new_tube)


       # 스타일 일괄 업데이트
       targets = set(src_list) | set(dest_list)
       for item in targets:
           self.update_button_style(item)


   def reset_selection_t1(self):
       for btn in list(self.t1_active_buttons):
           btn.setChecked(False)
       self.t1_active_buttons.clear()
       self.t1_selected_items.clear()
       self.t1_dest_items.clear()
       self.le_t1_selected.clear()
       self.le_t1_dest.clear()
       self.le_t1_input.clear()
       self.txt_log_t1.setText("[System] Ready...")


   def reset_selection_t2(self):
       for btn in list(self.t2_active_buttons):
           btn.setChecked(False)
       self.t2_active_buttons.clear()
       self.t2_selected_items.clear()
       self.t2_dest_items.clear()
       self.le_t2_selected.clear()
       self.le_t2_dest.clear()
       self.le_t2_input.clear()
       self.txt_log_t2.setText("[System] Ready...")


   def update_text_fields_t1(self):
       self.le_t1_selected.setText(", ".join(sorted(self.t1_selected_items)))
       self.le_t1_dest.setText(", ".join(sorted(self.t1_dest_items)))


   def update_text_fields_t2(self):
       self.le_t2_selected.setText(", ".join(sorted(self.t2_selected_items)))
       self.le_t2_dest.setText(", ".join(sorted(self.t2_dest_items)))


   # =========================
   # [ADDED: EMERGENCY STOP / HOME] START (2026-01-25)
   # =========================
   def on_emergency_stop_clicked(self):
       """
       UI -> main 으로 EMERGENCY STOP 1줄 명령 전송
       포맷: "EMERGENCY,STOP,NONE,NONE"
       """
       if self.ros_node is None:
           self.log_t1("❌ [EMERGENCY] ROS 노드 미연동")
           return

       # 큐/작업 여부와 무관하게 "긴급정지"는 우선 전송(요청대로 단순)
       self._set_busy_reason("EMERGENCY STOP 요청")
       self.show_busy_popup("🛑 긴급정지 요청 전송")
       self.log_t1(f"🛑 [EMERGENCY] 전송: {self.CMD_EMERGENCY_STOP}")

       ok = self.ros_node.send_tube_command_line(self.CMD_EMERGENCY_STOP)
       if not ok:
           self.log_t1("❌ [EMERGENCY] /tube_main_control 서버 연결 실패")

   def on_home_clicked(self):
       """
       UI -> main 으로 HOME 1줄 명령 전송
       포맷: "HOME,NONE,NONE,NONE"
       """
       if self.ros_node is None:
           self.log_t1("❌ [HOME] ROS 노드 미연동")
           return

       self._set_busy_reason("HOME 복귀 요청")
       self.show_busy_popup("🏠 홈 복귀 요청 전송")
       self.log_t1(f"🏠 [HOME] 전송: {self.CMD_HOME}")

       ok = self.ros_node.send_tube_command_line(self.CMD_HOME)
       if not ok:
           self.log_t1("❌ [HOME] /tube_main_control 서버 연결 실패")
   # =========================
   # [ADDED: EMERGENCY STOP / HOME] END (2026-01-25)
   # =========================


   def on_tube_clicked(self, checked, tube_id, btn_obj):
       if self.is_item_blocked(tube_id):
           btn_obj.setChecked(False)
           self.log_t1(f"⛔ [경고] {tube_id} 위치는 선택할 수 없습니다.")
           return


       mode_id = self.t1_mode_group.checkedId()


       # [수정] 경고창 로직 추가
       if mode_id == 1 and tube_id in self.inventory: # 입고인데 이미 있음
           btn_obj.setChecked(False)
           QMessageBox.warning(self, "경고", "튜브가 이미 존재합니다.")
           return
       if mode_id in (2, 4) and tube_id not in self.inventory: # 출고/폐기인데 없음
           btn_obj.setChecked(False)
           QMessageBox.warning(self, "경고", "튜브가 존재하지 않습니다.")
           return


       if mode_id == 3:
           if not checked:
               if self.le_t1_selected.text() == tube_id:
                   self.le_t1_selected.clear()
               elif self.le_t1_dest.text() == tube_id:
                   self.le_t1_dest.clear()
               self.t1_active_buttons.discard(btn_obj)
           else:
               self.t1_active_buttons.add(btn_obj)
               if not self.le_t1_selected.text():
                   self.le_t1_selected.setText(tube_id)
               elif not self.le_t1_dest.text():
                   self.le_t1_dest.setText(tube_id)
               else:
                   btn_obj.setChecked(False)
                   self.t1_active_buttons.discard(btn_obj)
                   self.log_t1("⚠️ [안내] 이동은 2개(sel/dest)만 선택 가능합니다.")
           return


       target_set = self.t1_dest_items if mode_id == 1 else self.t1_selected_items
       if checked:
           target_set.add(tube_id)
           self.t1_active_buttons.add(btn_obj)
       else:
           target_set.discard(tube_id)
           self.t1_active_buttons.discard(btn_obj)


       self.update_text_fields_t1()


   def on_rack_clicked(self, checked, rack_id, btn_obj):
       if self.is_item_blocked(rack_id):
           btn_obj.setChecked(False)
           self.log_t2(f"⛔ [경고] {rack_id} 렉은 선택할 수 없습니다.")
           return


       mode_id = self.t2_mode_group.checkedId()
      
       # [수정] 버튼 선택 시 유효성 검사 로직 개선
       if checked:
           # 1. 입고 (반드시 빈 곳이어야 함)
           if mode_id == 1:
               if rack_id in self.inventory:
                   btn_obj.setChecked(False)
                   QMessageBox.warning(self, "경고", "이미 렉이 존재하는 위치입니다.")
                   return


           # 2. 출고 (반드시 렉이 있어야 함)
           elif mode_id == 2:
               if rack_id not in self.inventory:
                   btn_obj.setChecked(False)
                   QMessageBox.warning(self, "경고", "렉이 존재하지 않는 위치입니다.")
                   return


           # 3. 이동 (출발지는 있어야 하고, 목적지는 비어있어야 함)
           elif mode_id == 3:
               # 현재 출발지가 선택되어 있는지 확인
               current_source = self.le_t2_selected.text()


               if not current_source:
                   # 출발지 선택 단계 -> 렉이 있어야 함
                   if rack_id not in self.inventory:
                       btn_obj.setChecked(False)
                       QMessageBox.warning(self, "경고", "이동할 렉(출발지)이 없습니다.")
                       return
               else:
                   # 목적지 선택 단계 -> 렉이 없어야 함 (비어있어야 함)
                   if rack_id in self.inventory:
                       btn_obj.setChecked(False)
                       QMessageBox.warning(self, "경고", "목적지에 이미 렉이 있습니다.\n비어있는 위치를 선택해주세요.")
                       return


       # --- 이하 기존 선택 처리 로직 ---
       if mode_id == 3:
           if not checked:
               # 체크 해제 로직
               if self.le_t2_selected.text() == rack_id:
                   self.le_t2_selected.clear()
               elif self.le_t2_dest.text() == rack_id:
                   self.le_t2_dest.clear()
               self.t2_active_buttons.discard(btn_obj)
           else:
               # 체크 설정 로직
               self.t2_active_buttons.add(btn_obj)
               if not self.le_t2_selected.text():
                   self.le_t2_selected.setText(rack_id)
               else:
                   self.le_t2_dest.setText(rack_id)
           return


       # 입고/출고 모드 처리
       target_set = self.t2_dest_items if mode_id == 1 else self.t2_selected_items
       if checked:
           target_set.add(rack_id)
           self.t2_active_buttons.add(btn_obj)
       else:
           target_set.discard(rack_id)
           self.t2_active_buttons.discard(btn_obj)


       self.update_text_fields_t2()


   def on_confirm_t1(self):
       if self._is_busy_global():
           self.show_busy_popup("현재 작업이 끝난 후 다시 시도하세요.")
           return


       mode_id = self.t1_mode_group.checkedId()
       sel_list = list(self.t1_selected_items)
       dest_list = list(self.t1_dest_items)


       if mode_id == 3:
           src = self.le_t1_selected.text().strip()
           dst = self.le_t1_dest.text().strip()
           if not src or not dst:
               self.log_t1("[경고] 이동: 대상/목적지 필요")
               return
           sel_list = [src]
           dest_list = [dst]
       else:
           if mode_id == 1 and not dest_list:
               self.log_t1("[경고] 입고: 목적지 필요")
               return
           if mode_id in (2, 4) and not sel_list:
               self.log_t1("[경고] 출고/폐기: 대상 필요")
               return


       jobs = []
       try:
           if mode_id == 1:
               for dst in sorted(dest_list):
                   jobs.append((1, [], [dst], f"TUBE,IN,NONE,{dst}"))
           elif mode_id == 2:
               for src in sorted(sel_list):
                   jobs.append((2, [src], [], f"TUBE,OUT,{src},NONE"))
           elif mode_id == 3:
               src = sel_list[0]
               dst = dest_list[0]
               jobs.append((3, [src], [dst], f"TUBE,MOVE,{src},{dst}"))
           elif mode_id == 4:
               for src in sorted(sel_list):
                   jobs.append((4, [src], [], f"TUBE,WASTE,{src},NONE"))
           else:
               self.log_t1("[경고] 알 수 없는 모드")
               return
       except Exception as e:
           self.log_t1(f"❌ [parse] {e}")
           return


       if not jobs:
           self.log_t1("[경고] 실행할 작업이 없습니다.")
           return


       self._tube_job_queue = jobs
       self._tube_job_running = False


       for btn in list(self.t1_active_buttons):
           try:
               btn.setChecked(False)
           except Exception:
               pass
       self.t1_active_buttons.clear()
       self.t1_selected_items.clear()
       self.t1_dest_items.clear()
       self.le_t1_selected.clear()
       self.le_t1_dest.clear()
       self.le_t1_input.clear()


       self._start_next_tube_job()


   def on_confirm_t2(self):
       if self._is_busy_global():
           self.show_busy_popup("현재 작업이 끝난 후 다시 시도하세요.")
           return


       mode_id = self.t2_mode_group.checkedId()
       sel_list = list(self.t2_selected_items)
       dest_list = list(self.t2_dest_items)


       if mode_id == 3:
           src = self.le_t2_selected.text().strip()
           dst = self.le_t2_dest.text().strip()
           if not src or not dst:
               self.log_t2("[경고] 이동: 대상/목적지 필요")
               return
           sel_list = [src]
           dest_list = [dst]
       else:
           if mode_id == 1 and not dest_list:
               self.log_t2("[경고] 입고: 목적지 필요")
               return
           if mode_id == 2 and not sel_list:
               self.log_t2("[경고] 출고: 대상 필요")
               return


       if self.ros_node is None:
           self.log_t2("❌ [Action] ROS 노드 미연동")
           return


       if mode_id == 1:
           dst = sorted(dest_list)[0]
           ok = self.ros_node.send_rack_command("IN", "NONE", dst)
       elif mode_id == 2:
           src = sorted(sel_list)[0]
           ok = self.ros_node.send_rack_command("OUT", src, "NONE")
       else:
           ok = self.ros_node.send_rack_command("MOVE", sel_list[0], dest_list[0])


       if not ok:
           return


       self._rack_job_running = True
       self._set_busy_reason(self._format_busy_reason_rack(mode_id, sel_list, dest_list))
       self._pending_rack_change = (mode_id, sel_list, dest_list)


       # 작업 대상 점등 시작
       targets = set(sel_list) | set(dest_list)
       self.start_blinking(targets)


       # [수정] 렉 버튼 선택 상태 초기화
       for btn in list(self.t2_active_buttons):
           btn.setChecked(False)
       self.t2_active_buttons.clear()
       self.t2_selected_items.clear()
       self.t2_dest_items.clear()
       self.le_t2_selected.clear()
       self.le_t2_dest.clear()
       self.le_t2_input.clear()


   def _format_busy_reason_rack(self, mode_id, src_list, dest_list):
       if mode_id == 1:
           return f"Rack IN -> {dest_list}"
       elif mode_id == 2:
           return f"Rack OUT <- {src_list}"
       elif mode_id == 3:
           return f"Rack MOVE {src_list} -> {dest_list}"
       return "Rack Action"


   def create_rack_widget(self, storage_name, rack_idx, mode="tube"):
       frame = QFrame()
       frame.setProperty("class", "RackFrame")
       frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
       layout = QVBoxLayout(frame)
       layout.setSpacing(5)
       layout.setContentsMargins(5, 5, 5, 5)


       title = f"{storage_name}-{rack_idx}"
       # is_blocked = self.is_item_blocked(title) # blocked 로직 제거


       if mode == "tube":
           lbl = QLabel(title)
           lbl.setStyleSheet("color: #FFFFFF; font-size: 18px; font-weight: bold;")
           lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
           layout.addWidget(lbl)
           layout.addStretch(1)


           for i in range(1, 5):
               btn = QToolButton()
               btn_id = f"{title}-{i}"
               self.widget_map[btn_id] = (btn, "tube")
              
               # 초기 스타일 설정
               if btn_id in self.inventory:
                   cls = "TubeBtnOccupied"
               else:
                   cls = "TubeBtn"
               btn.setProperty("class", cls)
               btn.setCheckable(True)
               btn.setFixedSize(52, 52)
               btn.setCursor(Qt.CursorShape.PointingHandCursor)
               btn.clicked.connect(lambda checked, bid=btn_id, b_obj=btn: self.on_tube_clicked(checked, bid, b_obj))
               layout.addWidget(btn, alignment=Qt.AlignmentFlag.AlignCenter)
           layout.addStretch(1)


       else:
           btn_sel = QPushButton(title)
           self.widget_map[title] = (btn_sel, "rack")
          
           # 초기 스타일 설정
           if title in self.inventory:
               cls = "RackSelectBtnOccupied"
           else:
               cls = "RackSelectBtn"
           btn_sel.setProperty("class", cls)
           btn_sel.setCheckable(True)
           btn_sel.setCursor(Qt.CursorShape.PointingHandCursor)
           btn_sel.clicked.connect(lambda checked, rid=title, b_obj=btn_sel: self.on_rack_clicked(checked, rid, b_obj))
           layout.addWidget(btn_sel)
           layout.addStretch(1)


           for _ in range(1, 5):
               ind = QLabel()
               ind.setFixedSize(36, 36)
               ind.setStyleSheet("background-color: #64748B; border-radius: 6px;")
               layout.addWidget(ind, alignment=Qt.AlignmentFlag.AlignCenter)
           layout.addStretch(1)


       return frame


   def create_storage_grid(self, mode="tube"):
       scroll = QScrollArea()
       scroll.setWidgetResizable(True)
       scroll.setFrameShape(QFrame.Shape.NoFrame)
       content = QWidget()
       grid = QGridLayout(content)
       grid.setSpacing(20)
       grid.setContentsMargins(10, 10, 10, 10)
       grid.setRowStretch(0, 1)
       grid.setRowStretch(1, 1)
       grid.setColumnStretch(0, 1)
       grid.setColumnStretch(1, 1)


       layout_map = [("C", 0, 0), ("D", 0, 1), ("A", 1, 0), ("B", 1, 1)]
       for name, r, c in layout_map:
           group = QGroupBox(f"Storage {name}")
           hbox = QHBoxLayout(group)
           hbox.setSpacing(10)
           hbox.setContentsMargins(10, 25, 10, 10)
           for i in range(1, 4):
               hbox.addWidget(self.create_rack_widget(name, i, mode))
           grid.addWidget(group, r, c)
       scroll.setWidget(content)
       return scroll


   def create_right_panel(self, title, items, is_tube=True):
       panel = QFrame()
       panel.setMinimumWidth(300)
       panel.setStyleSheet("background-color: #FFFFFF; border-left: 1px solid #E2E8F0;")
       vbox = QVBoxLayout(panel)
       vbox.setContentsMargins(15, 15, 15, 15)
       vbox.setSpacing(10)


       lbl = QLabel(title)
       lbl.setStyleSheet("font-size: 18px; font-weight: bold; color: #1E293B;")
       vbox.addWidget(lbl)


       line = QFrame()
       line.setFrameShape(QFrame.Shape.HLine)
       line.setFrameShadow(QFrame.Shadow.Sunken)
       vbox.addWidget(line)


       grp = QGroupBox("작업 모드")
       v_r = QVBoxLayout(grp)
       v_r.setContentsMargins(10, 15, 10, 10)


       group_obj = self.t1_mode_group if is_tube else self.t2_mode_group
       for i, txt in enumerate(items, 1):
           rb = QRadioButton(txt)
           group_obj.addButton(rb, i)
           if "폐기" in txt:
               rb.setStyleSheet("color: #EF4444; font-weight: bold;")
           if i == 1:
               rb.setChecked(True)
           v_r.addWidget(rb)
       vbox.addWidget(grp)


       form = QFormLayout()
       form.setVerticalSpacing(10)


       le_in = QLineEdit()
       le_in.setPlaceholderText("바코드...")
       le_sel = QLineEdit()
       le_sel.setReadOnly(True)
       le_dest = QLineEdit()
       le_dest.setReadOnly(True)


       form.addRow("바코드 :", le_in)
       form.addRow("선택 객체 :", le_sel)
       form.addRow("목적지 :", le_dest)
       vbox.addLayout(form)


       if is_tube:
           self.le_t1_input = le_in
           self.le_t1_selected = le_sel
           self.le_t1_dest = le_dest
       else:
           self.le_t2_input = le_in
           self.le_t2_selected = le_sel
           self.le_t2_dest = le_dest


       h_btn = QHBoxLayout()
       btn_ok = QPushButton("확인")
       btn_ok.setObjectName("btnConfirm")
       btn_ok.setCursor(Qt.CursorShape.PointingHandCursor)
       btn_cancel = QPushButton("취소")
       btn_cancel.setCursor(Qt.CursorShape.PointingHandCursor)
       btn_ok.clicked.connect(self.on_confirm_t1 if is_tube else self.on_confirm_t2)
       btn_cancel.clicked.connect(self.reset_selection_t1 if is_tube else self.reset_selection_t2)
       h_btn.addWidget(btn_ok)
       h_btn.addWidget(btn_cancel)
       vbox.addLayout(h_btn)


       btn_reset = QPushButton("초기화")
       btn_reset.setCursor(Qt.CursorShape.PointingHandCursor)
       btn_reset.clicked.connect(self.reset_selection_t1 if is_tube else self.reset_selection_t2)
       vbox.addWidget(btn_reset)

       # =========================
       # [ADDED: EMERGENCY STOP / HOME] START (2026-01-25)
       # - 기존 팝업/로그 구조 그대로 재사용
       # - 버튼은 "튜브/렉 패널" 어디서든 접근 가능하도록 양쪽 패널에 동일하게 추가
       # =========================
       btn_home = QPushButton("HOME")
       btn_home.setCursor(Qt.CursorShape.PointingHandCursor)
       btn_home.clicked.connect(self.on_home_clicked)
       vbox.addWidget(btn_home)

       btn_estop = QPushButton("EMERGENCY STOP")
       btn_estop.setCursor(Qt.CursorShape.PointingHandCursor)
       btn_estop.setStyleSheet("background-color: #FECACA; border: 2px solid #EF4444; color: #7F1D1D; font-weight: bold;")
       btn_estop.clicked.connect(self.on_emergency_stop_clicked)
       vbox.addWidget(btn_estop)
       # =========================
       # [ADDED: EMERGENCY STOP / HOME] END (2026-01-25)
       # =========================


       grp_log = QGroupBox("로그 (History)")
       grp_log.setFixedHeight(180)
       v_l = QVBoxLayout(grp_log)
       v_l.setContentsMargins(5, 15, 5, 5)
       txt = QTextEdit()
       txt.setReadOnly(True)
       txt.setText("[System] Ready...")
       v_l.addWidget(txt)
       vbox.addWidget(grp_log)
       vbox.addStretch(1)


       if is_tube:
           self.txt_log_t1 = txt
       else:
           self.txt_log_t2 = txt


       return panel


   def setup_tab1(self):
       tab = QWidget()
       layout = QHBoxLayout(tab)
       layout.setContentsMargins(0, 0, 0, 0)
       layout.addWidget(self.create_storage_grid(mode="tube"), stretch=7)
       layout.addWidget(self.create_right_panel("검체 제어 패널", ["입고", "출고", "이동", "폐기"], True), stretch=3)
       self.tabs.addTab(tab, "튜브 관리")


   def setup_tab2(self):
       tab = QWidget()
       layout = QHBoxLayout(tab)
       layout.setContentsMargins(0, 0, 0, 0)
       layout.addWidget(self.create_storage_grid(mode="rack"), stretch=7)
       layout.addWidget(self.create_right_panel("렉(Rack) 제어 패널", ["렉 입고", "렉 출고", "렉 이동"], False), stretch=3)
       self.tabs.addTab(tab, "렉 관리")




def main(args=None):
   if args is None:
       args = sys.argv
   try:
       from rclpy.utilities import remove_ros_args
       qt_argv = remove_ros_args(args)
   except Exception:
       qt_argv = list(args)


   rclpy.init(args=args)


   app = QApplication(qt_argv)
   window = BioBankApp()
   window.showMaximized()


   ros_node = UiActionClientNode(window)
   window.set_ros_node(ros_node)


   timer = QTimer()
   timer.setInterval(10)
   timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.0))
   timer.start()


   try:
       exit_code = app.exec()
   finally:
       try:
           ros_node.destroy_node()
       except Exception:
           pass
       try:
           rclpy.shutdown()
       except Exception:
           pass


   return exit_code




if __name__ == "__main__":
   raise SystemExit(main())
