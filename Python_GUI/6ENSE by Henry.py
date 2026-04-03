import threading
import queue
import time
import math
import re
import asyncio
import flet as ft
import serial
from serial.tools import list_ports

# ================= CONSTANTS & CONFIG =================
APP_TITLE = "6ENSE by Henry"
BAUD_DEFAULT = 9600

# Step Degrees Default
STEP_GRIP = 5
STEP_LIFT = 5
STEP_BASE = 5
STEP_NECK = 5

SERVO_LIMITS = {1:[0,180], 2:[0,180], 3:[0,180], 4:[0,180]}

# [จุดที่แก้ 1] ใช้ค่ามุมจริง (Physical) ตรงๆ เพื่อให้สอดคล้องกับปุ่มกดแบบใหม่
# ค่านี้มาจาก Code Arduino ที่คุณใช้ล่าสุด (S2=110, S4=160)
HOME_ANGLES  = {1:80, 2:70, 3:90, 4:25}

# Config defaults (ปรับค่าเริ่มต้นให้ตรงกับ Home ใหม่)
S2_UP_DEFAULT, S2_DOWN_DEFAULT = 110, 160 
S4_UP_DEFAULT, S4_DOWN_DEFAULT = 160, 40

MICRO_STEP_DEG = 1
MICRO_DELAY_MS = 20
BASE_STEP_DEG  = 3
BASE_DELAY_MS  = 25
BASE_POST_SLEEP_S = 0.25
MANUAL_DELAY_MS = 100 # ดีเลย์สำหรับปุ่มกดค้าง (ms)

SUPPRESS_PATTERNS = [
    re.compile(r'(?i)use:\s*<\s*servo\s*>\s*<\s*angle\s*>'),
    re.compile(r'(?i)<\s*idx\s*>'),
    re.compile(r'(?i)<\s*angle\s*>|<\s*angel\s*>'),
    re.compile(r'(?i)delay\s*<\s*sec\s*>'),
]

# ================= UTILS =================
def clamp(v, lo, hi): return max(lo, min(hi, v))
def ceil_div(a, b):   return int(math.ceil(a / float(b)))
def format_mmss(sec: int) -> str:
    m = sec // 60; s = sec % 60
    return f"{m:02d}:{s:02d}"

# ================= SERIAL WORKER =================
class SerialWorker(threading.Thread):
    def __init__(self, log_func, line_queue):
        super().__init__(daemon=True)
        self.cmd_q = queue.Queue()
        self.stop_event = threading.Event()
        self.ser = None
        self.log = log_func
        self.line_queue = line_queue

    def open(self, port, baud):
        try:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.1)
            self.log(f"[INFO] Connected to {port} @ {baud}")
            return True
        except Exception as e:
            self.ser = None
            self.log(f"[ERR] {e}")
            return False

    def close(self):
        if self.ser:
            try: self.ser.close()
            except: pass
            self.log("[INFO] Disconnected")
        self.ser = None

    def enqueue(self, text_line): self.cmd_q.put(text_line)

    def run(self):
        while not self.stop_event.is_set():
            if self.ser and self.ser.in_waiting:
                try:
                    line = self.ser.readline().decode(errors="ignore").strip()
                    if line:
                        try: self.line_queue.put_nowait(line)
                        except queue.Full: pass
                except Exception as e:
                    self.log(f"[READ ERR] {e}")

            try:    cmd = self.cmd_q.get_nowait()
            except queue.Empty: cmd = None

            if cmd and self.ser:
                try:
                    if not cmd.endswith("\n"): cmd += "\n"
                    self.ser.write(cmd.encode("utf-8"))
                    self.log(f">> {cmd.strip()}")
                except Exception as e:
                    self.log(f"[WRITE ERR] {e}")
            time.sleep(0.01)
        self.close()

# ================= MAIN FLET APP =================
class RobotApp:
    def __init__(self, page: ft.Page):
        self.page = page
        self.page.title = APP_TITLE
        self.page.theme_mode = ft.ThemeMode.LIGHT
        self.page.window_width = self.page.window_height = None
        self.page.window_maximized = True

        self._hold_started = False
        self._hold_start_time = None
        self.HOLD_THRESHOLD = 0.25  # 0.25 วินาที

        # --- Variables ---
        self.connected = False
        self.current_angles = HOME_ANGLES.copy()
        self.serial_line_q = queue.Queue(maxsize=500)
        
        self.S2_UP, self.S2_DOWN = S2_UP_DEFAULT, S2_DOWN_DEFAULT
        self.S4_UP, self.S4_DOWN = S4_UP_DEFAULT, S4_DOWN_DEFAULT
        
        self.step_base = STEP_BASE
        self.step_lift = STEP_LIFT
        self.step_neck = STEP_NECK
        self.step_grip = STEP_GRIP

        # (เพิ่ม) ตัวแปร invert servo
        self.invert_s1 = ft.Checkbox(label="Invert S1")
        self.invert_s2 = ft.Checkbox(label="Invert S2")
        self.invert_s3 = ft.Checkbox(label="Invert S3")
        self.invert_s4 = ft.Checkbox(label="Invert S4")
        
        # [จุดที่แก้ 2] ตั้งเป็น False ทั้งหมด (เพราะเราแก้ปุ่มกดให้เป็น + แล้ว)
        self.is_inverted = {1: False, 2: False, 3: False, 4: False}

        self.start_cfg_saved = None
        self.play_thread = None
        self.abort_flag = threading.Event()
        self._s1_allow_once = False
        
        # (เพิ่ม) ตัวแปรสำหรับปุ่มกดค้าง
        self._hold_cb = None

        # (เพิ่ม) ตัวแปรสำหรับ Dynamic Delay
        self.dynamic_delay_container = ft.Column(
            scroll=ft.ScrollMode.AUTO, 
            height=150 
        )
        self.delay_text_fields = []  # List สำหรับเก็บ <ft.TextField>
        self.beaker_delays = [5, 5, 5, 5] # ค่าเริ่มต้น (สำหรับ 4 บีกเกอร์)

        # UI References
        self.log_lv = ft.ListView(expand=True, spacing=2, auto_scroll=True)
        self.port_dd = ft.Dropdown(width=200, hint_text="Select Port")
        self.baud_dd = ft.Dropdown(width=120, value=str(BAUD_DEFAULT), options=[ft.dropdown.Option("9600"), ft.dropdown.Option("115200")])
        
        self.btn_connect = ft.ElevatedButton("Connect", on_click=self.handle_connect, icon=ft.Icons.USB)
        self.status_txt = ft.Text("Status: Disconnected", color="red", weight="bold")
        
        # Progress UI
        self.prog_bar = ft.ProgressBar(value=0, color="green", visible=False)
        self.prog_text = ft.Text("", size=12)
        self._start_epoch = None
        self._total_time_est = 0

        self.sliders = {}
        self.slider_txts = {}

        # Init Worker
        self.serial_worker = SerialWorker(self.append_log, self.serial_line_q)
        self.serial_worker.start()

        # Build UI
        self._build_ui()
        
        # (เพิ่ม) สร้าง UI 4 ช่องเริ่มต้น
        self._update_delay_fields(4)
        
        self.refresh_ports()
        
        # Start UI Loop
        self.page.run_task(self._ui_loop)

    # --- UI Builders ---
    def _create_spinbox(self, label, min_val, max_val, default_val, ref_variable_name=None, on_change_func=None):
        txt_val = ft.TextField(value=str(default_val), width=50, text_align="center", read_only=True)
        if ref_variable_name: setattr(self, ref_variable_name, txt_val)

        def adjust(delta):
            cur = int(txt_val.value)
            new_val = clamp(cur + delta, min_val, max_val)
            txt_val.value = str(new_val)
            if on_change_func: on_change_func(new_val)
            txt_val.update()

        return ft.Column([
            ft.Text(label, size=12),
            ft.Row([
                ft.IconButton(ft.Icons.REMOVE, on_click=lambda _: adjust(-1), icon_size=16),
                txt_val,
                ft.IconButton(ft.Icons.ADD, on_click=lambda _: adjust(1), icon_size=16),
            ], spacing=0)
        ], alignment=ft.MainAxisAlignment.CENTER, horizontal_alignment=ft.CrossAxisAlignment.CENTER)

    # (อัปเกรด) ฟังก์ชันช่วยสร้างปุ่มกดค้าง (เวอร์ชันมีแอนิเมชั่น + สีตัวอักษร)
    def _create_hold_button(self, text: str, func_to_run, width, height, size=16, color="blue100", color_pressed="blue200", text_color="primary"):
        return ft.GestureDetector(
            on_tap_down=lambda e, f=func_to_run: self._on_tap_down_handler(e, f),
            on_tap_up=self._on_tap_up_handler,
            on_long_press_end=self._on_tap_up_handler,
            on_pan_end=self._on_tap_up_handler,
            content=ft.Container(
                content=ft.Text(value=text, size=size, color=text_color, weight="bold"),
                width=width, height=height, alignment=ft.alignment.center, bgcolor=color, border_radius=8,
                scale=ft.Scale(1.0), animate_scale=ft.Animation(100, ft.AnimationCurve.EASE_OUT),
                on_hover=lambda e: (
                    setattr(e.control, 'bgcolor', color_pressed if e.data == 'true' else color),
                    setattr(e.control, 'scale', ft.Scale(1.05) if e.data == 'true' else ft.Scale(1.0)),
                    e.control.update()
                )
            )
        )

    def save_invert_settings(self, e):
        try:
            # [จุดที่แก้ 3] เอา logic กลับด้านออก (not) เพราะเราใช้ตรรกะปกติแล้ว
            new_states = {
                1: self.invert_s1.value,
                2: self.invert_s2.value,
                3: self.invert_s3.value,
                4: self.invert_s4.value
            }

            log_changes = []

            for i in range(1, 5):
                if self.is_inverted[i] != new_states[i]:
                    # คำนวณกลับค่า
                    old_angle = self.current_angles[i]
                    new_angle = 180 - old_angle
                    self.current_angles[i] = new_angle

                    # อัปเดต UI
                    if i in self.sliders:
                        self.sliders[i].value = new_angle
                        self.sliders[i].update()
                    if i in self.slider_txts:
                        self.slider_txts[i].value = str(new_angle)
                        self.slider_txts[i].update()
                    
                    log_changes.append(f"S{i}: {old_angle}->{new_angle}")

                self.is_inverted[i] = new_states[i]
                self.set_servo(i, self.current_angles[i])

            msg_text = "Invert Settings Saved"
            if log_changes:
                msg_text += f" (Auto Flip: {', '.join(log_changes)})"
            
            sb = ft.SnackBar(content=ft.Text(msg_text))
            self.page.overlay.append(sb)
            sb.open = True
            self.page.update()

        except Exception as err:
            print("Save Invert Error:", err)
            sb = ft.SnackBar(content=ft.Text(f"Error: {err}"), bgcolor="red400")
            self.page.overlay.append(sb)
            sb.open = True
            self.page.update()

    def _on_mode_changed(self, e):
        mode = self.dd_repeat_mode.value
        if mode == "no":
            self.txt_repeat_times.value = "1"     # บังคับเป็น 1
            self.txt_repeat_times.disabled = True # ล็อคช่อง
        else:
            self.txt_repeat_times.disabled = False # ปลดล็อค
        self.txt_repeat_times.update()

    def _build_ui(self):
        # 1. Top Bar
        top_bar = ft.Container(
            content=ft.Row([
                ft.Text("Port:", weight="bold", size= 16),
                self.port_dd,
                ft.IconButton(ft.Icons.REFRESH, on_click=lambda _: self.refresh_ports(), tooltip="Refresh Ports"),
                ft.Text("Baud:", weight="bold", size= 16),
                self.baud_dd,
                self.btn_connect,
                ft.VerticalDivider(),
                self.status_txt
            ], alignment=ft.MainAxisAlignment.START),
            padding=10, bgcolor="#f8f9ff", border_radius=5, border=ft.border.all(width=1, color="#c3c7cf"),
        )
        

        # 2. Left Panel (Controls)
        btn_size = 150
        
        self.txt_step_s1 = ft.TextField(value=str(STEP_GRIP), width=60, text_align="center")
        self.txt_step_s2 = ft.TextField(value=str(STEP_LIFT), width=60, text_align="center")
        self.txt_step_s3 = ft.TextField(value=str(STEP_BASE), width=60, text_align="center")
        self.txt_step_s4 = ft.TextField(value=str(STEP_NECK), width=60, text_align="center")

        control_pad = ft.Container(
            content=ft.Column([
                ft.Text("Manual Control", size=20, weight="bold"),
                ft.Row([
                    ft.Column([ # Left/Right/Base
                        # --- (แก้) เปลี่ยนเป็นปุ่มกดค้าง ---
                        self._create_hold_button("Up (S4)", self.neck_up, width=btn_size*1.2, height=50, size=16, ),
                        ft.Row([
                            self._create_hold_button("Left (S3)", self.turn_left, width=btn_size, height=50, size=16),
                            self._create_hold_button("Right (S3)", self.turn_right, width=btn_size, height=50, size=16),
                        ]),
                        self._create_hold_button("Down (S4)", self.neck_down, width=btn_size*1.2, height=50, size=16),
                    ], horizontal_alignment=ft.CrossAxisAlignment.CENTER),
                    ft.VerticalDivider(width=10),
                    ft.Column([ # Lift / Grip
                         self._create_hold_button("Lift Up (S2)", self.move_lift_up, width=btn_size*1.2, height=50, size=16),
                         ft.Row([
                             self._create_hold_button("Open (S1)", self.grip_ccw, width=btn_size, height=50, size=16),
                             self._create_hold_button("Close (S1)", self.grip_cw, width=btn_size, height=50, size=16),
                         ]),
                         self._create_hold_button("Lift Down (S2)", self.move_lift_down, width=btn_size*1.2, height=50, size=16),    
                    ], horizontal_alignment=ft.CrossAxisAlignment.CENTER)
                ], alignment=ft.MainAxisAlignment.CENTER),
                
                ft.Divider(),
                ft.Text("Step Sizes", size=16, weight="bold"),
                ft.Row(
                    [
                        ft.Column([ft.Text("S1"), self.txt_step_s1], horizontal_alignment=ft.CrossAxisAlignment.CENTER), 
                        ft.Column([ft.Text("S2"), self.txt_step_s2], horizontal_alignment=ft.CrossAxisAlignment.CENTER),
                        ft.Column([ft.Text("S3"), self.txt_step_s3], horizontal_alignment=ft.CrossAxisAlignment.CENTER),
                        ft.Column([ft.Text("S4"), self.txt_step_s4], horizontal_alignment=ft.CrossAxisAlignment.CENTER),
                        ft.ElevatedButton("Save Steps", on_click=self.save_step_sizes, tooltip="Save Steps", icon=ft.Icons.SAVE),
                    ], 
                    alignment=ft.MainAxisAlignment.CENTER, 
                    vertical_alignment=ft.CrossAxisAlignment.END,
                    spacing=15
                ),
                ft.Divider(),
                ft.Text("Invert Servo Direction", size=16, weight="bold"),
                ft.Row([
                    self.invert_s1,
                    self.invert_s2,
                    self.invert_s3,
                    self.invert_s4,
                ft.ElevatedButton("Save Invert", icon=ft.Icons.SAVE, on_click=self.save_invert_settings),
                ], alignment=ft.MainAxisAlignment.CENTER),
            ], spacing=15),
            padding=10
        )

        # Macro / Config Tab
        self.txt_num_beakers = ft.TextField(value="4", width=60, text_align="center")
        self.txt_base_start = ft.TextField(value="0", width=60, text_align="center")
        self.txt_base_end = ft.TextField(value="90", width=60, text_align="center")
        self.txt_base_result = ft.TextField(value="0,30,60,90", expand=True)
                
        self.dd_repeat_mode = ft.Dropdown(
            value="no", 
            options=[
                ft.dropdown.Option("no", "No"),
                ft.dropdown.Option("reverse", "Reverse"),
                ft.dropdown.Option("again", "Again")
            ], 
            width=125,
            on_change=self._on_mode_changed
        )
        self.txt_repeat_times = ft.TextField(value="1", width=60, text_align="center", disabled=True)

        self.txt_cfg_s2_down = ft.TextField(value=str(S2_DOWN_DEFAULT), width=60, text_align="center")
        self.txt_cfg_s4_down = ft.TextField(value=str(S4_DOWN_DEFAULT), width=60, text_align="center")
        self.txt_cfg_s2_up = ft.TextField(value=str(S2_UP_DEFAULT), width=60, text_align="center")
        self.txt_cfg_s4_up = ft.TextField(value=str(S4_UP_DEFAULT), width=60, text_align="center")
        
        self.txt_base_step_deg = ft.TextField(value=str(BASE_STEP_DEG), width=60, text_align="center")
        self.txt_base_delay_ms = ft.TextField(value=str(BASE_DELAY_MS), width=60, text_align="center")
        self.txt_micro_delay = ft.TextField(value=str(MICRO_DELAY_MS), width=60, text_align="center")

        # --- (เพิ่ม) ตัวแปร UI สำหรับ Sync ---
        self.txt_min_s2 = ft.TextField(value="1", width=60, text_align="center")
        self.txt_min_s4 = ft.TextField(value="0", width=60, text_align="center")
        self.sync_preview_text = ft.Text(value="(Not calculated yet)", italic=True, size=12)
        # -----------------------------------
        # ... (โค้ด self.txt_micro_delay) ...
        self.txt_micro_delay = ft.TextField(value=str(MICRO_DELAY_MS), width=60, text_align="center")

        # --- (เพิ่ม) UI Calibrate Min/Max/Home ---
        self.cal_lim = {} # Dict สำหรับเก็บ TextField min/max
        self.cal_home = {} # Dict สำหรับเก็บ TextField home
        for i in range(1, 5):
            # Min
            self.cal_lim[(i, 'min')] = ft.TextField(value=str(SERVO_LIMITS[i][0]), width=60, text_align="center")
            # Max
            self.cal_lim[(i, 'max')] = ft.TextField(value=str(SERVO_LIMITS[i][1]), width=60, text_align="center")
            # Home
            self.cal_home[i] = ft.TextField(value=str(HOME_ANGLES[i]), width=60, text_align="center")
        # ----------------------------------------

        # ...

        tab_config = ft.Container(
            content=ft.Column([
                ft.Text("Sequence Configuration", size=20, weight="bold"),
                # Beakers
                ft.Row([
                     ft.Text("Beakers:"), self.txt_num_beakers,
                     ft.Text("Start°:"), self.txt_base_start,
                     ft.Text("End°:"), self.txt_base_end,
                     ft.ElevatedButton("Calc", on_click=self._calc_bases, tooltip="คำนวณองศา")
                ], alignment=ft.MainAxisAlignment.START),
                ft.Row([ft.Text("Result:"), self.txt_base_result]),
                
                # --- (เพิ่ม) ส่วน UI Delay ใหม่ ---
                ft.Divider(),
                ft.Text("Delay per Beaker (s):", weight="bold", size=16),
                self.dynamic_delay_container, # <--- คอนเทนเนอร์สำหรับช่องกรอก
                ft.ElevatedButton(
                    "Save Delays",
                    icon=ft.Icons.SAVE_AS_OUTLINED,
                    on_click=self.save_delays, # <--- ฟังก์ชันใหม่
                    tooltip="Save individual delay values"
                ),
                # ----------------------------------------

                ft.Divider(),
                # Up Settings
                ft.Row([
                    ft.Text("Lift (Up):    "),
                    ft.Text("S2"), self.txt_cfg_s2_up,
                    ft.Text("S4"), self.txt_cfg_s4_up,
                    ft.IconButton(ft.Icons.DOWNLOAD, tooltip="ดึงค่าปัจจุบัน", on_click=lambda _: self.pull_up_values()),
                    ft.IconButton(ft.Icons.PLAY_ARROW, tooltip="เทสยก", on_click=lambda _: self.test_raise()),
                ]),
                                
                # Dip Settings
                ft.Row([
                    ft.Text("Dip (Down):"),
                    ft.Text("S2"), self.txt_cfg_s2_down,
                    ft.Text("S4"), self.txt_cfg_s4_down,
                    ft.IconButton(ft.Icons.DOWNLOAD, tooltip="ดึงค่าปัจจุบัน", on_click=lambda _: self.pull_dip_values()),
                    ft.IconButton(ft.Icons.PLAY_ARROW, tooltip="เทสจุ่ม", on_click=lambda _: self.test_dip()),
                ]),

                # --- (เพิ่ม) แถวสำหรับตั้งค่า Sync ---
                ft.Row([
                    ft.Text("Sync Min Step (Fill only one field):"),
                    ft.Text("S2"), self.txt_min_s2,
                    ft.Text("S4"), self.txt_min_s4,
                ]),
                ft.Row([
                    ft.ElevatedButton("Calc Sync", on_click=self.compute_pair_steps_preview, icon=ft.Icons.CALCULATE_OUTLINED),
                    self.sync_preview_text
                ], alignment=ft.MainAxisAlignment.START, vertical_alignment=ft.CrossAxisAlignment.CENTER),
                # ------------------------------------

                 ft.Divider(),
                 # Repeat
                 ft.Row([
                     ft.Text("Mode:"), self.dd_repeat_mode,
                     ft.Text("Times:"), self.txt_repeat_times,
                 ]),
                 # (แก้) ใช้ expand=True
                 ft.ElevatedButton("SAVE CONFIG & READY", on_click=self._save_config_start, width=200, bgcolor="green200", icon=ft.Icons.SAVE_AS)

            ], 
            scroll=ft.ScrollMode.AUTO,
            expand=True  # (แก้) เพิ่ม expand=True ให้ Column
            ),
            padding=10
        )

        tab_calib = ft.Container(
            content=ft.Column(
                [
                    ft.Text("Calibration & Fine Tune", size=20, weight="bold"),
                    ft.Divider(),
                    # --- (เพิ่ม) UI Grid สำหรับ Min/Max/Home ---
                    ft.Text("Servo Limits & Home", size=16, weight="bold"),
                ] + [ # สร้าง Row S1-S4 อัตโนมัติ
                    ft.Row(
                        [
                            ft.Text(f"S{i}", width=30),
                            ft.Text("Min:", width=40), self.cal_lim[(i, 'min')],
                            ft.Text("Max:", width=40), self.cal_lim[(i, 'max')],
                            ft.VerticalDivider(width=20),
                            ft.Text("Home:", width=50), self.cal_home[i],
                        ],
                        vertical_alignment=ft.CrossAxisAlignment.CENTER
                    ) for i in range(1, 5)
                ] + [ # เพิ่มส่วนที่เหลือ
                    # ---------------------------------------------
                    
                    ft.Divider(),
                    ft.Text("Speed Settings", size=16, weight="bold"),
                    ft.Row([ft.Text("S3 Step (deg):", width=150), self.txt_base_step_deg]),
                    ft.Row([ft.Text("S3 Delay (ms):", width=150), self.txt_base_delay_ms]),
                    ft.Row([ft.Text("Micro Delay (ms):", width=150), self.txt_micro_delay]),
                    ft.ElevatedButton("Apply Calibrate", on_click=self.apply_calibrate, expand=True, bgcolor="green200", icon=ft.Icons.AUTORENEW),
                    ft.Text("Note: Q3JlYXRlZCBieSBQaHV3aXMgV2FubmFwcmFwYQ==", size=12, italic=True)
                ], 
                expand=True,
                scroll=ft.ScrollMode.AUTO # (เพิ่ม) เผื่อหน้าจอเล็ก
            ), 
            padding=10
        )

        tabs_control = ft.Tabs(
            selected_index=0,
            animation_duration=300,
            tabs=[
                ft.Tab(
                    tab_content=ft.Text(value="Controls", size=16),
                    content=control_pad
                ),
                ft.Tab(
                    tab_content=ft.Text(value="Config Start", size=16),
                    content=tab_config
                ),
                ft.Tab(
                    tab_content=ft.Text(value="Calibrate", size=16),
                    content=tab_calib
                ),
            ],
            expand=True
        )

        # 3. Right Panel (Status & Log)
        sliders_col = ft.Column()
        for i in range(1, 5):
            lbl_val = ft.Text(str(HOME_ANGLES[i]), width=40, weight="bold", size=20)
            self.slider_txts[i] = lbl_val
            sl = ft.Slider(min=SERVO_LIMITS[i][0], max=SERVO_LIMITS[i][1], value=HOME_ANGLES[i], 
                           label="{value}", expand=True, 
                           on_change_end=lambda e, idx=i: self.set_servo_to_slider(idx, e.control.value))
            self.sliders[i] = sl
            sliders_col.controls.append(
                ft.Row([ft.Text(f"S{i}", weight="bold", size=20), sl, lbl_val])
            )

        actions = ft.Row([
            ft.FilledButton("START", on_click=lambda _: self.on_start_clicked(), icon=ft.Icons.PLAY_CIRCLE),
            ft.OutlinedButton("STOP", on_click=lambda _: self.on_stop_clicked(), icon=ft.Icons.STOP),
            # [จุดที่แก้ 3] ปุ่ม HOME เรียกฟังก์ชัน smooth
            ft.ElevatedButton("HOME", on_click=lambda _: self.on_go_home_clicked(), icon=ft.Icons.HOME),
            ft.ElevatedButton("RESET", on_click=lambda _: self.send_cmd("reset"), color="red", icon=ft.Icons.RESET_TV_SHARP),
        ], alignment=ft.MainAxisAlignment.CENTER)

        right_panel = ft.Container(
            content=ft.Column([
                ft.Text("Live Status", size=26, weight="bold"),
                sliders_col,
                ft.Divider(),
                actions,
                self.prog_bar,
                self.prog_text,
                ft.Divider(),
                ft.Row(
                    controls=[
                        ft.Text("Log Output:", weight="bold"),
                        ft.IconButton(icon=ft.Icons.DELETE_SWEEP_OUTLINED, tooltip="ล้างข้อมูล log", on_click=self.clear_log, icon_size=18)
                    ],
                    # (แก้) จัดปุ่มล้าง Log
                    alignment=ft.MainAxisAlignment.SPACE_BETWEEN,
                    vertical_alignment=ft.CrossAxisAlignment.CENTER
                ),
                
                ft.Container(content=self.log_lv, border=ft.border.all(1, "grey300"), border_radius=5, expand=True, padding=5)
            ]),
            padding=10, expand=True
        )

        # Main Layout
        self.page.add(
            top_bar,
            ft.Row([
                ft.Container(content=tabs_control, width=700, padding=5),
                ft.VerticalDivider(width=1),
                right_panel
            ], expand=True)
        )

    # --- LOGIC & HANDLERS ---
    
    # --- (เพิ่ม) Handlers สำหรับแอนิเมชั่นปุ่มกดค้าง ---
    def _on_tap_down_handler(self, e, func_to_run):
        e.control.content.scale = ft.Scale(0.95)
        e.control.update()

        self._hold_cb = func_to_run
        self._hold_started = False
        self._hold_start_time = time.time()

        self.page.run_task(self._hold_tick)


    def _on_tap_up_handler(self, e):
        # reset UI
        if e.control.content:
            e.control.content.scale = ft.Scale(1.0)
            e.control.update()

        # ตรวจว่าเป็น single click (ยังไม่ hold)
        if self._hold_cb and not self._hold_started:
            self._hold_cb()  # <-- เรียกครั้งเดียว

        # clear
        self._hold_cb = None
        self._hold_started = False

    # --- (เพิ่ม) LOGIC สำหรับการกดค้าง (Hold-to-Repeat) ---
    async def _hold_tick(self):
        if not self._hold_cb:
            return

        now = time.time()
        
        # ถ้ายังไม่เริ่ม hold-loop → เช็กก่อน
        if not self._hold_started:
            if now - self._hold_start_time >= self.HOLD_THRESHOLD:
                self._hold_started = True  # เริ่ม loop
            else:
                await asyncio.sleep(0.02)
                self.page.run_task(self._hold_tick)
                return

        # เรียกซ้ำเมื่อเป็นโหมด hold แล้ว
        self._hold_cb()
        await asyncio.sleep(MANUAL_DELAY_MS / 1000)
        self.page.run_task(self._hold_tick)

    def _on_hold_press(self, func_to_run):
        """
        ถูกเรียกโดย on_tap_down ของปุ่ม
        """
        self._hold_cb = func_to_run
        # (แก้) ไม่ต้องเรียก func() ซ้ำตรงนี้ เพราะ _on_tap_down_handler เรียกให้แล้ว
        # เริ่มลูป _hold_tick
        self.page.run_task(self._hold_tick)

    def _on_hold_release(self, e):
        """
        ถูกเรียกโดย on_tap_up, on_long_press_end ฯฯ
        """
        # หยุดลูปโดยการล้าง callback
        self._hold_cb = None
    # --- จบส่วน Hold-to-Repeat ---

    async def _ui_loop(self):
        while True:
            try:
                while True:
                    line = self.serial_line_q.get_nowait()
            except queue.Empty:
                pass
            
            if self.play_thread and self.play_thread.is_alive() and self._start_epoch:
                elapsed = time.time() - self._start_epoch
                left = max(0, self._total_time_est - elapsed)
                if self._total_time_est > 0:
                    pct = min(1.0, elapsed / self._total_time_est)
                    self.prog_bar.value = pct
                    self.prog_text.value = f"Time Left: {format_mmss(int(left))} | {pct*100:.1f}%"
                    self.prog_bar.update()
                    self.prog_text.update()
            
            await asyncio.sleep(0.1)

    def append_log(self, text):
        for pat in SUPPRESS_PATTERNS:
            if pat.search(text): return
        self.log_lv.controls.append(ft.Text(text, size=12, font_family="Consolas"))
        if len(self.log_lv.controls) > 100: 
            self.log_lv.controls.pop(0)
        self.log_lv.update()
    
    def clear_log(self, e):
        self.log_lv.controls.clear()
        self.log_lv.update()

    def refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.port_dd.options = [ft.dropdown.Option(p) for p in ports]
        if ports: self.port_dd.value = ports[0]
        self.port_dd.update()

    # --- (แก้) SnackBar ทั้งหมด ---
    def handle_connect(self, e):
        if self.connected: return
        port = self.port_dd.value
        if not port:
            sb = ft.SnackBar(ft.Text("Please select a port"))
            self.page.overlay.append(sb)
            sb.open = True
            self.page.update()
            return
        
        baud = int(self.baud_dd.value)
        if self.serial_worker.open(port, baud):
            self.append_log(f"Connected to {port}")
            self.connected = True
            self.status_txt.value = f"Connected: {port}"
            self.status_txt.color = "green"
            self.btn_connect.disabled = True
            self.btn_connect.update()
            self.status_txt.update()
        else:
            sb = ft.SnackBar(ft.Text("Connection Failed"), bgcolor="red400")
            self.page.overlay.append(sb)
            sb.open = True
            self.page.update()

    def send_cmd(self, text):
        if not self.connected:
             self.append_log("[WARN] Not connected")
             return
        self.serial_worker.enqueue(text)

    def _send_s1_guarded(self, angle):
        """ Bypasses the S1 guard just once to move the gripper. """
        self._s1_allow_once = True
        try:
            self._set_servo_core(1, angle)
        finally:
            self._s1_allow_once = False
    
    def _set_servo_core(self, idx, angle):
        # 1. UI Logic: แสดงค่าตามที่มนุษย์เข้าใจ (0-180 ปกติ บนหน้าจอ)
        angle = clamp(int(angle), SERVO_LIMITS[idx][0], SERVO_LIMITS[idx][1])
        self.current_angles[idx] = angle
        
        if idx in self.sliders and idx in self.slider_txts:
            self.sliders[idx].value = angle
            self.slider_txts[idx].value = str(angle)
            self.sliders[idx].update()
            self.slider_txts[idx].update()

        # 2. Hardware Logic: ถ้า Invert ให้กลับด้านตัวเลขก่อนส่งไป Arduino
        # เพื่อให้ Servo หมุนไปในทิศที่ถูกต้องตามฟิสิกส์
        final_angle_to_send = angle
        if self.is_inverted.get(idx, False):
            final_angle_to_send = 180 - angle

        self.send_cmd(f"{idx} {final_angle_to_send}")

    def set_servo(self, idx, angle):
        if idx == 1 and not self._s1_allow_once:
            # S1 ถูกล็อกไว้ (เช่น ระหว่าง auto-mode)
            # self.append_log("[WARN] S1 movement is locked.")
            return
        self._set_servo_core(idx, angle)

    def set_servo_to_slider(self, idx, val):
        val = int(round(float(val)))
        if idx == 1:
            self._send_s1_guarded(val) # ปุ่ม Send ของ S1 ต้องใช้ Guard
        else:
            self.set_servo(idx, val)

    # [จุดที่แก้ 4] ปุ่มกด: บังคับบวกลบตรงๆ ไม่อิง sign แล้ว
    def turn_left(self):  
        # S3: -
        self.set_servo(3, self.current_angles[3] - self.step_base)

    def turn_right(self): 
        # S3: +
        self.set_servo(3, self.current_angles[3] + self.step_base)

    def move_lift_up(self):   
        # S2: + (เพิ่มค่า)
        self.set_servo(2, self.current_angles[2] + self.step_lift)

    def move_lift_down(self): 
        # S2: - (ลดค่า)
        self.set_servo(2, self.current_angles[2] - self.step_lift)

    def neck_up(self):   
        # S4: + (เพิ่มค่า)
        self.set_servo(4, self.current_angles[4] + self.step_neck)

    def neck_down(self): 
        # S4: - (ลดค่า)
        self.set_servo(4, self.current_angles[4] - self.step_neck)

    def grip_ccw(self):  
        # S1: -
        self._send_s1_guarded(self.current_angles[1] - self.step_grip)

    def grip_cw(self):   
        # S1: +
        self._send_s1_guarded(self.current_angles[1] + self.step_grip)

    def save_step_sizes(self, e):
        try:
            self.step_grip = int(self.txt_step_s1.value)
            self.step_lift = int(self.txt_step_s2.value)
            self.step_base = int(self.txt_step_s3.value)
            self.step_neck = int(self.txt_step_s4.value)
            
            sb = ft.SnackBar(ft.Text("Steps Saved!"))
            self.page.overlay.append(sb)
            sb.open = True
            self.page.update()
        except:
            sb = ft.SnackBar(
                ft.Text("Invalid Step Values"),
                bgcolor="red400"
            )
            self.page.overlay.append(sb)
            sb.open = True
            self.page.update()
            
    # --- (เพิ่ม) ฟังก์ชัน Dynamic Delay ---
    def _update_delay_fields(self, beaker_count: int):
        """ล้างและสร้างช่องกรอก Delay ตามจำนวนบีกเกอร์"""
        self.dynamic_delay_container.controls.clear()
        self.delay_text_fields.clear()
        
        old_delays = self.beaker_delays if self.beaker_delays else []
        new_delays = []
        for i in range(beaker_count):
            default_val = str(old_delays[i]) if i < len(old_delays) else "5"
            
            label = ft.Text(f"Beaker {i+1} Delay:", width=110, text_align="LEFT")
            txt_field = ft.TextField(
                value=default_val, 
                width=60, 
                text_align="center"
            )
            
            self.delay_text_fields.append(txt_field)
            self.dynamic_delay_container.controls.append(
                ft.Row([label, txt_field], alignment=ft.MainAxisAlignment.START)
            )
            try: new_delays.append(int(default_val))
            except: new_delays.append(5)
            
        self.beaker_delays = new_delays
        self.dynamic_delay_container.update()

    def save_delays(self, e):
        """บันทึกค่าจาก TextField ลงใน List self.beaker_delays"""
        self.beaker_delays = []
        try:
            for i, txt_field in enumerate(self.delay_text_fields):
                delay_val = int(txt_field.value)
                self.beaker_delays.append(max(0, delay_val))
            
            sb = ft.SnackBar(ft.Text(f"Saved {len(self.beaker_delays)} delay values."))
            self.page.overlay.append(sb)
            sb.open = True
            self.page.update()

        except Exception as ex:
            sb = ft.SnackBar(ft.Text(f"Error: Invalid delay value."), bgcolor="red400")
            self.page.overlay.append(sb)
            sb.open = True
            self.page.update()

    # --- (เพิ่ม) ฟังก์ชัน "สมอง" สำหรับ Sync ---
    def _compute_rounds_for_pair(self, d2, d4):
        """
        [THE BRAIN] คำนวณ 'รอบ' ที่เหมาะสมที่สุดสำหรับการเคลื่อนที่แบบ Smooth
        จะคืนค่า (rounds, error_message)
        """
        min_s2 = 0
        min_s4 = 0
        try:
            # (แก้) อ่านค่าจาก Flet TextField
            min_s2 = int(self.txt_min_s2.value)
            min_s4 = int(self.txt_min_s4.value)
            if min_s2 < 0 or min_s4 < 0:
                 return (None, "Error: Min Step cannot be negative")
        except:
             return (None, "Error: Min Step cannot contain decimals")

        # --- (เพิ่ม) Logic การแจ้งเตือนตามที่คุณขอ ---
        # if min_s2 > 0 and min_s4 > 0:
        #     return (None, "Error: Please fill in only one field (S2 or S4)")

        if d2 == 0 and d4 == 0: return (0, None)
        g = math.gcd(d2, d4) if d2 and d4 else max(d2, d4)
        
        rounds = max(1, g)
        if min_s2 > 0:
            rounds = max(rounds, ceil_div(d2, max(1, min_s2)))
        elif min_s4 > 0:
            rounds = max(rounds, ceil_div(d4, max(1, min_s4)))
        return (max(1, rounds), None) # คืนค่าสำเร็จ

    # --- (เพิ่ม) ฟังก์ชัน "พรีวิว" สำหรับ Sync ---
    # --- (อัปเกรด) ฟังก์ชัน "พรีวิว" สำหรับ Sync (แก้กลับเป็นไทย) ---
    def compute_pair_steps_preview(self, e):
        """
        [THE PREVIEW] ถูกเรียกโดยปุ่ม 'คำนวณผล Sync'
        """
        try:
            s2_up = int(self.txt_cfg_s2_up.value)
            s4_up = int(self.txt_cfg_s4_up.value)
            s2_dn = int(self.txt_cfg_s2_down.value)
            s4_dn = int(self.txt_cfg_s4_down.value)
        except:
            # --- (แก้) เปลี่ยนกลับเป็นภาษาไทย ---
            err_msg = "Error: Please set the Up/Down values correctly"
            self.sync_preview_text.value = err_msg
            self.sync_preview_text.color = "red"
            
            sb = ft.SnackBar(ft.Text(err_msg), bgcolor="red400")
            self.page.overlay.append(sb)
            sb.open = True
            self.page.update()
            return

        d2 = abs(s2_dn - s2_up)
        d4 = abs(s4_dn - s4_up)

        if d2 == 0 and d4 == 0:
            # --- (แก้) เปลี่ยนกลับเป็นภาษาไทย ---
            self.sync_preview_text.value = "No movement on S2/S4"
            self.sync_preview_text.color = "black"
            self.sync_preview_text.update()
            return

        # เรียก "สมอง" มาคำนวณ
        rounds, err = self._compute_rounds_for_pair(d2, d4)
        
        if err:
            # ถ้ามี Error (เช่น กรอก 2 ช่อง)
            self.sync_preview_text.value = err
            self.sync_preview_text.color = "red"

            sb = ft.SnackBar(ft.Text(err), bgcolor="red400")
            self.page.overlay.append(sb)
            sb.open = True
        else:
            # ถ้าสำเร็จ
            step2_avg = d2 / float(rounds)
            step4_avg = d4 / float(rounds)
            # --- (แก้) เปลี่ยนกลับเป็นภาษาไทย (เหมือนใน Tkinter) ---
            result_text = f"Round={rounds} | S2≈{step2_avg:.2f}°/Round, S4≈{step4_avg:.2f}°/Round"
            self.sync_preview_text.value = result_text
            self.sync_preview_text.color = "black"

            # --- (แก้) เปลี่ยนกลับเป็นภาษาไทย ---
            sb = ft.SnackBar(ft.Text(f"Calc Sync Result: {result_text}"))
            self.page.overlay.append(sb)
            sb.open = True
            # --------------------------------------
        
        self.sync_preview_text.update()
        self.page.update() # (เพิ่ม) อัปเดต page เพื่อแสดง SnackBar

    def _calc_bases(self, e):
        try:
            n = int(self.txt_num_beakers.value)
            a0 = int(self.txt_base_start.value)
            a1 = int(self.txt_base_end.value)
            if n == 1: lst = [a0]
            else:
                step = (a1 - a0) / float(n - 1)
                lst = [int(round(a0 + i * step)) for i in range(n)]
            self.txt_base_result.value = ",".join(map(str, lst))
            self.txt_base_result.update()
            
            # (เพิ่ม) สั่งอัปเดต UI Delay
            self._update_delay_fields(n)
            sb = ft.SnackBar(ft.Text(f"Generated {n} delay fields."))
            self.page.overlay.append(sb)
            sb.open = True
            self.page.update()

        except: 
            sb = ft.SnackBar(ft.Text("Error calculating bases"), bgcolor="red400")
            self.page.overlay.append(sb)
            sb.open = True
            self.page.update()

    def pull_dip_values(self):
        self.txt_cfg_s2_down.value = str(self.current_angles[2])
        self.txt_cfg_s4_down.value = str(self.current_angles[4])
        self.page.update()

    def pull_up_values(self):
        self.txt_cfg_s2_up.value = str(self.current_angles[2])
        self.txt_cfg_s4_up.value = str(self.current_angles[4])
        self.page.update()

    def test_dip(self):
        s2 = int(self.txt_cfg_s2_down.value)
        s4 = int(self.txt_cfg_s4_down.value)
        self._micro_move_pair(s2, s4)

    def test_raise(self):
        s2 = int(self.txt_cfg_s2_up.value)
        s4 = int(self.txt_cfg_s4_up.value)
        self._micro_move_pair(s2, s4)

    def apply_calibrate(self, e):
        # (เพิ่ม) ประกาศ global เพื่อให้ค่าอัปเดตไปทั่วโปรแกรม
        global SERVO_LIMITS, HOME_ANGLES, BASE_STEP_DEG, BASE_DELAY_MS, MICRO_DELAY_MS
        
        try:
            # --- 1. Limits ---
            for i in range(1,5):
                lo = int(self.cal_lim[(i,'min')].value) # .get() -> .value
                hi = int(self.cal_lim[(i,'max')].value) # .get() -> .value
                if lo > hi: lo, hi = hi, lo
                lo = max(0, lo); hi = min(180, hi)
                SERVO_LIMITS[i] = [lo, hi]
                
                # --- Update Sliders (Flet way) ---
                self.sliders[i].min = lo    # .configure(from_=lo) -> .min
                self.sliders[i].max = hi    # .configure(to=hi) -> .max
                
                # Clamp ค่าปัจจุบันของ slider ให้อยู่ใน Limit ใหม่
                current_slider_val = self.sliders[i].value
                if not (lo <= current_slider_val <= hi):
                    self.sliders[i].value = max(lo, min(hi, current_slider_val))
                
                self.sliders[i].update()
                # ---------------------------------

                # --- Enforce Limits (ถ้า Servo อยู่นอกกรอบ) ---
                cur_angle = self.current_angles[i]
                if not (lo <= cur_angle <= hi):
                    clamped_val = max(lo, min(hi, cur_angle))
                    if i == 1:
                        self._send_s1_guarded(clamped_val)
                    else:
                        self.set_servo(i, clamped_val)

            # --- 2. Home ---
            for i in range(1,5):
                home_val = int(self.cal_home[i].value) # .get() -> .value
                # Clamp ค่า Home ให้อยู่ใน Limits ใหม่
                HOME_ANGLES[i] = max(SERVO_LIMITS[i][0], min(SERVO_LIMITS[i][1], home_val))
                self.cal_home[i].value = str(HOME_ANGLES[i]) # อัปเดต UI กลับ
                self.cal_home[i].update()


            # --- 3. S3/Micro Speed ---
            BASE_STEP_DEG = int(self.txt_base_step_deg.value)
            BASE_DELAY_MS = int(self.txt_base_delay_ms.value)
            MICRO_DELAY_MS = int(self.txt_micro_delay.value)

            log_msg = f"[CAL] Limits={SERVO_LIMITS} | Home={HOME_ANGLES} | S3(step={BASE_STEP_DEG}, delay={BASE_DELAY_MS}ms)"
            self.append_log(log_msg)
            
            # --- SnackBar (Flet new way) ---
            sb = ft.SnackBar(ft.Text("Calibration Applied!"))
            self.page.overlay.append(sb)
            sb.open = True
            self.page.update()
            
        except Exception as ex:
            self.append_log(f"[CAL ERR] {ex}")
            sb = ft.SnackBar(ft.Text(f"Error applying calibration: {ex}"), bgcolor="red400")
            self.page.overlay.append(sb)
            sb.open = True
            self.page.update()

    def _save_config_start(self, e):
        bases = [int(x) for x in self.txt_base_result.value.split(",") if x]
        
        # (เพิ่ม) ตรวจสอบว่าจำนวน Delay กับ Beaker ตรงกันไหม
        if not self.beaker_delays or len(self.beaker_delays) != len(bases):
            sb = ft.SnackBar(
                ft.Text("Error: Delay/Beaker count mismatch. Press 'Calc' and 'Save Delays' first."),
                bgcolor="red400"
            )
            self.page.overlay.append(sb)
            sb.open = True
            self.page.update()
            return

        self.start_cfg_saved = {
            "bases": bases,
            "repeat_mode": self.dd_repeat_mode.value,
            "repeat_times": int(self.txt_repeat_times.value),
            
            # (แก้) เปลี่ยนไปใช้ List Delay
            "delays_list": self.beaker_delays,
            
            "S2_UP": int(self.txt_cfg_s2_up.value),
            "S4_UP": int(self.txt_cfg_s4_up.value),
            "S2_DOWN": int(self.txt_cfg_s2_down.value),
            "S4_DOWN": int(self.txt_cfg_s4_down.value),
        }
        
        # ------------------------------------------------------------
        # (แก้) คำนวณเวลาใหม่ (ให้รองรับ Reverse และ Again)
        # ------------------------------------------------------------
        # 1. คำนวณเวลาต่อ "1 เที่ยว" (Forward) = เวลาแช่รวม + (เวลาย้าย 5วิ x จำนวนบีกเกอร์)
        one_pass_time = sum(self.beaker_delays) + (len(bases) * 5) 
        
        mode = self.dd_repeat_mode.value
        times = int(self.txt_repeat_times.value)

        if mode == "reverse":
            # Reverse = ไป + กลับ (คิดเป็น 2 เที่ยว ต่อ 1 รอบการทำซ้ำ)
            self._total_time_est = one_pass_time * 2 * times
        elif mode == "again":
            # Again = ทำซ้ำทางเดิม (1 เที่ยว ต่อ 1 รอบการทำซ้ำ)
            self._total_time_est = one_pass_time * times
        else: 
            # No = ทำเที่ยวเดียวจบ
            self._total_time_est = one_pass_time
        
        # ------------------------------------------------------------
        
        sb = ft.SnackBar(ft.Text("Config Saved & Ready"))
        self.page.overlay.append(sb)
        sb.open = True
        self.page.update()

    def on_start_clicked(self):
        if not self.start_cfg_saved:
            sb = ft.SnackBar(ft.Text("Please Save Config first"))
            self.page.overlay.append(sb)
            sb.open = True
            self.page.update()
            return
            
        self.abort_flag.clear()
        self.prog_bar.visible = True
        self._start_epoch = time.time()
        self.play_thread = threading.Thread(target=self._run_sequence, daemon=True)
        self.play_thread.start()
        self.page.update()

    def on_stop_clicked(self):
        self.abort_flag.set()
        self.send_cmd("stop")
        self.prog_bar.visible = False
        self.prog_text.value = "Stopped"
        self.page.update()
        
    # [จุดที่แก้ 3] ปุ่ม HOME: เช็คก่อนว่ามี thread วิ่งไหม ถ้าไม่มีให้สร้างใหม่เพื่อรัน smooth home
    def on_go_home_clicked(self):
        # เช็คว่าลำดับหลักทำงานอยู่ไหม
        if self.play_thread and self.play_thread.is_alive():
             self.append_log("[WARN] Sequence running, stop first.")
             return
        
        # --- [เพิ่มบรรทัดนี้] ---
        self.abort_flag.clear() # ล้างสถานะ Stop เก่าทิ้งก่อน ไม่งั้นมันจะไม่ขยับ
        # ---------------------

        # สร้าง Thread ชั่วคราวเพื่อรัน Home แบบ Smooth (เพราะต้องใช้ time.sleep)
        threading.Thread(target=self._run_smooth_home, daemon=True).start()

    # [ฟังก์ชันใหม่] กลับบ้านแบบนุ่มนวล (Smooth Home)
    def _run_smooth_home(self):
        self.append_log(">>> GOING HOME (Smoothly)...")
        
        # 1. ยกแขนขึ้น/เก็บคอ (S2, S4) แบบ Smooth Pair
        # ใช้ค่า Home ที่ตั้งไว้ในตัวแปร
        self._micro_move_pair(HOME_ANGLES[2], HOME_ANGLES[4])
        
        # 2. หมุนฐาน (S3) กลับมาตรงกลาง แบบ Smooth Single
        self._micro_move_single(3, HOME_ANGLES[3])
        
        # 3. ปรับกริป (S1)
        self._send_s1_guarded(HOME_ANGLES[1])
        
        self.append_log("[INFO] Arrived at Home.")

    # [จุดที่ 3] Run Sequence: รองรับ Mode + ลำดับถูกต้อง + Pre-Move + Smooth Stop + 100% Fix
    def _run_sequence(self):
        cfg = self.start_cfg_saved
        
        # 1. สร้างรายการงานพื้นฐาน 1 รอบ (Forward)
        base_pass = list(zip(cfg["bases"], cfg["delays_list"]))
        
        # 2. ขยายรายการงานตาม Mode และ Times
        full_jobs = []
        mode = cfg["repeat_mode"]
        times = cfg["repeat_times"]

        if mode == "no":
            full_jobs = base_pass
        elif mode == "again":
            for _ in range(times):
                full_jobs.extend(base_pass)
        elif mode == "reverse":
            for _ in range(times):
                full_jobs.extend(base_pass)          # ขาไป
                full_jobs.extend(base_pass[::-1])    # ขากลับ

        self.append_log(">>> STARTED SEQUENCE")
        self.append_log("[INFO] Moving to Start Position...")
        
        # ยกหัวขึ้นก่อนเริ่มงาน (Pre-Move)
        self._micro_move_pair(cfg["S2_UP"], cfg["S4_UP"])
        time.sleep(0.5)

        # เริ่มวนลูปตามรายการงานทั้งหมด
        for i, (base, delay_s) in enumerate(full_jobs):
            if self.abort_flag.is_set(): break
            
            self.append_log(f"--- Step {i+1} (Base: {base}°, Delay: {delay_s}s) ---")
            
            # 1. หมุนฐานไป (Move Base)
            self._micro_move_single(3, base)
            time.sleep(BASE_POST_SLEEP_S)
            if self.abort_flag.is_set(): break
            
            # 2. จุ่มลง (Dip DOWN)
            self._micro_move_pair(cfg["S2_DOWN"], cfg["S4_DOWN"])
            if self.abort_flag.is_set(): break
            
            # -------------------------------------------------------
            # 3. แช่ (Delay/Soak)
            # -------------------------------------------------------
            if delay_s > 0:
                self.append_log(f"[INFO] Soaking for {delay_s} seconds...")
                for _ in range(int(delay_s * 10)):
                    if self.abort_flag.is_set(): break
                    time.sleep(0.1)
            
            if self.abort_flag.is_set(): break
            
            # 4. ยกขึ้น (Lift UP)
            self._micro_move_pair(cfg["S2_UP"], cfg["S4_UP"])
            if self.abort_flag.is_set(): break

        # -------------------------------------------------------
        # สรุปผลและจัดการ Progress Bar
        # -------------------------------------------------------
        
        # [สำคัญ] หยุดตัวจับเวลา UI ทันที เพื่อป้องกัน Race Condition
        self._start_epoch = None 

        if self.abort_flag.is_set(): 
            self.append_log(">>> SEQUENCE ABORTED")
            self.prog_text.value = "Aborted!"
            self.page.update()
            self.abort_flag.clear() # เคลียร์ธงเพื่อให้ Home ทำงานต่อได้
        else: 
            self.append_log(">>> SEQUENCE FINISHED")
            
            # --- บังคับให้หลอดเต็ม 100% โชว์ความเท่ ---
            self.prog_bar.value = 1.0
            self.prog_text.value = "Completed! | 100%"
            self.page.update()     # สั่งอัปเดตหน้าจอทันที
            time.sleep(0.5)        # หน่วงเวลา 0.5 วิ ให้คนได้เชยชม 100%
            # -------------------------------------
        
        # กลับบ้านแบบนุ่มนวล
        self._run_smooth_home()

    def _micro_move_single(self, idx, target):
        cur = self.current_angles[idx]
        step = max(1, int(self.txt_base_step_deg.value)) if idx == 3 else MICRO_STEP_DEG
        delay_ms = max(1, int(self.txt_base_delay_ms.value)) if idx == 3 else MICRO_DELAY_MS
        delay_s = delay_ms / 1000.0
        
        if cur == target: return

        while cur != target:
            if self.abort_flag.is_set(): return
            diff = target - cur
            if abs(diff) < step: cur = target
            else: cur += step if diff > 0 else -step
            self.set_servo(idx, cur)
            time.sleep(delay_s)

    # --- (อัปเกรด) ฟังก์ชันขยับคู่แบบ Smooth ---
    def _micro_move_pair(self, s2_target, s4_target):
        s2 = int(self.current_angles[2]); s4 = int(self.current_angles[4])
        d2 = abs(s2_target - s2); d4 = abs(s4_target - s4)
        
        wait_ms = max(1, int(self.txt_micro_delay.value))
        wait_s = wait_ms / 1000.0

        if d2 == 0 and d4 == 0:
            return
        if d2 == 0:
            self._micro_move_single(4, s4_target)
            return
        if d4 == 0:
            self._micro_move_single(2, s2_target)
            return

        # --- (แก้) เรียก "สมอง" มาคำนวณ ---
        rounds, err = self._compute_rounds_for_pair(d2, d4)
        
        if err:
            # ถ้าผู้ใช้ตั้งค่าผิด (เช่น กรอก 2 ช่อง) ให้ Log และขยับแบบไม่ Smooth (ปลอดภัย)
            self.append_log(f"[SMOOTH ERR] {err}")
            self.append_log("[WARN] กลับไปขยับแบบทีละแกน")
            self._micro_move_single(2, s2_target)
            if self.abort_flag.is_set(): return
            self._micro_move_single(4, s4_target)
            return

        # --- Logic การขยับแบบ Smooth (เส้นทแยงมุม) ---
        dir2 = 1 if s2_target > s2 else -1
        dir4 = 1 if s4_target > s4 else -1

        for k in range(1, rounds + 1):
            if self.abort_flag.is_set(): return
            
            next2 = s2 + dir2 * int(round(k * d2 / float(rounds)))
            next4 = s4 + dir4 * int(round(k * d4 / float(rounds)))
            
            # สั่งขยับพร้อมกัน
            self.set_servo(2, next2)
            self.set_servo(4, next4)
            time.sleep(wait_s)

# ================= RUN =================
def main(page: ft.Page):
    app = RobotApp(page)

# (แก้) รันในโหมด Web Browser อัตโนมัติ
#ft.app(target=main, view=ft.AppView.WEB_BROWSER)
ft.app(target=main)