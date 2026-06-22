"""
TAP-FUSION Calibrator - KivyMD (Android-ready)
================================================
Port dari PyQt5 ke KivyMD.

Padanan pola threading:
    PyQt5                    ->  KivyMD
    QThread + pyqtSignal     ->  threading.Thread + Clock.schedule_once
    @pyqtSlot                ->  fungsi biasa dipanggil lewat Clock
    ser.read() blocking      ->  tetap blocking, TAPI di background thread

Logika Modbus RTU (CRC16, frame builder, parser) identik dengan versi PyQt5.

Install (desktop):
    pip install kivy==2.2.1 kivymd==1.1.1 pyserial

Build APK:
    Lihat buildozer.spec dan .github/workflows/build.yml
"""

import struct
import threading
from datetime import datetime

from kivy.clock import Clock
from kivy.lang import Builder
from kivy.metrics import dp
from kivy.properties import BooleanProperty, ListProperty, StringProperty
from kivy.uix.widget import Widget
from kivy.animation import Animation

from kivymd.app import MDApp
from kivymd.uix.label import MDLabel
from kivymd.uix.snackbar import Snackbar

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


# ─────────────────────────────────────────────────────────────
#  Konstanta Modbus (identik dengan versi PyQt5)
# ─────────────────────────────────────────────────────────────
MODBUS_SLAVE_ID      = 0x01
FC_WRITE_REGISTER    = 0x06
FC_WRITE_SINGLE_COIL = 0x05
FC_READ_HOLDING      = 0x03
CALIB_REGISTER       = 0x0001
COIL_REGISTER        = 0x000A


# ─────────────────────────────────────────────────────────────
#  Helper Modbus - IDENTIK dengan versi PyQt5, tidak ada perubahan
# ─────────────────────────────────────────────────────────────
def calc_crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return struct.pack('<H', crc)


def build_write_register(slave_id, register, value):
    payload = struct.pack('>BBHH', slave_id, FC_WRITE_REGISTER, register, value)
    return payload + calc_crc16(payload)


def build_write_single_coil(slave_id, register, value):
    payload = struct.pack('>BBHH', slave_id, FC_WRITE_SINGLE_COIL, register, value)
    return payload + calc_crc16(payload)


def build_read_register(slave_id, register, count=1):
    payload = struct.pack('>BBHH', slave_id, FC_READ_HOLDING, register, count)
    return payload + calc_crc16(payload)


def verify_crc(data: bytes) -> bool:
    return len(data) >= 2 and calc_crc16(data[:-2]) == data[-2:]


# ─────────────────────────────────────────────────────────────
#  Warna
# ─────────────────────────────────────────────────────────────
C_BG      = (0.055, 0.067, 0.090, 1)
C_SURFACE = (0.086, 0.106, 0.133, 1)
C_AMBER   = (1.000, 0.690, 0.125, 1)
C_CYAN    = (0.133, 0.827, 0.933, 1)
C_GREEN   = (0.239, 0.863, 0.518, 1)
C_RED     = (1.000, 0.322, 0.322, 1)
C_TEXT    = (0.902, 0.929, 0.953, 1)
C_DIM     = (0.545, 0.580, 0.620, 1)

LOG_COLORS = {
    "INFO":    "#00BFFF",
    "TX":      "#FFD700",
    "RX":      "#90EE90",
    "SUCCESS": "#00FF7F",
    "WARN":    "#FFA500",
    "ERROR":   "#FF4500",
}

MAX_LOG_LINES = 300


# ─────────────────────────────────────────────────────────────
#  Backend Serial
#  Setara dengan ModbusWorker(QObject) di versi PyQt5.
#  Semua method yang blocking HANYA dipanggil dari background thread.
#  Tidak ada satupun widget yang disentuh di sini.
# ─────────────────────────────────────────────────────────────
class SerialBackend:
    def __init__(self):
        self.ser         = None
        self.cancelling  = False
        self.state_pompa = False

    @property
    def is_open(self):
        return self.ser is not None and self.ser.is_open

    # ── Buka koneksi ─────────────────────────────────────────
    def connect(self, port, baudrate):
        """Blocking — panggil HANYA dari background thread."""
        self.cancelling = False
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.5,
            write_timeout=2,
        )

    # ── Tutup koneksi ────────────────────────────────────────
    def disconnect(self):
        """Blocking (cepat) — panggil dari background thread."""
        self.cancelling = True
        try:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.cancel_read()
                except Exception:
                    pass
                self.ser.close()
        finally:
            self.ser         = None
            self.cancelling  = False
            self.state_pompa = False

    # ── Kirim kalibrasi ──────────────────────────────────────
    def send_calibration(self, value):
        """Return (success: bool, logs: list[(msg, level)])"""
        logs = []
        if self.cancelling or not self.is_open:
            return False, [("Serial tidak terbuka.", "WARN")]

        int_value = int(round(value * 100))
        if not (0 <= int_value <= 65535):
            return False, [(f"Nilai {value} di luar jangkauan (0-655.35).", "ERROR")]

        frame = build_write_register(MODBUS_SLAVE_ID, CALIB_REGISTER, int_value)
        try:
            self.ser.reset_input_buffer()
            self.ser.write(frame)
            logs.append((
                f"TX  FC06  Reg=0x{CALIB_REGISTER:04X}  "
                f"Val={int_value} (={value:.2f})  Frame={frame.hex().upper()}", "TX"
            ))

            if self.cancelling:
                return False, logs
            response = self.ser.read(8)
            if self.cancelling:
                return False, logs

            if not response:
                logs.append(("Timeout: tidak ada response.", "WARN"))
                return False, logs
            if len(response) < 8:
                logs.append((f"Response tidak lengkap ({len(response)}/8): "
                             f"{response.hex().upper()}", "WARN"))
                return False, logs
            if not verify_crc(response):
                logs.append(("CRC tidak valid.", "ERROR"))
                return False, logs
            if response[1] & 0x80:
                logs.append((f"Exception Modbus: kode {response[2]:#04x}", "ERROR"))
                return False, logs

            logs.append((f"RX  FC06 OK  Frame={response.hex().upper()}", "RX"))
            logs.append((f"Kalibrasi {value:.2f} berhasil dikirim.", "SUCCESS"))
            return True, logs

        except serial.SerialException as e:
            if not self.cancelling:
                logs.append((f"Serial error: {e}", "ERROR"))
            return False, logs

    # ── Baca kalibrasi ───────────────────────────────────────
    def read_calibration(self):
        """Return (value: float|None, logs: list[(msg, level)])"""
        logs = []
        if self.cancelling or not self.is_open:
            return None, [("Serial tidak terbuka.", "WARN")]

        frame = build_read_register(MODBUS_SLAVE_ID, CALIB_REGISTER, 1)
        try:
            self.ser.reset_input_buffer()
            self.ser.write(frame)
            logs.append((
                f"TX  FC03  Reg=0x{CALIB_REGISTER:04X}  "
                f"Frame={frame.hex().upper()}", "TX"
            ))

            if self.cancelling:
                return None, logs
            response = self.ser.read(7)
            if self.cancelling:
                return None, logs

            if not response:
                logs.append(("Timeout: tidak ada response.", "WARN"))
                return None, logs
            if len(response) < 7:
                logs.append((f"Response tidak lengkap ({len(response)}/7): "
                             f"{response.hex().upper()}", "WARN"))
                return None, logs
            if not verify_crc(response):
                logs.append(("CRC tidak valid.", "ERROR"))
                return None, logs
            if response[1] & 0x80:
                logs.append((f"Exception Modbus: kode {response[2]:#04x}", "ERROR"))
                return None, logs

            raw   = struct.unpack('>H', response[3:5])[0]
            value = raw / 100.0
            logs.append((
                f"RX  FC03 OK  Raw={raw}  Val={value:.2f}  "
                f"Frame={response.hex().upper()}", "RX"
            ))
            return value, logs

        except serial.SerialException as e:
            if not self.cancelling:
                logs.append((f"Serial error: {e}", "ERROR"))
            return None, logs

    # ── Toggle pompa ─────────────────────────────────────────
    def toggle_pump(self):
        """Return (new_state: bool|None, logs: list[(msg, level)])"""
        logs = []
        if self.cancelling or not self.is_open:
            return None, [("Serial tidak terbuka.", "WARN")]

        val   = 0x0000 if self.state_pompa else 0xFF00
        frame = build_write_single_coil(MODBUS_SLAVE_ID, COIL_REGISTER, val)
        try:
            self.ser.reset_input_buffer()
            self.ser.write(frame)
            logs.append((
                f"TX  FC05  Reg=0x{COIL_REGISTER:04X}  "
                f"Frame={frame.hex().upper()}", "TX"
            ))

            if self.cancelling:
                return None, logs
            response = self.ser.read(8)
            if self.cancelling:
                return None, logs

            if not response:
                logs.append(("Timeout: tidak ada response.", "WARN"))
                return None, logs
            if len(response) < 8:
                logs.append((f"Response tidak lengkap ({len(response)}/8): "
                             f"{response.hex().upper()}", "WARN"))
                return None, logs
            if not verify_crc(response):
                logs.append(("CRC tidak valid.", "ERROR"))
                return None, logs
            if response[1] & 0x80:
                logs.append((f"Exception Modbus: kode {response[2]:#04x}", "ERROR"))
                return None, logs

            logs.append((f"RX  FC05 OK  Frame={response.hex().upper()}", "RX"))
            self.state_pompa = not self.state_pompa
            new_state = ((response[4] << 8) | response[5]) == 0xFF00
            return new_state, logs

        except serial.SerialException as e:
            if not self.cancelling:
                logs.append((f"Serial error: {e}", "ERROR"))
            return None, logs


# ─────────────────────────────────────────────────────────────
#  Widget LED indikator
# ─────────────────────────────────────────────────────────────
KV = '''
#:import dp kivy.metrics.dp

<LedDot>:
    canvas:
        Color:
            rgba: self.glow_color
        Ellipse:
            pos: self.x - dp(6), self.y - dp(6)
            size: dp(24), dp(24)
        Color:
            rgba: self.dot_color
        Ellipse:
            pos: self.x, self.y
            size: dp(12), dp(12)
    size_hint: None, None
    size: dp(12), dp(12)


MDScreen:
    md_bg_color: app.bg

    MDBoxLayout:
        orientation: "vertical"

        # ── Header ──────────────────────────────────────────
        MDBoxLayout:
            orientation: "vertical"
            size_hint_y: None
            height: dp(96)
            padding: dp(20), dp(14), dp(20), dp(10)
            spacing: dp(6)
            md_bg_color: app.surface

            MDLabel:
                text: "TAP-FUSION  ·  CALIBRATOR"
                bold: True
                font_style: "H6"
                theme_text_color: "Custom"
                text_color: app.amber
                size_hint_y: None
                height: dp(28)

            MDBoxLayout:
                spacing: dp(10)
                size_hint_y: None
                height: dp(36)

                LedDot:
                    id: led
                    pos_hint: {"center_y": 0.5}
                    dot_color: app.red
                    glow_color: app.red[0], app.red[1], app.red[2], 0.25

                MDLabel:
                    id: status_label
                    text: "Tidak terhubung"
                    bold: True
                    theme_text_color: "Custom"
                    text_color: app.dim

        # ── Konten ──────────────────────────────────────────
        ScrollView:
            MDBoxLayout:
                orientation: "vertical"
                spacing: dp(14)
                padding: dp(16), dp(16), dp(16), dp(24)
                size_hint_y: None
                height: self.minimum_height

                # Panel: Koneksi Serial
                MDBoxLayout:
                    orientation: "vertical"
                    padding: dp(18)
                    spacing: dp(12)
                    size_hint_y: None
                    height: self.minimum_height
                    md_bg_color: app.surface
                    radius: [14, 14, 14, 14]

                    MDLabel:
                        text: "KONEKSI SERIAL"
                        font_style: "Caption"
                        bold: True
                        theme_text_color: "Custom"
                        text_color: app.cyan
                        size_hint_y: None
                        height: dp(20)

                    MDBoxLayout:
                        spacing: dp(8)
                        size_hint_y: None
                        height: dp(48)

                        MDRectangleFlatButton:
                            id: port_btn
                            text: "PILIH PORT"
                            size_hint_x: 0.75
                            line_color: app.dim
                            theme_text_color: "Custom"
                            text_color: app.text
                            on_release: app.open_port_menu(self)

                        MDIconButton:
                            icon: "refresh"
                            theme_text_color: "Custom"
                            text_color: app.cyan
                            on_release: app.refresh_ports()

                    MDBoxLayout:
                        spacing: dp(8)
                        size_hint_y: None
                        height: dp(48)

                        MDRectangleFlatButton:
                            id: baud_btn
                            text: "9600"
                            line_color: app.dim
                            theme_text_color: "Custom"
                            text_color: app.text
                            on_release: app.open_baud_menu(self)

                    MDRectangleFlatIconButton:
                        id: connect_btn
                        text: "HUBUNGKAN"
                        icon: "power-plug-outline"
                        pos_hint: {"center_x": 0.5}
                        line_color: app.amber
                        theme_text_color: "Custom"
                        text_color: app.amber
                        on_release: app.toggle_connection()

                # Panel: Kalibrasi
                MDBoxLayout:
                    orientation: "vertical"
                    padding: dp(18)
                    spacing: dp(12)
                    size_hint_y: None
                    height: self.minimum_height
                    md_bg_color: app.surface
                    radius: [14, 14, 14, 14]

                    MDLabel:
                        text: "KALIBRASI"
                        font_style: "Caption"
                        bold: True
                        theme_text_color: "Custom"
                        text_color: app.cyan
                        size_hint_y: None
                        height: dp(20)

                    MDBoxLayout:
                        size_hint_y: None
                        height: dp(48)

                        MDLabel:
                            text: "Nilai saat ini:"
                            theme_text_color: "Custom"
                            text_color: app.dim
                            size_hint_x: 0.5

                        MDLabel:
                            id: current_val
                            text: "—"
                            bold: True
                            font_style: "H5"
                            halign: "right"
                            theme_text_color: "Custom"
                            text_color: app.cyan

                    MDBoxLayout:
                        spacing: dp(8)
                        size_hint_y: None
                        height: dp(44)

                        MDRectangleFlatIconButton:
                            id: read_btn
                            text: "BACA"
                            icon: "tray-arrow-down"
                            disabled: True
                            line_color: app.cyan
                            theme_text_color: "Custom"
                            text_color: app.cyan
                            size_hint_x: 0.5
                            on_release: app.read_calibration()

                        MDRectangleFlatIconButton:
                            id: pump_btn
                            text: "NYALAKAN POMPA"
                            icon: "engine-outline"
                            disabled: True
                            line_color: app.green
                            theme_text_color: "Custom"
                            text_color: app.green
                            size_hint_x: 0.5
                            on_release: app.toggle_pump()

                    MDTextField:
                        id: calib_input
                        hint_text: "Input Nilai Kalibrasi (0.00 - 655.35)"
                        text: "0.00"
                        input_filter: "float"
                        line_color_normal: app.dim
                        line_color_focus: app.amber
                        hint_text_color_normal: app.dim
                        hint_text_color_focus: app.amber
                        text_color_normal: app.text
                        text_color_focus: app.text

                    MDFillRoundFlatIconButton:
                        id: send_btn
                        text: "KIRIM KALIBRASI"
                        icon: "send"
                        disabled: True
                        pos_hint: {"center_x": 0.5}
                        md_bg_color: app.amber
                        theme_text_color: "Custom"
                        text_color: 0.05, 0.05, 0.05, 1
                        on_release: app.send_calibration()

                # Panel: Log
                MDBoxLayout:
                    orientation: "vertical"
                    padding: dp(18)
                    spacing: dp(10)
                    size_hint_y: None
                    height: dp(300)
                    md_bg_color: app.surface
                    radius: [14, 14, 14, 14]

                    MDBoxLayout:
                        size_hint_y: None
                        height: dp(20)

                        MDLabel:
                            text: "LOG"
                            font_style: "Caption"
                            bold: True
                            theme_text_color: "Custom"
                            text_color: app.cyan

                        MDIconButton:
                            icon: "delete-outline"
                            theme_text_color: "Custom"
                            text_color: app.dim
                            on_release: app.clear_log()

                    ScrollView:
                        id: log_scroll
                        do_scroll_x: False

                        MDBoxLayout:
                            id: log_box
                            orientation: "vertical"
                            spacing: dp(2)
                            padding: dp(4)
                            size_hint_y: None
                            height: self.minimum_height
'''


class LedDot(Widget):
    dot_color  = ListProperty(list(C_RED))
    glow_color = ListProperty([C_RED[0], C_RED[1], C_RED[2], 0.25])


class CalibratorApp(MDApp):
    # ── Properti warna ──────────────────────────────────────
    bg      = ListProperty(list(C_BG))
    surface = ListProperty(list(C_SURFACE))
    amber   = ListProperty(list(C_AMBER))
    cyan    = ListProperty(list(C_CYAN))
    green   = ListProperty(list(C_GREEN))
    red     = ListProperty(list(C_RED))
    text    = ListProperty(list(C_TEXT))
    dim     = ListProperty(list(C_DIM))

    # ── State ────────────────────────────────────────────────
    connected    = BooleanProperty(False)
    busy         = BooleanProperty(False)
    is_connecting = BooleanProperty(False)
    selected_port = StringProperty("")
    selected_baud = StringProperty("9600")

    def build(self):
        self.theme_cls.theme_style   = "Dark"
        self.theme_cls.primary_palette = "Amber"
        self.title   = "TAP-FUSION Calibrator"
        self.backend  = SerialBackend()
        self.port_menu = None
        self.baud_menu = None
        self.screen   = Builder.load_string(KV)
        return self.screen

    def on_start(self):
        self.refresh_ports()

    # ──────────────────────────────────────────────────────────
    #  Util: jalankan kerja blocking di background thread,
    #  kembalikan hasil ke main thread lewat Clock.schedule_once.
    #  Ini padanan langsung QThread + pyqtSignal.emit() di PyQt5.
    # ──────────────────────────────────────────────────────────
    def _run_in_thread(self, work_fn, on_result):
        def runner():
            result = work_fn()
            Clock.schedule_once(lambda dt: on_result(result))
        threading.Thread(target=runner, daemon=True).start()

    # ── Port & Baudrate ──────────────────────────────────────
    def refresh_ports(self):
        if not SERIAL_AVAILABLE:
            self.log("pyserial tidak terinstall.", "ERROR")
            return

        ports = [p.device for p in serial.tools.list_ports.comports()]
        if not ports:
            self.log("Tidak ada port serial ditemukan.", "WARN")
            self._snack("Tidak ada port serial terdeteksi")
            return

        self.log(f"Ditemukan {len(ports)} port: {', '.join(ports)}", "INFO")
        from kivymd.uix.menu import MDDropdownMenu
        self.port_menu = MDDropdownMenu(
            caller=self.screen.ids.port_btn,
            items=[{"text": p, "viewclass": "OneLineListItem",
                    "on_release": lambda x=p: self._set_port(x)}
                   for p in ports],
            width_mult=4,
        )

    def open_port_menu(self, instance):
        if self.port_menu:
            self.port_menu.open()
        else:
            self.refresh_ports()

    def _set_port(self, name):
        self.selected_port = name
        self.screen.ids.port_btn.text = name
        if self.port_menu:
            self.port_menu.dismiss()

    def open_baud_menu(self, instance):
        from kivymd.uix.menu import MDDropdownMenu
        bauds = ["1200", "2400", "4800", "9600", "19200", "38400", "57600", "115200"]
        self.baud_menu = MDDropdownMenu(
            caller=self.screen.ids.baud_btn,
            items=[{"text": b, "viewclass": "OneLineListItem",
                    "on_release": lambda x=b: self._set_baud(x)}
                   for b in bauds],
            width_mult=3,
        )
        self.baud_menu.open()

    def _set_baud(self, baud):
        self.selected_baud = baud
        self.screen.ids.baud_btn.text = baud
        if self.baud_menu:
            self.baud_menu.dismiss()

    # ── Koneksi ──────────────────────────────────────────────
    def toggle_connection(self):
        if self.busy or self.is_connecting:
            return
        if self.connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        if not self.selected_port:
            self._snack("Pilih port terlebih dahulu")
            return

        baudrate = int(self.selected_baud)

        # Update UI segera — feedback instan sebelum proses dimulai
        self.is_connecting = True
        self.screen.ids.connect_btn.text     = "MENGHUBUNGKAN..."
        self.screen.ids.connect_btn.disabled = True
        self.screen.ids.status_label.text       = f"Menghubungkan ke {self.selected_port}..."
        self.screen.ids.status_label.text_color = self.amber
        self._led("connecting")
        self.log(f"Mencoba koneksi ke {self.selected_port}...", "INFO")

        def work():
            try:
                self.backend.connect(self.selected_port, baudrate)
                return True, None
            except Exception as e:
                return False, str(e)

        self._run_in_thread(work, lambda r: self._on_connect(r, baudrate))

    def _on_connect(self, result, baudrate):
        success, error = result
        self.is_connecting = False
        self.screen.ids.connect_btn.disabled = False

        if success:
            self.connected = True
            self.log(f"Terhubung ke {self.selected_port} @ {baudrate} baud", "INFO")
            self.screen.ids.status_label.text       = f"ONLINE  ·  {self.selected_port}  ·  {baudrate} bps"
            self.screen.ids.status_label.text_color = self.green
            self._led("on")
            self.screen.ids.connect_btn.text = "PUTUSKAN"
            self.screen.ids.connect_btn.icon = "power-plug-off-outline"
            self._set_controls(True)
            self._snack("Berhasil terhubung")
            # Otomatis baca kalibrasi saat terhubung — sama seperti versi PyQt5
            self.read_calibration()
        else:
            self.connected = False
            self.log(f"Gagal terhubung: {error}", "ERROR")
            self._led("off")
            self.screen.ids.connect_btn.text       = "HUBUNGKAN"
            self.screen.ids.connect_btn.icon       = "power-plug-outline"
            self.screen.ids.status_label.text       = "Tidak terhubung"
            self.screen.ids.status_label.text_color = self.dim
            self._snack(f"Gagal: {error}")

    def _disconnect(self):
        self.busy = True
        self.screen.ids.connect_btn.disabled = True
        self.log("Memutuskan koneksi...", "INFO")

        self._run_in_thread(
            lambda: (self.backend.disconnect(), True)[1],
            lambda r: self._on_disconnect()
        )

    def _on_disconnect(self):
        self.busy      = False
        self.connected = False
        self.log("Koneksi serial ditutup.", "INFO")
        self._led("off")
        self.screen.ids.status_label.text       = "Tidak terhubung"
        self.screen.ids.status_label.text_color = self.dim
        self.screen.ids.connect_btn.text        = "HUBUNGKAN"
        self.screen.ids.connect_btn.icon        = "power-plug-outline"
        self.screen.ids.connect_btn.disabled    = False
        self.screen.ids.current_val.text        = "—"
        self._set_controls(False)
        self._set_pump_ui(False)

    # ── Baca kalibrasi ───────────────────────────────────────
    def read_calibration(self):
        if self.busy or not self.connected:
            return
        self.busy = True
        self.screen.ids.read_btn.disabled = True

        self._run_in_thread(self.backend.read_calibration, self._on_read)

    def _on_read(self, result):
        value, logs = result
        self._flush_logs(logs)
        self.busy = False
        self.screen.ids.read_btn.disabled = not self.connected
        if value is not None:
            self.screen.ids.current_val.text = f"{value:.2f}"

    # ── Kirim kalibrasi ──────────────────────────────────────
    def send_calibration(self):
        if self.busy or not self.connected:
            return
        try:
            value = float(self.screen.ids.calib_input.text)
        except ValueError:
            self._snack("Nilai kalibrasi tidak valid")
            return

        self.busy = True
        self.screen.ids.send_btn.disabled = True

        self._run_in_thread(
            lambda: self.backend.send_calibration(value),
            self._on_send
        )

    def _on_send(self, result):
        success, logs = result
        self._flush_logs(logs)
        self.busy = False
        self.screen.ids.send_btn.disabled = not self.connected
        if success:
            self._snack("Kalibrasi berhasil dikirim")
            self.read_calibration()

    # ── Toggle pompa ─────────────────────────────────────────
    def toggle_pump(self):
        if self.busy or not self.connected:
            return
        self.busy = True
        self.screen.ids.pump_btn.disabled = True

        self._run_in_thread(self.backend.toggle_pump, self._on_pump)

    def _on_pump(self, result):
        new_state, logs = result
        self._flush_logs(logs)
        self.busy = False
        self.screen.ids.pump_btn.disabled = not self.connected
        if new_state is not None:
            self._set_pump_ui(new_state)

    def _set_pump_ui(self, state):
        btn = self.screen.ids.pump_btn
        if state:
            btn.text       = "MATIKAN POMPA"
            btn.line_color = self.red
            btn.text_color = self.red
        else:
            btn.text       = "NYALAKAN POMPA"
            btn.line_color = self.green
            btn.text_color = self.green

    # ── LED indikator ────────────────────────────────────────
    def _led(self, state):
        led = self.screen.ids.led
        Animation.cancel_all(led)
        led.opacity = 1

        if state == "on":
            led.dot_color  = list(self.green)
            led.glow_color = [*self.green[:3], 0.30]
            anim = (Animation(opacity=0.35, duration=0.7) +
                    Animation(opacity=1.0,  duration=0.7))
            anim.repeat = True
            anim.start(led)
        elif state == "connecting":
            led.dot_color  = list(self.amber)
            led.glow_color = [*self.amber[:3], 0.30]
            anim = (Animation(opacity=0.35, duration=0.4) +
                    Animation(opacity=1.0,  duration=0.4))
            anim.repeat = True
            anim.start(led)
        else:
            led.dot_color  = list(self.red)
            led.glow_color = [*self.red[:3], 0.25]

    # ── Helper UI ────────────────────────────────────────────
    def _set_controls(self, enabled):
        for widget_id in ("read_btn", "send_btn", "pump_btn"):
            self.screen.ids[widget_id].disabled = not enabled

    # ── Log ──────────────────────────────────────────────────
    def _flush_logs(self, logs):
        for msg, level in logs:
            self.log(msg, level)

    def log(self, message, level="INFO"):
        ts    = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        color = LOG_COLORS.get(level, "#D4D4D4")
        text  = (
            f"[color=888888]{ts}[/color]  "
            f"[color={color}][b]{level:<7}[/b][/color]  "
            f"[color=D4D4D4]{message}[/color]"
        )
        label = MDLabel(
            text=text,
            markup=True,
            font_style="Caption",
            size_hint_y=None,
            height=dp(20),
            shorten=False,
        )
        log_box = self.screen.ids.log_box
        log_box.add_widget(label)

        # Batasi jumlah baris agar tidak bocor memori
        while len(log_box.children) > MAX_LOG_LINES:
            log_box.remove_widget(log_box.children[0])

        # Auto-scroll ke bawah
        Clock.schedule_once(
            lambda dt: setattr(self.screen.ids.log_scroll, "scroll_y", 0)
        )

    def clear_log(self):
        self.screen.ids.log_box.clear_widgets()

    def _snack(self, message):
        Snackbar(text=message, size_hint_x=0.9).open()

    # ── Cleanup ──────────────────────────────────────────────
    def on_stop(self):
        if self.backend.is_open:
            self.backend.disconnect()


if __name__ == "__main__":
    CalibratorApp().run()
