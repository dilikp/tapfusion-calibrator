"""
TAP-FUSION Calibrator - Versi KivyMD (Android-ready)
======================================================
Port dari aplikasi PyQt5 desktop ke KivyMD, mempertahankan logika Modbus RTU
mentah (CRC16 manual, frame builder manual) apa adanya - hanya lapisan UI dan
threading yang diganti.

Padanan pola threading:
    PyQt5 (asli)              ->  KivyMD (versi ini)
    -----------------------------------------------------
    QThread + QObject worker  ->  threading.Thread + class SerialBackend
    pyqtSignal.emit()         ->  Clock.schedule_once(callback)
    @pyqtSlot                 ->  fungsi biasa, dipanggil lewat Clock

Aturan inti: SEMUA operasi serial (connect/disconnect/read/write) berjalan
di background thread. UI (widget Kivy) HANYA boleh diubah dari main thread,
makanya setiap hasil dari background thread dikembalikan lewat
Clock.schedule_once() sebelum menyentuh widget manapun.

Install dependency:
    pip install kivy kivymd==1.1.1 pyserial

CATATAN UNTUK BUILD KE ANDROID:
    pyserial TIDAK bisa mengakses USB Host Android secara langsung.
    Untuk APK yang benar-benar membaca USB-to-RS485 di HP, bagian
    `import serial` di SerialBackend perlu diganti dengan `usb4a` /
    `usbserial4a` (lihat catatan di akhir file). Struktur kode ini sengaja
    dipisah ke class SerialBackend supaya penggantian itu cukup dilakukan
    di SATU tempat saja, tanpa mengubah UI.
"""

import struct
import threading
from datetime import datetime

from kivy.clock import Clock
from kivy.lang import Builder
from kivy.metrics import dp
from kivy.properties import BooleanProperty, ListProperty, StringProperty
from kivy.uix.widget import Widget
from kivy.utils import platform

from kivymd.app import MDApp
from kivymd.uix.label import MDLabel
from kivymd.uix.snackbar import Snackbar

IS_ANDROID = (platform == "android")

if IS_ANDROID:
    # --- Backend Android: akses USB lewat USB Host API via pyjnius ---
    try:
        from usb4a import usb
        from usbserial4a import serial4a
        ANDROID_USB_AVAILABLE = True
    except ImportError:
        ANDROID_USB_AVAILABLE = False
    SERIAL_AVAILABLE = ANDROID_USB_AVAILABLE
else:
    # --- Backend Desktop: pyserial biasa (untuk testing di laptop) ---
    try:
        import serial
        import serial.tools.list_ports
        SERIAL_AVAILABLE = True
    except ImportError:
        SERIAL_AVAILABLE = False


# ----------------------------------------------------------------------
# Konstanta Modbus (identik dengan versi PyQt5)
# ----------------------------------------------------------------------
MODBUS_SLAVE_ID = 0x01
FC_WRITE_REGISTER = 0x06
FC_WRITE_SINGLE_COIL = 0x05
FC_READ_HOLDING = 0x03
CALIB_REGISTER = 0x0001
COIL_REGISTER = 0x000A


# ----------------------------------------------------------------------
# Helper Modbus murni Python - tidak ada perubahan dari versi asli
# ----------------------------------------------------------------------
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


# ----------------------------------------------------------------------
# PALET WARNA - selaras dengan aplikasi RS485 Reader sebelumnya
# ----------------------------------------------------------------------
COL_BG = (0.055, 0.067, 0.090, 1)
COL_SURFACE = (0.086, 0.106, 0.133, 1)
COL_AMBER = (1.0, 0.690, 0.125, 1)
COL_CYAN = (0.133, 0.827, 0.933, 1)
COL_GREEN = (0.239, 0.863, 0.518, 1)
COL_RED = (1.0, 0.322, 0.322, 1)
COL_TEXT = (0.902, 0.929, 0.953, 1)
COL_TEXT_DIM = (0.545, 0.580, 0.620, 1)

LEVEL_COLORS = {
    "INFO": "#00BFFF",
    "TX": "#FFD700",
    "RX": "#90EE90",
    "SUCCESS": "#00FF7F",
    "WARN": "#FFA500",
    "ERROR": "#FF4500",
}


# ----------------------------------------------------------------------
# SerialBackend
# ------------------------------------------------------------------
# Setara dengan ModbusWorker (QObject) di versi PyQt5, TAPI di sini bukan
# class yang "hidup" di thread terpisah secara permanen. Sebagai gantinya,
# setiap method-nya dipanggil dari dalam thread sekali pakai yang dibuat
# App (lihat ModbusApp._run_in_thread di bawah). Ini lebih sederhana di
# Kivy dan tetap memenuhi syarat: operasi blocking TIDAK PERNAH terjadi
# di main/UI thread.
# ----------------------------------------------------------------------
class SerialBackend:
    def __init__(self):
        self.ser = None
        self.lock = threading.Lock()   # cegah dua command serial jalan bersamaan
        self.cancelling = False
        self.state_pompa = False

    @property
    def is_open(self):
        return self.ser is not None and self.ser.is_open

    def connect(self, port, baudrate):
        """Blocking - panggil ini HANYA dari background thread."""
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

    def disconnect(self):
        """Blocking (tapi cepat) - panggil dari background thread."""
        self.cancelling = True
        try:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.cancel_read()  # batalkan ser.read() yang sedang blocking
                except Exception:
                    pass
                self.ser.close()
        finally:
            self.ser = None
            self.cancelling = False
            self.state_pompa = False

    def send_calibration(self, value):
        """Return: (success: bool, logs: list[(msg, level)])"""
        logs = []
        if self.cancelling or not self.is_open:
            return False, [("Operasi dibatalkan: serial tidak terbuka.", "WARN")]

        int_value = int(round(value * 100))
        if not (0 <= int_value <= 65535):
            return False, [(f"Nilai {value} di luar jangkauan (0-655.35).", "ERROR")]

        frame = build_write_register(MODBUS_SLAVE_ID, CALIB_REGISTER, int_value)
        try:
            self.ser.reset_input_buffer()
            self.ser.write(frame)
            logs.append((
                f"TX  FC06  Reg=0x{CALIB_REGISTER:04X}  Val={int_value} "
                f"(={value:.2f})  Frame={frame.hex().upper()}", "TX"
            ))

            if self.cancelling:
                return False, logs
            response = self.ser.read(8)
            if self.cancelling:
                return False, logs

            if not response:
                logs.append(("Timeout: tidak ada response dari perangkat.", "WARN"))
                return False, logs
            if len(response) < 8:
                logs.append((
                    f"Response tidak lengkap ({len(response)}/8 byte): "
                    f"{response.hex().upper()}", "WARN"
                ))
                return False, logs
            if not verify_crc(response):
                logs.append(("CRC response tidak valid.", "ERROR"))
                return False, logs
            if response[1] & 0x80:
                logs.append((f"Exception Modbus: kode {response[2]:#04x}", "ERROR"))
                return False, logs

            logs.append((f"RX  FC06 OK  Frame={response.hex().upper()}", "RX"))
            logs.append((f"Kalibrasi {value:.2f} berhasil dikirim.", "SUCCESS"))
            return True, logs

        except serial.SerialException as e:
            if not self.cancelling:
                logs.append((f"Serial error saat kirim: {e}", "ERROR"))
            return False, logs

    def read_calibration(self):
        """Return: (value: float|None, logs: list[(msg, level)])"""
        logs = []
        if self.cancelling or not self.is_open:
            return None, [("Operasi dibatalkan: serial tidak terbuka.", "WARN")]

        frame = build_read_register(MODBUS_SLAVE_ID, CALIB_REGISTER, 1)
        try:
            self.ser.reset_input_buffer()
            self.ser.write(frame)
            logs.append((
                f"TX  FC03  Reg=0x{CALIB_REGISTER:04X}  Frame={frame.hex().upper()}", "TX"
            ))

            if self.cancelling:
                return None, logs
            response = self.ser.read(7)
            if self.cancelling:
                return None, logs

            if not response:
                logs.append(("Timeout: tidak ada response saat baca.", "WARN"))
                return None, logs
            if len(response) < 7:
                logs.append((
                    f"Response baca tidak lengkap ({len(response)}/7 byte): "
                    f"{response.hex().upper()}", "WARN"
                ))
                return None, logs
            if not verify_crc(response):
                logs.append(("CRC response baca tidak valid.", "ERROR"))
                return None, logs
            if response[1] & 0x80:
                logs.append((f"Exception Modbus saat baca: kode {response[2]:#04x}", "ERROR"))
                return None, logs

            raw = struct.unpack('>H', response[3:5])[0]
            value = raw / 100.0
            logs.append((
                f"RX  FC03 OK  Raw={raw}  Val={value:.2f}  "
                f"Frame={response.hex().upper()}", "RX"
            ))
            return value, logs

        except serial.SerialException as e:
            if not self.cancelling:
                logs.append((f"Serial error saat baca: {e}", "ERROR"))
            return None, logs

    def toggle_pump(self):
        """Return: (new_state: bool|None, logs: list[(msg, level)])"""
        logs = []
        if self.cancelling or not self.is_open:
            return None, [("Operasi dibatalkan: serial tidak terbuka.", "WARN")]

        val = 0x0000 if self.state_pompa else 0xFF00
        frame = build_write_single_coil(MODBUS_SLAVE_ID, COIL_REGISTER, val)
        try:
            self.ser.reset_input_buffer()
            self.ser.write(frame)
            logs.append((
                f"TX  FC05  Reg=0x{COIL_REGISTER:04X}  Frame={frame.hex().upper()}", "TX"
            ))

            if self.cancelling:
                return None, logs
            response = self.ser.read(8)
            if self.cancelling:
                return None, logs

            if not response:
                logs.append(("Timeout: tidak ada response saat baca.", "WARN"))
                return None, logs
            if len(response) < 8:
                logs.append((
                    f"Response baca tidak lengkap ({len(response)}/8 byte): "
                    f"{response.hex().upper()}", "WARN"
                ))
                return None, logs
            if not verify_crc(response):
                logs.append(("CRC response baca tidak valid.", "ERROR"))
                return None, logs
            if response[1] & 0x80:
                logs.append((f"Exception Modbus saat baca: kode {response[2]:#04x}", "ERROR"))
                return None, logs

            logs.append((f"RX  FC05 OK  Frame={response.hex().upper()}", "RX"))
            self.state_pompa = not self.state_pompa
            new_state = ((response[4] << 8) | response[5]) == 0xFF00
            return new_state, logs

        except serial.SerialException as e:
            if not self.cancelling:
                logs.append((f"Serial error saat baca: {e}", "ERROR"))
            return None, logs


# ----------------------------------------------------------------------
# AndroidSerialBackend
# ------------------------------------------------------------------
# Drop-in pengganti SerialBackend di atas, khusus saat APK berjalan di
# Android. Method-nya SENGAJA punya signature & return value yang sama
# persis (connect, disconnect, send_calibration, read_calibration,
# toggle_pump) - jadi ModbusApp tidak perlu tahu lagi platform mana yang
# aktif, tinggal pilih instance backend yang tepat saat startup.
#
# Perbedaan utama dari sisi pemakaian:
#   - "port" di desktop = nama COM/tty (mis. "COM3", "/dev/ttyUSB0")
#   - "port" di Android  = device_name USB (mis. "/dev/bus/usb/001/002"),
#     didapat dari usb.get_usb_device_list(), BUKAN diketik manual.
#
# CATATAN PENTING:
#   - Kode ini belum bisa diuji langsung di environment pembuatan kode ini
#     karena perlu HP Android fisik + USB-RS485 converter yang tercolok.
#     Wajib diuji & kemungkinan disesuaikan sedikit (terutama timing izin
#     USB) langsung di perangkat sebelum dipakai produksi.
#   - usbserial4a otomatis mendeteksi chip USB-to-serial yang didukung:
#     CP210x, FTDI, CH340/CH341, PL2303. Pastikan converter RS485 kamu
#     pakai salah satu chip ini.
# ----------------------------------------------------------------------
class AndroidSerialBackend:
    def __init__(self):
        self.serial_port = None
        self.cancelling = False
        self.state_pompa = False

    @property
    def is_open(self):
        return self.serial_port is not None

    @staticmethod
    def list_devices():
        """Return list nama device USB yang terdeteksi (untuk ditampilkan di UI)."""
        if not ANDROID_USB_AVAILABLE:
            return []
        try:
            return [d.getDeviceName() for d in usb.get_usb_device_list()]
        except Exception:
            return []

    @staticmethod
    def has_permission(device_name):
        device = AndroidSerialBackend._find_device(device_name)
        return device is not None and usb.has_usb_permission(device)

    @staticmethod
    def request_permission(device_name):
        """Memicu dialog izin USB bawaan Android. HARUS dipanggil dari main thread."""
        device = AndroidSerialBackend._find_device(device_name)
        if device is not None:
            usb.request_usb_permission(device)

    @staticmethod
    def _find_device(device_name):
        for d in usb.get_usb_device_list():
            if d.getDeviceName() == device_name:
                return d
        return None

    def connect(self, device_name, baudrate):
        """Blocking - panggil dari background thread, SETELAH izin USB diberikan."""
        self.cancelling = False
        port = serial4a.get_serial_port(
            device_name,
            baudrate,
            8,    # data bits
            1,    # stop bits (1 = satu stop bit)
            "N",  # parity: None
        )
        if not port.is_open:
            port.open()
        port.timeout = 0.5
        self.serial_port = port

    def disconnect(self):
        self.cancelling = True
        try:
            if self.serial_port is not None:
                self.serial_port.close()
        finally:
            self.serial_port = None
            self.cancelling = False
            self.state_pompa = False

    def send_calibration(self, value):
        logs = []
        if self.cancelling or not self.is_open:
            return False, [("Operasi dibatalkan: serial tidak terbuka.", "WARN")]

        int_value = int(round(value * 100))
        if not (0 <= int_value <= 65535):
            return False, [(f"Nilai {value} di luar jangkauan (0-655.35).", "ERROR")]

        frame = build_write_register(MODBUS_SLAVE_ID, CALIB_REGISTER, int_value)
        try:
            self.serial_port.write(frame)
            logs.append((
                f"TX  FC06  Reg=0x{CALIB_REGISTER:04X}  Val={int_value} "
                f"(={value:.2f})  Frame={frame.hex().upper()}", "TX"
            ))

            if self.cancelling:
                return False, logs
            response = self.serial_port.read(8)
            if self.cancelling:
                return False, logs

            if not response:
                logs.append(("Timeout: tidak ada response dari perangkat.", "WARN"))
                return False, logs
            if len(response) < 8:
                logs.append((f"Response tidak lengkap: {response.hex().upper()}", "WARN"))
                return False, logs
            if not verify_crc(response):
                logs.append(("CRC response tidak valid.", "ERROR"))
                return False, logs
            if response[1] & 0x80:
                logs.append((f"Exception Modbus: kode {response[2]:#04x}", "ERROR"))
                return False, logs

            logs.append((f"RX  FC06 OK  Frame={response.hex().upper()}", "RX"))
            logs.append((f"Kalibrasi {value:.2f} berhasil dikirim.", "SUCCESS"))
            return True, logs

        except Exception as e:
            if not self.cancelling:
                logs.append((f"Serial error saat kirim: {e}", "ERROR"))
            return False, logs

    def read_calibration(self):
        logs = []
        if self.cancelling or not self.is_open:
            return None, [("Operasi dibatalkan: serial tidak terbuka.", "WARN")]

        frame = build_read_register(MODBUS_SLAVE_ID, CALIB_REGISTER, 1)
        try:
            self.serial_port.write(frame)
            logs.append((
                f"TX  FC03  Reg=0x{CALIB_REGISTER:04X}  Frame={frame.hex().upper()}", "TX"
            ))

            if self.cancelling:
                return None, logs
            response = self.serial_port.read(7)
            if self.cancelling:
                return None, logs

            if not response:
                logs.append(("Timeout: tidak ada response saat baca.", "WARN"))
                return None, logs
            if len(response) < 7:
                logs.append((f"Response tidak lengkap: {response.hex().upper()}", "WARN"))
                return None, logs
            if not verify_crc(response):
                logs.append(("CRC response baca tidak valid.", "ERROR"))
                return None, logs
            if response[1] & 0x80:
                logs.append((f"Exception Modbus saat baca: kode {response[2]:#04x}", "ERROR"))
                return None, logs

            raw = struct.unpack('>H', response[3:5])[0]
            value = raw / 100.0
            logs.append((
                f"RX  FC03 OK  Raw={raw}  Val={value:.2f}  "
                f"Frame={response.hex().upper()}", "RX"
            ))
            return value, logs

        except Exception as e:
            if not self.cancelling:
                logs.append((f"Serial error saat baca: {e}", "ERROR"))
            return None, logs

    def toggle_pump(self):
        logs = []
        if self.cancelling or not self.is_open:
            return None, [("Operasi dibatalkan: serial tidak terbuka.", "WARN")]

        val = 0x0000 if self.state_pompa else 0xFF00
        frame = build_write_single_coil(MODBUS_SLAVE_ID, COIL_REGISTER, val)
        try:
            self.serial_port.write(frame)
            logs.append((
                f"TX  FC05  Reg=0x{COIL_REGISTER:04X}  Frame={frame.hex().upper()}", "TX"
            ))

            if self.cancelling:
                return None, logs
            response = self.serial_port.read(8)
            if self.cancelling:
                return None, logs

            if not response:
                logs.append(("Timeout: tidak ada response saat baca.", "WARN"))
                return None, logs
            if len(response) < 8:
                logs.append((f"Response tidak lengkap: {response.hex().upper()}", "WARN"))
                return None, logs
            if not verify_crc(response):
                logs.append(("CRC response baca tidak valid.", "ERROR"))
                return None, logs
            if response[1] & 0x80:
                logs.append((f"Exception Modbus saat baca: kode {response[2]:#04x}", "ERROR"))
                return None, logs

            logs.append((f"RX  FC05 OK  Frame={response.hex().upper()}", "RX"))
            self.state_pompa = not self.state_pompa
            new_state = ((response[4] << 8) | response[5]) == 0xFF00
            return new_state, logs

        except Exception as e:
            if not self.cancelling:
                logs.append((f"Serial error saat baca: {e}", "ERROR"))
            return None, logs


# ----------------------------------------------------------------------
# Widget LED indikator (sama seperti aplikasi sebelumnya)
# ----------------------------------------------------------------------
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
    md_bg_color: app.bg_color

    MDBoxLayout:
        orientation: "vertical"

        # ============== HEADER ==============
        MDBoxLayout:
            orientation: "vertical"
            size_hint_y: None
            height: dp(96)
            padding: dp(20), dp(14), dp(20), dp(10)
            spacing: dp(6)
            md_bg_color: app.surface_color

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
                    text_color: app.text_dim

        ScrollView:
            MDBoxLayout:
                orientation: "vertical"
                spacing: dp(14)
                padding: dp(16), dp(16), dp(16), dp(24)
                size_hint_y: None
                height: self.minimum_height

                # ---------- PANEL: Koneksi ----------
                MDBoxLayout:
                    orientation: "vertical"
                    padding: dp(18)
                    spacing: dp(12)
                    size_hint_y: None
                    height: self.minimum_height
                    md_bg_color: app.surface_color
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
                            id: port_dropdown
                            text: "PILIH PORT"
                            size_hint_x: 0.55
                            line_color: app.text_dim
                            theme_text_color: "Custom"
                            text_color: app.text_main
                            on_release: app.open_port_menu(self)

                        MDRectangleFlatButton:
                            id: baud_dropdown
                            text: "9600"
                            size_hint_x: 0.3
                            line_color: app.text_dim
                            theme_text_color: "Custom"
                            text_color: app.text_main
                            on_release: app.open_baud_menu(self)

                        MDIconButton:
                            icon: "refresh"
                            theme_text_color: "Custom"
                            text_color: app.cyan
                            on_release: app.refresh_ports()

                    MDRectangleFlatIconButton:
                        id: connect_btn
                        text: "HUBUNGKAN"
                        icon: "power-plug-outline"
                        pos_hint: {"center_x": 0.5}
                        line_color: app.amber
                        theme_text_color: "Custom"
                        text_color: app.amber
                        on_release: app.toggle_connection()

                # ---------- PANEL: Kalibrasi ----------
                MDBoxLayout:
                    orientation: "vertical"
                    padding: dp(18)
                    spacing: dp(12)
                    size_hint_y: None
                    height: self.minimum_height
                    md_bg_color: app.surface_color
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
                        spacing: dp(8)

                        MDLabel:
                            text: "Nilai saat ini:"
                            theme_text_color: "Custom"
                            text_color: app.text_dim
                            size_hint_x: 0.5

                        MDLabel:
                            id: current_value_label
                            text: "—"
                            bold: True
                            font_style: "H5"
                            theme_text_color: "Custom"
                            text_color: app.cyan
                            halign: "right"

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

                    MDBoxLayout:
                        spacing: dp(8)
                        size_hint_y: None
                        height: dp(48)

                        MDTextField:
                            id: calib_input
                            hint_text: "Input Nilai Kalibrasi (0 - 655.35)"
                            text: "0.00"
                            input_filter: "float"
                            line_color_normal: app.text_dim
                            line_color_focus: app.amber
                            hint_text_color_normal: app.text_dim
                            hint_text_color_focus: app.amber
                            text_color_normal: app.text_main
                            text_color_focus: app.text_main

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

                # ---------- PANEL: Log ----------
                MDBoxLayout:
                    orientation: "vertical"
                    padding: dp(18)
                    spacing: dp(10)
                    size_hint_y: None
                    height: dp(320)
                    md_bg_color: app.surface_color
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
                            text_color: app.text_dim
                            on_release: app.clear_log()

                    ScrollView:
                        id: log_scroll
                        do_scroll_x: False

                        MDBoxLayout:
                            id: log_box
                            orientation: "vertical"
                            spacing: dp(2)
                            padding: dp(8)
                            size_hint_y: None
                            height: self.minimum_height
'''


class LedDot(Widget):
    dot_color = ListProperty([1, 0, 0, 1])
    glow_color = ListProperty([1, 0, 0, 0.25])


class CalibratorApp(MDApp):
    # ---- properti tema (sama seperti aplikasi RS485 Reader) ----
    bg_color = ListProperty(COL_BG)
    surface_color = ListProperty(COL_SURFACE)
    text_main = ListProperty(COL_TEXT)
    text_dim = ListProperty(COL_TEXT_DIM)
    amber = ListProperty(COL_AMBER)
    cyan = ListProperty(COL_CYAN)
    green = ListProperty(COL_GREEN)
    red = ListProperty(COL_RED)

    connected = BooleanProperty(False)
    busy = BooleanProperty(False)        # True selama ada operasi serial berjalan
    pump_on = BooleanProperty(False)
    selected_port = StringProperty("")
    selected_baud = StringProperty("9600")

    def build(self):
        self.theme_cls.theme_style = "Dark"
        self.theme_cls.primary_palette = "Amber"
        self.title = "TAP-FUSION Calibrator"
        # Pilih backend sesuai platform - inilah satu-satunya percabangan
        # platform yang dibutuhkan, sisanya (UI & logic) sama untuk keduanya.
        self.backend = AndroidSerialBackend() if IS_ANDROID else SerialBackend()
        self.port_menu = None
        self.baud_menu = None
        self.screen = Builder.load_string(KV)
        return self.screen

    def on_start(self):
        self.refresh_ports()

    # ==================================================================
    # Util: jalankan fungsi blocking di background thread, lalu
    # kembalikan hasilnya ke main thread lewat Clock.schedule_once.
    # Ini padanan langsung dari pola QThread + pyqtSignal.emit().
    # ==================================================================
    def _run_in_thread(self, work_fn, on_result):
        def runner():
            result = work_fn()
            Clock.schedule_once(lambda dt: on_result(result))
        threading.Thread(target=runner, daemon=True).start()

    def _guard_busy(self):
        """True jika sedang ada operasi serial berjalan -> abaikan klik baru."""
        return self.busy

    # ------------------------------------------------------------------
    # Port & Baudrate
    # ------------------------------------------------------------------
    def refresh_ports(self):
        if not SERIAL_AVAILABLE:
            if IS_ANDROID:
                self.log("Modul usb4a/usbserial4a tidak tersedia.", "ERROR")
            else:
                self.log("pyserial tidak terinstall. Jalankan: pip install pyserial", "ERROR")
            return

        if IS_ANDROID:
            ports = AndroidSerialBackend.list_devices()
        else:
            ports = [p.device for p in serial.tools.list_ports.comports()]

        if not ports:
            self.log("Tidak ada port/device USB yang ditemukan.", "WARN")
            self.show_snackbar("Tidak ada port serial terdeteksi")
            return

        self.log(f"Ditemukan {len(ports)} port: {', '.join(ports)}", "INFO")

        from kivymd.uix.menu import MDDropdownMenu
        self.port_menu = MDDropdownMenu(
            caller=self.screen.ids.port_dropdown,
            items=[
                {"text": p, "viewclass": "OneLineListItem",
                 "on_release": lambda x=p: self._set_port(x)}
                for p in ports
            ],
            width_mult=4,
        )

    def open_port_menu(self, instance):
        if self.port_menu:
            self.port_menu.open()
        else:
            self.refresh_ports()

    def _set_port(self, port_name):
        self.selected_port = port_name
        self.screen.ids.port_dropdown.text = port_name
        if self.port_menu:
            self.port_menu.dismiss()

    def open_baud_menu(self, instance):
        from kivymd.uix.menu import MDDropdownMenu
        baud_options = ["9600", "19200", "38400", "57600", "115200"]
        self.baud_menu = MDDropdownMenu(
            caller=self.screen.ids.baud_dropdown,
            items=[
                {"text": b, "viewclass": "OneLineListItem",
                 "on_release": lambda x=b: self._set_baud(x)}
                for b in baud_options
            ],
            width_mult=3,
        )
        self.baud_menu.open()

    def _set_baud(self, baud):
        self.selected_baud = baud
        self.screen.ids.baud_dropdown.text = baud
        if self.baud_menu:
            self.baud_menu.dismiss()

    # ------------------------------------------------------------------
    # Koneksi
    # ------------------------------------------------------------------
    def toggle_connection(self):
        if self._guard_busy():
            return
        if self.connected:
            self._do_disconnect()
        else:
            self._do_connect()

    def _do_connect(self):
        if not self.selected_port:
            self.show_snackbar("Pilih port terlebih dahulu")
            return

        if IS_ANDROID:
            self._do_connect_android()
        else:
            self._do_connect_desktop()

    def _do_connect_desktop(self):
        baudrate = int(self.selected_baud)
        self.busy = True
        self.screen.ids.connect_btn.text = "MENGHUBUNGKAN..."
        self.screen.ids.connect_btn.disabled = True
        self._set_led("connecting")
        self.log(f"Mencoba koneksi ke {self.selected_port}...", "INFO")

        def work():
            try:
                self.backend.connect(self.selected_port, baudrate)
                return True, None
            except Exception as e:
                return False, str(e)

        self._run_in_thread(work, lambda r: self._on_connect_result(r, baudrate))

    def _do_connect_android(self):
        """
        Di Android, izin USB Host harus diminta dari MAIN THREAD (memicu
        dialog sistem "Izinkan aplikasi mengakses perangkat USB?"). Tidak
        boleh dipanggil dari background thread. Setelah izin didapat,
        baru proses buka port dipindah ke thread seperti biasa.
        """
        baudrate = int(self.selected_baud)

        if AndroidSerialBackend.has_permission(self.selected_port):
            self._open_android_port(baudrate)
            return

        self.log("Meminta izin akses USB...", "INFO")
        self.show_snackbar("Izinkan akses USB di dialog yang muncul")
        AndroidSerialBackend.request_permission(self.selected_port)

        # Poll status izin setiap 0.5 detik selama maks 15 detik
        # (menunggu user menekan "Allow" di dialog sistem Android)
        self._permission_attempts = 0

        def check_permission(dt):
            self._permission_attempts += 1
            if AndroidSerialBackend.has_permission(self.selected_port):
                self._open_android_port(baudrate)
            elif self._permission_attempts >= 30:
                self.log("Izin USB tidak diberikan (timeout).", "ERROR")
                self.show_snackbar("Izin USB ditolak atau timeout")
            else:
                Clock.schedule_once(check_permission, 0.5)

        Clock.schedule_once(check_permission, 0.5)

    def _open_android_port(self, baudrate):
        self.busy = True
        self.screen.ids.connect_btn.text = "MENGHUBUNGKAN..."
        self.screen.ids.connect_btn.disabled = True
        self._set_led("connecting")
        self.log(f"Membuka koneksi ke {self.selected_port}...", "INFO")

        def work():
            try:
                self.backend.connect(self.selected_port, baudrate)
                return True, None
            except Exception as e:
                return False, str(e)

        self._run_in_thread(work, lambda r: self._on_connect_result(r, baudrate))

    def _on_connect_result(self, result, baudrate):
        success, error = result
        self.busy = False
        self.screen.ids.connect_btn.disabled = False

        if success:
            self.connected = True
            self.log(f"Terhubung ke {self.selected_port} @ {baudrate} baud", "INFO")
            self.screen.ids.status_label.text = f"Terhubung: {self.selected_port} @ {baudrate} baud"
            self.screen.ids.status_label.text_color = self.green
            self._set_led("on")
            self.screen.ids.connect_btn.text = "PUTUSKAN"
            self.screen.ids.connect_btn.icon = "power-plug-off-outline"
            self.screen.ids.read_btn.disabled = False
            self.screen.ids.send_btn.disabled = False
            self.screen.ids.pump_btn.disabled = False
            self.show_snackbar("Berhasil terhubung")
            # otomatis baca nilai kalibrasi saat ini, seperti versi PyQt5
            self.read_calibration()
        else:
            self.connected = False
            self.log(f"Gagal buka port: {error}", "ERROR")
            self._set_led("off")
            self.screen.ids.connect_btn.text = "HUBUNGKAN"
            self.screen.ids.status_label.text = "Tidak terhubung"
            self.screen.ids.status_label.text_color = self.text_dim
            self.show_snackbar(f"Gagal terhubung: {error}")

    def _do_disconnect(self):
        self.busy = True
        self.screen.ids.connect_btn.disabled = True
        self.log("Memutuskan koneksi...", "INFO")

        def work():
            self.backend.disconnect()
            return True

        self._run_in_thread(work, lambda r: self._on_disconnect_result())

    def _on_disconnect_result(self):
        self.busy = False
        self.connected = False
        self.pump_on = False
        self.log("Koneksi serial ditutup.", "INFO")

        self._set_led("off")
        self.screen.ids.status_label.text = "Tidak terhubung"
        self.screen.ids.status_label.text_color = self.text_dim
        self.screen.ids.connect_btn.text = "HUBUNGKAN"
        self.screen.ids.connect_btn.icon = "power-plug-outline"
        self.screen.ids.connect_btn.disabled = False
        self.screen.ids.read_btn.disabled = True
        self.screen.ids.send_btn.disabled = True
        self.screen.ids.pump_btn.disabled = True
        self.screen.ids.current_value_label.text = "—"
        self._set_pump_button(False)

    # ------------------------------------------------------------------
    # Baca kalibrasi
    # ------------------------------------------------------------------
    def read_calibration(self):
        if self._guard_busy() or not self.connected:
            return
        self.busy = True
        self.screen.ids.read_btn.disabled = True

        self._run_in_thread(self.backend.read_calibration, self._on_read_result)

    def _on_read_result(self, result):
        value, logs = result
        self._flush_logs(logs)
        self.busy = False
        self.screen.ids.read_btn.disabled = not self.connected

        if value is not None:
            self.screen.ids.current_value_label.text = f"{value:.2f}"
        if not self.connected:
            return  # koneksi mungkin terputus saat operasi berjalan

    # ------------------------------------------------------------------
    # Kirim kalibrasi
    # ------------------------------------------------------------------
    def send_calibration(self):
        if self._guard_busy() or not self.connected:
            return

        try:
            value = float(self.screen.ids.calib_input.text)
        except ValueError:
            self.show_snackbar("Nilai kalibrasi tidak valid")
            return

        self.busy = True
        self.screen.ids.send_btn.disabled = True

        self._run_in_thread(
            lambda: self.backend.send_calibration(value),
            self._on_send_result,
        )

    def _on_send_result(self, result):
        success, logs = result
        self._flush_logs(logs)
        self.busy = False
        self.screen.ids.send_btn.disabled = not self.connected

        if success:
            self.show_snackbar("Kalibrasi berhasil dikirim")
            self.read_calibration()  # refresh nilai terbaru

    # ------------------------------------------------------------------
    # Nyala/matikan pompa
    # ------------------------------------------------------------------
    def toggle_pump(self):
        if self._guard_busy() or not self.connected:
            return
        self.busy = True
        self.screen.ids.pump_btn.disabled = True

        self._run_in_thread(self.backend.toggle_pump, self._on_pump_result)

    def _on_pump_result(self, result):
        new_state, logs = result
        self._flush_logs(logs)
        self.busy = False
        self.screen.ids.pump_btn.disabled = not self.connected

        if new_state is not None:
            self.pump_on = new_state
            self._set_pump_button(new_state)

    def _set_pump_button(self, state):
        btn = self.screen.ids.pump_btn
        if state:
            btn.text = "MATIKAN POMPA"
            btn.line_color = self.red
            btn.text_color = self.red
        else:
            btn.text = "NYALAKAN POMPA"
            btn.line_color = self.green
            btn.text_color = self.green

    # ------------------------------------------------------------------
    # LED status
    # ------------------------------------------------------------------
    def _set_led(self, state):
        from kivy.animation import Animation
        led = self.screen.ids.led
        Animation.cancel_all(led)
        led.opacity = 1

        if state == "on":
            led.dot_color = self.green
            led.glow_color = (*self.green[:3], 0.30)
            anim = (Animation(opacity=0.35, duration=0.7) +
                    Animation(opacity=1, duration=0.7))
            anim.repeat = True
            anim.start(led)
        elif state == "connecting":
            led.dot_color = self.amber
            led.glow_color = (*self.amber[:3], 0.30)
            anim = (Animation(opacity=0.35, duration=0.4) +
                    Animation(opacity=1, duration=0.4))
            anim.repeat = True
            anim.start(led)
        else:  # off
            led.dot_color = self.red
            led.glow_color = (*self.red[:3], 0.25)

    # ------------------------------------------------------------------
    # Log
    # ------------------------------------------------------------------
    def _flush_logs(self, logs):
        for message, level in logs:
            self.log(message, level)

    def log(self, message, level="INFO"):
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        color = LEVEL_COLORS.get(level, "#D4D4D4")
        text = (
            f"[color=888888]{timestamp}[/color]  "
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

        # batasi jumlah baris log agar memori tidak membengkak
        if len(log_box.children) > 300:
            log_box.remove_widget(log_box.children[0])

        # auto-scroll ke bawah
        Clock.schedule_once(lambda dt: setattr(self.screen.ids.log_scroll, "scroll_y", 0))

    def clear_log(self):
        self.screen.ids.log_box.clear_widgets()

    # ------------------------------------------------------------------
    def show_snackbar(self, message):
        Snackbar(text=message, size_hint_x=0.9).open()

    # ------------------------------------------------------------------
    # Cleanup saat aplikasi ditutup - pastikan port serial dilepas
    # ------------------------------------------------------------------
    def on_stop(self):
        if self.backend.is_open:
            self.backend.disconnect()


if __name__ == "__main__":
    CalibratorApp().run()
