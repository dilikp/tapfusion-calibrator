[app]
title = TAP-FUSION Calibrator
package.name = tapfusioncalibrator
package.domain = org.tapfusion

source.dir = .
source.include_exts = py,png,jpg,kv,atlas

version = 1.0
orientation = portrait
fullscreen = 0

# --- Dependency Python yang dibutuhkan ---
# pyserial tetap dimasukkan untuk berjaga-jaga / kompatibilitas import,
# tapi yang BENAR-BENAR dipakai untuk akses USB di Android adalah usb4a
# dan usbserial4a (lihat AndroidSerialBackend di main.py).
# SEBELUM (hapus usb4a dan usbserial4a dari sini)
requirements = python3,kivy==2.2.1,kivymd==1.1.1,pyserial,usb4a,usbserial4a

# SESUDAH
requirements = python3,kivy==2.2.1,kivymd==1.1.1,pyserial

# --- Permission Android ---
android.permissions = INTERNET

# --- WAJIB untuk akses USB Host (USB-to-RS485 converter) ---
android.add_uses_feature = android.hardware.usb.host:required=false
android.sdk_path = /usr/local/lib/android/sdk
android.ndk_path = /usr/local/lib/android/sdk/ndk/25.2.9519653

# Minimum & target SDK - sesuaikan jika perlu
android.minapi = 24
android.api = 33
android.ndk = 25b

android.build_tools_version = 37.0.0

# Arsitektur target (arm64-v8a mencakup hampir semua HP modern)
android.archs = arm64-v8a, armeabi-v7a

# Izinkan resource intent-filter USB device attached (opsional tapi disarankan
# supaya app bisa langsung terbuka saat converter dicolok)
# Lihat catatan di bawah jika ingin menambahkan ini secara manual lewat
# android.manifest_intent_filters jika dibutuhkan.

# Hapus tanda # di bawah ini dan siapkan file icon.png (ukuran 512x512)
# Arahkan ke p4a yang sudah diclone dan dipatch di workflow
p4a.source_dir = .buildozer/android/platform/python-for-android
# di folder project jika ingin app punya icon custom.
# icon.filename = %(source.dir)s/icon.png

[buildozer]
log_level = 2
warn_on_root = 1
