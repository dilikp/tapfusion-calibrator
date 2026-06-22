[app]
title = TAP-FUSION Calibrator
package.name = tapfusioncalibrator
package.domain = org.tapfusion

source.dir = .
source.include_exts = py,png,jpg,kv,atlas

version = 1.0
orientation = portrait
fullscreen = 0

# Requirements minimal yang stabil - tanpa usb4a/usbserial4a
# karena keduanya tidak tersedia sebagai binary wheel untuk Android
requirements = python3,kivy==2.2.1,kivymd==1.1.1,pyserial

android.permissions = INTERNET
android.minapi = 21
android.api = 33
android.ndk = 25b
android.build_tools_version = 33.0.0
android.archs = arm64-v8a

# Pin p4a ke versi 2023 yang masih pakai Python 3.11 untuk hostpython3
# Ini mencegah error "No module named cgi" yang terjadi di p4a master (Python 3.14)
p4a.branch = v2023.9.16

[buildozer]
log_level = 2
warn_on_root = 1
