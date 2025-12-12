import cv2
import numpy as np
import pykinect_azure as pykinect
import time
import csv
import ctypes
import serial  # SERIAL UNTUK ESP32

from pykinect_azure.k4a import _k4a
from pykinect_azure import (
    K4A_CALIBRATION_TYPE_COLOR,
    K4A_CALIBRATION_TYPE_DEPTH,
    K4ABT_JOINT_HEAD,
    K4ABT_JOINT_HAND_RIGHT,
    K4ABT_JOINT_HAND_LEFT,
    K4ABT_JOINT_PELVIS,
    K4ABT_JOINT_KNEE_RIGHT,
    K4ABT_JOINT_ANKLE_RIGHT,
    K4ABT_JOINT_ANKLE_LEFT,
    K4ABT_JOINT_COUNT,
    K4ABT_JOINT_NAMES,
    # tambahan joint untuk gambar skeleton / workout
    K4ABT_JOINT_NECK,
    K4ABT_JOINT_SPINE_CHEST,
    K4ABT_JOINT_SPINE_NAVEL,
    K4ABT_JOINT_CLAVICLE_RIGHT,
    K4ABT_JOINT_SHOULDER_RIGHT,
    K4ABT_JOINT_ELBOW_RIGHT,
    K4ABT_JOINT_WRIST_RIGHT,
    K4ABT_JOINT_CLAVICLE_LEFT,
    K4ABT_JOINT_SHOULDER_LEFT,
    K4ABT_JOINT_ELBOW_LEFT,
    K4ABT_JOINT_WRIST_LEFT,
    K4ABT_JOINT_HIP_RIGHT,
    K4ABT_JOINT_HIP_LEFT,
    K4ABT_JOINT_KNEE_LEFT,
    K4ABT_JOINT_KNEE_RIGHT,
    K4ABT_JOINT_ANKLE_LEFT,
    K4ABT_JOINT_ANKLE_RIGHT
)

# 1. INISIALISASI
print("Menginisialisasi library...")
pykinect.initialize_libraries(track_body=True)

# Konfigurasi perangkat
device_config = pykinect.default_configuration
device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_720P
device_config.depth_mode = pykinect.K4A_DEPTH_MODE_NFOV_UNBINNED
device_config.synchronized_images_only = True

# Mulai perangkat
print("Membuka kamera Kinect...")
device = pykinect.start_device(config=device_config)

# Dapatkan kalibrasi (penting untuk memetakan 3D ke 2D)
calibration = device.get_calibration(device_config.depth_mode, device_config.color_resolution)

# Mulai body tracker
print("Memulai Body Tracker...")
body_tracker = pykinect.start_body_tracker(calibration)

# 2. PERSIAPAN FILE CSV
print("Membuka file CSV untuk menyimpan data...")
try:
    csv_file = open('data_gerakan.csv', 'w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(['timestamp', 'body_index', 'joint_name', 'pos_x', 'pos_y', 'pos_z'])
except IOError as e:
    print(f"Error membuka file CSV: {e}. Pastikan file tidak sedang dibuka program lain.")
    exit()

# 2b. PERSIAPAN SERIAL KE ESP32
try:
    # GANTI 'COM8' SESUAI PORT ESP32 KAMU
    ser = serial.Serial('COM8', 115200, timeout=1)
    time.sleep(2)  # beri waktu ESP32 reset
    print("Terhubung ke ESP32 via serial.")
except Exception as e:
    print(f"Gagal membuka port serial: {e}")
    ser = None  # supaya aman kalau serial tidak tersedia

print("🎥 Sistem siap. Tekan 'q' untuk keluar.")

# Pilihan exercise di awal
selected_exercise = input("Pilih exercise (knee/shoulder/sidebend/all) [all]: ").strip().lower() or "all"
if selected_exercise not in ("knee", "shoulder", "sidebend", "all"):
    print("Pilihan tidak valid, menggunakan 'all'")
    selected_exercise = "all"

# Tambahkan: state & counter untuk tiap body (persisten di luar loop)
# body_index -> {
#   knee_count, knee_state,
#   shoulder_count, shoulder_state,
#   sidebend_count, sidebend_state,
#   success_sent,
#   knee_led_sent, shoulder_led_sent, side_led_sent
# }
exercise_state = {}

# ==========================
# THRESHOLD GERAKAN BARU
# ==========================

# 1) STANDING KNEE RAISE
# gunakan jarak vertikal pelvis - rata-rata lutut (semakin kecil = lutut makin naik)
KNEE_UP_THRESH = 80     # kalau |pelvis.y - knee.y| < ini -> dianggap KNEE UP
KNEE_DOWN_THRESH = 130  # kalau |pelvis.y - knee.y| > ini -> kembali DOWN
KNEE_DEADZONE = 10      # zona aman biar tidak jitter

# 2) SHOULDER FRONT RAISE
# gunakan selisih Y pergelangan tangan rata-rata terhadap bahu rata-rata
# wrist_y - shoulder_y:
#   > 60  => tangan turun (di bawah bahu)
#   <  0  => tangan naik (sejajar / di atas bahu)
SHOULDER_UP_THRESH = 0      # lebih kecil dari ini + deadzone -> dianggap RAISE
SHOULDER_DOWN_THRESH = 60   # lebih besar dari ini + deadzone -> dianggap turun
SHOULDER_DEADZONE = 10

# 3) SIDE BEND
# gunakan selisih X kepala terhadap pelvis -> miring kiri/kanan
SIDEBEND_RIGHT_THRESH = 70    # head.x - pelvis.x > ini -> miring ke kanan
SIDEBEND_LEFT_THRESH = -70    # head.x - pelvis.x < ini -> miring ke kiri
SIDEBEND_CENTER_THRESH = 30   # |diff| < ini -> dianggap kembali ke tengah


def convert_to_2d(calibration, position_3d):
    """Helper function to convert 3D coordinates to 2D screen coordinates"""
    point3d = _k4a.k4a_float3_t()
    point2d = _k4a.k4a_float2_t()
    valid = ctypes.c_long()

    point3d.xyz.x = ctypes.c_float(position_3d.x)
    point3d.xyz.y = ctypes.c_float(position_3d.y)
    point3d.xyz.z = ctypes.c_float(position_3d.z)

    result = _k4a.k4a_calibration_3d_to_2d(
        calibration._handle,
        point3d,
        K4A_CALIBRATION_TYPE_DEPTH,
        K4A_CALIBRATION_TYPE_COLOR,
        point2d,
        valid
    )

    if result == 0 and valid.value:  # K4A_RESULT_SUCCEEDED = 0
        return (int(point2d.xy.x), int(point2d.xy.y))
    return None


# Utility kecil untuk membuat objek 3D dari angka
class Vec3:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


# 3. LOOP UTAMA
selected_body_index = None  # indeks body yang dipilih berdasarkan ROI tengah

while True:
    capture = device.update()
    if not capture:
        print("Gagal mendapatkan capture")
        continue

    ret_color, color_image = capture.get_color_image()
    if not ret_color or color_image is None:
        continue

    # Manual ROI config
    ROI_W = 320  # width in pixels
    ROI_H = 480  # height in pixels
    ROI_CX = None  # set to an int to fix center X; keep None to use image center
    ROI_CY = None  # set to an int to fix center Y; keep None to use image center

    h, w, _ = color_image.shape
    roi_w, roi_h = ROI_W, ROI_H

    roi_cx = ROI_CX if ROI_CX is not None else w // 2
    roi_cy = ROI_CY if ROI_CY is not None else h // 2

    roi_x1 = max(0, roi_cx - roi_w // 2)
    roi_y1 = max(0, roi_cy - roi_h // 2)
    roi_x2 = min(w - 1, roi_cx + roi_w // 2)
    roi_y2 = min(h - 1, roi_cy + roi_h // 2)

    cv2.rectangle(color_image, (roi_x1, roi_y1), (roi_x2, roi_y2), (0, 255, 255), 2)
    cv2.putText(color_image, "ROI", (roi_x1 + 6, roi_y1 + 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    # Masukkan capture ke body tracker untuk diproses
    body_frame = body_tracker.update(capture)
    if not body_frame:
        print("Gagal mendapatkan body frame")
        continue

    # Dapatkan jumlah tubuh yang terdeteksi
    num_bodies = body_frame.get_num_bodies()

    # Pilih satu body yang berada paling dekat di tengah ROI (berdasarkan pundak)
    target_candidate = None
    min_dist = float("inf")

    for i in range(num_bodies):
        try:
            b = body_frame.get_body(i)
            sr = b.joints[K4ABT_JOINT_SHOULDER_RIGHT].position
            sl = b.joints[K4ABT_JOINT_SHOULDER_LEFT].position
            # rata-rata 3D
            sc = Vec3((sr.x + sl.x) / 2.0,
                      (sr.y + sl.y) / 2.0,
                      (sr.z + sl.z) / 2.0)
            p2 = convert_to_2d(calibration, sc)
            if p2 is None:
                continue
            sx, sy = p2

            # Jika shoulder center berada di dalam ROI, pertimbangkan sebagai kandidat
            if roi_x1 <= sx <= roi_x2 and roi_y1 <= sy <= roi_y2:
                # hitung jarak ke pusat ROI (2D)
                dist = (sx - roi_cx) ** 2 + (sy - roi_cy) ** 2
                if dist < min_dist:
                    min_dist = dist
                    target_candidate = i
        except Exception:
            pass

    # Jika ada kandidat, set selected_body_index ke kandidat; jika tidak, clear (tidak menghitung)
    selected_body_index = target_candidate

    # 4. HITUNG JARAK ANTAR ORANG (SHOULDER TO SHOULDER) - opsional tampil
    if num_bodies >= 2:
        for i in range(num_bodies):
            for j in range(i + 1, num_bodies):
                try:
                    shoulder_right1 = body_frame.get_body(i).joints[K4ABT_JOINT_SHOULDER_RIGHT].position
                    shoulder_left1 = body_frame.get_body(i).joints[K4ABT_JOINT_SHOULDER_LEFT].position
                    center1 = np.array([
                        (shoulder_right1.x + shoulder_left1.x) / 2,
                        (shoulder_right1.y + shoulder_left1.y) / 2,
                        (shoulder_right1.z + shoulder_left1.z) / 2
                    ])

                    shoulder_right2 = body_frame.get_body(j).joints[K4ABT_JOINT_SHOULDER_RIGHT].position
                    shoulder_left2 = body_frame.get_body(j).joints[K4ABT_JOINT_SHOULDER_LEFT].position
                    center2 = np.array([
                        (shoulder_right2.x + shoulder_left2.x) / 2,
                        (shoulder_right2.y + shoulder_left2.y) / 2,
                        (shoulder_right2.z + shoulder_left2.z) / 2
                    ])

                    distance = np.sqrt(np.sum((center2 - center1) ** 2))
                    distance_m = distance / 1000.0  # Konversi ke meter

                    y_offset = 50 + (i * 30)
                    cv2.putText(color_image,
                                f"Jarak {i+1}-{j+1}: {distance_m:.2f} m",
                                (50, y_offset),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.8, (0, 0, 250), 2)
                except Exception:
                    pass

    # 5. LOOP SETIAP TUBUH
    for body_index in range(num_bodies):
        body = body_frame.get_body(body_index)
        joints = body.joints

        # Inisialisasi label gesture/default
        gesture_label = "..."

        # 6. LOGIKA WORKOUT DETECTION (knee raise, shoulder front raise, side bend)
        try:
            # Ambil posisi 3D dari sendi yang relevan
            head_pos = joints[K4ABT_JOINT_HEAD].position
            pelvis_pos = joints[K4ABT_JOINT_PELVIS].position
            shoulder_right_pos = joints[K4ABT_JOINT_SHOULDER_RIGHT].position
            shoulder_left_pos = joints[K4ABT_JOINT_SHOULDER_LEFT].position
            knee_right_pos = joints[K4ABT_JOINT_KNEE_RIGHT].position
            knee_left_pos = joints[K4ABT_JOINT_KNEE_LEFT].position
            wrist_right_pos = joints[K4ABT_JOINT_WRIST_RIGHT].position
            wrist_left_pos = joints[K4ABT_JOINT_WRIST_LEFT].position

            # Inisialisasi state untuk body ini jika belum ada
            if body_index not in exercise_state:
                exercise_state[body_index] = {
                    'knee_count': 0,
                    'knee_state': 'down',
                    'shoulder_count': 0,
                    'shoulder_state': 'down',
                    'sidebend_count': 0,
                    'sidebend_state': 'center',
                    'success_sent': False,
                    'knee_led_sent': False,
                    'shoulder_led_sent': False,
                    'side_led_sent': False
                }

            st = exercise_state[body_index]

            # Hitung metrik dasar
            knee_avg_y = (knee_left_pos.y + knee_right_pos.y) / 2
            shoulder_avg_y = (shoulder_left_pos.y + shoulder_right_pos.y) / 2
            wrist_avg_y = (wrist_left_pos.y + wrist_right_pos.y) / 2

            # Standing Knee Raise -> jarak vertikal pelvis-ke-lutut
            knee_dist = abs(pelvis_pos.y - knee_avg_y)

            # Shoulder Front Raise -> selisih Y pergelangan tangan dan bahu
            wrist_shoulder_diff = wrist_avg_y - shoulder_avg_y

            # Side Bend -> selisih X kepala dan pelvis
            head_pelvis_x_diff = head_pos.x - pelvis_pos.x

            # Debug singkat
            print(f"[Body {body_index}] "
                  f"knee_dist: {knee_dist:.1f}, "
                  f"wrist-shoulder: {wrist_shoulder_diff:.1f}, "
                  f"head-pelvis-x: {head_pelvis_x_diff:.1f} "
                  f"target={body_index == selected_body_index}")

            # Hanya update counter jika body ini target
            is_target = (selected_body_index is not None and body_index == selected_body_index)

            # -------- STANDING KNEE RAISE --------
            if is_target and selected_exercise in ("knee", "all"):
                # Knee UP: lutut naik mendekati pelvis (knee_dist kecil)
                if knee_dist < (KNEE_UP_THRESH - KNEE_DEADZONE):
                    if st['knee_state'] == 'down':
                        st['knee_state'] = 'up'
                        st['knee_count'] += 1
                        print(f"Knee count: {st['knee_count']}")

                        # Kirim RED saat tepat 7x
                        if st['knee_count'] == 7 and not st['knee_led_sent']:
                            if ser is not None:
                                try:
                                    ser.write(b"RED\n")
                                    print("Mengirim RED (Knee Raise 7x)")
                                except Exception as e:
                                    print(f"Gagal kirim RED: {e}")
                            st['knee_led_sent'] = True

                # Knee DOWN: kaki lurus lagi (knee_dist besar)
                elif knee_dist > (KNEE_DOWN_THRESH + KNEE_DEADZONE):
                    st['knee_state'] = 'down'

            # -------- SHOULDER FRONT RAISE --------
            if is_target and selected_exercise in ("shoulder", "all"):
                # tangan naik cukup tinggi (di sekitar / di atas bahu)
                if wrist_shoulder_diff < (SHOULDER_UP_THRESH - SHOULDER_DEADZONE):
                    if st['shoulder_state'] == 'down':
                        st['shoulder_state'] = 'up'
                        st['shoulder_count'] += 1
                        print(f"Shoulder count: {st['shoulder_count']}")

                        # Kirim YELLOW saat tepat 7x
                        if st['shoulder_count'] == 7 and not st['shoulder_led_sent']:
                            if ser is not None:
                                try:
                                    ser.write(b"YELLOW\n")
                                    print("Mengirim YELLOW (Shoulder Raise 7x)")
                                except Exception as e:
                                    print(f"Gagal kirim YELLOW: {e}")
                            st['shoulder_led_sent'] = True

                # tangan kembali turun (jauh di bawah bahu)
                elif wrist_shoulder_diff > (SHOULDER_DOWN_THRESH + SHOULDER_DEADZONE):
                    st['shoulder_state'] = 'down'

            # -------- SIDE BEND (MIRING KIRI–KANAN) --------
            if is_target and selected_exercise in ("sidebend", "all"):
                # dari center, cek apakah miring cukup ke kiri / kanan
                if st['sidebend_state'] == 'center':
                    if head_pelvis_x_diff > SIDEBEND_RIGHT_THRESH:
                        st['sidebend_state'] = 'right'
                    elif head_pelvis_x_diff < SIDEBEND_LEFT_THRESH:
                        st['sidebend_state'] = 'left'
                else:
                    # jika sudah di kiri/kanan, tunggu sampai kembali ke tengah -> hitung 1 repetisi
                    if abs(head_pelvis_x_diff) < SIDEBEND_CENTER_THRESH:
                        st['sidebend_count'] += 1
                        print(f"Sidebend count: {st['sidebend_count']}")

                        # Kirim GREEN saat tepat 7x
                        if st['sidebend_count'] == 7 and not st['side_led_sent']:
                            if ser is not None:
                                try:
                                    ser.write(b"GREEN\n")
                                    print("Mengirim GREEN (Side Bend 7x)")
                                except Exception as e:
                                    print(f"Gagal kirim GREEN: {e}")
                            st['side_led_sent'] = True

                        st['sidebend_state'] = 'center'

            # -------- CEK TOTAL REP & KIRIM NOTIF KE ESP32 --------
            # syarat: knee + shoulder + sidebend >= 15
            if is_target:
                total_rep = st['knee_count'] + st['shoulder_count'] + st['sidebend_count']
                if total_rep >= 15 and not st['success_sent']:
                    if ser is not None:
                        try:
                            ser.write(b"SUCCESS\n")
                            print(f"Mengirim SUCCESS ke ESP32 (total rep: {total_rep})")
                        except Exception as e:
                            print(f"Gagal mengirim SUCCESS: {e}")
                    st['success_sent'] = True

            # Tampilkan ringkasan hanya untuk target, atau "Other" untuk non-target
            if is_target:
                if selected_exercise == "knee":
                    gesture_label = f"Knee:{st['knee_count']}"
                elif selected_exercise == "shoulder":
                    gesture_label = f"Sh:{st['shoulder_count']}"
                elif selected_exercise == "sidebend":
                    gesture_label = f"Side:{st['sidebend_count']}"
                else:
                    gesture_label = (f"Knee:{st['knee_count']} "
                                     f"Sh:{st['shoulder_count']} "
                                     f"Side:{st['sidebend_count']}")
            else:
                gesture_label = "Other"

        except Exception as e:
            print(f"Error deteksi workout: {e}")
            gesture_label = "Other" if (selected_body_index is None or body_index != selected_body_index) else "..."

        # 7. VISUALISASI KERANGKA
        joint_2d_positions = {}
        head_2d_pos = None

        for j in range(K4ABT_JOINT_COUNT):
            joint = joints[j]
            pos_3d = joint.position

            # Konversi 3D ke 2D menggunakan helper function
            pos_2d = convert_to_2d(calibration, pos_3d)
            if pos_2d is not None:
                x, y = pos_2d
                if 0 <= x < w and 0 <= y < h:
                    joint_2d_positions[j] = (x, y)

                    if j == K4ABT_JOINT_HEAD:
                        head_2d_pos = (x, y)

                    color = (0, 255, 0) if (selected_body_index == body_index) else (80, 80, 80)
                    cv2.circle(color_image, (x, y), 3, color, -1)

            # 8. SIMPAN DATA CSV (satu baris per joint)
            timestamp = time.time()
            joint_name = K4ABT_JOINT_NAMES[j]
            csv_writer.writerow([timestamp, body_index, joint_name, pos_3d.x, pos_3d.y, pos_3d.z])

        # Gambar TULANG (bones)
        bones = [
            (K4ABT_JOINT_HEAD, K4ABT_JOINT_NECK),
            (K4ABT_JOINT_NECK, K4ABT_JOINT_SPINE_CHEST),
            (K4ABT_JOINT_SPINE_CHEST, K4ABT_JOINT_SPINE_NAVEL),
            (K4ABT_JOINT_SPINE_NAVEL, K4ABT_JOINT_PELVIS),

            (K4ABT_JOINT_SPINE_CHEST, K4ABT_JOINT_CLAVICLE_RIGHT),
            (K4ABT_JOINT_CLAVICLE_RIGHT, K4ABT_JOINT_SHOULDER_RIGHT),
            (K4ABT_JOINT_SHOULDER_RIGHT, K4ABT_JOINT_ELBOW_RIGHT),
            (K4ABT_JOINT_ELBOW_RIGHT, K4ABT_JOINT_WRIST_RIGHT),

            (K4ABT_JOINT_SPINE_CHEST, K4ABT_JOINT_CLAVICLE_LEFT),
            (K4ABT_JOINT_CLAVICLE_LEFT, K4ABT_JOINT_SHOULDER_LEFT),
            (K4ABT_JOINT_SHOULDER_LEFT, K4ABT_JOINT_ELBOW_LEFT),
            (K4ABT_JOINT_ELBOW_LEFT, K4ABT_JOINT_WRIST_LEFT),

            (K4ABT_JOINT_PELVIS, K4ABT_JOINT_HIP_RIGHT),
            (K4ABT_JOINT_HIP_RIGHT, K4ABT_JOINT_KNEE_RIGHT),
            (K4ABT_JOINT_KNEE_RIGHT, K4ABT_JOINT_ANKLE_RIGHT),

            (K4ABT_JOINT_PELVIS, K4ABT_JOINT_HIP_LEFT),
            (K4ABT_JOINT_HIP_LEFT, K4ABT_JOINT_KNEE_LEFT),
            (K4ABT_JOINT_KNEE_LEFT, K4ABT_JOINT_ANKLE_LEFT),
        ]

        for joint1, joint2 in bones:
            if joint1 in joint_2d_positions and joint2 in joint_2d_positions:
                pos1 = joint_2d_positions[joint1]
                pos2 = joint_2d_positions[joint2]
                line_color = (255, 0, 0) if (selected_body_index == body_index) else (120, 120, 120)
                cv2.line(color_image, pos1, pos2, line_color, 2)

        # Tampilkan LABEL di atas kepala
        if head_2d_pos:
            text_x = max(5, head_2d_pos[0] - 80)
            text_y = max(20, head_2d_pos[1] - 40)
            (text_w, text_h), _ = cv2.getTextSize(gesture_label,
                                                  cv2.FONT_HERSHEY_SIMPLEX,
                                                  0.8, 2)
            cv2.rectangle(color_image,
                          (text_x-6, text_y-text_h-6),
                          (text_x+text_w+6, text_y+6),
                          (0, 0, 0), -1)
            cv2.putText(color_image, gesture_label, (text_x, text_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        else:
            cv2.putText(color_image, gesture_label,
                        (10, 30 + 20 * body_index),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

    # 9. TAMPILKAN HASIL
    cv2.imshow("Azure Kinect Body Tracking", color_image)

    # Keluar jika tombol 'q' ditekan
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

print("Menutup sistem...")
csv_file.close()
device.stop_cameras()
cv2.destroyAllWindows()
if ser is not None:
    ser.close()
print("Selesai.")
