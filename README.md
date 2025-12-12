# Azure Kinect Workout Tracker x ESP32

Aplikasi komputer visi untuk menghitung repetisi latihan (standing knee raise, shoulder front raise, side bend) memakai Azure Kinect Body Tracking. Hasil hitung dan data pose dicatat ke CSV, lalu milestone dikirim via serial ke ESP32 untuk menyalakan LED indikator dan menampilkan status di OLED.

## Arsitektur Singkat
- Azure Kinect DK + Body Tracking SDK untuk deteksi skeleton real-time.
- Skrip Python `workout.py` memfilter tubuh dalam ROI, menghitung repetisi, dan menyimpan pose ke `data_gerakan.csv`.
- Komunikasi serial ke ESP32 (`COM8` contoh), mengirim pesan `RED`, `YELLOW`, `GREEN`, `SUCCESS` sesuai progres.
- ESP32 (`esp32_oled_workout.ino`) menyalakan LED Merah/Kuning/Hijau dan menampilkan status di OLED SSD1306 128x64 (I2C 0x3C, pin LED: 14, 27, 26).

## Fitur Utama
- Pemilihan tubuh otomatis di tengah ROI, cocok untuk area gym ramai.
- Hitung repetisi tiga gerakan dengan threshold terpisah dan deadzone untuk mengurangi jitter.
- Log lengkap pose 3D semua joint ke CSV untuk analisis atau training model lanjut.
- Notifikasi hardware: milestone 7x per gerakan + total 15 rep dikirim ke ESP32 (LED + OLED).
- Pilihan mode latihan lewat prompt: `knee`, `shoulder`, `sidebend`, atau `all`.

## Prasyarat
**Perangkat keras:** Azure Kinect DK, PC Windows, ESP32 board, OLED SSD1306 I2C, LED Merah/Kuning/Hijau + resistor.
**Perangkat lunak PC:**
- Python 3.9+.
- Azure Kinect SDK + Body Tracking SDK (pastikan environment sudah mengenali kamera).
- Paket Python: `opencv-python`, `numpy`, `pykinect-azure`, `pyserial`.
**Perangkat lunak ESP32:**
- Arduino IDE / PlatformIO.
- Library `Adafruit_SSD1306` dan `Adafruit_GFX`.

## Cara Menjalankan di PC
1) Siapkan environment (opsional):
```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -U opencv-python numpy pykinect-azure pyserial
```
2) Hubungkan Azure Kinect, buka area pandang, dan pastikan SDK berjalan.
3) Hubungkan ESP32 ke USB, catat port COM lalu sesuaikan di `workout.py` (default `COM8`).
4) Jalankan skrip:
```powershell
python workout.py
```
5) Pilih mode latihan saat diminta. Tekan `q` di jendela kamera untuk keluar. File `data_gerakan.csv` akan diisi otomatis.

## Cara Menyiapkan ESP32
1) Buka `esp32_oled_workout.ino` di Arduino IDE.
2) Pastikan pin LED sesuai wiring (Merah=14, Kuning=27, Hijau=26) dan OLED I2C address 0x3C.
3) Install library `Adafruit_SSD1306` dan `Adafruit_GFX` via Library Manager.
4) Upload ke board ESP32, lalu sambungkan ke PC. Serial 115200 baud.

## Protokol Serial
- `RED`    -> Knee Raise selesai 7x, nyalakan LED merah.
- `YELLOW` -> Shoulder Front Raise selesai 7x, nyalakan LED kuning.
- `GREEN`  -> Side Bend selesai 7x, nyalakan LED hijau.
- `SUCCESS`-> Total rep (knee+shoulder+sidebend) >= 15, tampil pesan selesai.

## Data yang Disimpan
`data_gerakan.csv` berisi: `timestamp`, `body_index`, `joint_name`, `pos_x`, `pos_y`, `pos_z`. Satu baris per joint per frame untuk analisis lanjut atau training model.

## Tips Penggunaan
- Pastikan tubuh target berada di dalam kotak ROI (garis kuning) agar terpilih.
- Jaga jarak kamera 1.5-3 m dan pencahayaan cukup untuk stabilitas tracking.
- Sesuaikan threshold jika ingin gerakan lebih/kurang sensitif (lihat konstanta di `workout.py`).
- Jika port serial berbeda, ganti `COM8` sesuai Device Manager.

## Rencana Lanjut
- Tambah video demo dan diagram sistem (akan kamu lampirkan).
- Simpan log kesalahan atau durasi per rep untuk analitik sederhana.
- Tambah mode latihan baru atau kalibrasi otomatis.
  
## **Diagram System**
<img src="system%20task%20diagram4.png" alt="Pinout Diagram" style="max-width: 600px; height: auto;">

## Uji Coba
<img src="Ex_4.gif" alt="Uji Coba GIF" style="max-width: 600px; height: auto">

Selamat berlatih! Jika ada error SDK atau serial, cek koneksi perangkat dan izin port.

