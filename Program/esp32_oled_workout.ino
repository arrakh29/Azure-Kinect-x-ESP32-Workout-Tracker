#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// PIN LED – sesuaikan wiring kamu
#define LED_RED     14   // LED Merah  -> GPIO14
#define LED_YELLOW  27   // LED Kuning -> GPIO27
#define LED_GREEN   26   // LED Hijau  -> GPIO26

// Fungsi bantu untuk menampilkan tulisan di OLED
void showMessage(const char* line1, const char* line2 = "") {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println(line1);
  if (line2 && line2[0] != '\0') {
    display.println(line2);
  }
  display.display();
}

void setup() {
  Serial.begin(115200);

  // Inisialisasi OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    // Kalau gagal, bisa debug via Serial:
    // Serial.println("Gagal inisialisasi OLED");
    return;
  }

  // Inisialisasi pin LED
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  // Matikan semua LED di awal
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_GREEN, LOW);

  // Pesan awal di OLED
  showMessage("Belum ada data");
  Serial.println("ESP32 siap, menunggu data serial...");
}

void loop() {
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    msg.trim();  // buang spasi & newline

    if (msg.length() == 0) return;

    Serial.print("Terima dari PC: ");
    Serial.println(msg);

    if (msg == "RED") {
      // Knee Raise 7x
      showMessage("Knee Raise", "7x Selesai");
      digitalWrite(LED_RED, HIGH);
      // Kalau mau hanya 1 LED yang nyala, bisa:
      // digitalWrite(LED_YELLOW, LOW);
      // digitalWrite(LED_GREEN, LOW);
    }
    else if (msg == "YELLOW") {
      // Shoulder Front Raise 7x
      showMessage("Shoulder Raise", "7x Selesai");
      digitalWrite(LED_YELLOW, HIGH);
      // digitalWrite(LED_RED, LOW);
      // digitalWrite(LED_GREEN, LOW);
    }
    else if (msg == "GREEN") {
      // Side Bend 7x
      showMessage("Side Bend", "7x Selesai");
      digitalWrite(LED_GREEN, HIGH);
      // digitalWrite(LED_RED, LOW);
      // digitalWrite(LED_YELLOW, LOW);
    }
    else if (msg == "SUCCESS") {
      // Total rep >= 15
      showMessage("Semua Latihan", "Selesai!");
      // Kalau mau semua LED ON:
      // digitalWrite(LED_RED, HIGH);
      // digitalWrite(LED_YELLOW, HIGH);
      // digitalWrite(LED_GREEN, HIGH);
    }
    else {
      // Data lain, kalau nanti kamu kirim teks berbeda
      showMessage("Data:", msg.c_str());
    }
  }

  // Tidak pakai delay besar supaya respon ke serial tetap cepat
}
