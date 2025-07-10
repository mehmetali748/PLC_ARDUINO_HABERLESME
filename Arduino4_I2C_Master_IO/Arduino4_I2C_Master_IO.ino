#include <Wire.h>

#define SLAVE_ADDR 0x08

bool receivedFlags[5];    // I2C'den gelen 5 bayrak (0–3: çıkışlar, 4: global reset)
bool sendFlags[4];        // Dijital girişlerden okunan değerler
bool latchedFlags[4] = {false, false, false, false}; // Çıkışların mühür durumları

void setup() {
  Serial.begin(9600);
  Serial.println("🔧 Arduino4 Başlatıldı - I2C Master + Dijital IO");

  Wire.begin(); // I2C master olarak başlat

  // Dijital çıkış pinleri (2–5)
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);

  // Dijital giriş pinleri (6–9), pull-up aktif
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
}

void loop() {
  Serial.println("------------------------------------------------");

  // 📥 I2C'den 5 bayt veri çek
  Wire.requestFrom(SLAVE_ADDR, (uint8_t)5);
  for (uint8_t i = 0; i < 5 && Wire.available(); i++) {
    receivedFlags[i] = Wire.read() != 0;
  }

  // 📖 Dijital giriş pinlerinden veri oku (6–9)
  sendFlags[0] = digitalRead(6);
  sendFlags[1] = digitalRead(7);
  sendFlags[2] = digitalRead(8);
  sendFlags[3] = digitalRead(9);

  // 🧹 I2C ile gelen 5. bayrak (index 4) true ise: TÜM mühürleri sıfırla
  if (receivedFlags[4]) {
    Serial.println("🧹 GLOBAL RESET AKTİF! Tüm mühürler sıfırlandı.");
    for (int i = 0; i < 4; i++) {
      latchedFlags[i] = false;
      digitalWrite(i + 2, LOW);
    }
  }

  // 🔁 Mühürleme ve sıfırlama kontrolü (çıkış pinleri: 2–5)
  for (int i = 0; i < 4; i++) {
    uint8_t outPin = i + 2;
    bool inputState = sendFlags[i]; // Bağlı sensör durumu (6+i)

    if (latchedFlags[i]) {
      if (inputState == LOW) {
        // Sensör tetiklendi (LOW), mühür çözülür
        latchedFlags[i] = false;
        digitalWrite(outPin, LOW);
        Serial.print("🔓 MÜHÜR BOZULDU → Pin ");
        Serial.print(outPin); Serial.println(" → LOW");
      } else {
        // Mühürlü kalır
        digitalWrite(outPin, HIGH);
        Serial.print("🔒 Pin "); Serial.print(outPin); Serial.println(" mühürlü → HIGH");
      }
    } else {
      if (receivedFlags[i] && inputState == HIGH) {
        // Sensör serbestken (HIGH) ve veri true → mühürle
        latchedFlags[i] = true;
        digitalWrite(outPin, HIGH);
        Serial.print("🔐 MÜHÜRLENDİ → Pin ");
        Serial.print(outPin); Serial.println(" → HIGH");
      } else {
        digitalWrite(outPin, LOW);
        Serial.print("⏳ Bekliyor → Pin ");
        Serial.print(outPin); Serial.println(" → LOW");
      }
    }
  }

  // 📤 Dijital giriş verilerini I2C ile slave'e geri gönder
  Wire.beginTransmission(SLAVE_ADDR);
  for (uint8_t i = 0; i < 4; i++) {
    Wire.write(sendFlags[i] ? 1 : 0);
  }
  byte result = Wire.endTransmission();

  if (result == 0) {
    Serial.println("✅ I2C veri gönderimi başarılı.");
  } else {
    Serial.print("❌ I2C HATASI! Kod: ");
    Serial.println(result);
  }

  for (uint8_t i = 0; i < 4 ; i++) {
    receivedFlags[i] = false;
  }
  delay(500); // 500ms bekle
}
