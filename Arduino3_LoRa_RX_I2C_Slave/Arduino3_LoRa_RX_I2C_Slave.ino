#include <Wire.h>
#include <SoftwareSerial.h>
#include "LoRa_E22.h"

#define M0 7
#define M1 6
#define I2C_SLAVE_ADDR 0x08
#define LORA_ADDR 6
#define LORA_CHAN 18

const uint8_t ACK_BYTE = 0x06;

SoftwareSerial mySerial(3, 4);
LoRa_E22 FixajSS(&mySerial);

uint8_t i2cFlags[4], lastI2CFlags[4] = {0};
volatile bool gotFromI2C = false;
bool receivedFromLoRa = false;

unsigned long lastLoRaSendTime = 0;
const unsigned long loRaSendInterval = 50;

struct Signal {
  char type[15];
  uint8_t role_ileri_a;
  uint8_t role_geri_a;
  uint8_t role_ileri_k;
  uint8_t role_geri_k;
  uint8_t role_a_fren;
  uint8_t crc;
};

struct SensorData {
  char type[15];
  uint8_t sensor_1;
  uint8_t sensor_2;
  uint8_t sensor_3;
  uint8_t sensor_4;
  uint8_t veri_5;
  uint8_t crc;
};

Signal dataToSend;
Signal lastReceivedSignal;
SensorData sensorData = { "Fixaj.com", 0, 0, 0, 0, 0, 0 };

uint8_t calculateCRC8(uint8_t *data, uint8_t length) {
  uint8_t crc = 0x00;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
    }
  }
  return crc;
}

void onI2CReceive(int count) {
  if (count >= 4) {
    for (int i = 0; i < 4; i++) {
      i2cFlags[i] = Wire.read();
    }
    gotFromI2C = true;
  } else {
    while (Wire.available()) Wire.read();
  }
}

void onI2CRequest() {
  uint8_t buf[5] = {
    dataToSend.role_ileri_a,
    dataToSend.role_geri_a,
    dataToSend.role_ileri_k,
    dataToSend.role_geri_k,
    dataToSend.role_a_fren
  };
  Wire.write(buf, 5);
  Serial.println("[I2C] Master komut istedi, gönderildi.");
}

void setup() {
  Serial.begin(9600);
  Wire.begin(I2C_SLAVE_ADDR);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);

  FixajSS.begin();
  pinMode(M0, OUTPUT); pinMode(M1, OUTPUT);
  digitalWrite(M0, LOW); digitalWrite(M1, HIGH);
  delay(100);

  auto c = FixajSS.getConfiguration();
  Configuration cfg = *(Configuration*)c.data;
  cfg.ADDL = lowByte(LORA_ADDR); cfg.ADDH = highByte(LORA_ADDR);
  cfg.NETID = 0; cfg.CHAN = LORA_CHAN;
  cfg.SPED.airDataRate = AIR_DATA_RATE_010_24;
  cfg.OPTION.transmissionPower = POWER_22;
  cfg.SPED.uartBaudRate = UART_BPS_9600;
  cfg.SPED.uartParity = MODE_00_8N1;
  cfg.TRANSMISSION_MODE.fixedTransmission = FT_FIXED_TRANSMISSION;
  FixajSS.setConfiguration(cfg, WRITE_CFG_PWR_DWN_SAVE);
  c.close();

  digitalWrite(M0, LOW); digitalWrite(M1, LOW);
  Serial.println("[Arduino2] Sistem başlatıldı.");
}

void loop() {
  // LoRa'dan komut al
  if (FixajSS.available() >= sizeof(Signal)) {
    auto rsp = FixajSS.receiveMessage(sizeof(Signal));
    Signal incoming;
    memcpy(&incoming, rsp.data, sizeof(incoming));
    rsp.close();

    uint8_t calc_crc = calculateCRC8((uint8_t*)&incoming, sizeof(incoming) - 1);
    if (calc_crc == incoming.crc) {
      bool changed = false;
      changed |= incoming.role_ileri_a != lastReceivedSignal.role_ileri_a;
      changed |= incoming.role_geri_a  != lastReceivedSignal.role_geri_a;
      changed |= incoming.role_ileri_k != lastReceivedSignal.role_ileri_k;
      changed |= incoming.role_geri_k  != lastReceivedSignal.role_geri_k;
      changed |= incoming.role_a_fren  != lastReceivedSignal.role_a_fren;

      if (changed) {
        lastReceivedSignal = incoming;
        dataToSend = incoming;

        Serial.println("====== GELEN VERİ (LoRa'dan Komut) ======");
        Serial.print("ileri_a: "); Serial.println(dataToSend.role_ileri_a);
        Serial.print("geri_a : "); Serial.println(dataToSend.role_geri_a);
        Serial.print("ileri_k: "); Serial.println(dataToSend.role_ileri_k);
        Serial.print("geri_k : "); Serial.println(dataToSend.role_geri_k);
        Serial.print("fren   : "); Serial.println(dataToSend.role_a_fren);
        Serial.println("=========================================");
      } else {
        Serial.println("[LoRa] Komut verisi aynı, işlenmedi.");
      }

      receivedFromLoRa = true;
      FixajSS.sendFixedMessage(highByte(5), lowByte(5), LORA_CHAN, &ACK_BYTE, sizeof(ACK_BYTE));
      Serial.println("[LoRa] ACK gönderildi.");
      delay(30);
    } else {
      Serial.println("[LoRa] CRC HATASI: Komut paketi geçersiz.");
    }
  }

  // Sensör verisini LoRa ile gönder
  if (gotFromI2C) {
    bool changed = false;
    Serial.print("[DEBUG] i2cFlags: ");
    for (int i = 0; i < 4; i++) { Serial.print(i2cFlags[i]); Serial.print(" "); }
    Serial.println();
    Serial.print("[DEBUG] lastI2CFlags: ");
    for (int i = 0; i < 4; i++) { Serial.print(lastI2CFlags[i]); Serial.print(" "); }
    Serial.println();

    for (int i = 0; i < 4; i++) {
      if (i2cFlags[i] != lastI2CFlags[i]) {
        changed = true;
        break;
      }
    }

    bool forceSend = false;
    if (!changed && receivedFromLoRa) {
      forceSend = true;
      Serial.println("[LoRa] Sensör verisi aynı ama komut alındığı için ZORLA GÖNDERİLİYOR.");
    }

    if ((changed || forceSend) && (millis() - lastLoRaSendTime >= loRaSendInterval)) {
      memcpy(&sensorData.sensor_1, i2cFlags, 4);
      sensorData.veri_5 = 0;
      strcpy(sensorData.type, "Fixaj.com");
      sensorData.crc = calculateCRC8((uint8_t*)&sensorData, sizeof(sensorData) - 1);

      FixajSS.sendFixedMessage(highByte(5), lowByte(5), LORA_CHAN, &sensorData, sizeof(sensorData));
      Serial.println("====== GİDEN VERİ (LoRa'ya Sensör) ======");
      Serial.print("sensor_1: "); Serial.println(sensorData.sensor_1);
      Serial.print("sensor_2: "); Serial.println(sensorData.sensor_2);
      Serial.print("sensor_3: "); Serial.println(sensorData.sensor_3);
      Serial.print("sensor_4: "); Serial.println(sensorData.sensor_4);
      Serial.print("veri_5  : "); Serial.println(sensorData.veri_5);
      Serial.println("=========================================");

      lastLoRaSendTime = millis();
      if (changed || forceSend) memcpy(lastI2CFlags, i2cFlags, 4);
      receivedFromLoRa = false;
      delay(30);
    }

    gotFromI2C = false;
  }
}
