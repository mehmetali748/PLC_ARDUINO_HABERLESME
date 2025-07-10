#include <Wire.h>
#include <SoftwareSerial.h>
#include "LoRa_E22.h"

#define M0 7
#define M1 6
#define SLAVE_ADDR 0x08
#define GONDEREN_ADDR 6
#define Kanal 18

const uint8_t ACK_BYTE = 0x06;

SoftwareSerial mySerial(3, 4);
LoRa_E22 FixajSS(&mySerial);

uint8_t receivedFlags[5], previousFlags[5] = {0};
bool newData = false;
bool receivedFromLoRa = false;

unsigned long lastLoRaSendTime = 0;
const unsigned long loRaSendInterval = 50;
unsigned long lastCommandTime = 0;

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

SensorData lastSensorData;
Signal dataToSend = { "Fixaj.com", 0, 0, 0, 0, 0, 0 };

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

// ✅ I2C Master veri istediğinde 4 byte döndür
void onI2CRequest() {
  Wire.write(lastSensorData.sensor_1);
  Wire.write(lastSensorData.sensor_2);
  Wire.write(lastSensorData.sensor_3);
  Wire.write(lastSensorData.sensor_4);
}

void trySendLoRaDataWithRetry() {
  const int maxRetries = 5;
  int attempt = 0;
  bool ackReceived = false;

  dataToSend.crc = calculateCRC8((uint8_t*)&dataToSend, sizeof(dataToSend) - 1);

  while (attempt < maxRetries && !ackReceived) {
    Serial.print("[LoRa] Gönderim denemesi: "); Serial.println(attempt + 1);

    FixajSS.sendFixedMessage(highByte(GONDEREN_ADDR), lowByte(GONDEREN_ADDR), Kanal, &dataToSend, sizeof(dataToSend));
    unsigned long ackWaitStart = millis();

    while (millis() - ackWaitStart < 300) {
      if (FixajSS.available()) {
        uint8_t ackByte;
        auto ackRsp = FixajSS.receiveMessage(1);
        memcpy(&ackByte, ackRsp.data, 1);
        ackRsp.close();

        if (ackByte == ACK_BYTE) {
          ackReceived = true;
          Serial.println("[LoRa] ACK alındı.");
          break;
        }
      }
    }

    if (!ackReceived) {
      Serial.println("[LoRa] ACK alınamadı. Tekrar deneniyor...");
      delay(50);
    }

    attempt++;
  }

  if (!ackReceived) {
    Serial.println("[LoRa] ACK başarısız. Gönderim başarısız sayıldı.");
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin(SLAVE_ADDR);

  Wire.onReceive([](int count) {
    if (count >= 5) {
      for (int i = 0; i < 5; i++) {
        receivedFlags[i] = Wire.read();
      }
      lastCommandTime = millis();
      newData = true;
    }
  });

  Wire.onRequest(onI2CRequest);

  FixajSS.begin();
  pinMode(M0, OUTPUT); pinMode(M1, OUTPUT);
  digitalWrite(M0, LOW); digitalWrite(M1, HIGH);
  delay(100);

  auto c = FixajSS.getConfiguration();
  Configuration cfg = *(Configuration*)c.data;
  cfg.ADDL = lowByte(5); cfg.ADDH = highByte(5); cfg.NETID = 0;
  cfg.CHAN = Kanal;
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
  if (FixajSS.available() >= sizeof(SensorData)) {
    auto rsp = FixajSS.receiveMessage(sizeof(SensorData));
    SensorData incoming;
    memcpy(&incoming, rsp.data, sizeof(incoming));
    rsp.close();

    uint8_t calc_crc = calculateCRC8((uint8_t*)&incoming, sizeof(incoming) - 1);
    if (calc_crc == incoming.crc) {
      bool changed = memcmp(&incoming, &lastSensorData, sizeof(SensorData)) != 0;

      if (changed) {
        lastSensorData = incoming;

        Serial.println("====== GELEN VERİ (LoRa'dan Sensör) ======");
        Serial.print("sensor_1: "); Serial.println(incoming.sensor_1);
        Serial.print("sensor_2: "); Serial.println(incoming.sensor_2);
        Serial.print("sensor_3: "); Serial.println(incoming.sensor_3);
        Serial.print("sensor_4: "); Serial.println(incoming.sensor_4);
        Serial.print("veri_5  : "); Serial.println(incoming.veri_5);
        Serial.println("==========================================");

        Serial.println("[I2C] Sensör verisi master'a hazırlandı.");
      }

      receivedFromLoRa = true;
    } else {
      Serial.println("[LoRa] CRC HATASI: Sensör verisi reddedildi.");
    }
  }

  if (newData) {
    uint8_t changeMask = 0;
    for (int i = 0; i < 5; i++) {
      if (receivedFlags[i] != previousFlags[i]) {
        changeMask |= (1 << i);
      }
    }

    bool changed = (changeMask != 0);
    bool forceSend = (!changed && receivedFromLoRa);

    if ((changed || forceSend || (millis() - lastCommandTime < 100)) &&
        (millis() - lastLoRaSendTime >= loRaSendInterval)) {

      memcpy(&dataToSend.role_ileri_a, receivedFlags, 5);
      strcpy(dataToSend.type, "Fixaj.com");

      Serial.println("====== GİDEN VERİ (LoRa'ya Komut) ======");
      Serial.print("ileri_a: "); Serial.println(dataToSend.role_ileri_a);
      Serial.print("geri_a : "); Serial.println(dataToSend.role_geri_a);
      Serial.print("ileri_k: "); Serial.println(dataToSend.role_ileri_k);
      Serial.print("geri_k : "); Serial.println(dataToSend.role_geri_k);
      Serial.print("fren   : "); Serial.println(dataToSend.role_a_fren);
      Serial.println("=========================================");

      trySendLoRaDataWithRetry();

      memcpy(previousFlags, receivedFlags, 5);
      receivedFromLoRa = false;
      lastLoRaSendTime = millis();
    } else {
      Serial.println("[LoRa] Komut gönderilmedi (değişmedi veya debounce).");
    }

    newData = false;
  }
}
