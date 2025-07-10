#include <Wire.h>
#include <SPI.h>
#include <Ethernet.h>
#include <Modbus.h>
#include <ModbusIP.h>

const uint8_t SLAVE_ADDR = 0x08;

const int setpoint_HR  = 100;
const int setpoint_HRC = 101;
const int setpoint_HR1 = 102;
const int setpoint_HR2 = 103;
const int setpoint_HR3 = 104;
const int reg5  = 10;
const int reg6  = 11;
const int reg7  = 12;
const int reg8  = 13;
const int reg9  = 14;

bool receivedFlags[4] = {0};  // Artık sadece 4 byte
bool previousReceivedFlags[4] = {0};

bool sendFlags[5] = {0};
bool previousSendFlags[5] = {0};

ModbusIP mb;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
  byte ip[]  = {192, 168, 6, 100};
  mb.config(mac, ip);

  mb.addIreg(setpoint_HR);  // 0
  mb.addIreg(setpoint_HRC); // 1
  mb.addIreg(setpoint_HR1); // 2
  mb.addIreg(setpoint_HR2); // 3
  mb.addIreg(setpoint_HR3); // 4 - kullanılmıyor ama duruyor
  mb.addHreg(reg5); mb.addHreg(reg6);
  mb.addHreg(reg7); mb.addHreg(reg8); mb.addHreg(reg9);

  Serial.println("🔌 [Başlatıldı] Modbus + I2C Master Hazır.");
}

void loop() {
  mb.task();

  // 🟢 I2C'den 4 byte sensör verisi al
  Wire.requestFrom(SLAVE_ADDR, (uint8_t)4);
  bool changedRecv = false;
  for (uint8_t i = 0; i < 4 && Wire.available(); i++) {
    receivedFlags[i] = Wire.read() != 0;
    if (receivedFlags[i] != previousReceivedFlags[i]) {
      changedRecv = true;
    }
  }

  if (changedRecv) {
    for (int i = 0; i < 4; i++) previousReceivedFlags[i] = receivedFlags[i];

    Serial.print("📥 [I2C] Yeni sensör verisi alındı: ");
    for (int i = 0; i < 4; i++) Serial.print(receivedFlags[i]), Serial.print(" ");
    Serial.println();

    // Modbus Input Register’lara yaz
    mb.Ireg(setpoint_HR,  receivedFlags[0]);
    mb.Ireg(setpoint_HRC, receivedFlags[1]);
    mb.Ireg(setpoint_HR1, receivedFlags[2]);
    mb.Ireg(setpoint_HR2, receivedFlags[3]);
    // setpoint_HR3 (index 4) kullanılmıyor artık
  }

  // 🔄 Modbus Holding Register’dan komut oku
  int vals[] = {
    mb.Hreg(reg5), mb.Hreg(reg6),
    mb.Hreg(reg7), mb.Hreg(reg8),
    mb.Hreg(reg9)
  };

  bool changedSend = false;
  for (int i = 0; i < 5; i++) {
    sendFlags[i] = (vals[i] == 100);
    if (sendFlags[i] != previousSendFlags[i]) {
      changedSend = true;
    }
  }

  if (changedSend) {
    for (int i = 0; i < 5; i++) previousSendFlags[i] = sendFlags[i];

    Wire.beginTransmission(SLAVE_ADDR);
    for (uint8_t i = 0; i < 5; i++) {
      Wire.write(sendFlags[i] ? 1 : 0);
    }
    Wire.endTransmission();

    Serial.print("📤 [Modbus→I2C] Yeni komut verisi gönderildi: ");
    for (int i = 0; i < 5; i++) Serial.print(sendFlags[i]), Serial.print(" ");
    Serial.println();

    Serial.print("📘 [Modbus] Holding Reg -> ");
    for (int i = 0; i < 5; i++) Serial.print(vals[i]), Serial.print(" ");
    Serial.println();
  }

  delay(300);
}
