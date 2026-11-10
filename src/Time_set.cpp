#include <Wire.h>
#include "RTClib.h"

RTC_DS3231 rtc;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);

  if (!rtc.begin()) {
    Serial.println("[ERROR] RTC not found on I2C bus!");
    while (1);
  }

  // 1. Read existing Control Register (0x0E)
  Wire.beginTransmission(0x68);
  Wire.write(0x0E);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 1);
  byte ctl = Wire.read();

  Serial.printf("Current Control Register (0x0E): 0x%02X\n", ctl);

  if (ctl & (1 << 7)) {
    Serial.println("[FOUND ISSUE] Bit 7 (!EOSC) was set to 1! The oscillator was programmed to halt on battery.");
  } else {
    Serial.println("[OK] Bit 7 (!EOSC) was already 0.");
  }

  // 2. Clear Bit 7 (!EOSC = 0 forces oscillator to RUN on battery)
  ctl &= ~(1 << 7);
  Wire.beginTransmission(0x68);
  Wire.write(0x0E);
  Wire.write(ctl);
  Wire.endTransmission();

  // 3. Clear Oscillator Stop Flag (OSF in 0x0F)
  Wire.beginTransmission(0x68);
  Wire.write(0x0F);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 1);
  byte stat = Wire.read();
  stat &= ~(1 << 7);

  Wire.beginTransmission(0x68);
  Wire.write(0x0F);
  Wire.write(stat);
  Wire.endTransmission();

  // 4. Synchronize clock to current compile time
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  Serial.println("\nTime set and oscillator unlocked for battery operation!");
}

void loop() {
  DateTime now = rtc.now();
  Serial.printf("Live RTC: %04d-%02d-%02d %02d:%02d:%02d\n",
                now.year(), now.month(), now.day(),
                now.hour(), now.minute(), now.second());
  delay(1000);
}