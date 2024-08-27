#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int servoMin = 140;
int servoMax = 680;

int currentAngle = 0;
unsigned long lastPrintTime = 0;
const unsigned long printInterval = 1000;

void setup() {
  pwm.begin();
  pwm.setPWMFreq(60);

  Serial.begin(115200);
  Serial.println("Servo Controller siap. Masukkan sudut antara 0 hingga 180.");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int angle = input.toInt();

    if (angle >= 0 && angle <= 360) {
      int pulseLength = map(angle, 0, 180, servoMin, servoMax);

      // Control all 16 channels with the same pulse length
      for (int channel = 0; channel < 16; channel++) {
        pwm.setPWM(channel, 0, pulseLength);
      }

      currentAngle = angle;

      Serial.print("Servo bergerak ke ");
      Serial.print(angle);
      Serial.println(" derajat pada semua channel");
    } else {
      Serial.println("Nilai tidak valid. Masukkan sudut antara 0 hingga 180.");
    }
  }

  if (millis() - lastPrintTime >= printInterval) {
    lastPrintTime = millis();
    Serial.print("Posisi servo saat ini: ");
    Serial.print(currentAngle);
    Serial.println(" derajat pada semua channel");
  }
}
