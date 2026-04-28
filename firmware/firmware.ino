const int fsrPins[] = {36, 39, 34, 35};
const int whiteLed = 22;
const int redLed = 23;

unsigned long lastSend = 0;

void setup() {
  Serial.begin(115200);
  ledcAttach(whiteLed, 5000, 8);
  ledcAttach(redLed, 5000, 8);
}

void loop() {
  if (millis() - lastSend >= 100) {
    lastSend = millis();
    Serial.printf("%d,%d,%d,%d\n",
      analogRead(fsrPins[0]), analogRead(fsrPins[1]),
      analogRead(fsrPins[2]), analogRead(fsrPins[3]));
  }

  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    int comma = line.indexOf(',');
    if (comma > 0) {
      int whiteDuty = constrain(line.substring(0, comma).toInt(), 0, 100);
      int redDuty = constrain(line.substring(comma + 1).toInt(), 0, 100);
      ledcWrite(whiteLed, map(whiteDuty, 0, 100, 0, 255));
      ledcWrite(redLed, map(redDuty, 0, 100, 0, 255));
    }
  }
}
