// ====== Simple serial servo firmware for the Python GUI ======
#include <Arduino.h>
#include <Servo.h>

// ================== Pins ==================
const uint8_t PIN_S1 = 3;   // grip  (ย้ายมา D3 ตามที่แจ้ง)
const uint8_t PIN_S2 = 5;   // lift
const uint8_t PIN_S3 = 8;   // base
const uint8_t PIN_S4 = 11;  // neck

// ====== Calibrate pulse ranges (ปรับได้ตามรุ่นเซอร์โว) ======
// S1 ให้ช่วงกว้างขึ้นเพื่อดึงกำลัง/มุมให้มาเต็ม
const int MIN_US1 = 585,  MAX_US1 = 1900;
const int MIN_US2 = 600, MAX_US2 = 2550;
const int MIN_US3 = 600, MAX_US3 = 2550;
const int MIN_US4 = 1000, MAX_US4 = 2000;

// ====== Home angles (มุมพักตอนรีเซ็ต/จบงาน) ======
int HOME1 = 0, HOME2 = 90, HOME3 = 90, HOME4 = 90;

// หน่วงก่อนรีเซ็ตตอนเปิดเครื่อง/เพิ่งเสียบไฟ (ตามที่ขอ 2 วินาที)
const unsigned long WAIT_BEFORE_RESET_MS = 2000;

// --------------------------------------------------------------
Servo s1, s2, s3, s4;

static int angleToUs(int angle, int min_us, int max_us) {
  angle = constrain(angle, 0, 180);
  long us = (long)min_us + (long)(max_us - min_us) * angle / 180L;
  return (int)us;
}

static void writeAngle(Servo &sv, int servoIdx, int angle) {
  int us;
  switch (servoIdx) {
    case 1: us = angleToUs(angle, MIN_US1, MAX_US1); break;
    case 2: us = angleToUs(angle, MIN_US2, MAX_US2); break;
    case 3: us = angleToUs(angle, MIN_US3, MAX_US3); break;
    case 4: us = angleToUs(angle, MIN_US4, MAX_US4); break;
    default: return;
  }
  sv.writeMicroseconds(us);
  // รายงานกลับไป GUI เพื่ออัปเดตสไลเดอร์สด ๆ
  Serial.print("S"); Serial.print(servoIdx); Serial.print(" = ");
  Serial.print(constrain(angle, 0, 180));
  Serial.println(" deg");
}

static void setServo(int idx, int angle) {
  switch (idx) {
    case 1: writeAngle(s1, 1, angle); break;
    case 2: writeAngle(s2, 2, angle); break;
    case 3: writeAngle(s3, 3, angle); break;
    case 4: writeAngle(s4, 4, angle); break;
  }
}

static void resetPose() {
  setServo(3, HOME3);
  setServo(2, HOME2);
  setServo(1, HOME1);
  setServo(4, HOME4);
  Serial.println("[INFO] Reset done");
}

// --------------------------------------------------------------
void setup() {
  Serial.begin(9600);

  // แนบด้วยช่วงพัลส์ของแต่ละตัว (สำคัญกับ S1 มาก)
  s1.attach(PIN_S1, MIN_US1, MAX_US1);
  s2.attach(PIN_S2, MIN_US2, MAX_US2);
  s3.attach(PIN_S3, MIN_US3, MAX_US3);
  s4.attach(PIN_S4, MIN_US4, MAX_US4);

  delay(WAIT_BEFORE_RESET_MS);  // รอ 2 วินาทีให้ระบบนิ่ง
  resetPose();
  Serial.println("[READY]");
}

void loop() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  // คำสั่งตัวอักษร
  if (line.startsWith("reset")) {
    resetPose();
    return;
  }
  if (line.startsWith("stop")) {
    Serial.println("[STOP] (no onboard sequence running)");
    return;
  }
  if (line.startsWith("start")) {
    // ลำดับหลักให้ GUI รันเอง
    Serial.println("[INFO] Start command (host-driven)");
    return;
  }
  if (line.startsWith("delay")) {
    int sec = line.substring(5).toInt();
    if (sec < 0) sec = 0;
    Serial.print("[DELAY] "); Serial.print(sec); Serial.println(" s");
    delay((unsigned long)sec * 1000UL);
    return;
  }

  // คำสั่งตัวเลข: "<idx> <angle>"
  int idx = -1, ang = -1;
  if (sscanf(line.c_str(), "%d %d", &idx, &ang) == 2) {
    if (idx >= 1 && idx <= 4) {
      ang = constrain(ang, 0, 180);
      setServo(idx, ang);
    } else {
      Serial.println("[ERR] idx must be 1..4");
    }
  } else {
    Serial.print("Unknown command: ");
    Serial.println(line);
  }
}
