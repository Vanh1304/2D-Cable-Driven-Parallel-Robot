// // motion.cpp
// #include "motion.h"

// // ===== GPIO BANK-AWARE MACRO (GPIO 32/33 khác bank) =====
// #define GPIO_SET(pin) do { if((pin)<32) GPIO.out_w1ts = (1UL<<(pin)); \
//                            else GPIO.out1_w1ts.val = (1UL<<((pin)-32)); } while(0)
// #define GPIO_CLR(pin) do { if((pin)<32) GPIO.out_w1tc = (1UL<<(pin)); \
//                            else GPIO.out1_w1tc.val = (1UL<<((pin)-32)); } while(0)

// // ===== ĐẢO CHIỀU DIR — dựa vào test thực tế phần cứng =====
// // Motor 0: HIGH=NHA (thực tế) → cần đảo
// // Motor 1: HIGH=THU (thực tế) → đúng, giữ nguyên
// // Motor 2: HIGH=NHA (thực tế) → cần đảo
// // Motor 3: HIGH=THU (thực tế) → đúng, giữ nguyên
// const bool DIR_INVERTED[4] = {true, false, true, false};

// // ===== TRẠNG THÁI TỪNG MOTOR =====
// hw_timer_t *timers[4] = {NULL, NULL, NULL, NULL};
// portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// volatile bool stepState[4]      = {false,false,false,false};
// volatile long stepsRemaining[4] = {0,0,0,0};
// volatile long stepsDone[4]      = {0,0,0,0};

// float   vMaxPerMotor[4]        = {V_MAX_HZ, V_MAX_HZ, V_MAX_HZ, V_MAX_HZ};
// int     rampStepsPerMotor[4]   = {RAMP_STEPS, RAMP_STEPS, RAMP_STEPS, RAMP_STEPS};
// long    targetStepsPerMotor[4] = {0,0,0,0};

// // ===== 4 ISR — chỉ toggle + đếm =====
// void IRAM_ATTR isr0() {
//   if (stepsRemaining[0] <= 0) return;
//   stepState[0] = !stepState[0];
//   if (stepState[0]) GPIO_SET(STEP_PINS[0]); else GPIO_CLR(STEP_PINS[0]);
//   if (!stepState[0]) { stepsRemaining[0]--; stepsDone[0]++; }
// }
// void IRAM_ATTR isr1() {
//   if (stepsRemaining[1] <= 0) return;
//   stepState[1] = !stepState[1];
//   if (stepState[1]) GPIO_SET(STEP_PINS[1]); else GPIO_CLR(STEP_PINS[1]);
//   if (!stepState[1]) { stepsRemaining[1]--; stepsDone[1]++; }
// }
// void IRAM_ATTR isr2() {
//   if (stepsRemaining[2] <= 0) return;
//   stepState[2] = !stepState[2];
//   if (stepState[2]) GPIO_SET(STEP_PINS[2]); else GPIO_CLR(STEP_PINS[2]);
//   if (!stepState[2]) { stepsRemaining[2]--; stepsDone[2]++; }
// }
// void IRAM_ATTR isr3() {
//   if (stepsRemaining[3] <= 0) return;
//   stepState[3] = !stepState[3];
//   if (stepState[3]) GPIO_SET(STEP_PINS[3]); else GPIO_CLR(STEP_PINS[3]);
//   if (!stepState[3]) { stepsRemaining[3]--; stepsDone[3]++; }
// }
// void (*isrList[4])() = {isr0, isr1, isr2, isr3};

// // ===== ĐỔI TỐC ĐỘ 1 MOTOR =====
// static void setFreq(int i, float hz) {
//   if (hz < 1.0f) hz = 1.0f;
//   uint64_t halfUs = (uint64_t)(500000.0f / hz);
//   timerAlarmWrite(timers[i], halfUs, true);
//   timerAlarmEnable(timers[i]);
// }

// void motionHoldEnable(bool en) {
//   digitalWrite(EN_PIN, en ? LOW : HIGH);   // active-LOW
// }

// void motionInit() {
//   pinMode(EN_PIN, OUTPUT);
//   digitalWrite(EN_PIN, HIGH);   // tắt trong lúc init

//   for (int i = 0; i < 4; i++) {
//     pinMode(STEP_PINS[i], OUTPUT);
//     pinMode(DIR_PINS[i],  OUTPUT);
//     digitalWrite(STEP_PINS[i], LOW);

//     timers[i] = timerBegin(i, 80, true);
//     timerAttachInterrupt(timers[i], isrList[i], true);
//     setFreq(i, V_START_HZ);
//   }
//   motionHoldEnable(true);
//   Serial.println("[MOTION] San sang!");
// }

// // ===== TÍNH ĐỒNG BỘ THEO TỈ LỆ QUÃNG ĐƯỜNG =====
// void motionStart(const long steps[4], const bool dir[4]) {
//   long maxSteps = 0;
//   for (int i = 0; i < 4; i++) if (steps[i] > maxSteps) maxSteps = steps[i];

//   if (maxSteps == 0) {
//     Serial.println("[MOTION] Khong co chuyen dong (steps=0)");
//     return;
//   }

//   for (int i = 0; i < 4; i++) {
//     bool actualDir = dir[i] ^ DIR_INVERTED[i];   // XOR đảo nếu cần
//     digitalWrite(DIR_PINS[i], actualDir ? HIGH : LOW);

//     targetStepsPerMotor[i] = steps[i];

//     if (steps[i] == 0) {
//       vMaxPerMotor[i] = V_START_HZ;
//       rampStepsPerMotor[i] = 0;
//       continue;
//     }

//     float ratio = (float)steps[i] / (float)maxSteps;
//     vMaxPerMotor[i] = V_START_HZ + ratio * (V_MAX_HZ - V_START_HZ);

//     rampStepsPerMotor[i] = min((long)RAMP_STEPS, steps[i] / 3);
//     if (rampStepsPerMotor[i] < 1) rampStepsPerMotor[i] = 1;
//   }

//   delayMicroseconds(5);   // chờ DIR ổn định

//   portENTER_CRITICAL(&timerMux);
//   for (int i = 0; i < 4; i++) {
//     stepsDone[i]      = 0;
//     stepsRemaining[i] = steps[i];
//   }
//   portEXIT_CRITICAL(&timerMux);

//   for (int i = 0; i < 4; i++) setFreq(i, V_START_HZ);
// }

// // ===== CẬP NHẬT RAMP — gọi liên tục trong loop() =====
// void motionUpdate() {
//   for (int i = 0; i < 4; i++) {
//     if (targetStepsPerMotor[i] == 0) continue;
//     if (stepsRemaining[i] <= 0) continue;

//     long done      = stepsDone[i];
//     long remaining = stepsRemaining[i];
//     int  ramp      = rampStepsPerMotor[i];

//     float hz;
//     if (done < ramp) {
//       float t = (float)done / ramp;
//       hz = V_START_HZ + t * (vMaxPerMotor[i] - V_START_HZ);
//     } else if (remaining < ramp) {
//       float t = (float)remaining / ramp;
//       hz = V_START_HZ + t * (vMaxPerMotor[i] - V_START_HZ);
//     } else {
//       hz = vMaxPerMotor[i];
//     }
//     setFreq(i, hz);
//   }
// }

// bool motionAllDone() {
//   for (int i = 0; i < 4; i++)
//     if (stepsRemaining[i] > 0) return false;
//   return true;
// }

// // motion.cpp — thêm vào cuối file (không xóa gì phần cũ)

// static bool motorMoving[4]  = {false, false, false, false};
// static bool lastDirCmd[4]   = {false, false, false, false};

// // Cộng dồn thêm bước vào motor đang chạy — không reset về 0
// // Trả về false nếu motor đang chạy chiều ngược, cần chờ nó xong trước
// bool motionAppend(int i, long deltaSteps, bool dir) {
//   if (deltaSteps == 0) return true;

//   bool actualDir = dir ^ DIR_INVERTED[i];

//   if (motorMoving[i] && actualDir != lastDirCmd[i]) {
//     return false;   // đang chạy chiều khác — caller phải chờ rồi gọi lại
//   }

//   if (!motorMoving[i]) {
//     // Bắt đầu 1 đoạn mới — set DIR, reset đếm
//     digitalWrite(DIR_PINS[i], actualDir ? HIGH : LOW);
//     delayMicroseconds(5);
//     portENTER_CRITICAL(&timerMux);
//     stepsDone[i]      = 0;
//     stepsRemaining[i] = 0;
//     portEXIT_CRITICAL(&timerMux);
//     lastDirCmd[i]  = actualDir;
//     motorMoving[i] = true;
//     setFreq(i, V_START_HZ);
//   }

//   portENTER_CRITICAL(&timerMux);
//   stepsRemaining[i] += deltaSteps;   // CỘNG DỒN — không ghi đè
//   portEXIT_CRITICAL(&timerMux);
//   return true;
// }

// // Cập nhật tốc độ cho chế độ streaming — gọi liên tục thay cho motionUpdate()
// void motionStreamUpdate(float cruiseHz) {
//   for (int i = 0; i < 4; i++) {
//     if (!motorMoving[i]) continue;
//     if (stepsRemaining[i] <= 0) { motorMoving[i] = false; continue; }

//     long done      = stepsDone[i];
//     long remaining = stepsRemaining[i];
//     float hz;

//     if (done < RAMP_STEPS) {
//       float t = (float)done / RAMP_STEPS;
//       hz = V_START_HZ + t * (cruiseHz - V_START_HZ);
//     } else if (remaining < RAMP_STEPS) {
//       // Chỉ thực sự ramp-down nếu KHÔNG có thêm bước mới được cộng vào kịp lúc
//       float t = (float)remaining / RAMP_STEPS;
//       hz = V_START_HZ + t * (cruiseHz - V_START_HZ);
//     } else {
//       hz = cruiseHz;
//     }
//     setFreq(i, hz);
//   }
// }

// bool motionStreamMoving() {
//   for (int i = 0; i < 4; i++) if (motorMoving[i]) return true;
//   return false;
// }
// // motion.cpp — thêm vào cuối file

// long motionGetStepsDone(int i) {
//   return stepsDone[i];
// }

// long motionGetStepsRemaining(int i) {
//   return stepsRemaining[i];
// }

// // Thêm bước bổ sung vào motor đang chạy — KHÔNG đổi DIR
// // Dùng cho tension correction — chỉ nới rộng quãng đường theo chiều đã set sẵn
// void motionAddSteps(int i, long extraSteps) {
//   portENTER_CRITICAL(&timerMux);
//   stepsRemaining[i] += extraSteps;
//   if (stepsRemaining[i] < 0) stepsRemaining[i] = 0;
//   portEXIT_CRITICAL(&timerMux);
// }

// motion.cpp
#include "motion.h"

#define GPIO_SET(pin) do { if((pin)<32) GPIO.out_w1ts = (1UL<<(pin)); \
                           else GPIO.out1_w1ts.val = (1UL<<((pin)-32)); } while(0)
#define GPIO_CLR(pin) do { if((pin)<32) GPIO.out_w1tc = (1UL<<(pin)); \
                           else GPIO.out1_w1tc.val = (1UL<<((pin)-32)); } while(0)

const bool DIR_INVERTED[4] = {true, false, true, false};

hw_timer_t *timers[4] = {NULL, NULL, NULL, NULL};
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool stepState[4]      = {false,false,false,false};
volatile long stepsRemaining[4] = {0,0,0,0};
volatile long stepsDone[4]      = {0,0,0,0};

float vMaxPerMotor[4]        = {V_MAX_HZ, V_MAX_HZ, V_MAX_HZ, V_MAX_HZ};
int   rampStepsPerMotor[4]   = {RAMP_STEPS, RAMP_STEPS, RAMP_STEPS, RAMP_STEPS};
long  targetStepsPerMotor[4] = {0,0,0,0};

static bool motorMoving[4]   = {false, false, false, false};
static bool lastDirCmd[4]    = {false, false, false, false};
static long pendingSteps[4]  = {0, 0, 0, 0};
static bool pendingDirVal[4] = {false, false, false, false};
static bool pendingValid[4]  = {false, false, false, false};

void IRAM_ATTR isr0() {
  if (stepsRemaining[0] <= 0) return;
  stepState[0] = !stepState[0];
  if (stepState[0]) GPIO_SET(STEP_PINS[0]); else GPIO_CLR(STEP_PINS[0]);
  if (!stepState[0]) { stepsRemaining[0]--; stepsDone[0]++; }
}
void IRAM_ATTR isr1() {
  if (stepsRemaining[1] <= 0) return;
  stepState[1] = !stepState[1];
  if (stepState[1]) GPIO_SET(STEP_PINS[1]); else GPIO_CLR(STEP_PINS[1]);
  if (!stepState[1]) { stepsRemaining[1]--; stepsDone[1]++; }
}
void IRAM_ATTR isr2() {
  if (stepsRemaining[2] <= 0) return;
  stepState[2] = !stepState[2];
  if (stepState[2]) GPIO_SET(STEP_PINS[2]); else GPIO_CLR(STEP_PINS[2]);
  if (!stepState[2]) { stepsRemaining[2]--; stepsDone[2]++; }
}
void IRAM_ATTR isr3() {
  if (stepsRemaining[3] <= 0) return;
  stepState[3] = !stepState[3];
  if (stepState[3]) GPIO_SET(STEP_PINS[3]); else GPIO_CLR(STEP_PINS[3]);
  if (!stepState[3]) { stepsRemaining[3]--; stepsDone[3]++; }
}
void (*isrList[4])() = {isr0, isr1, isr2, isr3};

static void setFreq(int i, float hz) {
  if (hz < 1.0f) hz = 1.0f;
  uint64_t halfUs = (uint64_t)(500000.0f / hz);
  timerAlarmWrite(timers[i], halfUs, true);
  timerAlarmEnable(timers[i]);
}

void motionHoldEnable(bool en) {
  digitalWrite(EN_PIN, en ? LOW : HIGH);
}

void motionInit() {
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);
  for (int i = 0; i < 4; i++) {
    pinMode(STEP_PINS[i], OUTPUT);
    pinMode(DIR_PINS[i],  OUTPUT);
    digitalWrite(STEP_PINS[i], LOW);
    timers[i] = timerBegin(i, 80, true);
    timerAttachInterrupt(timers[i], isrList[i], true);
    setFreq(i, V_START_HZ);
  }
  motionHoldEnable(true);
  Serial.println("[MOTION] San sang!");
}

// ===== CHẾ ĐỘ RỜI RẠC — dùng cho moveToPoint() từng điểm =====
void motionStart(const long steps[4], const bool dir[4]) {
  long maxSteps = 0;
  for (int i = 0; i < 4; i++) if (steps[i] > maxSteps) maxSteps = steps[i];
  if (maxSteps == 0) { Serial.println("[MOTION] Khong co chuyen dong"); return; }

  for (int i = 0; i < 4; i++) {
    bool actualDir = dir[i] ^ DIR_INVERTED[i];
    digitalWrite(DIR_PINS[i], actualDir ? HIGH : LOW);
    targetStepsPerMotor[i] = steps[i];
    if (steps[i] == 0) { vMaxPerMotor[i] = V_START_HZ; rampStepsPerMotor[i] = 0; continue; }
    float ratio = (float)steps[i] / (float)maxSteps;
    vMaxPerMotor[i] = V_START_HZ + ratio * (V_MAX_HZ - V_START_HZ);
    rampStepsPerMotor[i] = min((long)RAMP_STEPS, steps[i] / 3);
    if (rampStepsPerMotor[i] < 1) rampStepsPerMotor[i] = 1;
  }
  delayMicroseconds(5);
  portENTER_CRITICAL(&timerMux);
  for (int i = 0; i < 4; i++) { stepsDone[i] = 0; stepsRemaining[i] = steps[i]; }
  portEXIT_CRITICAL(&timerMux);
  for (int i = 0; i < 4; i++) setFreq(i, V_START_HZ);
}

void motionUpdate() {
  for (int i = 0; i < 4; i++) {
    if (targetStepsPerMotor[i] == 0) continue;
    if (stepsRemaining[i] <= 0) continue;
    long done = stepsDone[i], remaining = stepsRemaining[i];
    int ramp = rampStepsPerMotor[i];
    float hz;
    if (done < ramp) { float t=(float)done/ramp; hz=V_START_HZ+t*(vMaxPerMotor[i]-V_START_HZ); }
    else if (remaining < ramp) { float t=(float)remaining/ramp; hz=V_START_HZ+t*(vMaxPerMotor[i]-V_START_HZ); }
    else hz = vMaxPerMotor[i];
    setFreq(i, hz);
  }
}

bool motionAllDone() {
  for (int i = 0; i < 4; i++) if (stepsRemaining[i] > 0) return false;
  return true;
}

// ===== CHẾ ĐỘ STREAMING — dùng cho đường liên tục (circle, trajectory mượt) =====
bool motionAppend(int i, long deltaSteps, bool dir) {
  if (deltaSteps == 0) return true;
  bool actualDir = dir ^ DIR_INVERTED[i];

  if (!motorMoving[i]) {
    digitalWrite(DIR_PINS[i], actualDir ? HIGH : LOW);
    delayMicroseconds(5);
    portENTER_CRITICAL(&timerMux);
    stepsDone[i] = 0; stepsRemaining[i] = deltaSteps;
    portEXIT_CRITICAL(&timerMux);
    lastDirCmd[i] = actualDir; motorMoving[i] = true;
    setFreq(i, V_START_HZ);
    return true;
  }

  if (actualDir == lastDirCmd[i]) {
    portENTER_CRITICAL(&timerMux);
    stepsRemaining[i] += deltaSteps;
    portEXIT_CRITICAL(&timerMux);
    return true;
  }

  // Đổi chiều — xếp hàng chờ, KHÔNG chặn caller
  if (pendingValid[i] && pendingDirVal[i] == actualDir) {
    pendingSteps[i] += deltaSteps;
  } else {
    pendingSteps[i] = deltaSteps;
    pendingDirVal[i] = actualDir;
    pendingValid[i] = true;
  }
  return true;
}

void motionStreamUpdate(float cruiseHz) {
  for (int i = 0; i < 4; i++) {
    if (!motorMoving[i] && pendingValid[i]) {
      digitalWrite(DIR_PINS[i], pendingDirVal[i] ? HIGH : LOW);
      delayMicroseconds(5);
      portENTER_CRITICAL(&timerMux);
      stepsDone[i] = 0; stepsRemaining[i] = pendingSteps[i];
      portEXIT_CRITICAL(&timerMux);
      lastDirCmd[i] = pendingDirVal[i];
      motorMoving[i] = true;
      pendingValid[i] = false;
      setFreq(i, V_START_HZ);
    }

    if (!motorMoving[i]) continue;
    if (stepsRemaining[i] <= 0) { motorMoving[i] = false; continue; }

    long done = stepsDone[i], remaining = stepsRemaining[i];
    float hz;
    if (done < RAMP_STEPS) { float t=(float)done/RAMP_STEPS; hz=V_START_HZ+t*(cruiseHz-V_START_HZ); }
    else if (remaining < RAMP_STEPS) { float t=(float)remaining/RAMP_STEPS; hz=V_START_HZ+t*(cruiseHz-V_START_HZ); }
    else hz = cruiseHz;
    setFreq(i, hz);
  }
}

bool motionStreamMoving() {
  for (int i = 0; i < 4; i++) if (motorMoving[i] || pendingValid[i]) return true;
  return false;
}

long motionGetStepsDone(int i)      { return stepsDone[i]; }
long motionGetStepsRemaining(int i) { return stepsRemaining[i]; }
void motionAddSteps(int i, long extraSteps) {
  portENTER_CRITICAL(&timerMux);
  stepsRemaining[i] += extraSteps;
  if (stepsRemaining[i] < 0) stepsRemaining[i] = 0;
  portEXIT_CRITICAL(&timerMux);
}