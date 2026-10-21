// position_control.cpp
#include <Arduino.h>
#include "position_control.h"

// ===== HỆ SỐ PID — CẦN TUNE THỰC NGHIỆM, bắt đầu bảo thủ =====
const float POS_KP = 100.0f;   // N / mm — lực bổ sung cho mỗi mm sai lệch
const float POS_KI = 0.0f;  // N / (mm·lần) — tích lũy sai số dai dẳng
const float POS_KD = 0.0f;   // N / (mm/lần) — giảm dao động

// ===== GIỚI HẠN AN TOÀN — tránh wrench bổ sung quá lớn phá vỡ QP =====
const float MAX_WRENCH_BIAS = 0.3f;   // N — không vượt quá 30% T_MIN_N

static float integralX = 0.0f, integralY = 0.0f;
static float prevErrorX = 0.0f, prevErrorY = 0.0f;
static bool  firstRun = true;

void positionControlReset() {
  integralX = 0.0f; integralY = 0.0f;
  prevErrorX = 0.0f; prevErrorY = 0.0f;
  firstRun = true;
  Serial.println("[POS_PID] Reset bo nho PID vi tri");
}

void positionControlUpdate(float targetX, float targetY, float fkX, float fkY,
                            float &outWx, float &outWy) {
  float errorX = targetX - fkX;
  float errorY = targetY - fkY;

  integralX += errorX;
  integralY += errorY;

  // Chống windup — giới hạn tích lũy để không bị "bùng nổ" khi sai số kéo dài
  const float INTEGRAL_LIMIT = 50.0f;
  if (integralX > INTEGRAL_LIMIT) integralX = INTEGRAL_LIMIT;
  if (integralX < -INTEGRAL_LIMIT) integralX = -INTEGRAL_LIMIT;
  if (integralY > INTEGRAL_LIMIT) integralY = INTEGRAL_LIMIT;
  if (integralY < -INTEGRAL_LIMIT) integralY = -INTEGRAL_LIMIT;

  float derivativeX = firstRun ? 0.0f : (errorX - prevErrorX);
  float derivativeY = firstRun ? 0.0f : (errorY - prevErrorY);
  firstRun = false;

  outWx = POS_KP * errorX + POS_KI * integralX + POS_KD * derivativeX;
  outWy = POS_KP * errorY + POS_KI * integralY + POS_KD * derivativeY;

  // Giới hạn an toàn — không cho wrench bổ sung vượt quá mức cho phép
  if (outWx > MAX_WRENCH_BIAS) outWx = MAX_WRENCH_BIAS;
  if (outWx < -MAX_WRENCH_BIAS) outWx = -MAX_WRENCH_BIAS;
  if (outWy > MAX_WRENCH_BIAS) outWy = MAX_WRENCH_BIAS;
  if (outWy < -MAX_WRENCH_BIAS) outWy = -MAX_WRENCH_BIAS;

  prevErrorX = errorX;
  prevErrorY = errorY;

  Serial.print("[POS_PID] error=("); Serial.print(errorX,2);
  Serial.print(","); Serial.print(errorY,2);
  Serial.print(")mm  wrench_bias=("); Serial.print(outWx,4);
  Serial.print(","); Serial.print(outWy,4); Serial.println(")N");
}