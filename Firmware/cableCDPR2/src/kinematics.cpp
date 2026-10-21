// kinematics.cpp
#include "kinematics.h"
#include <math.h>

bool isInWorkspace(float x, float y) {
  return (fabs(x) <= WFW_X_MAX && fabs(y) <= WFW_Y_MAX);
}

bool computeIK(float x, float y, float outLength[4]) {
  // IK đơn giản: chiều dài cáp = khoảng cách Euclid từ điểm neo đến end-effector
  // Hợp lệ vì end-effector chỉ tịnh tiến (không xoay)
  for (int i = 0; i < 4; i++) {
    float dx = x - ANCHOR_X[i];
    float dy = y - ANCHOR_Y[i];
    outLength[i] = sqrtf(dx*dx + dy*dy);
  }
  return true;
}

IKResult computeMotion(float targetX, float targetY,
                       float currentX, float currentY,
                       const float currentLength[4]) {
  IKResult result;
  result.valid = false;

  // Kiểm tra WFW
  if (!isInWorkspace(targetX, targetY)) {
    Serial.println("[IK] ERROR: toa do ngoai WFW!");
    return result;
  }

  // Tính chiều dài cáp tại vị trí đích
  float targetLength[4];
  computeIK(targetX, targetY, targetLength);

  for (int i = 0; i < 4; i++) {
    result.cableLength[i] = targetLength[i];
    result.deltaLength[i] = currentLength[i] - targetLength[i];
    // deltaLength > 0 → cáp cần ngắn lại → thu cáp → dir = true
    // deltaLength < 0 → cáp cần dài ra  → nhả cáp → dir = false

    result.dir[i]   = (result.deltaLength[i] > 0);
    result.steps[i] = (long)(fabs(result.deltaLength[i]) * STEPS_PER_MM);
  }

  result.valid = true;
  return result;
}