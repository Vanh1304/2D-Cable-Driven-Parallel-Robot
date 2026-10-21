// kinematics.h
#pragma once
#include "config.h"
#include <Arduino.h>    
// Kết quả IK cho 1 lần tính
struct IKResult {
  float cableLength[4];   // chiều dài tuyệt đối từng cáp (mm)
  float deltaLength[4];   // độ thay đổi so với vị trí hiện tại (mm), + = thu, - = nhả
  long  steps[4];         // số bước tương ứng, luôn dương
  bool  dir[4];           // true = thu cáp (DIR HIGH), false = nhả cáp (DIR LOW)
  bool  valid;            // false nếu tọa độ nằm ngoài WFW
};

// Tính chiều dài cáp tại vị trí (x, y) — mm, gốc tọa độ = tâm khung
bool computeIK(float x, float y, float outLength[4]);

// Kiểm tra tọa độ có nằm trong WFW không
bool isInWorkspace(float x, float y);

// Tính đầy đủ IK + steps + direction so với vị trí hiện tại
IKResult computeMotion(float targetX, float targetY,
                       float currentX, float currentY,
                       const float currentLength[4]);