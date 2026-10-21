// forward_kinematics.h
#pragma once
#include "config.h"

struct FKResult {
  float x, y;
  float residual;   // sai số khớp — càng nhỏ càng đáng tin cậy
  bool  valid;
};

// Tính vị trí (x,y) ước lượng từ 4 chiều dài cáp đo được/tính được
FKResult forwardKinematics(const float length[4]);