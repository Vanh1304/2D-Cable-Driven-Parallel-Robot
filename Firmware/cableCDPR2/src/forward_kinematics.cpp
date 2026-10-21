// forward_kinematics.cpp
#include "forward_kinematics.h"
#include <math.h>

static bool solve2x2(float a, float b, float c, float d,
                      float e, float f, float &x, float &y) {
  float det = a*d - b*c;
  if (fabs(det) < 1e-9f) return false;
  x = (e*d - b*f) / det;
  y = (a*f - e*c) / det;
  return true;
}

FKResult forwardKinematics(const float length[4]) {
  FKResult res;
  res.valid = false;

  // ===== BƯỚC 1: Xây 3 phương trình tuyến tính (i=1,2,3 so với mốc i=0) =====
  // Dạng: Ai*x + Bi*y = Ci
  float Arow[3], Brow[3], Crow[3];

  float L0sq  = length[0] * length[0];
  float Ax0sq = ANCHOR_X[0]*ANCHOR_X[0];
  float Ay0sq = ANCHOR_Y[0]*ANCHOR_Y[0];

  for (int k = 1; k <= 3; k++) {
    float Lisq = length[k] * length[k];
    Arow[k-1] = -2.0f * (ANCHOR_X[k] - ANCHOR_X[0]);
    Brow[k-1] = -2.0f * (ANCHOR_Y[k] - ANCHOR_Y[0]);
    Crow[k-1] = (Lisq - L0sq)
              - (ANCHOR_X[k]*ANCHOR_X[k] - Ax0sq)
              - (ANCHOR_Y[k]*ANCHOR_Y[k] - Ay0sq);
    Crow[k-1] = -Crow[k-1];   // đưa về đúng vế phải: Ai*x+Bi*y = Ci
  }

  // ===== BƯỚC 2: Normal equations — (AᵀA)*[x,y]ᵀ = Aᵀb, giải hệ 2x2 =====
  float ATA00 = 0, ATA01 = 0, ATA11 = 0;
  float ATb0  = 0, ATb1  = 0;

  for (int k = 0; k < 3; k++) {
    ATA00 += Arow[k]*Arow[k];
    ATA01 += Arow[k]*Brow[k];
    ATA11 += Brow[k]*Brow[k];
    ATb0  += Arow[k]*Crow[k];
    ATb1  += Brow[k]*Crow[k];
  }

  float x, y;
  if (!solve2x2(ATA00, ATA01, ATA01, ATA11, ATb0, ATb1, x, y)) {
    return res;   // suy biến — không giải được (rất hiếm khi xảy ra với 4 neo không thẳng hàng)
  }

  res.x = x;
  res.y = y;

  // ===== BƯỚC 3: Tính residual — đo độ "khớp" của nghiệm =====
  // So sánh chiều dài lý thuyết tại (x,y) vừa tìm với chiều dài đầu vào
  float sumSqErr = 0.0f;
  for (int i = 0; i < 4; i++) {
    float dx = x - ANCHOR_X[i];
    float dy = y - ANCHOR_Y[i];
    float predictedLen = sqrtf(dx*dx + dy*dy);
    float err = predictedLen - length[i];
    sumSqErr += err * err;
  }
  res.residual = sqrtf(sumSqErr / 4.0f);   // RMS error, đơn vị mm
  res.valid = true;

  return res;
}