// tension_distribution.cpp
#include "tension_distribution.h"
#include <math.h>

// Giải hệ 2x2:  [a b; c d] * [x;y] = [e;f]
static bool solve2x2(float a, float b, float c, float d,
                     float e, float f, float &x, float &y) {
  float det = a*d - b*c;
  if (fabs(det) < 1e-6f) return false;   // suy biến — vị trí đặc biệt, tránh xa
  x = (e*d - b*f) / det;
  y = (a*f - e*c) / det;
  return true;
}

TensionSolution solveTensionDistribution(float x, float y) {
  TensionSolution sol;
  sol.feasible = true;

  // ===== BƯỚC 1: Tính ma trận cấu trúc A (2x4) =====
  // u[i] = vector đơn vị từ end-effector hướng về điểm neo i
  float ux[4], uy[4];
  for (int i = 0; i < 4; i++) {
    float dx = ANCHOR_X[i] - x;
    float dy = ANCHOR_Y[i] - y;
    float len = sqrtf(dx*dx + dy*dy);
    if (len < 1e-3f) len = 1e-3f;   // tránh chia 0 nếu trùng điểm neo
    ux[i] = dx / len;
    uy[i] = dy / len;
  }

  // ===== BƯỚC 2: Tách B (cột 0,1) và C (cột 2,3) =====
  float B00 = ux[0], B01 = ux[1];
  float B10 = uy[0], B11 = uy[1];
  float C00 = ux[2], C01 = ux[3];
  float C10 = uy[2], C11 = uy[3];

  const float T_NOMINAL = T_NOMINAL_N;

  // ===== BƯỚC 3: Duyệt lưới nhỏ (T3,T4) quanh T_NOMINAL, chọn nghiệm hợp lệ tốt nhất =====
  // Cách đơn giản, ổn định cho ESP32 — thay vì giải QP tổng quát phức tạp
  float bestT[4];
  float bestCost = 1e9f;
  bool  foundFeasible = false;

  const int GRID_STEPS = 48;   // độ mịn lưới — đủ tốt cho thời gian thực trên ESP32
  float step = (T_MAX_N - T_MIN_N) / GRID_STEPS;

  for (int i3 = 0; i3 <= GRID_STEPS; i3++) {
    float T3 = T_MIN_N + i3 * step;
    for (int i4 = 0; i4 <= GRID_STEPS; i4++) {
      float T4 = T_MIN_N + i4 * step;

      // Tính T1, T2 từ T3, T4 đã chọn
      float rhsX = -W_EXT_X - (C00*T3 + C01*T4);
      float rhsY = -W_EXT_Y - (C10*T3 + C11*T4);

      float T1, T2;
      if (!solve2x2(B00, B01, B10, B11, rhsX, rhsY, T1, T2)) continue;

      // Kiểm tra ràng buộc hộp
      if (T1 < T_MIN_N || T1 > T_MAX_N) continue;
      if (T2 < T_MIN_N || T2 > T_MAX_N) continue;

      // Chi phí: tổng bình phương độ lệch so với T_nominal — càng đều càng tốt
      float cost = (T1-T_NOMINAL)*(T1-T_NOMINAL) + (T2-T_NOMINAL)*(T2-T_NOMINAL)
                 + (T3-T_NOMINAL)*(T3-T_NOMINAL) + (T4-T_NOMINAL)*(T4-T_NOMINAL);

      if (cost < bestCost) {
        bestCost = cost;
        bestT[0] = T1; bestT[1] = T2; bestT[2] = T3; bestT[3] = T4;
        foundFeasible = true;
      }
    }
  }

  if (foundFeasible) {
    for (int i = 0; i < 4; i++) sol.T[i] = bestT[i];
    sol.feasible = true;
  } else {
    // Không tìm được nghiệm hợp lệ — dùng T_nominal đều cho cả 4, đánh dấu không khả thi
    for (int i = 0; i < 4; i++) sol.T[i] = T_NOMINAL;
    sol.feasible = false;
  }

  return sol;
}

// ===== THÊM VÀO CUỐI tension_distribution.cpp =====

TensionSolution solveTensionDistributionWithBias(float x, float y, float biasWx, float biasWy) {
  // Tính toán y hệt bản gốc, chỉ khác: cộng thêm biasWx/biasWy vào ngoại lực
  TensionSolution sol;
  sol.feasible = true;

  float ux[4], uy[4];
  for (int i = 0; i < 4; i++) {
    float dx = ANCHOR_X[i] - x;
    float dy = ANCHOR_Y[i] - y;
    float len = sqrtf(dx*dx + dy*dy);
    if (len < 1e-3f) len = 1e-3f;
    ux[i] = dx / len;
    uy[i] = dy / len;
  }

  float B00 = ux[0], B01 = ux[1];
  float B10 = uy[0], B11 = uy[1];
  float C00 = ux[2], C01 = ux[3];
  float C10 = uy[2], C11 = uy[3];

  const float T_NOMINAL = T_NOMINAL_N;

  float bestT[4];
  float bestCost = 1e9f;
  bool foundFeasible = false;

  const int GRID_STEPS = 48;
  float step = (T_MAX_N - T_MIN_N) / GRID_STEPS;

  // ===== KHÁC BIỆT DUY NHẤT: W_EXT + bias thay vì chỉ W_EXT =====
  float totalWx = W_EXT_X + biasWx;
  float totalWy = W_EXT_Y + biasWy;

  for (int i3 = 0; i3 <= GRID_STEPS; i3++) {
    float T3 = T_MIN_N + i3 * step;
    for (int i4 = 0; i4 <= GRID_STEPS; i4++) {
      float T4 = T_MIN_N + i4 * step;

      float rhsX = -totalWx - (C00*T3 + C01*T4);
      float rhsY = -totalWy - (C10*T3 + C11*T4);

      float det = B00*B11 - B01*B10;
      if (fabs(det) < 1e-6f) continue;
      float T1 = (rhsX*B11 - B01*rhsY) / det;
      float T2 = (B00*rhsY - rhsX*B10) / det;

      if (T1 < T_MIN_N || T1 > T_MAX_N) continue;
      if (T2 < T_MIN_N || T2 > T_MAX_N) continue;

      float cost = (T1-T_NOMINAL)*(T1-T_NOMINAL) + (T2-T_NOMINAL)*(T2-T_NOMINAL)
                 + (T3-T_NOMINAL)*(T3-T_NOMINAL) + (T4-T_NOMINAL)*(T4-T_NOMINAL);

      if (cost < bestCost) {
        bestCost = cost;
        bestT[0]=T1; bestT[1]=T2; bestT[2]=T3; bestT[3]=T4;
        foundFeasible = true;
      }
    }
  }

  if (foundFeasible) {
    for (int i = 0; i < 4; i++) sol.T[i] = bestT[i];
    sol.feasible = true;
  } else {
    for (int i = 0; i < 4; i++) sol.T[i] = T_NOMINAL;
    sol.feasible = false;
  }
  return sol;
}