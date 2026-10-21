// tension_distribution.h
#pragma once
#include "config.h"

struct TensionSolution {
  float T[4];      // lực căng tối ưu từng cáp (N)
  bool  feasible;  // false nếu không tìm được nghiệm hợp lệ trong [T_MIN, T_MAX]
};

// Tính bộ lực căng tối ưu tại vị trí (x, y)
TensionSolution solveTensionDistribution(float x, float y);
TensionSolution solveTensionDistributionWithBias(float x, float y, float biasWx, float biasWy);