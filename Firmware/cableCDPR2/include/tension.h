// tension.h
#pragma once
#include <Arduino.h>
#include "config.h"

struct TensionReading {
  float f_loadcell[4];   // lực tác động lên loadcell (N)
  float t_cable[4];      // lực căng cáp thực tế (N)
  bool  valid[4];        // false nếu không đọc được
};

void tensionInit();
void tensionTareAll();

// Đọc nhanh 1 lần — dùng khi motor đang chạy (realtime)
TensionReading tensionReadRealtime();

// Đọc ổn định — chờ 5s, trung bình 20 lần — dùng lúc khởi tạo lực căng ban đầu
TensionReading tensionReadStable();