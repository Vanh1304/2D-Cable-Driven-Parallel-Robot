// tension.cpp
#include "tension.h"

static long tareValues[4] = {0, 0, 0, 0};

// ===== CHỜ CẢ 4 DOUT VỀ LOW (data ready) =====
static bool waitAllReady(uint32_t timeoutMs = 500) {
  uint32_t t = millis();
  while (millis() - t < timeoutMs) {
    bool allReady = true;
    for (int i = 0; i < 4; i++) {
      if (digitalRead(HX_DOUT_PINS[i]) == HIGH) { allReady = false; break; }
    }
    if (allReady) return true;
  }
  return false;
}

// ===== ĐỌC 4 HX711 CÙNG 1 CHUỖI CLK — đồng bộ tuyệt đối =====
static bool readAllRaw(long out[4]) {
  if (!waitAllReady(500)) return false;

  long data[4] = {0, 0, 0, 0};

  for (int bit = 23; bit >= 0; bit--) {
    digitalWrite(HX_SCK_PIN, HIGH);
    delayMicroseconds(1);
    for (int i = 0; i < 4; i++) {
      if (digitalRead(HX_DOUT_PINS[i]) == HIGH) data[i] |= (1L << bit);
    }
    digitalWrite(HX_SCK_PIN, LOW);
    delayMicroseconds(1);
  }

  // Xung thứ 25 — set gain 128, channel A cho lần đọc kế tiếp
  digitalWrite(HX_SCK_PIN, HIGH);
  delayMicroseconds(1);
  digitalWrite(HX_SCK_PIN, LOW);
  delayMicroseconds(1);

  for (int i = 0; i < 4; i++) {
    if (data[i] & 0x800000) data[i] |= 0xFF000000;  // bù 2, số âm
    out[i] = data[i];
  }
  return true;
}

// ===== RESET CỨNG TOÀN BỘ 4 HX711 =====
static void resetAll() {
  digitalWrite(HX_SCK_PIN, HIGH);
  delay(100);            // >> 60us tối thiểu để vào power-down
  digitalWrite(HX_SCK_PIN, LOW);
  delay(500);            // chờ hoàn thành 1 chu kỳ conversion (~100ms ở 10Hz)
}

void tensionInit() {
  Serial.println("[TENSION] Khoi tao...");
  pinMode(HX_SCK_PIN, OUTPUT);
  for (int i = 0; i < 4; i++) pinMode(HX_DOUT_PINS[i], INPUT);

  resetAll();

  if (!waitAllReady(3000)) {
    Serial.println("[TENSION] WARNING: co HX711 khong san sang!");
  } else {
    Serial.println("[TENSION] Tat ca san sang!");
  }

  tensionTareAll();
  Serial.println("[TENSION] Hoan tat!");
}

void tensionTareAll() {
  Serial.println("[TENSION] Tare (de trong, khong cham vao)...");
  long sum[4] = {0, 0, 0, 0};
  int count = 0;
  long raw[4];

  for (int n = 0; n < 20; n++) {
    if (readAllRaw(raw)) {
      for (int i = 0; i < 4; i++) sum[i] += raw[i];
      count++;
    }
    delay(20);
  }

  for (int i = 0; i < 4; i++) {
    tareValues[i] = (count > 0) ? sum[i] / count : 0;
    Serial.print("[TENSION] Tare #"); Serial.print(i);
    Serial.print(" = "); Serial.println(tareValues[i]);
  }
}

// ===== CHUYỂN ĐỔI: raw → gram → F_loadcell (N) → T_cable (N) =====
static void rawToForce(const long raw[4], float fLoadcell[4], float tCable[4], bool valid[4]) {
  for (int i = 0; i < 4; i++) {
    long offset  = raw[i] - tareValues[i];
    float gram   = (float)offset / CALIB_FACTORS[i];
    float newton = fabs(gram) * 9.81f / 1000.0f;   // fabs() vì LC#2 dấu dương ngược
    fLoadcell[i] = newton;
    tCable[i]    = newton * TENSION_FACTOR;
    valid[i]     = true;
  }
}

// ===== CHẾ ĐỘ 1: ĐỌC NHANH — dùng khi motor đang chạy =====
TensionReading tensionReadRealtime() {
  TensionReading result;
  long raw[4];

  if (!readAllRaw(raw)) {
    for (int i = 0; i < 4; i++) {
      result.valid[i] = false;
      result.f_loadcell[i] = 0.0f;
      result.t_cable[i] = 0.0f;
    }
    return result;
  }

  rawToForce(raw, result.f_loadcell, result.t_cable, result.valid);
  return result;
}

// ===== CHẾ ĐỘ 2: ĐỌC ỔN ĐỊNH — chờ 5s, trung bình 20 lần =====
// Dùng lúc khởi tạo lực căng ban đầu, cần độ chính xác cao, không vội
TensionReading tensionReadStable() {
  Serial.println("[TENSION] Cho on dinh 5 giay...");
  delay(5000);

  Serial.println("[TENSION] Doc trung binh 20 lan...");
  long sum[4] = {0, 0, 0, 0};
  int count = 0;
  long raw[4];

  for (int n = 0; n < 20; n++) {
    if (readAllRaw(raw)) {
      for (int i = 0; i < 4; i++) sum[i] += raw[i];
      count++;
    }
    delay(50);
  }

  TensionReading result;
  if (count == 0) {
    for (int i = 0; i < 4; i++) {
      result.valid[i] = false;
      result.f_loadcell[i] = 0.0f;
      result.t_cable[i] = 0.0f;
    }
    Serial.println("[TENSION] LOI: khong doc duoc lan nao!");
    return result;
  }

  long avgRaw[4];
  for (int i = 0; i < 4; i++) avgRaw[i] = sum[i] / count;

  rawToForce(avgRaw, result.f_loadcell, result.t_cable, result.valid);

  Serial.println("[TENSION] Ket qua trung binh:");
  for (int i = 0; i < 4; i++) {
    Serial.print("  T_cable["); Serial.print(i); Serial.print("] = ");
    Serial.print(result.t_cable[i], 3); Serial.println(" N");
  }
  return result;
}