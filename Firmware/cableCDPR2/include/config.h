// config.h
#pragma once
#include <math.h>

// ===== KÍCH THƯỚC KHUNG =====
#define FRAME_W         300.0f      // mm
#define FRAME_H         300.0f      // mm

// Tọa độ 4 điểm neo — gốc tọa độ = tâm khung
// Thứ tự: LC#1(top-left), LC#2(top-right), LC#3(bottom-right), LC#4(bottom-left)
const float ANCHOR_X[4] = {-140.0f,  140.0f,  140.0f, -140.0f};
const float ANCHOR_Y[4] = { 160.0f,  160.0f, -160.0f, -160.0f};

// ===== CƠ HỌC MOTOR =====
#define STEPS_PER_REV       200
#define MICROSTEP            16
#define DRUM_DIAMETER_MM     35.0f
#define DRUM_CIRCUMFERENCE  (3.14159f * DRUM_DIAMETER_MM)  // = 109.96mm
#define STEPS_PER_MM        ((STEPS_PER_REV * MICROSTEP) / DRUM_CIRCUMFERENCE)

// ===== GPIO MOTOR =====
const int STEP_PINS[4] = {14, 26, 33, 13};
const int DIR_PINS[4]  = {27, 25, 32, 2};
#define EN_PIN  17

// ===== GPIO LOADCELL =====
#define HX_SCK_PIN  4
const int HX_DOUT_PINS[4] = {36, 39, 34, 35};

// ===== CALIB LOADCELL =====
// Dấu âm = loadcell lắp ngược chiều lực (bình thường)
// LC#2 (index 1) dấu dương = lắp ngược so với 3 con còn lại
const float CALIB_FACTORS[4] = {
  -1107.561f,   // LC#1 — GPIO 36
   1042.253f,   // LC#2 — GPIO 39
  -1077.644f,   // LC#3 — GPIO 34
  -1090.602f    // LC#4 — GPIO 35
};

// ===== THÔNG SỐ 3-PULLEY =====
// T_cable = F_loadcell / (2 × cos(θ/2))
// θ = góc giữa 2 đoạn dây tại pulley loadcell — ĐO THỰC TẾ RỒI ĐIỀN VÀO
#define PULLEY_THETA_DEG    38.68f   // ← chưa đo, tạm để 90°
#define TENSION_FACTOR      (1.0f / (2.0f * cosf(PULLEY_THETA_DEG * 3.14159f / 360.0f)))

// ===== NGOẠI LỰC (trọng lượng end-effector) =====
// CẦN CÂN THỰC TẾ end-effector (bao gồm mọi thứ gắn lên nó)
#define EFFECTOR_MASS_KG   0.00f
#define GRAVITY_MS2        9.81f

// Giả định mặt phẳng làm việc THẲNG ĐỨNG, trục Y trong ANCHOR_Y là phương trọng lực
// Nếu khung của bạn nằm NGANG (mặt bàn) → đổi W_EXT_Y = 0.0f
#define W_EXT_X   0.0f
#define W_EXT_Y   (-EFFECTOR_MASS_KG * GRAVITY_MS2)

// ===== LỰC CĂNG =====
#define T_MIN_N     0.4f    // N 
#define T_MAX_N     2.0f    // N
#define T_NOMINAL_N   0.5f    // N — MỤC TIÊU lực căng

// ===== TỐC ĐỘ MOTOR =====
#define V_START_HZ   300.0f
#define V_MAX_HZ    5000.0f
#define RAMP_STEPS    500

// ===== WORKSPACE =====
#define WFW_X_MAX   140.0f  // mm từ tâm
#define WFW_Y_MAX   140.0f  // mm từ tâm