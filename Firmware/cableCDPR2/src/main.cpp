// main.cpp
#include <Arduino.h>
#include "config.h"
#include "kinematics.h"
#include "motion.h"
#include "tension.h"
#include "tension_distribution.h"
#include "forward_kinematics.h"
#include "position_control.h"

float currentX = 0.0f;
float currentY = 0.0f;
float posBiasWx = 0.0f;
float posBiasWy = 0.0f;
float tRef[4] = {0, 0, 0, 0};
bool  tRefCalibrated = false;
float currentLength[4];

// ===== THÔNG SỐ FEEDBACK LỰC CĂNG =====
const float TENSION_KP = 30.0f;          // steps per Newton sai lệch — chỉnh nếu quá nhạy/chậm
const float TENSION_ERR_THRESHOLD = 0.08f; // N — dưới ngưỡng này coi như đủ tốt, không chỉnh
const long  MAX_CORRECTION_STEPS = 4;     // giới hạn mỗi lần chỉnh — an toàn, tránh nhảy vọt

// ===== NHẬN LỆNH X,Y TỪ SERIAL =====
bool readXY(float &x, float &y) {
  static String buf = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      int commaIdx = buf.indexOf(',');
      if (commaIdx < 0) { buf = ""; return false; }
      x = buf.substring(0, commaIdx).toFloat();
      y = buf.substring(commaIdx + 1).toFloat();
      buf = "";
      return true;
    } else if (c != '\r') {
      buf += c;
    }
  }
  return false;
}

// ===== GIỮ LỰC CĂNG KHỞI TẠO — XEN KẼ 4 CÁP, GIỮ ĐỐI XỨNG =====
void holdInitialTension() {
  Serial.println("\n[INIT] Tinh phan bo luc cang toi uu tai vi tri hien tai...");

  TensionSolution ts = solveTensionDistribution(currentX, currentY);
  if (!ts.feasible) {
    Serial.println("[INIT] WARNING: khong tim duoc nghiem hop le, dung T_nominal!");
  }
  Serial.println("[INIT] Muc tieu luc cang tung cap:");
  for (int i = 0; i < 4; i++) {
    Serial.print("  T_target["); Serial.print(i); Serial.print("] = ");
    Serial.println(ts.T[i], 3);
  }

  const int MAX_ROUNDS = 100;   // số vòng xoay tối đa
  const int STEP_CHUNK = 1;     // mỗi lần chỉnh RẤT nhỏ — giữ đối xứng
  bool converged[4] = {false, false, false, false};

  long totalSteps[4] = {0, 0, 0, 0};
  bool moveDir[4]     = {false, false, false, false};

  for (int round = 0; round < MAX_ROUNDS; round++) {
    bool allDone = true;

    for (int i = 0; i < 4; i++) {
      if (converged[i]) continue;
      allDone = false;

      TensionReading trFast = tensionReadRealtime();
      if (!trFast.valid[i]) continue;

      float t = trFast.t_cable[i];
      float target = ts.T[i];

      if (t >= target * 0.9f && t <= target * 1.1f) {
        converged[i] = true;
        Serial.print("[INIT] Cap "); Serial.print(i);
        Serial.print(" dat T="); Serial.println(t, 3);
        continue;
      }

      long steps[4] = {0,0,0,0};
      bool dir[4]   = {false,false,false,false};
      steps[i] = STEP_CHUNK;
      dir[i]   = (t < target);
      moveDir[i] = dir[i];

      motionStart(steps, dir);
      while (!motionAllDone()) motionUpdate();

      totalSteps[i] += STEP_CHUNK;
      delay(15);   // chờ lực ổn định ngắn giữa các lần chỉnh xen kẽ
    }

    if (allDone) break;
  }

  for (int i = 0; i < 4; i++) {
    if (!converged[i]) {
      Serial.print("[INIT] WARNING: Cap "); Serial.print(i);
      Serial.println(" khong hoi tu sau MAX_ROUNDS!");
    }
  }

  // ===== CẬP NHẬT currentLength[] theo BƯỚC THỰC TẾ đã chạy =====
  for (int i = 0; i < 4; i++) {
    float deltaMM = (float)totalSteps[i] / STEPS_PER_MM;
    if (moveDir[i]) {
      currentLength[i] -= deltaMM;   // đã THU → cáp ngắn lại
    } else {
      currentLength[i] += deltaMM;   // đã NHA → cáp dài ra
    }
  }

  // ===== DÙNG FORWARD KINEMATICS để biết vị trí THỰC sau khi giữ lực căng =====
  FKResult fk = forwardKinematics(currentLength);
  if (fk.valid) {
    currentX = fk.x;
    currentY = fk.y;
    Serial.print("[INIT] Vi tri thuc sau khi giu luc cang: (");
    Serial.print(currentX); Serial.print(", "); Serial.print(currentY);
    Serial.print(")  residual="); Serial.println(fk.residual, 3);
  } else {
    Serial.println("[INIT] WARNING: Forward kinematics khong giai duoc!");
  }

  Serial.println("[INIT] Hoan tat giu luc cang toi uu!");
}

// ===== CALIB — chỉ chạy 1 lần, khi ban đã căn tay đúng tâm khung =====
void calibrateHomeTension() {
  Serial.println("\n[CALIB] Dam bao end-effector dang o DUNG TAM KHUNG bang tay/mot!");
  Serial.println("Nhan phim bat ky de do luc tham chieu...");
  while (!Serial.available()) delay(10);
  Serial.read();

  TensionReading tr = tensionReadStable();

  bool allValid = true;
  for (int i = 0; i < 4; i++) {
    if (!tr.valid[i]) allValid = false;
    tRef[i] = tr.t_cable[i];
  }

  if (!allValid) {
    Serial.println("[CALIB] WARNING: co loadcell khong doc duoc, ket qua co the sai!");
  }

  tRefCalibrated = true;
  Serial.println("[CALIB] Da luu T_ref:");
  for (int i = 0; i < 4; i++) {
    Serial.print("  T_ref["); Serial.print(i); Serial.print("] = ");
    Serial.println(tRef[i], 3);
  }
  Serial.println("[CALIB] Hoan tat! Tu gio co the dung lenh 'home' de ve tam.");
}

// ===== VỀ TÂM BẰNG LỰC CĂNG =====
void homeByTension() {
  if (!tRefCalibrated) {
    Serial.println("[HOME] ERROR: chua calib T_ref! Go 'calib_home' truoc.");
    return;
  }

  Serial.println("[HOME] Bat dau ve tam bang luc cang...");

  const int   MAX_ROUNDS  = 300;
  const float TOLERANCE   = 0.05f;
  const int   STEP_CHUNK  = 1;

  bool converged[4]  = {false, false, false, false};
  long totalSteps[4] = {0, 0, 0, 0};
  bool moveDir[4]    = {false, false, false, false};

  for (int round = 0; round < MAX_ROUNDS; round++) {
    bool allDone = true;

    for (int i = 0; i < 4; i++) {
      if (converged[i]) continue;
      allDone = false;

      TensionReading tr = tensionReadRealtime();
      if (!tr.valid[i]) continue;

      float error = tRef[i] - tr.t_cable[i];

      if (fabs(error) < TOLERANCE) {
        converged[i] = true;
        Serial.print("[HOME] Cap "); Serial.print(i);
        Serial.print(" hoi tu, T="); Serial.println(tr.t_cable[i], 3);
        continue;
      }

      long steps[4] = {0,0,0,0};
      bool dir[4]   = {false,false,false,false};
      steps[i] = STEP_CHUNK;
      dir[i]   = (error > 0);
      moveDir[i] = dir[i];

      motionStart(steps, dir);
      while (!motionAllDone()) motionUpdate();

      totalSteps[i] += STEP_CHUNK;
      delay(15);
    }

    if (allDone) {
      Serial.println("[HOME] Da hoi tu ve tam!");
      break;
    }
  }

  for (int i = 0; i < 4; i++) {
    if (!converged[i]) {
      Serial.print("[HOME] WARNING: Cap "); Serial.print(i);
      Serial.println(" khong hoi tu sau MAX_ROUNDS!");
    }
  }

  for (int i = 0; i < 4; i++) {
    float deltaMM = (float)totalSteps[i] / STEPS_PER_MM;
    if (moveDir[i]) currentLength[i] -= deltaMM;
    else            currentLength[i] += deltaMM;
  }

  FKResult fk = forwardKinematics(currentLength);
  if (fk.valid) {
    currentX = fk.x;
    currentY = fk.y;
    Serial.print("[HOME] Vi tri sau khi home: (");
    Serial.print(currentX); Serial.print(", "); Serial.print(currentY);
    Serial.print(")  residual="); Serial.println(fk.residual, 3);

    if (fk.residual > 5.0f) {
      Serial.println("[HOME] WARNING: residual cao, ket qua co the khong dang tin!");
    }
  } else {
    Serial.println("[HOME] WARNING: FK khong giai duoc, gan tam (0,0) theo mac dinh");
    currentX = 0; currentY = 0;
    computeIK(0, 0, currentLength);
  }

  Serial.println("[HOME] Hoan tat!");
}

// ===== HÀM tensionControlUpdate — đặt sau holdInitialTension(), TRƯỚC moveToPoint() =====
void tensionControlUpdate(float startX, float startY, float targetX, float targetY,
                          const bool segDir[4]) {
  // Ước lượng % hoàn thành — trung bình theo motor có steps > 0
  float fracSum = 0.0f; int cnt = 0;
  for (int i = 0; i < 4; i++) {
    long done = motionGetStepsDone(i);
    long remaining = motionGetStepsRemaining(i);
    long total = done + remaining;
    if (total > 0) { fracSum += (float)done / total; cnt++; }
  }
  float frac = (cnt > 0) ? (fracSum / cnt) : 1.0f;

  float estX = startX + frac * (targetX - startX);
  float estY = startY + frac * (targetY - startY);

  TensionSolution ts = solveTensionDistributionWithBias(estX, estY, posBiasWx, posBiasWy);
  TensionReading  tr = tensionReadRealtime();

  for (int i = 0; i < 4; i++) {
    if (!tr.valid[i]) continue;

    float error = ts.T[i] - tr.t_cable[i];   // + = thieu luc, can thu them
    if (fabs(error) < TENSION_ERR_THRESHOLD) continue;

    bool neededDir = (error > 0);   // true = can thu (tang luc)

    if (neededDir != segDir[i]) {
      // Chieu can chinh nguoc voi chieu dang chay — bo qua de an toan
      continue;
    }

    long corr = (long)(fabs(error) * TENSION_KP);
    if (corr > MAX_CORRECTION_STEPS) corr = MAX_CORRECTION_STEPS;
    if (corr < 1) continue;

    motionAddSteps(i, corr);
  }
}

// ===== DI CHUYỂN ĐẾN 1 ĐIỂM — IK + motion song song tension realtime =====
bool moveToPoint(float targetX, float targetY) {
  if (!isInWorkspace(targetX, targetY)) {
    Serial.println("[ERROR] Toa do ngoai WFW! Bo qua lenh.");
    return false;
  }

  IKResult r = computeMotion(targetX, targetY, currentX, currentY, currentLength);
  if (!r.valid) {
    Serial.println("[ERROR] IK khong hop le!");
    return false;
  }

  float startX = currentX, startY = currentY;   // lưu điểm đầu để nội suy

  float prevLength[4];
  for (int i = 0; i < 4; i++) prevLength[i] = currentLength[i];   // lưu chiều dài cáp TRƯỚC khi chạy

  Serial.print("[MOTION] Di chuyen den ("); Serial.print(targetX);
  Serial.print(", "); Serial.print(targetY); Serial.println(")");

  motionStart(r.steps, r.dir);

  unsigned long lastTensionControl = 0;
  unsigned long lastTensionPrint   = 0;

  while (!motionAllDone()) {
    motionUpdate();

    // Feedback lực căng — chỉnh chủ động, không chỉ cảnh báo
    if (millis() - lastTensionControl > 100) {   // 10Hz
      lastTensionControl = millis();
      tensionControlUpdate(startX, startY, targetX, targetY, r.dir);
    }

    // In Teleplot giám sát
    if (millis() - lastTensionPrint > 50) {
      lastTensionPrint = millis();
      TensionReading tr = tensionReadRealtime();
      for (int i = 0; i < 4; i++) {
        Serial.print(">T_cable_"); Serial.print(i);
        Serial.print(":"); Serial.println(tr.t_cable[i], 3);
        if (tr.valid[i] && tr.t_cable[i] > T_MAX_N) {
          Serial.print("[WARNING] Cap "); Serial.print(i);
          Serial.println(" vuot T_MAX!");
        }
      }
    }
  }

  // Dùng SỐ BƯỚC THỰC TẾ đã chạy (IK + correction lực căng), không dùng IK ly thuyet
  for (int i = 0; i < 4; i++) {
    long actualSteps = motionGetStepsDone(i);
    float actualDeltaMM = (float)actualSteps / STEPS_PER_MM;

    if (r.dir[i]) {
      // dir=true -> THU cap (rut ngan)
      currentLength[i] = prevLength[i] - actualDeltaMM;
    } else {
      // dir=false -> NHA cap (keo dai)
      currentLength[i] = prevLength[i] + actualDeltaMM;
    }
  }

// ===== DÙNG FORWARD KINEMATICS =====
  FKResult fk = forwardKinematics(currentLength);
  if (fk.valid && fk.residual < 5.0f) {   // ngưỡng tin cậy 5mm — chỉnh nếu cần
    currentX = fk.x;
    currentY = fk.y;
    Serial.print("[FK] Vi tri uoc luong: (");
    Serial.print(currentX); Serial.print(", "); Serial.print(currentY);
    Serial.print(")  residual="); Serial.println(fk.residual, 3);
// ===== THÊM: cập nhật PID vị trí, sinh wrench bias cho lần chạy sau =====
    positionControlUpdate(targetX, targetY, currentX, currentY, posBiasWx, posBiasWy);

  } else {
    // FK không đáng tin (residual cao) — tạm dùng target làm dự phòng
    currentX = targetX;
    currentY = targetY;
    Serial.print("[FK] WARNING: residual cao ("); Serial.print(fk.residual, 2);
    Serial.println("mm), dung target lam du phong!");
  }

  Serial.println("[MOTION] Hoan thanh!");
  return true;
}

// ===== CHẠY QUỸ ĐẠO — danh sách điểm liên tiếp =====
// Sinh N điểm trên đường tròn bán kính r, tâm (cx, cy)

void runTrajectory(float waypoints[][2], int n) {
  Serial.println("[TRAJECTORY] Bat dau quy dao...");
  for (int i = 0; i < n; i++) {
    Serial.print("[TRAJECTORY] Diem "); Serial.print(i+1);
    Serial.print("/"); Serial.println(n);

    bool ok = moveToPoint(waypoints[i][0], waypoints[i][1]);
    if (!ok) {
      Serial.println("[TRAJECTORY] Dung lai do loi!");
      return;
    }
  }
  Serial.println("[TRAJECTORY] Hoan thanh toan bo quy dao!");
}

// THAY bản cũ (có while anyPending) bằng bản đơn giản này
void appendSegment(const float fromLen[4], const float toLen[4]) {
  for (int i = 0; i < 4; i++) {
    float delta = fromLen[i] - toLen[i];
    bool  dir   = (delta > 0);
    long  steps = (long)(fabs(delta) * STEPS_PER_MM);
    motionAppend(i, steps, dir);   // luôn thành công ngay, không cần vòng lặp chờ
  }
}

// Sinh N điểm trên đường tròn
void generateCircle(float waypoints[][2], int n, float cx, float cy, float radius) {
  for (int i = 0; i < n; i++) {
    float angle = (2.0f * PI * i) / n;
    waypoints[i][0] = cx + radius * cosf(angle);
    waypoints[i][1] = cy + radius * sinf(angle);
  }
}


void runContinuousPath(float waypoints[][2], int n, float cruiseHz) {
  Serial.println("[STREAM] Bat dau duong lien tuc...");

  moveToPoint(waypoints[0][0], waypoints[0][1]);

  float prevLen[4];
  for (int i = 0; i < 4; i++) prevLen[i] = currentLength[i];

  unsigned long lastPrint = 0;

  for (int p = 1; p < n; p++) {
    if (!isInWorkspace(waypoints[p][0], waypoints[p][1])) {
      Serial.println("[STREAM] ERROR: diem ngoai WFW, dung lai!");
      break;
    }
    float curLen[4];
    computeIK(waypoints[p][0], waypoints[p][1], curLen);
    appendSegment(prevLen, curLen);
    for (int i = 0; i < 4; i++) prevLen[i] = curLen[i];

    motionStreamUpdate(cruiseHz);

    if (millis() - lastPrint > 50) {
      lastPrint = millis();
      TensionReading tr = tensionReadRealtime();
      for (int i = 0; i < 4; i++) {
        Serial.print(">T_cable_"); Serial.print(i);
        Serial.print(":"); Serial.println(tr.t_cable[i], 3);
      }
    }
  }

  while (motionStreamMoving()) {
    motionStreamUpdate(cruiseHz);
  }

  currentX = waypoints[n-1][0];
  currentY = waypoints[n-1][1];
  for (int i = 0; i < 4; i++) currentLength[i] = prevLen[i];

  Serial.println("[STREAM] Hoan thanh duong lien tuc!");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n===== KHOI DONG CDPR =====");

  tensionInit();
  motionInit();

  // Khởi tạo tại tâm khung (0,0)
  computeIK(0, 0, currentLength);
  currentX = 0; currentY = 0;
  Serial.println("[INIT] Vi tri bat dau: tam khung (0,0)");

  holdInitialTension();

  Serial.println("\n===== SAN SANG =====");
  Serial.println("Nhap toa do: X,Y (vi du: 5,5) roi Enter");
  Serial.println("Hoac go 'square or circle' de chay quy dao vuong mau");
}

void loop() {
  static String cmdBuf = "";

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      cmdBuf.trim();

      if (cmdBuf == "square") {
        float square[][2] = {{80,80}, {80,-80}, {-80,-80}, {-80,80}, {80,80}, {0,0}};
        runTrajectory(square, 6);
      }
      else if (cmdBuf == "circle") {
        const int N_POINTS = 200;
        const float RADIUS  = 80.0f;
        const float CRUISE_HZ = 1000.0f;

        float circlePoints[N_POINTS + 1][2];
        generateCircle(circlePoints, N_POINTS, 0.0f, 0.0f, RADIUS);
        circlePoints[N_POINTS][0] = circlePoints[0][0];
        circlePoints[N_POINTS][1] = circlePoints[0][1];

        runContinuousPath(circlePoints, N_POINTS + 1, CRUISE_HZ);
      }
      else if (cmdBuf == "calib_home") {
        calibrateHomeTension();
      }
      else if (cmdBuf == "home") {
        homeByTension();
      }
      else if (cmdBuf == "reset_pid") {
        positionControlReset();
        posBiasWx = 0.0f; posBiasWy = 0.0f;
        Serial.println("Da reset PID vi tri!");
      }
      else if (cmdBuf == "tune_test") {
  const int N_REPEAT = 10;
  Serial.println("[TUNE] Bat dau chuoi test lap lai...");
  for (int i = 0; i < N_REPEAT; i++) {
    Serial.print("[TUNE] Lan "); Serial.print(i+1); Serial.print("/"); Serial.println(N_REPEAT);
    moveToPoint(20, 20);
    delay(500);
    moveToPoint(0, 0);
    delay(500);
  }
  Serial.println("[TUNE] Hoan tat chuoi test!");
}
      else {
        int commaIdx = cmdBuf.indexOf(',');
        if (commaIdx > 0) {
          float x = cmdBuf.substring(0, commaIdx).toFloat();
          float y = cmdBuf.substring(commaIdx + 1).toFloat();
          moveToPoint(x, y);
        }
      }

      cmdBuf = "";
      Serial.println("Nhap lenh tiep theo:");
    } else if (c != '\r') {
      cmdBuf += c;
    }
  }
}

// #include "tension.h"

// void setup() {
//   Serial.begin(115200);
//   tensionInit();

//   // Đo lực khởi tạo — chờ ổn định, chính xác
//   TensionReading initTension = tensionReadStable();
//   // ... dùng initTension để quyết định giữ/điều chỉnh lực ban đầu
// }

// void loop() {
//   // Trong lúc motor chạy — đọc nhanh, không chờ
//   TensionReading tr = tensionReadRealtime();
//   for (int i = 0; i < 4; i++) {
//     Serial.print(">T_cable_"); Serial.print(i);
//     Serial.print(":"); Serial.println(tr.t_cable[i], 3);
//   }
// }

