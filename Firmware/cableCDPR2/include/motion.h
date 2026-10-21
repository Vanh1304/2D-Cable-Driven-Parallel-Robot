// // motion.h
// #pragma once
// #include <Arduino.h>
// #include "config.h"

// void motionInit();
// void motionStart(const long steps[4], const bool dir[4]);
// void motionUpdate();          // gọi liên tục trong loop()
// bool motionAllDone();
// void motionHoldEnable(bool en);   // bật/tắt driver

// // motion.h — thêm các dòng này vào cuối file (giữ nguyên phần cũ)
// bool motionAppend(int motorIdx, long deltaSteps, bool dir);
// void motionStreamUpdate(float cruiseHz);
// bool motionStreamMoving();
// // motion.h — thêm vào cuối file, giữ nguyên phần cũ
// long motionGetStepsDone(int i);
// long motionGetStepsRemaining(int i);
// void motionAddSteps(int i, long extraSteps);

// motion.h
#pragma once
#include <Arduino.h>
#include "config.h"

void motionInit();
void motionHoldEnable(bool en);

// Chế độ rời rạc — từng điểm, có ramp riêng
void motionStart(const long steps[4], const bool dir[4]);
void motionUpdate();
bool motionAllDone();

// Chế độ streaming — đường liên tục, không dừng giữa các điểm
bool motionAppend(int motorIdx, long deltaSteps, bool dir);
void motionStreamUpdate(float cruiseHz);
bool motionStreamMoving();

long motionGetStepsDone(int i);
long motionGetStepsRemaining(int i);
void motionAddSteps(int i, long extraSteps);