// position_control.h
#pragma once
#include "config.h"

// Reset bộ nhớ PID — gọi khi bắt đầu 1 phiên làm việc mới hoặc sau khi home
void positionControlReset();

// Cập nhật PID vị trí — gọi SAU KHI 1 chuyển động hoàn thành và FK đã tính xong
// targetX/Y: vị trí đích lý thuyết
// fkX/Y: vị trí ước lượng thực tế từ Forward Kinematics
// Trả về: wrench bổ sung (N) cần cộng vào trọng lực khi tính QP
void positionControlUpdate(float targetX, float targetY, float fkX, float fkY,
                            float &outWx, float &outWy);