# """
# track_trajectory.py
# --------------------
# Theo dõi (track) chấm đỏ trong video, vẽ quỹ đạo thực tế lên lưới tọa độ (cm),
# so sánh với quỹ đạo chuẩn (hình vuông cạnh 8cm hoặc hình tròn bán kính 8cm),
# và tính sai số (error) giữa quỹ đạo thực tế và quỹ đạo lý tưởng.

# Yêu cầu thư viện:
#     pip install opencv-python numpy matplotlib scipy --break-system-packages

# Cách dùng cơ bản:
#     python track_trajectory.py --video path/to/video.mp4 --shape square --size 8 --scale 10

# Trong đó:
#     --shape   : "square" hoặc "circle"
#     --size    : kích thước quỹ đạo chuẩn (cm) - cạnh hình vuông hoặc bán kính hình tròn
#     --scale   : số pixel tương ứng với 1 cm (pixels/cm). Nếu không biết,
#                 dùng --calib để tự động tính từ 2 điểm bạn click trên khung hình đầu tiên.
# """

# import cv2
# import numpy as np
# import matplotlib.pyplot as plt
# import argparse
# import sys
# from scipy.spatial import cKDTree


# # ----------------------------------------------------------------------
# # 1. PHÁT HIỆN CHẤM ĐỎ TRONG TỪNG KHUNG HÌNH
# # ----------------------------------------------------------------------
# def detect_red_dot(frame):
#     """Trả về tọa độ pixel (x, y) tâm chấm đỏ lớn nhất trong frame, hoặc None."""
#     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

#     # Màu đỏ nằm ở 2 đầu của dải Hue trong không gian HSV -> cần 2 mask
#     lower_red1 = np.array([0, 100, 80])
#     upper_red1 = np.array([10, 255, 255])
#     lower_red2 = np.array([170, 100, 80])
#     upper_red2 = np.array([180, 255, 255])

#     mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
#     mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
#     mask = cv2.bitwise_or(mask1, mask2)

#     # Lọc nhiễu
#     kernel = np.ones((5, 5), np.uint8)
#     mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
#     mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

#     contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#     if not contours:
#         return None

#     # Chọn contour lớn nhất (giả định là chấm đỏ cần theo dõi)
#     c = max(contours, key=cv2.contourArea)
#     if cv2.contourArea(c) < 15:  # bỏ qua nhiễu quá nhỏ
#         return None

#     M = cv2.moments(c)
#     if M["m00"] == 0:
#         return None
#     cx = M["m10"] / M["m00"]
#     cy = M["m01"] / M["m00"]
#     return (cx, cy)


# # ----------------------------------------------------------------------
# # 2. CALIBRATION: CHỌN THỦ CÔNG 2 ĐIỂM ĐỂ TÍNH TỈ LỆ PIXEL/CM
# # ----------------------------------------------------------------------
# def calibrate_scale(video_path, known_distance_cm):
#     """
#     Mở khung hình đầu tiên của video, cho người dùng click 2 điểm
#     tương ứng với một khoảng cách đã biết (known_distance_cm),
#     trả về scale = pixels / cm.
#     """
#     cap = cv2.VideoCapture(video_path)
#     ret, frame = cap.read()
#     cap.release()
#     if not ret:
#         raise RuntimeError("Không đọc được khung hình đầu tiên của video.")

#     points = []

#     def on_click(event, x, y, flags, param):
#         if event == cv2.EVENT_LBUTTONDOWN and len(points) < 2:
#             points.append((x, y))
#             cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)
#             cv2.imshow("Calibration - click 2 diem", frame)

#     cv2.imshow("Calibration - click 2 diem", frame)
#     cv2.setMouseCallback("Calibration - click 2 diem", on_click)
#     print(f"Hãy click 2 điểm trên khung hình cách nhau {known_distance_cm} cm, "
#           f"rồi nhấn phím bất kỳ để tiếp tục...")
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()

#     if len(points) != 2:
#         raise RuntimeError("Cần chọn đúng 2 điểm để calibrate.")

#     (x1, y1), (x2, y2) = points
#     pixel_dist = np.hypot(x2 - x1, y2 - y1)
#     scale = pixel_dist / known_distance_cm  # pixels / cm
#     print(f"Scale tính được: {scale:.3f} pixels/cm")
#     return scale


# # ----------------------------------------------------------------------
# # 3. TRACK TOÀN BỘ VIDEO -> DANH SÁCH ĐIỂM (cm)
# # ----------------------------------------------------------------------
# def track_video(video_path, scale_px_per_cm, origin=None):
#     """
#     Duyệt toàn bộ video, phát hiện chấm đỏ ở mỗi khung hình,
#     chuyển tọa độ pixel -> cm.
#     origin: (ox, oy) pixel gốc tọa độ. Nếu None, dùng điểm đầu tiên phát hiện được làm gốc.
#     Trả về: mảng numpy Nx2 tọa độ (x_cm, y_cm), và (ox, oy) gốc tọa độ pixel đã dùng.
#     """
#     cap = cv2.VideoCapture(video_path)
#     if not cap.isOpened():
#         raise RuntimeError(f"Không mở được video: {video_path}")

#     traj_px = []
#     frame_idx = 0
#     while True:
#         ret, frame = cap.read()
#         if not ret:
#             break
#         pt = detect_red_dot(frame)
#         if pt is not None:
#             traj_px.append(pt)
#         frame_idx += 1
#     cap.release()

#     if len(traj_px) == 0:
#         raise RuntimeError("Không phát hiện được chấm đỏ trong toàn bộ video. "
#                             "Hãy kiểm tra lại ngưỡng màu HSV.")

#     traj_px = np.array(traj_px)
#     print(f"Đã xử lý {frame_idx} khung hình, phát hiện chấm đỏ ở {len(traj_px)} khung hình.")

#     if origin is None:
#         ox, oy = traj_px[0]
#     else:
#         ox, oy = origin

#     # Đổi trục y (ảnh có gốc tọa độ trên-trái, y hướng xuống) -> hệ tọa độ toán học (y hướng lên)
#     traj_cm = np.zeros_like(traj_px)
#     traj_cm[:, 0] = (traj_px[:, 0] - ox) / scale_px_per_cm
#     traj_cm[:, 1] = -(traj_px[:, 1] - oy) / scale_px_per_cm

#     return traj_cm, (ox, oy)


# # ----------------------------------------------------------------------
# # 4. TẠO QUỸ ĐẠO CHUẨN (LÝ TƯỞNG)
# # ----------------------------------------------------------------------
# def ideal_square(side_cm, n_points_per_side=200, center=(0, 0)):
#     """Tạo các điểm của hình vuông cạnh side_cm, tâm tại center, các cạnh song song trục."""
#     h = side_cm
#     cx, cy = center
#     corners = np.array([
#         [-h, -h], [h, -h], [h, h], [-h, h], [-h, -h]
#     ]) + np.array([cx, cy])
#     pts = []
#     for i in range(4):
#         p0, p1 = corners[i], corners[i + 1]
#         t = np.linspace(0, 1, n_points_per_side, endpoint=False)[:, None]
#         seg = p0 + t * (p1 - p0)
#         pts.append(seg)
#     return np.vstack(pts)


# def ideal_circle(radius_cm, n_points=800, center=(0, 0)):
#     """Tạo các điểm của hình tròn bán kính radius_cm, tâm tại center."""
#     theta = np.linspace(0, 2 * np.pi, n_points, endpoint=False)
#     cx, cy = center
#     x = cx + radius_cm * np.cos(theta)
#     y = cy + radius_cm * np.sin(theta)
#     return np.column_stack([x, y])


# # ----------------------------------------------------------------------
# # 5. TÍNH SAI SỐ: KHOẢNG CÁCH TỪ MỖI ĐIỂM THỰC TẾ ĐẾN QUỸ ĐẠO CHUẨN GẦN NHẤT
# # ----------------------------------------------------------------------
# def compute_error(actual_pts, ideal_pts):
#     """
#     Với mỗi điểm thực tế, tìm khoảng cách ngắn nhất tới quỹ đạo chuẩn (tập điểm dày đặc).
#     Trả về mảng sai số (cm) và các chỉ số thống kê.
#     """
#     tree = cKDTree(ideal_pts)
#     dists, _ = tree.query(actual_pts)
#     stats = {
#         "mean_error_cm": float(np.mean(dists)),
#         "max_error_cm": float(np.max(dists)),
#         "rmse_cm": float(np.sqrt(np.mean(dists ** 2))),
#         "std_error_cm": float(np.std(dists)),
#     }
#     return dists, stats


# # ----------------------------------------------------------------------
# # 6. VẼ KẾT QUẢ: QUỸ ĐẠO THỰC TẾ + CHUẨN + LƯỚI TỌA ĐỘ + SAI SỐ
# # ----------------------------------------------------------------------
# def plot_result(actual_pts, ideal_pts, stats, shape, size_cm, output_path):
#     fig, axes = plt.subplots(1, 2, figsize=(14, 6))

#     # --- Bên trái: quỹ đạo trên lưới tọa độ ---
#     ax = axes[0]
#     margin = size_cm * 0.6
#     ax.plot(ideal_pts[:, 0], ideal_pts[:, 1], "--", color="tab:blue",
#              linewidth=1.5, label=f"Quỹ đạo chuẩn ({shape}, {size_cm}cm)")
#     sc = ax.scatter(actual_pts[:, 0], actual_pts[:, 1], c=range(len(actual_pts)),
#                      cmap="autumn", s=10, label="Quỹ đạo thực tế")
#     ax.plot(actual_pts[:, 0], actual_pts[:, 1], color="tab:red", alpha=0.3, linewidth=1)

#     ax.set_xlim(-size_cm/2 - margin, size_cm/2 + margin)
#     ax.set_ylim(-size_cm/2 - margin, size_cm/2 + margin)
#     ax.set_xticks(np.arange(-size_cm - margin, size_cm + margin + 1, 1))
#     ax.set_yticks(np.arange(-size_cm - margin, size_cm + margin + 1, 1))
#     ax.grid(True, linestyle=":", linewidth=0.6)
#     ax.set_aspect("equal")
#     ax.set_xlabel("X (cm)")
#     ax.set_ylabel("Y (cm)")
#     ax.set_title("Quỹ đạo thực tế vs Quỹ đạo chuẩn (lưới 1cm)")
#     ax.legend(loc="upper right", fontsize=9)
#     fig.colorbar(sc, ax=ax, label="Thứ tự khung hình (thời gian)")

#     # --- Bên phải: biểu đồ sai số theo thời gian ---
#     ax2 = axes[1]
#     dists, _ = compute_error(actual_pts, ideal_pts)
#     ax2.plot(dists, color="tab:red", linewidth=1)
#     ax2.axhline(stats["mean_error_cm"], color="tab:blue", linestyle="--",
#                 label=f"Sai số trung bình = {stats['mean_error_cm']:.3f} cm")
#     ax2.set_xlabel("Chỉ số điểm (theo thời gian)")
#     ax2.set_ylabel("Sai số (cm)")
#     ax2.set_title("Sai số quỹ đạo theo thời gian")
#     ax2.legend(fontsize=9)
#     ax2.grid(True, linestyle=":", linewidth=0.6)

#     stats_text = (
#         f"Sai số trung bình: {stats['mean_error_cm']:.3f} cm\n"
#         f"Sai số RMSE: {stats['rmse_cm']:.3f} cm\n"
#         f"Sai số lớn nhất: {stats['max_error_cm']:.3f} cm\n"
#         f"Độ lệch chuẩn: {stats['std_error_cm']:.3f} cm"
#     )
#     fig.text(0.5, -0.02, stats_text, ha="center", fontsize=10,
#               bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5))

#     plt.tight_layout()
#     plt.savefig(output_path, dpi=200, bbox_inches="tight")
#     print(f"Đã lưu ảnh kết quả tại: {output_path}")
#     plt.close(fig)


# # ----------------------------------------------------------------------
# # 7. MAIN
# # ----------------------------------------------------------------------
# def main():
#     parser = argparse.ArgumentParser(description="Track quỹ đạo chấm đỏ và tính sai số so với quỹ đạo chuẩn.")
#     parser.add_argument("--video", required=True, help="Đường dẫn tới file video")
#     parser.add_argument("--shape", choices=["square", "circle"], required=True,
#                          help="Hình dạng quỹ đạo chuẩn: square (hình vuông) hoặc circle (hình tròn)")
#     parser.add_argument("--size", type=float, default=8.0,
#                          help="Kích thước (cm): cạnh hình vuông hoặc bán kính hình tròn. Mặc định 8cm")
#     parser.add_argument("--scale", type=float, default=None,
#                          help="Số pixel / 1cm. Nếu không cung cấp, dùng --calib để tự tính")
#     parser.add_argument("--calib", type=float, default=None,
#                          help="Khoảng cách thực tế (cm) giữa 2 điểm bạn sẽ click để calibrate")
#     parser.add_argument("--output", default="/mnt/user-data/outputs/trajectory_result.png",
#                          help="Đường dẫn file ảnh kết quả")
#     args = parser.parse_args()

#     # --- Bước 1: xác định scale (pixels/cm) ---
#     if args.scale is not None:
#         scale = args.scale
#     elif args.calib is not None:
#         scale = calibrate_scale(args.video, args.calib)
#     else:
#         print("Lỗi: cần cung cấp --scale (pixels/cm) hoặc --calib (khoảng cách calibrate, cm).")
#         sys.exit(1)

#     # --- Bước 2: track chấm đỏ trong video ---
#     actual_pts, origin = track_video(args.video, scale)

#     # --- Bước 3: dịch chuyển quỹ đạo về tâm hình học để so sánh công bằng ---
#     center_actual = actual_pts.mean(axis=0)

#     # --- Bước 4: tạo quỹ đạo chuẩn tại cùng tâm ---
#     if args.shape == "square":
#         ideal_pts = ideal_square(args.size, center=center_actual)
#     else:
#         ideal_pts = ideal_circle(args.size, center=center_actual)

#     # --- Bước 5: tính sai số ---
#     dists, stats = compute_error(actual_pts, ideal_pts)
#     print("Thống kê sai số (cm):", stats)

#     # --- Bước 6: vẽ và lưu ảnh ---
#     plot_result(actual_pts, ideal_pts, stats, args.shape, args.size, args.output)


# if __name__ == "__main__":
#     main()

"""
track_trajectory.py
--------------------
Theo dõi (track) chấm đỏ trong video, vẽ quỹ đạo thực tế lên lưới tọa độ (cm),
so sánh với quỹ đạo chuẩn (hình vuông cạnh 8cm hoặc hình tròn bán kính 8cm),
và tính sai số (error) giữa quỹ đạo thực tế và quỹ đạo lý tưởng.

Yêu cầu thư viện:
    pip install opencv-python numpy matplotlib scipy --break-system-packages

Cách dùng cơ bản:
    python track_trajectory.py --video path/to/video.mp4 --shape square --size 8 --scale 10

Trong đó:
    --shape   : "square" hoặc "circle"
    --size    : kích thước quỹ đạo chuẩn (cm) - cạnh hình vuông hoặc bán kính hình tròn
    --scale   : số pixel tương ứng với 1 cm (pixels/cm). Nếu không biết,
                dùng --calib để tự động tính từ 2 điểm bạn click trên khung hình đầu tiên.
"""

import cv2
import numpy as np
import matplotlib.pyplot as plt
import argparse
import sys
from scipy.spatial import cKDTree


# ----------------------------------------------------------------------
# 1. PHÁT HIỆN CHẤM ĐỎ TRONG TỪNG KHUNG HÌNH
# ----------------------------------------------------------------------
def detect_red_dot(frame):
    """Trả về tọa độ pixel (x, y) tâm chấm đỏ lớn nhất trong frame, hoặc None."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Màu đỏ nằm ở 2 đầu của dải Hue trong không gian HSV -> cần 2 mask
    lower_red1 = np.array([0, 100, 80])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 100, 80])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    # Lọc nhiễu
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    # Chọn contour lớn nhất (giả định là chấm đỏ cần theo dõi)
    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < 15:  # bỏ qua nhiễu quá nhỏ
        return None

    M = cv2.moments(c)
    if M["m00"] == 0:
        return None
    cx = M["m10"] / M["m00"]
    cy = M["m01"] / M["m00"]
    return (cx, cy)


# ----------------------------------------------------------------------
# 2. CALIBRATION: CHỌN THỦ CÔNG 2 ĐIỂM ĐỂ TÍNH TỈ LỆ PIXEL/CM
# ----------------------------------------------------------------------
def calibrate_scale(video_path, known_distance_cm):
    """
    Mở khung hình đầu tiên của video, cho người dùng click 2 điểm
    tương ứng với một khoảng cách đã biết (known_distance_cm),
    trả về scale = pixels / cm.
    """
    cap = cv2.VideoCapture(video_path)
    ret, frame = cap.read()
    cap.release()
    if not ret:
        raise RuntimeError("Không đọc được khung hình đầu tiên của video.")

    points = []

    def on_click(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(points) < 2:
            points.append((x, y))
            cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)
            cv2.imshow("Calibration - click 2 diem", frame)

    cv2.imshow("Calibration - click 2 diem", frame)
    cv2.setMouseCallback("Calibration - click 2 diem", on_click)
    print(f"Hãy click 2 điểm trên khung hình cách nhau {known_distance_cm} cm, "
          f"rồi nhấn phím bất kỳ để tiếp tục...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    if len(points) != 2:
        raise RuntimeError("Cần chọn đúng 2 điểm để calibrate.")

    (x1, y1), (x2, y2) = points
    pixel_dist = np.hypot(x2 - x1, y2 - y1)
    scale = pixel_dist / known_distance_cm  # pixels / cm
    print(f"Scale tính được: {scale:.3f} pixels/cm")
    return scale


# ----------------------------------------------------------------------
# 3. TRACK TOÀN BỘ VIDEO -> DANH SÁCH ĐIỂM (cm)
# ----------------------------------------------------------------------
def track_video(video_path, scale_px_per_cm, origin=None, skip_start=0, skip_end=0):
    """
    Duyệt toàn bộ video, phát hiện chấm đỏ ở mỗi khung hình,
    chuyển tọa độ pixel -> cm.
    origin: (ox, oy) pixel gốc tọa độ. Nếu None, dùng điểm đầu tiên phát hiện được làm gốc.
    skip_start / skip_end: số điểm (đã phát hiện được, không phải số khung hình gốc)
        sẽ bị cắt bỏ ở đầu / cuối danh sách quỹ đạo, dùng để loại bỏ đoạn
        robot "đi vào vị trí" trước khi vẽ hoặc đoạn nhiễu ở cuối.
    Trả về: mảng numpy Nx2 tọa độ (x_cm, y_cm), và (ox, oy) gốc tọa độ pixel đã dùng.
    """
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise RuntimeError(f"Không mở được video: {video_path}")

    traj_px = []
    frame_idx = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        pt = detect_red_dot(frame)
        if pt is not None:
            traj_px.append(pt)
        frame_idx += 1
    cap.release()

    if len(traj_px) == 0:
        raise RuntimeError("Không phát hiện được chấm đỏ trong toàn bộ video. "
                            "Hãy kiểm tra lại ngưỡng màu HSV.")

    traj_px = np.array(traj_px)
    print(f"Đã xử lý {frame_idx} khung hình, phát hiện chấm đỏ ở {len(traj_px)} khung hình.")

    # Cắt bỏ đoạn đầu/cuối (thường là đoạn tiếp cận vị trí bắt đầu hoặc nhiễu)
    if skip_start > 0 or skip_end > 0:
        end_idx = len(traj_px) - skip_end if skip_end > 0 else len(traj_px)
        traj_px = traj_px[skip_start:end_idx]
        print(f"Đã cắt bỏ {skip_start} điểm đầu và {skip_end} điểm cuối. "
              f"Còn lại {len(traj_px)} điểm để đánh giá.")
        if len(traj_px) == 0:
            raise RuntimeError("Cắt quá nhiều, không còn điểm nào để đánh giá.")

    if origin is None:
        ox, oy = traj_px[0]
    else:
        ox, oy = origin

    # Đổi trục y (ảnh có gốc tọa độ trên-trái, y hướng xuống) -> hệ tọa độ toán học (y hướng lên)
    traj_cm = np.zeros_like(traj_px)
    traj_cm[:, 0] = (traj_px[:, 0] - ox) / scale_px_per_cm
    traj_cm[:, 1] = -(traj_px[:, 1] - oy) / scale_px_per_cm

    return traj_cm, (ox, oy)


# ----------------------------------------------------------------------
# 4. TẠO QUỸ ĐẠO CHUẨN (LÝ TƯỞNG)
# ----------------------------------------------------------------------
def bbox_center(pts):
    """
    Tâm hình học dựa trên bounding box (trung điểm min/max theo X và Y).
    Ổn định hơn nhiều so với trung bình cộng khi quỹ đạo có lẫn đoạn nhiễu/tiếp cận,
    vì chỉ phụ thuộc vào 4 giá trị cực trị chứ không bị "kéo" bởi nhiều điểm lệch.
    """
    xmin, ymin = pts.min(axis=0)
    xmax, ymax = pts.max(axis=0)
    return np.array([(xmin + xmax) / 2, (ymin + ymax) / 2])


def ideal_square(side_cm, n_points_per_side=200, center=(0, 0)):
    """Tạo các điểm của hình vuông cạnh side_cm, tâm tại center, các cạnh song song trục."""
    h = side_cm
    cx, cy = center
    corners = np.array([
        [-h, -h], [h, -h], [h, h], [-h, h], [-h, -h]
    ]) + np.array([cx, cy])
    pts = []
    for i in range(4):
        p0, p1 = corners[i], corners[i + 1]
        t = np.linspace(0, 1, n_points_per_side, endpoint=False)[:, None]
        seg = p0 + t * (p1 - p0)
        pts.append(seg)
    return np.vstack(pts)


def ideal_circle(radius_cm, n_points=800, center=(0, 0)):
    """Tạo các điểm của hình tròn bán kính radius_cm, tâm tại center."""
    theta = np.linspace(0, 2 * np.pi, n_points, endpoint=False)
    cx, cy = center
    x = cx + radius_cm * np.cos(theta)
    y = cy + radius_cm * np.sin(theta)
    return np.column_stack([x, y])


# ----------------------------------------------------------------------
# 5. TÍNH SAI SỐ: KHOẢNG CÁCH TỪ MỖI ĐIỂM THỰC TẾ ĐẾN QUỸ ĐẠO CHUẨN GẦN NHẤT
# ----------------------------------------------------------------------
def compute_error(actual_pts, ideal_pts):
    """
    Với mỗi điểm thực tế, tìm khoảng cách ngắn nhất tới quỹ đạo chuẩn (tập điểm dày đặc).
    Trả về mảng sai số (cm) và các chỉ số thống kê.
    """
    tree = cKDTree(ideal_pts)
    dists, _ = tree.query(actual_pts)
    stats = {
        "mean_error_cm": float(np.mean(dists)),
        "max_error_cm": float(np.max(dists)),
        "rmse_cm": float(np.sqrt(np.mean(dists ** 2))),
        "std_error_cm": float(np.std(dists)),
    }
    return dists, stats


# ----------------------------------------------------------------------
# 6. VẼ KẾT QUẢ: QUỸ ĐẠO THỰC TẾ + CHUẨN + LƯỚI TỌA ĐỘ + SAI SỐ
# ----------------------------------------------------------------------
def plot_result(actual_pts, ideal_pts, stats, shape, size_cm, output_path):
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    # --- Bên trái: quỹ đạo trên lưới tọa độ ---
    ax = axes[0]
    margin = size_cm * 0.6
    ax.plot(ideal_pts[:, 0], ideal_pts[:, 1], "--", color="tab:blue",
             linewidth=1.5, label=f"Quỹ đạo chuẩn ({shape}, {size_cm}cm)")
    sc = ax.scatter(actual_pts[:, 0], actual_pts[:, 1], c=range(len(actual_pts)),
                     cmap="autumn", s=10, label="Quỹ đạo thực tế")
    ax.plot(actual_pts[:, 0], actual_pts[:, 1], color="tab:red", alpha=0.3, linewidth=1)

    ax.set_xlim(-size_cm/2 - margin, size_cm/2 + margin)
    ax.set_ylim(-size_cm/2 - margin, size_cm/2 + margin)
    ax.set_xticks(np.arange(-size_cm - margin, size_cm + margin + 1, 1))
    ax.set_yticks(np.arange(-size_cm - margin, size_cm + margin + 1, 1))
    ax.grid(True, linestyle=":", linewidth=0.6)
    ax.set_aspect("equal")
    ax.set_xlabel("X (cm)")
    ax.set_ylabel("Y (cm)")
    ax.set_title("Quỹ đạo thực tế vs Quỹ đạo chuẩn (lưới 1cm)")
    ax.legend(loc="upper right", fontsize=9)
    fig.colorbar(sc, ax=ax, label="Thứ tự khung hình (thời gian)")

    # --- Bên phải: biểu đồ sai số theo thời gian ---
    ax2 = axes[1]
    dists, _ = compute_error(actual_pts, ideal_pts)
    ax2.plot(dists, color="tab:red", linewidth=1)
    ax2.axhline(stats["mean_error_cm"], color="tab:blue", linestyle="--",
                label=f"Sai số trung bình = {stats['mean_error_cm']:.3f} cm")
    ax2.set_xlabel("Chỉ số điểm (theo thời gian)")
    ax2.set_ylabel("Sai số (cm)")
    ax2.set_title("Sai số quỹ đạo theo thời gian")
    ax2.legend(fontsize=9)
    ax2.grid(True, linestyle=":", linewidth=0.6)

    stats_text = (
        f"Sai số trung bình: {stats['mean_error_cm']:.3f} cm\n"
        f"Sai số RMSE: {stats['rmse_cm']:.3f} cm\n"
        f"Sai số lớn nhất: {stats['max_error_cm']:.3f} cm\n"
        f"Độ lệch chuẩn: {stats['std_error_cm']:.3f} cm"
    )
    fig.text(0.5, -0.02, stats_text, ha="center", fontsize=10,
              bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5))

    plt.tight_layout()
    plt.savefig(output_path, dpi=200, bbox_inches="tight")
    print(f"Đã lưu ảnh kết quả tại: {output_path}")
    plt.close(fig)


# ----------------------------------------------------------------------
# 7. MAIN
# ----------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Track quỹ đạo chấm đỏ và tính sai số so với quỹ đạo chuẩn.")
    parser.add_argument("--video", required=True, help="Đường dẫn tới file video")
    parser.add_argument("--shape", choices=["square", "circle"], required=True,
                         help="Hình dạng quỹ đạo chuẩn: square (hình vuông) hoặc circle (hình tròn)")
    parser.add_argument("--size", type=float, default=8.0,
                         help="Kích thước (cm): cạnh hình vuông hoặc bán kính hình tròn. Mặc định 8cm")
    parser.add_argument("--scale", type=float, default=None,
                         help="Số pixel / 1cm. Nếu không cung cấp, dùng --calib để tự tính")
    parser.add_argument("--calib", type=float, default=None,
                         help="Khoảng cách thực tế (cm) giữa 2 điểm bạn sẽ click để calibrate")
    parser.add_argument("--output", default="/mnt/user-data/outputs/trajectory_result.png",
                         help="Đường dẫn file ảnh kết quả")
    parser.add_argument("--skip-start", type=int, default=0,
                         help="Số điểm đầu quỹ đạo cần bỏ qua (đoạn robot đi vào vị trí bắt đầu)")
    parser.add_argument("--skip-end", type=int, default=0,
                         help="Số điểm cuối quỹ đạo cần bỏ qua (đoạn nhiễu/dừng lại)")
    args = parser.parse_args()

    # --- Bước 1: xác định scale (pixels/cm) ---
    if args.scale is not None:
        scale = args.scale
    elif args.calib is not None:
        scale = calibrate_scale(args.video, args.calib)
    else:
        print("Lỗi: cần cung cấp --scale (pixels/cm) hoặc --calib (khoảng cách calibrate, cm).")
        sys.exit(1)

    # --- Bước 2: track chấm đỏ trong video ---
    actual_pts, origin = track_video(args.video, scale,
                                      skip_start=args.skip_start, skip_end=args.skip_end)

    # --- Bước 3: dịch chuyển quỹ đạo về tâm hình học để so sánh công bằng ---
    # Dùng tâm bounding-box (min/max) thay vì trung bình cộng, vì trung bình cộng
    # dễ bị lệch nếu quỹ đạo còn sót đoạn nhiễu/tiếp cận chưa cắt hết.
    center_actual = bbox_center(actual_pts)

    xmin, ymin = actual_pts.min(axis=0)
    xmax, ymax = actual_pts.max(axis=0)
    print(f"Kích thước quỹ đạo thực tế đo được: "
          f"rộng {xmax - xmin:.2f}cm x cao {ymax - ymin:.2f}cm "
          f"(so sánh với --size {args.size}cm bạn đã đặt)")

    # --- Bước 4: tạo quỹ đạo chuẩn tại cùng tâm ---
    if args.shape == "square":
        ideal_pts = ideal_square(args.size, center=center_actual)
    else:
        ideal_pts = ideal_circle(args.size, center=center_actual)

    # --- Bước 5: tính sai số ---
    dists, stats = compute_error(actual_pts, ideal_pts)
    print("Thống kê sai số (cm):", stats)

    # --- Bước 6: vẽ và lưu ảnh ---
    plot_result(actual_pts, ideal_pts, stats, args.shape, args.size, args.output)


if __name__ == "__main__":
    main()