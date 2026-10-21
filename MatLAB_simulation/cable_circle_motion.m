clc; clear; close all;

% ==== 1. Kích thước khung ====
L = 0.4; % chiều dài cạnh khung vuông
halfL = L/2;

% ==== 2. Tọa độ các góc cố định (motor positions) ====
A1 = [-halfL, -halfL];
A2 = [ halfL, -halfL];
A3 = [ halfL,  halfL];
A4 = [-halfL,  halfL];
A = [A1; A2; A3; A4];

% ==== 3. Quỹ đạo tròn của end-effector ====
r_traj = 0.15;        % bán kính quỹ đạo
center = [0, 0];      % tâm quỹ đạo
t = linspace(0, 2*pi, 200); % góc quỹ đạo
x_traj = center(1) + r_traj*cos(t);
y_traj = center(2) + r_traj*sin(t);

% ==== 4. Thiết lập đồ thị ====
figure;
hold on; axis equal;
xlabel('X'); ylabel('Y');
title('Mô phỏng chuyển động tròn của Cable Robot');
axis([-0.5 0.5 -0.5 0.5]);
grid on;

% Vẽ khung cố định
plot([-halfL halfL halfL -halfL -halfL], ...
     [-halfL -halfL halfL halfL -halfL], 'b', 'LineWidth', 2);

% Vẽ vùng làm việc
r_workspace = 0.2;
theta = linspace(0, 2*pi, 200);
plot(r_workspace*cos(theta), r_workspace*sin(theta), 'm--', 'LineWidth', 1.5);

% ==== 5. Tạo các đối tượng đồ họa để cập nhật ====
hCables = gobjects(4,1);
for i = 1:4
    hCables(i) = plot([A(i,1) 0], [A(i,2) 0], 'g', 'LineWidth', 2);
end
hEE = plot(0, 0, 'r.', 'MarkerSize', 20); % End-effector (điểm đỏ)
hPath = plot(x_traj, y_traj, 'k:', 'LineWidth', 1); % Quỹ đạo

% ==== 6. Mô phỏng chuyển động ====
for k = 1:length(t)
    % Cập nhật vị trí end-effector
    Px = x_traj(k);
    Py = y_traj(k);

    % Cập nhật từng dây cáp
    for i = 1:4
        set(hCables(i), 'XData', [A(i,1) Px], 'YData', [A(i,2) Py]);
    end

    % Cập nhật vị trí end-effector
    set(hEE, 'XData', Px, 'YData', Py);

    drawnow;
end
