%% Inverse Kinematics of a Four-Cable-Driven Parallel Robot
% Paper: "Four-cable-driven parallel robot"
% XueJun Jin, Dae Ik Jun, Andreas Pott, et al., ICCAS 2013
%
% KEY EQUATION (4):
%   l_i = a_i - r + R*b_i
%
% where:
%   a_i   = anchor points on the rigid frame (proximal, world frame) [mm]
%   b_i   = attachment points on end-effector (distal, local frame) [mm]
%   r     = [x; y] position of end-effector center in world frame [mm]
%   theta = orientation of end-effector [rad]
%   R     = 2x2 rotation matrix
%   l_i   = cable vector; cable_length = norm(l_i)
%
% Simulation reproduces:
%   Fig. 8 - Circular path with radius = 100 mm
%   Fig. 9 - Cable length variation over time (3 revolutions in 30 s)

clear; clc; close all;

%% =========================================================
%  1. ROBOT PARAMETERS  (Table 1, Fig. 7)
%     Frame: 2000 mm x 2000 mm, origin at center
% =========================================================
frame  = 300;   % half frame size [mm]
b_size = sqrt(2)/2;     % half-size of end-effector attachment square [mm]

% Proximal anchor points on frame (world frame) [mm]
%   A1: bottom-right   A2: bottom-left
%   A3: top-left       A4: top-right
a = [ frame, -frame;   % A1
     -frame, -frame;   % A2
     -frame,  frame;   % A3
      frame,  frame]'; % A4   (2 x 4 matrix, each column = one anchor)

% Distal attachment points on end-effector (local frame) [mm]
b = [ b_size, -b_size;   % b1 (Cable 1)
     -b_size, -b_size;   % b2 (Cable 2)
     -b_size,  b_size;   % b3 (Cable 3)
      b_size,  b_size]'; % b4 (Cable 4)  (2 x 4 matrix)

%% =========================================================
%  2. TRAJECTORY  (Section 4.1, Fig. 8)
%     Circular path, radius = 100 mm, 3 revolutions in 30 s
% =========================================================
radius       = 100;    % [mm]
n_revolutions = 3;     % number of full circles
T_total      = 30;     % total time [s]
N_keypts     = 300;    % key-points for spline fitting

% Key-point angles and positions
t_kp  = linspace(0, T_total, N_keypts + 1);
t_kp  = t_kp(1:end-1);   % remove duplicate
phi   = linspace(0, n_revolutions * 2 * pi, N_keypts + 1);
phi   = phi(1:end-1);

x_kp  = radius * cos(phi);
y_kp  = radius * sin(phi);

% Cubic-spline interpolation (smooth path)
t_fine = linspace(0, T_total, 3000);
x_fine = spline(t_kp, x_kp, t_fine);
y_fine = spline(t_kp, y_kp, t_fine);

% Orientation: theta = 0 (no rotation in this simulation)
theta_fine = zeros(size(t_fine));

%% =========================================================
%  3. INVERSE KINEMATICS  — Equation (4)
%     For each pose (x, y, theta), compute cable lengths
% =========================================================
n_steps = length(t_fine);
L       = zeros(4, n_steps);    % cable lengths [mm]
L_vec   = zeros(2, 4, n_steps); % cable vectors (optional)

for k = 1 : n_steps
    r     = [x_fine(k); y_fine(k)];
    theta = theta_fine(k);

    % Rotation matrix  R(theta)
    R = [cos(theta), -sin(theta);
         sin(theta),  cos(theta)];

    for i = 1 : 4
        li_vec        = a(:,i) - r + R * b(:,i);   % Eq. (4)
        L(i, k)       = norm(li_vec);
        L_vec(:,i,k)  = li_vec;
    end
end

% Cable length deviation from mean (for display, matching paper Fig.9)
L_mean = mean(L, 2);        % mean length of each cable [mm]
L_dev  = L - L_mean;        % deviation [mm]

%% =========================================================
%  4. PLOT — Fig. 8: Circular path
% =========================================================
figure('Name', 'Fig.8 - Generated Circular Path', ...
       'Position', [80, 200, 430, 400]);

plot(x_fine, y_fine, 'r-', 'LineWidth', 2);
hold on;
plot(0, 0, 'k+', 'MarkerSize', 8, 'LineWidth', 1.5);   % origin
axis equal;
xlim([-120, 120]);  ylim([-120, 120]);
set(gca, 'XTick', -100:50:100, 'YTick', -100:50:100);
grid on;
xlabel('x [mm]');  ylabel('y [mm]');
title('Fig. 8 — Generated circular path with 100 mm of radius');

%% =========================================================
%  5. PLOT — Fig. 9: Cable length variation
% =========================================================
cable_colors = {'b','b','g','m'};
cable_names  = {'Cable 1','Cable 2','Cable 3','Cable 4'};

figure('Name', 'Fig.9 - Generated Cable Lengths', ...
       'Position', [120, 80, 700, 560]);

for i = 1 : 4
    subplot(2, 2, i);
    plot(t_fine, L_dev(i,:), cable_colors{i}, 'LineWidth', 0.9);
    title(cable_names{i}, 'FontSize', 10);
    xlim([0, 30]);
    ylim([-120, 120]);
    set(gca, 'YTick', -100:50:100);
    grid on;
    if i > 2
        xlabel('Time [s]');
    end
    if mod(i, 2) == 1
        ylabel('[mm]');
    end
end
sgtitle('Fig. 9 — Generated cable lengths for circular motion');

%% =========================================================
%  6. PRINT SUMMARY
% =========================================================
fprintf('\n========== Inverse Kinematics Summary ==========\n');
fprintf('Frame size     : 2000 x 2000 mm\n');
fprintf('Circular radius: %d mm\n', radius);
fprintf('Revolutions    : %d  (in %.0f s)\n', n_revolutions, T_total);
fprintf('-------------------------------------------------\n');
for i = 1 : 4
    fprintf('Cable %d: mean = %6.1f mm,  range = [%6.1f, %6.1f] mm\n', ...
        i, L_mean(i), min(L(i,:)), max(L(i,:)));
end
fprintf('=================================================\n');
