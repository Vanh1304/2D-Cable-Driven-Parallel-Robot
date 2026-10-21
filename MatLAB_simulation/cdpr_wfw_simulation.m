    %% ================================================================
%  CDPR STATIC WORKSPACE SIMULATION
%  Paper: "A Study of Tension Distribution for Control of Planar
%          Cable Driven Parallel Robot Using QP Algorithm"
%          Int. J. Mech. Eng. Rob. Res, Vol.11, No.7, July 2022
%
%  Config: Frame 300x300mm | Platform vuong canh sqrt(2) cm
%  Ket qua tuong duong: Fig.4, Fig.7, Fig.8 trong paper
%  Can: Optimization Toolbox (quadprog)
%% ================================================================

clear; clc; close all;

%% ============================================================
%%  PHAN 1: THONG SO HE THONG
%% ============================================================

% --- Khung (frame) ---
a = 150;                  % mm -- nua canh khung (300x300mm)

% --- Moving platform ---
% Hinh vuong canh = 3 cm = 3*10 mm
% => b (offset doc moi truc trong Eq.2) = canh/2 = sqrt(2)*10/2 = 5*sqrt(2)
b =  2 ;          % mm ~= 7.07 mm
% Khoang cach thuc tu tam P den diem gan cap = b*sqrt(2) = 10 mm

% --- Anchor points (goc toa do tai tam khung) -- Eq.(3) ---
D = [-a,-a;  % D1: goc duoi trai
     -a,+a;  % D2: goc tren trai
     +a,+a;  % D3: goc tren phai
     +a,-a]; % D4: goc duoi phai

fprintf('=== THONG SO ===\n');
fprintf('Frame: %dx%d mm (a=%d mm)\n', 2*a, 2*a, a);
fprintf('Platform: canh=%.2f mm, b=%.3f mm, |P->Ei|=%.2f mm\n', ...
         2*b, b, b*sqrt(2));

%% -----------------------------------------------------------
%%  GIAI THICH CHON TAU_MIN VA TAU_MAX
%% -----------------------------------------------------------
%
%  TAU_MIN = 1 N -- Ly do:
%    (1) Day cuoc Daiwa 0.28mm: luc dut ~35N, he so an toan 5 -> tau_max=7N
%    (2) HX711 + load cell 1kg: nguong nhieu 3-sigma ~= 0.15 N
%    (3) Ma sat 3 puly (mu=0.05, goc om pi/2):
%        He so truyen muc = e^(0.05*pi/2)^3 = 1.26 -> can them 26% bao dam
%    (4) Margin an toan chong chung cap: chon tau_min = 0.5 N
%
%  TAU_MAX -- Chon de TY LE giong paper (de WFW co hinh dang tuong tu):
%    Paper Fig.4(a): tau_max/tau_min = 200/50 = 4
%    Paper Fig.4(b): tau_max/tau_min = 300/50 = 6
%    => Case1: tau_max = 4 * 0.5 = 2.0 N
%    => Case2: tau_max = 6 * 0.5 = 3.0 N
%    (trong thuc te he cua ban: tau_max co the len den 5-8 N tuy tai trong)

TAU(1).min = 0.5;  TAU(1).max = 2.0;   % Case 1: ratio=4, nhu Fig.4(a)
TAU(2).min = 0.5;  TAU(2).max = 3.0;   % Case 2: ratio=6, nhu Fig.4(b)
fprintf('tau_min=%.1fN | Case1 tau_max=%.1fN(ratio=%.0f) | Case2 tau_max=%.1fN(ratio=%.0f)\n',...
    TAU(1).min, TAU(1).max, TAU(1).max/TAU(1).min, TAU(2).max, TAU(2).max/TAU(2).min);

%% ============================================================
%%  PHAN 2: TINH WFW -- tuong duong Fig.4 paper
%% ============================================================

fprintf('\n--- Tinh WFW (co the mat 1-2 phut)...\n');

res    = 3;          % mm -- do phan giai luoi
margin = b + 5;      % mm -- bo vien de tranh diem neo
xv = (-a+margin):res:(a-margin);
yv = (-a+margin):res:(a-margin);
[XX,YY] = meshgrid(xv, yv);
pts  = [XX(:), YY(:)];
Npts = size(pts,1);

WFW  = false(Npts, 2);
opts = optimoptions('quadprog','Display','off','Algorithm','interior-point-convex');

tic;
for idx = 1:Npts
    px = pts(idx,1);  py = pts(idx,2);
    [A, ok] = build_A(px, py, b, D);
    if ~ok, continue; end
    for c = 1:2
        WFW(idx,c) = qp_feasible(A, TAU(c).min, TAU(c).max, opts);
    end
end
fprintf('Hoan thanh: %d diem, %.1f giay\n', Npts, toc);

for c = 1:2
    n   = sum(WFW(:,c));
    pct = n/Npts*100;
    fprintf('Case%d (tau_max=%.1fN): %d/%d diem (%.1f%%), dien tich~%.1f cm2\n',...
        c, TAU(c).max, n, Npts, pct, n*res^2/100);
end

%% ============================================================
%%  PHAN 3: QUY DAO HINH CHU NHAT -- tuong duong Fig.7, Fig.8
%% ============================================================

fprintf('\n--- Tinh quy dao...\n');

rw = 60; rh = 40;           % mm -- kich thuoc hinh chu nhat
x0 = -rw/2; y0 = -rh/2;    % goc duoi trai
N_traj = 200;               % so diem tren quy dao
n4     = N_traj/4;
ts     = linspace(0,1,n4);

% 4 doan (theo chieu kim dong ho)
seg1 = [x0+rw*ts',  repmat(y0,     n4,1)];
seg2 = [repmat(x0+rw,n4,1), y0+rh*ts'];
seg3 = [x0+rw-rw*ts', repmat(y0+rh, n4,1)];
seg4 = [repmat(x0,n4,1), y0+rh-rh*ts'];
traj = [seg1; seg2; seg3; seg4];
Nt   = size(traj,1);

% Dung Case 1 cho quy dao
c_traj  = 1;
tau_min = TAU(c_traj).min;
tau_max = TAU(c_traj).max;
tau_bar = (tau_min + tau_max)/2;

L_traj   = zeros(Nt,4);   % chieu dai cap (IK)
tau_traj = zeros(Nt,4);   % suc cang toi uu (QP)

for k = 1:Nt
    px = traj(k,1); py = traj(k,2);

    % IK -- Eq.(1)
    E = get_E(px, py, b);
    for i = 1:4
        L_traj(k,i) = hypot(D(i,1)-E(i,1), D(i,2)-E(i,2));
    end

    % QP -- Eq.(15)(17)(18)
    [A, ok] = build_A(px, py, b, D);
    if ~ok, continue; end

    m = 4;
    H   = 2*eye(m);
    f   = -2*tau_bar*ones(m,1);
    Aeq = A;
    beq = zeros(3,1);   % w_p = 0 (khong co ngoai luc)
    lb  = tau_min*ones(m,1);
    ub  = tau_max*ones(m,1);

    [tau_opt,~,ef] = quadprog(H,f,[],[],Aeq,beq,lb,ub,[],opts);
    if ef == 1, tau_traj(k,:) = tau_opt'; end
end

%% ============================================================
%%  PHAN 4: VE KET QUA
%% ============================================================

figure('Position',[30,30,1400,900],'Color','w','Name','CDPR Simulation');

c_wfw  = {[0.12 0.35 0.78], [0.08 0.55 0.22]};
c_line = {[0.85 0.20 0.20], [0.05 0.46 0.85], ...
           [0.36 0.72 0.20], [0.94 0.68 0.05]};
case_title = {'(a) \tau_{min}=0.5N, \tau_{max}=2.0N', ...
              '(b) \tau_{min}=0.5N, \tau_{max}=3.0N'};

%% --- Fig.4(a) va (b): WFW ---
for c = 1:2
    subplot(2,3,c);
    in = pts(WFW(:,c),:);
    scatter(in(:,1), in(:,2), 7, c_wfw{c}, 'filled', 'MarkerFaceAlpha',0.65);
    hold on;
    rectangle('Position',[-a,-a,2*a,2*a],'EdgeColor',[0.2 0.2 0.2],...
              'LineWidth',2,'LineStyle','--');
    plot(D(:,1),D(:,2),'ks','MarkerSize',10,'MarkerFaceColor','k');
    for di = 1:4
        dx=[-14,-14,4,4]; dy=[-11,6,6,-11];
        text(D(di,1)+dx(di),D(di,2)+dy(di),sprintf('D_%d',di),'FontSize',9);
    end
    plot([-a-5,a+5],[0,0],'k-','LineWidth',0.5);
    plot([0,0],[-a-5,a+5],'k-','LineWidth',0.5);
    set(gca,'FontSize',10); axis equal; grid on; box on;
    xlim([-a-18,a+18]); ylim([-a-18,a+18]);
    xlabel('x (mm)','FontSize',11); ylabel('y (mm)','FontSize',11);
    title(case_title{c},'FontSize',10.5);
end

%% --- Overlay so sanh ---
subplot(2,3,3);
hold on;
for c = 1:2
    in = pts(WFW(:,c),:);
    scatter(in(:,1),in(:,2),4+2*(2-c),c_wfw{c},'filled',...
            'MarkerFaceAlpha',0.35+0.15*c,...
            'DisplayName',sprintf('Case %d (ratio=%d)',c,round(TAU(c).max/TAU(c).min)));
end
rectangle('Position',[-a,-a,2*a,2*a],'EdgeColor',[0.2 0.2 0.2],'LineWidth',2,'LineStyle','--');
plot(D(:,1),D(:,2),'ks','MarkerSize',10,'MarkerFaceColor','k');
plot([-a-5,a+5],[0,0],'k-','LineWidth',0.5);
plot([0,0],[-a-5,a+5],'k-','LineWidth',0.5);
legend('Location','NorthEast','FontSize',10);
set(gca,'FontSize',10); axis equal; grid on; box on;
xlim([-a-18,a+18]); ylim([-a-18,a+18]);
xlabel('x (mm)','FontSize',11); ylabel('y (mm)','FontSize',11);
title('(c) So sanh WFW -- WFW mo rong khi tang \tau_{max}','FontSize',10.5);

%% --- Fig.7: Quy dao ---
subplot(2,3,4);
in1 = pts(WFW(:,c_traj),:);
scatter(in1(:,1),in1(:,2),4,c_wfw{c_traj},'filled','MarkerFaceAlpha',0.2);
hold on;
plot(traj(:,1),traj(:,2),'r-','LineWidth',2.5,'DisplayName','Quy dao');
plot(traj(1,1),traj(1,2),'go','MarkerSize',10,'MarkerFaceColor','g','DisplayName','Xuat phat');
rectangle('Position',[-a,-a,2*a,2*a],'EdgeColor',[0.2 0.2 0.2],'LineWidth',2,'LineStyle','--');
plot(D(:,1),D(:,2),'ks','MarkerSize',10,'MarkerFaceColor','k','DisplayName','Anchor');
plot([-a-5,a+5],[0,0],'k-','LineWidth',0.5);
plot([0,0],[-a-5,a+5],'k-','LineWidth',0.5);
legend('Location','NorthEast','FontSize',9);
set(gca,'FontSize',10); axis equal; grid on; box on;
xlim([-a-18,a+18]); ylim([-a-18,a+18]);
xlabel('x (mm)','FontSize',11); ylabel('y (mm)','FontSize',11);
title('(d) Quy dao hinh chu nhat 60x40mm (cf. Fig.7 paper)','FontSize',10.5);

%% --- Fig.8 (tren): Chieu dai cap ---
subplot(2,3,5);
t_arr = 1:Nt;
hold on;
for i = 1:4
    plot(t_arr, L_traj(:,i), 'Color',c_line{i}, 'LineWidth',1.8,...
         'DisplayName',sprintf('L_%d',i));
end
grid on; box on;
legend('Location','NorthEast','FontSize',10);
xlabel('Buoc thoi gian','FontSize',11);
ylabel('Chieu dai cap (mm)','FontSize',11);
title('(e) Chieu dai cap L_i (cf. Fig.8 paper)','FontSize',10.5);
set(gca,'FontSize',10);

%% --- Fig.8 (duoi): Suc cang ---
subplot(2,3,6);
hold on;
for i = 1:4
    plot(t_arr, tau_traj(:,i), 'Color',c_line{i}, 'LineWidth',1.8,...
         'DisplayName',sprintf('\\tau_%d',i));
end
yline(tau_min,'k--','LineWidth',1.2,'DisplayName',sprintf('\\tau_{min}=%.1fN',tau_min));
yline(tau_max,'k:','LineWidth',1.2,'DisplayName',sprintf('\\tau_{max}=%.1fN',tau_max));
yline(tau_bar,'Color',[0.5 0.5 0.5],'LineWidth',0.8,...
      'DisplayName',sprintf('\\bar{\\tau}=%.2fN',tau_bar));
grid on; box on;
legend('Location','NorthEast','FontSize',9,'NumColumns',2);
xlabel('Buoc thoi gian','FontSize',11);
ylabel('Suc cang (N)','FontSize',11);
title(sprintf('(f) Phan phoi suc cang toi uu QP (cf. Fig.8 paper)'),'FontSize',10.5);
ylim([0, tau_max*1.35]);
set(gca,'FontSize',10);

sgtitle('CDPR Planar 300x300mm  |  Platform canh = sqrt(2) cm  |  tau_{min} = 0.5 N',...
        'FontSize',13,'FontWeight','bold');

%% ============================================================
%%  HAM PHU TRO
%% ============================================================

function E = get_E(px, py, b)
% Diem gan cap tren end-effector -- Eq.(2)
    E = [px-b, py-b;
         px-b, py+b;
         px+b, py+b;
         px+b, py-b];
end

function [A, ok] = build_A(px, py, b, D)
% Xay dung ma tran cau truc A (3x4) -- Eq.(9)(11)
    E  = get_E(px, py, b);
    A  = zeros(3,4);
    ok = true;
    for i = 1:4
        lx = D(i,1)-E(i,1);
        ly = D(i,2)-E(i,2);
        Li = hypot(lx, ly);
        if Li < 1e-6, ok = false; return; end

        ui = [lx; ly] / Li;           % Eq.(9)

        % Moment arm: si = (Ei-P) x ui -- Eq.(6)
        off = [E(i,1)-px; E(i,2)-py];
        si  = off(1)*ui(2) - off(2)*ui(1);

        A(1:2,i) = ui;
        A(3,  i) = si;
    end
end

function ok = qp_feasible(A, tau_min, tau_max, opts)
% Kiem tra: ton tai tau in [tau_min,tau_max]^4 s.t. A*tau=0
% => QP: min ||tau-tau_bar||^2  s.t. A*tau=0, bounds -- Eq.(15)(17)(18)
    m       = 4;
    tau_bar = (tau_min+tau_max)/2;
    H   = 2*eye(m);
    f   = -2*tau_bar*ones(m,1);
    Aeq = A;
    beq = zeros(3,1);
    lb  = tau_min*ones(m,1);
    ub  = tau_max*ones(m,1);
    try
        [~,~,ef] = quadprog(H,f,[],[],Aeq,beq,lb,ub,[],opts);
        ok = (ef == 1);
    catch
        ok = false;
    end
end
