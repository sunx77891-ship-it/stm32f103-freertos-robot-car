%% 航向辨识 — diff_spd → wz 的 ARX 模型（无需工具箱）
% =========================================================================
% 使用方法：
%   1. STM32 烧录 YAW_IDENTIFY_MODE=1 固件
%   2. 上电后会自动跑 PRBS 约 102 秒，USART1(115200) 输出 CSV
%   3. 串口助手保存为 csv/yaw_data.csv
%   4. 改 YAW_IDENTIFY_MODE=0 重新烧录恢复正常控制
%   5. 运行此脚本，输出 yaw_plant.mat
%
% 数据格式: sample,time_ms,diff_spd,wz,yaw
% 辨识对象: diff_spd(脉冲/10ms) → wz(°/s)
% 由 wz 积分得 yaw，所以 diff_spd → yaw = (diff_spd → wz) × (1/s)
% =========================================================================

clear; clc; close all;

%% 1. 加载数据
data = csvread('csv/yawdata3.csv', 1, 0);
sample  = data(:,1);
time_ms = data(:,2);
diff_spd = data(:,3);   % 输入 u
wz      = data(:,5);    % 输出 y (辨识对象)
yaw     = data(:,4);    % 用于后续验证

Ts = 0.01;   % 10ms 采样

fprintf('数据点数: %d, 时长: %.1f s\n', length(sample), time_ms(end)/1000);

%% 2. 原始数据查看
figure('Name', '原始数据');
subplot(3,1,1);
plot(time_ms/1000, diff_spd, 'b-', 'LineWidth', 1);
ylabel('diff\_spd (脉冲/10ms)'); xlabel('时间 (s)'); grid on;
title('PRBS 输入');

subplot(3,1,2);
plot(time_ms/1000, wz, 'r-', 'LineWidth', 1);
ylabel('wz (°/s)'); xlabel('时间 (s)'); grid on;
title('IMU 实测角速度');

subplot(3,1,3);
plot(time_ms/1000, yaw, 'g-', 'LineWidth', 1);
ylabel('yaw (°)'); xlabel('时间 (s)'); grid on;
title('IMU 实测航向角');

%% 3. 预处理（去均值 + 滑动平均）
u  = diff_spd - mean(diff_spd);
y  = wz      - mean(wz);

win = 5;   % 5 点滑动平均
u  = movmean(u, win);
y  = movmean(y, win);

%% 4. 阶数扫描（常规 ARX）
best_fit   = -inf;
best_theta = [];
best_na = 0; best_nb = 0;

fprintf('\n=== ARX 阶数扫描 ===\n');
fprintf('na\\nb');
for test_nb = 1:4
    fprintf('       nb=%d', test_nb);
end
fprintf('\n');

for test_na = 1:3
    fprintf('na=%d ', test_na);
    for test_nb = 1:4
        th = arx_ls(y, u, test_na, test_nb, 0);
        yh = arx_sim(u, th, test_na, test_nb, 0);

        mid = round(length(y) / 2);
        f = 100 * (1 - norm(y(mid:end) - yh(mid:end)) / norm(y(mid:end) - mean(y(mid:end))));

        fprintf('  %5.1f%%  ', f);

        if f > best_fit
            best_fit = f; best_theta = th;
            best_na = test_na; best_nb = test_nb;
        end
    end
    fprintf('\n');
end

fprintf('\n=== 最优模型 ===\n');
fprintf('阶数: na=%d, nb=%d, 拟合度: %.1f%%\n', best_na, best_nb, best_fit);
fprintf('theta = ');
fprintf('%+.6f ', best_theta);
fprintf('\n');

%% 5. 模型验证
y_hat = arx_sim(u, best_theta, best_na, best_nb, 0);
fit_all = 100 * (1 - norm(y - y_hat) / norm(y - mean(y)));

figure('Name', '模型验证 (wz)');
plot(time_ms/1000, y, 'b-', 'LineWidth', 1); hold on;
plot(time_ms/1000, y_hat, 'r--', 'LineWidth', 1);
ylabel('wz (°/s)'); xlabel('时间 (s)'); grid on;
legend(sprintf('实测 (全段拟合度 = %.1f%%)', fit_all), 'ARX 模型输出');
title('diff\_spd → wz 模型验证');

%% 6. 伯德图（diff_spd → wz）
[num_wz, den_wz] = arx2tf(best_theta, best_na, best_nb, 0);
w = logspace(-1, 2.5, 300);
[mag_wz, phase_wz] = bode_manual(num_wz, den_wz, Ts, w);

% diff_spd → yaw = (diff_spd → wz) × (1/s)
% 离散积分: y(k) = y(k-1) + Ts*u(k), 传递函数 1/(1 - z^-1) × Ts
% 串联到 wz 模型后 = num_wz(z) / [den_wz(z) × (1 - z^-1) / Ts]
num_yaw = num_wz * Ts;
den_yaw = conv(den_wz, [1, -1]);
[mag_yaw, phase_yaw] = bode_manual(num_yaw, den_yaw, Ts, w);

figure('Name', '伯德图');
subplot(2,1,1);
semilogx(w, 20*log10(mag_wz), 'r-', 'LineWidth', 1); hold on;
semilogx(w, 20*log10(mag_yaw), 'g-', 'LineWidth', 1);
ylabel('幅值 (dB)'); grid on; title('伯德图');
legend('|wz/diff\_spd|', '|yaw/diff\_spd|');

subplot(2,1,2);
semilogx(w, phase_wz*180/pi, 'r-', 'LineWidth', 1); hold on;
semilogx(w, phase_yaw*180/pi, 'g-', 'LineWidth', 1);
ylabel('相位 (°)'); xlabel('频率 (rad/s)'); grid on;
legend('wz', 'yaw');

%% 7. 保存模型
% 同时保存积分器形式以便仿真使用
save('yaw_plant.mat', ...
     'best_theta', 'best_na', 'best_nb', ...
     'num_wz','den_wz', 'num_yaw','den_yaw', ...
     'fit_all', 'best_fit', 'Ts');

fprintf('\n已保存 yaw_plant.mat\n');
fprintf('  - best_theta (ARX 参数向量)\n');
fprintf('  - num_wz/den_wz (diff_spd→wz 传递函数分子分母)\n');
fprintf('  - num_yaw/den_yaw (diff_spd→yaw 传递函数分子分母)\n');
fprintf('  - Ts (采样周期 0.01s)\n');

%% ========== 辅助函数（与 identify_motor.m 一致） ==========
function theta = arx_ls(y, u, na, nb, nk)
    N = length(y);
    max_lag = max(na, nb + nk);
    Phi = zeros(N - max_lag, na + nb);
    for i = 1:na
        Phi(:, i) = -y(max_lag - i + 1 : N - i);
    end
    for j = 1:nb
        Phi(:, na + j) = u(max_lag - nk - j + 1 : N - nk - j);
    end
    Y = y(max_lag + 1 : N);
    theta = Phi \ Y;
end

function y_hat = arx_sim(u, theta, na, nb, nk)
    N = length(u);
    y_hat = zeros(N, 1);
    for k = max(na, nb + nk) + 1 : N
        y_hat(k) = -sum(theta(1:na) .* y_hat(k - (1:na))) ...
                    + sum(theta(na+1:end) .* u(k - ((1:nb) + nk)));
    end
end

function [num, den] = arx2tf(theta, na, nb, nk)
    den = [1, theta(1:na)'];
    num = [zeros(1, nk), theta(na+1:na+nb)'];
end

function [mag, phase] = bode_manual(num, den, Ts, w)
    z = exp(1j * w * Ts);
    H = zeros(size(w));
    for i = 1:length(w)
        num_z = sum(num .* (z(i).^(-(0:length(num)-1))));
        den_z = sum(den .* (z(i).^(-(0:length(den)-1))));
        H(i) = num_z / den_z;
    end
    mag = abs(H);
    phase = angle(H);
end
