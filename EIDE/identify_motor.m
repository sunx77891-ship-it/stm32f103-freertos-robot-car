%% M 序列辨识 — 常规 ARX 最小二乘法（无需任何工具箱）
% 使用方法：
%   1. 将串口保存的 CSV 放至 csv/data3.csv
%   2. 运行此脚本
% 数据格式: sample,time_ms,ccr,speed_a,speed_b

clear; clc; close all;

%% 1. 加载数据
data = csvread('csv/data3.csv', 1, 0);
sample   = data(:,1);
time_ms  = data(:,2);
ccr      = data(:,3);
speed_a  = data(:,4);
speed_b  = data(:,5);

Ts = 0.01;

%% 2. 查看原始数据
figure('Name', '原始数据');
subplot(2,1,1);
plot(time_ms/1000, ccr, 'b-', 'LineWidth', 1);
ylabel('CCR'); xlabel('时间 (s)'); grid on;
title('M 序列 PRBS 信号');

subplot(2,1,2);
plot(time_ms/1000, speed_b, 'r-', 'LineWidth', 1); hold on;
plot(time_ms/1000, speed_a, 'g-', 'LineWidth', 1);
ylabel('编码器脉冲/10ms'); xlabel('时间 (s)'); grid on;
legend('电机 B', '电机 A');
title('电机转速响应');

%% 3. 预处理
u = ccr - mean(ccr);
y_b = speed_b - mean(speed_b);
y_a = speed_a - mean(speed_a);

win = 5;
u = movmean(u, win);
y_b = movmean(y_b, win);
y_a = movmean(y_a, win);

%% 4. 模型阶数扫描（常规 ARX）
best_fit_a = -inf; best_fit_b = -inf;
best_theta_a = []; best_theta_b = [];

for test_na = 1:4
    for test_nb = 1:4
        th_b = arx_ls(y_b, u, test_na, test_nb, 0);
        th_a = arx_ls(y_a, u, test_na, test_nb, 0);
        yh_b = arx_sim(u, th_b, test_na, test_nb, 0);
        yh_a = arx_sim(u, th_a, test_na, test_nb, 0);

        mid = round(length(y_b) / 2);
        f_b = 100 * (1 - norm(y_b(mid:end) - yh_b(mid:end)) / norm(y_b(mid:end) - mean(y_b(mid:end))));
        f_a = 100 * (1 - norm(y_a(mid:end) - yh_a(mid:end)) / norm(y_a(mid:end) - mean(y_a(mid:end))));

        if f_b > best_fit_b
            best_fit_b = f_b; best_theta_b = th_b;
            best_na_b = test_na; best_nb_b = test_nb;
        end
        if f_a > best_fit_a
            best_fit_a = f_a; best_theta_a = th_a;
            best_na_a = test_na; best_nb_a = test_nb;
        end
    end
end

fprintf('=== 电机 B ===\n');
fprintf('阶数: na=%d, nb=%d, 拟合度: %.1f%%\n', best_na_b, best_nb_b, best_fit_b);
fprintf('theta = [%+.6f, %+.6f, %+.6f, %+.6f]\n\n', best_theta_b);

fprintf('=== 电机 A ===\n');
fprintf('阶数: na=%d, nb=%d, 拟合度: %.1f%%\n', best_na_a, best_nb_a, best_fit_a);
fprintf('theta = [%+.6f, %+.6f, %+.6f, %+.6f]\n', best_theta_a);

%% 5. 模型验证
y_hat_b = arx_sim(u, best_theta_b, best_na_b, best_nb_b, 0);
y_hat_a = arx_sim(u, best_theta_a, best_na_a, best_nb_a, 0);

fit_b = 100 * (1 - norm(y_b - y_hat_b) / norm(y_b - mean(y_b)));
fit_a = 100 * (1 - norm(y_a - y_hat_a) / norm(y_a - mean(y_a)));

figure('Name', '模型验证');
subplot(2,1,1);
plot(time_ms/1000, y_b, 'b-', 'LineWidth', 1); hold on;
plot(time_ms/1000, y_hat_b, 'r--', 'LineWidth', 1);
ylabel('电机 B'); xlabel('时间 (s)'); grid on;
legend(sprintf('实测 (拟合度 = %.1f%%)', fit_b), '模型输出');

subplot(2,1,2);
plot(time_ms/1000, y_a, 'g-', 'LineWidth', 1); hold on;
plot(time_ms/1000, y_hat_a, 'r--', 'LineWidth', 1);
ylabel('电机 A'); xlabel('时间 (s)'); grid on;
legend(sprintf('实测 (拟合度 = %.1f%%)', fit_a), '模型输出');

%% 6. 伯德图
figure('Name', '伯德图');
[num_b, den_b] = arx2tf(best_theta_b, best_na_b, best_nb_b, 0);
[num_a, den_a] = arx2tf(best_theta_a, best_na_a, best_nb_a, 0);
w = logspace(-1, 2, 200);
[mag_b, phase_b] = bode_manual(num_b, den_b, Ts, w);
[mag_a, phase_a] = bode_manual(num_a, den_a, Ts, w);

subplot(2,1,1);
semilogx(w, 20*log10(mag_b), 'r-', 'LineWidth', 1); hold on;
semilogx(w, 20*log10(mag_a), 'g-', 'LineWidth', 1);
ylabel('幅值 (dB)'); grid on; title('伯德图');
legend('电机 B', '电机 A');

subplot(2,1,2);
semilogx(w, phase_b*180/pi, 'r-', 'LineWidth', 1); hold on;
semilogx(w, phase_a*180/pi, 'g-', 'LineWidth', 1);
ylabel('相位 (°)'); xlabel('频率 (rad/s)'); grid on;
legend('电机 B', '电机 A');

%% 7. 保存
save('identified_model.mat', 'best_theta_b', 'best_theta_a', 'fit_b', 'fit_a');

%% 辅助函数
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
