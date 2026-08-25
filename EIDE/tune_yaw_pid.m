%% 航向级联 PID 整定 — 非线性时域仿真（含抗扰测试）
% 内环: WZ_PI (wzErr -> diffSpd)  外环: YAW_PID (yawErr -> wzRef)
% 包含: WZ_INT_LIM / WZ_OUT_MAX / YAW_INT_LIM / YAW_OUT_MAX 非线性
% 抗扰: t=8s 注入 30° 阶跃扰动
% STEP2 仍用线性 tf() 分析极点, PSO/验证改用非线性时域仿真
clear; clc; close all;

load('yaw_plant.mat');
ts = Ts;
na = best_na;  nb = best_nb;
a  = best_theta(1:na)';
b  = best_theta(na+1:end)';

fprintf('=== 航向模型 ===  G(z) = diffSpd -> wz,  拟合度 %.1f%%\n\n', fit_all);

% 限幅常量（匹配 Ctrl.c 固件）
WZ_INT_LIM  = 3599;
WZ_OUT_MAX  = 3599;
YAW_INT_LIM = 3599;
YAW_OUT_MAX = 3599;

%% ========================================================================
%  STEP 1: 内环 WZ PI PSO 优化
% ========================================================================
fprintf('=== WZ 内环 PI PSO 优化 ===\n');
nSim     = 1000;
wzTarget = 100;

wzObjFunc = @(kp, ki) wzScoreFunc(kp, ki, a, b, ts, nSim, wzTarget, ...
                                   WZ_INT_LIM, WZ_OUT_MAX);

optsWz.swarmSize = 30;
optsWz.maxIter   = 50;
optsWz.w         = 0.8;
optsWz.wDamp     = 0.98;
optsWz.c1        = 1.5;
optsWz.c2        = 2.0;
optsWz.display   = true;

tic;
wzResult = pso_pid(wzObjFunc, 2, [1, 1], [30, 20], optsWz);
bestWz.kp = wzResult.kp;
bestWz.ki = wzResult.ki;
[~, bestWz.os, bestWz.rise, bestWz.settle] = ...
    wzScoreFunc(bestWz.kp, bestWz.ki, a, b, ts, nSim, wzTarget, WZ_INT_LIM, WZ_OUT_MAX);
fprintf('\nPSO 搜索完成: %.1fs  最优: Kp=%.2f, Ki=%.2f  过冲=%.1f%%  上升=%.2fs  调节=%.2fs\n\n', ...
    toc, bestWz.kp, bestWz.ki, bestWz.os, bestWz.rise, bestWz.settle);

%% ========================================================================
%  STEP 2: 构建闭环传递函数
% ========================================================================
numC = [bestWz.kp + bestWz.ki*ts,  -bestWz.kp];
denC = [1, -1];
numG = [zeros(1, nb), b];
denG = [1, a];

numCG = conv(numC, numG);
denCG = conv(denC, denG);

len = max(length(denCG), length(numCG));
numCGpad = [zeros(1, len-length(numCG)), numCG];
denCGpad = [zeros(1, len-length(denCG)), denCG];
denCL = denCGpad + numCGpad;
numCL = numCG;

numInt = [0, ts];
denInt = [1, -1];
numP = conv(numCL, numInt);
denP = conv(denCL, denInt);

innerCls = tf(numCL, denCL, ts);
outerPln = tf(numP, denP, ts);

fprintf('=== 内环闭环建立 ===  极点: ');
fprintf('%.3f ', pole(innerCls)); fprintf('\n\n');

%% ========================================================================
%  STEP 3: 外环 YAW PID 扫参（含抗扰测试，非线性时域仿真）
%  跟踪: 时域仿真 30/90/180
%  抗扰: t=8s 注入 30° 阶跃扰动
%  包含: WZ_INT_LIM/WZ_OUT_MAX/YAW_INT_LIM/YAW_OUT_MAX
%  综合评分 = 跟踪分 + 抗扰分
% ========================================================================
fprintf('=== YAW 外环 PID 扫参（含抗扰测试） ===\n');

nYaw      = 2000;
distMag   = 30;
distStep  = 800;
scoreTargets = [30, 90, 180];

% 目标函数：给定 (kp, ki, kd) 返回综合评分（跟踪+抗扰）
yawObjFunc = @(kp, ki, kd) yawScoreFunc(kp, ki, kd, a, b, nb, bestWz, ts, ...
                                         nYaw, distMag, distStep, scoreTargets, ...
                                         WZ_INT_LIM, WZ_OUT_MAX, YAW_INT_LIM, YAW_OUT_MAX);

opts.swarmSize = 40;
opts.maxIter   = 80;
opts.w         = 0.9;      % 初始惯性大 → 全局探索
opts.wDamp     = 0.97;     % 缓慢衰减 → 后期收敛
opts.c1        = 1.8;      % 个体认知
opts.c2        = 1.8;      % 社会学习（平衡）
opts.display   = true;

tic;
bestYaw = pso_pid(yawObjFunc, 3, [0.1, 0, 0], [8, 0.5, 3], opts);
fprintf('\nPSO 搜索完成: %.1fs\n\n', toc);

%% ========================================================================
%  STEP 4: 验证并出图（含抗扰响应，非线性时域仿真）
% ========================================================================
fprintf('=== 多目标性能（含抗扰，非线性仿真） ===\n');
figure('Name', 'YAW 角环阶跃响应（含抗扰）');

for ti = 1:length(scoreTargets)
    target = scoreTargets(ti);
    yawBuf = zeros(nYaw,1); wzBuf = zeros(nYaw,1); dBuf = zeros(nYaw,1);
    wzRef = 0; wzInteg = 0; yawInteg = 0;
    tResp = (0:nYaw-1)*ts;

    for k = 5:nYaw
        % --- 航向角环 PID（50ms = 每5步） ---
        if mod(k-1, 5) == 0
            yawErr = target - yawBuf(k-1);
            if yawErr > 180; yawErr = yawErr - 360; end
            if yawErr < -180; yawErr = yawErr + 360; end

            yawInteg = yawInteg + yawErr * bestYaw.ki * 5 * ts;
            intY = bestYaw.ki * yawInteg;
            if bestYaw.ki > 0
                if intY > YAW_INT_LIM; yawInteg = YAW_INT_LIM / bestYaw.ki; end
                if intY < -YAW_INT_LIM; yawInteg = -YAW_INT_LIM / bestYaw.ki; end
            end

            wzRef = bestYaw.kp * yawErr + bestYaw.ki * yawInteg - bestYaw.kd * wzBuf(k-1);
            if wzRef > YAW_OUT_MAX; wzRef = YAW_OUT_MAX; end
            if wzRef < -YAW_OUT_MAX; wzRef = -YAW_OUT_MAX; end
        end

        % --- 角速度环 PI（10ms） ---
        wzErr = wzRef - wzBuf(k-1);
        wzInteg = wzInteg + wzErr * ts;
        intW = bestWz.ki * wzInteg;
        if bestWz.ki > 0
            if intW > WZ_INT_LIM; wzInteg = WZ_INT_LIM / bestWz.ki; end
            if intW < -WZ_INT_LIM; wzInteg = -WZ_INT_LIM / bestWz.ki; end
        end
        dOut = bestWz.kp * wzErr + bestWz.ki * wzInteg;
        if dOut > WZ_OUT_MAX; dOut = WZ_OUT_MAX; end
        if dOut < -WZ_OUT_MAX; dOut = -WZ_OUT_MAX; end
        dBuf(k) = dOut;

        % ARX 模型: diffSpd → wz
        wzBuf(k) = -a(1)*wzBuf(k-1) - a(2)*wzBuf(k-2) - a(3)*wzBuf(k-3) ...
                   + b(1)*dBuf(k-1)+b(2)*dBuf(k-2)+b(3)*dBuf(k-3)+b(4)*dBuf(k-4);
        if ~isfinite(wzBuf(k)) || abs(wzBuf(k)) > 1e4; wzBuf(k) = 0; end
        yawBuf(k) = yawBuf(k-1) + wzBuf(k) * ts;

        % 扰动注入
        if k == distStep; yawBuf(k) = yawBuf(k) + distMag; end
    end

    yFull = yawBuf;

    subplot(2,2,ti);
    plot(tResp, target*ones(size(tResp)), 'k--', 'LineWidth', 1); hold on;
    plot(tResp, yFull, 'b-', 'LineWidth', 1.5);
    xline(8, 'r--', '扰动'); hold off;
    xlabel('时间 (s)'); ylabel('yaw (°)'); grid on;
    title(sprintf('目标 %d° (含扰动)', target)); legend('目标', '响应', '扰动');

    osPct = max(0, max(yFull(1:500))-target)/target*100;
    r10 = find(yFull(1:500)>=target*0.1,1); r90 = find(yFull(1:500)>=target*0.9,1);
    rt = 99; if ~isempty(r10)&&~isempty(r90); rt=(r90-r10)*ts; end
    si = find(abs(yFull(1:500)-target)/target>0.05,1,'last'); st=0;
    if ~isempty(si); st=si*ts; end
    yA = yFull(distStep+30:min(distStep+700,length(yFull)));
    maxDev = max(abs(yA-target));
    recIdx = find(abs(yA-target)<target*0.05,1,'first');
    recT = 5; if ~isempty(recIdx); recT=recIdx*ts; end
    fprintf('目标=%3d°: 过冲=%.0f%% 上升=%.2fs  |  扰动后: 最大偏差=%4.1f° 恢复=%.2fs\n', ...
        target, osPct, rt, maxDev, recT);
end
sgtitle(sprintf('YAW角环 Kp=%.2f Ki=%.2f Kd=%.2f', bestYaw.kp, bestYaw.ki, bestYaw.kd));

%% 内环 WZ 阶跃响应图
figure('Name', 'WZ 率环阶跃响应');
wzV = zeros(nSim,1); dV = zeros(nSim,1); integV = 0;
for k = 5:nSim
    eV = wzTarget - wzV(k-1);
    integV = integV + eV*ts;
    intValV = bestWz.ki * integV;
    if bestWz.ki > 0
        if intValV > WZ_INT_LIM; integV = WZ_INT_LIM / bestWz.ki; end
        if intValV < -WZ_INT_LIM; integV = -WZ_INT_LIM / bestWz.ki; end
    end
    dV(k) = bestWz.kp*eV + bestWz.ki*integV;
    if dV(k) > WZ_OUT_MAX; dV(k) = WZ_OUT_MAX; end
    if dV(k) < -WZ_OUT_MAX; dV(k) = -WZ_OUT_MAX; end
    wzV(k) = -a(1)*wzV(k-1) - a(2)*wzV(k-2) - a(3)*wzV(k-3) ...
             + b(1)*dV(k-1)+b(2)*dV(k-2)+b(3)*dV(k-3)+b(4)*dV(k-4);
    if ~isfinite(wzV(k)) || abs(wzV(k))>1e4; wzV(k)=0; end
end
t = (0:nSim-1)*ts;
subplot(2,1,1);
plot(t, wzTarget*ones(nSim,1),'k--',t,wzV,'b-','LineWidth',1.5);
ylabel('wz (°/s)'); grid on; legend('目标','响应');
title(sprintf('WZ率环 Kp=%.2f Ki=%.2f', bestWz.kp, bestWz.ki));
subplot(2,1,2);
plot(t, dV, 'r-'); ylabel('diffSpd'); xlabel('时间 (s)'); grid on;

%% 输出固件参数
fprintf('\n========================================\n');
fprintf('=== 推荐固件参数（复制到 Ctrl.c） ===\n');
fprintf('========================================\n\n');
fprintf('/* ===== 角速度环 PI（内环 10ms） ===== */\n');
fprintf('#define WZ_KP        %.2ff\n', bestWz.kp);
fprintf('#define WZ_KI        %.2ff\n', bestWz.ki);
fprintf('#define WZ_INT_LIM   %.2ff\n',WZ_INT_LIM);
fprintf('#define WZ_OUT_MAX   %.2ff\n\n',WZ_OUT_MAX);
fprintf('/* ===== 航向角环 PID（外环 50ms） ===== */\n');
fprintf('#define YAW_KP       %.2ff\n', bestYaw.kp);
fprintf('#define YAW_KI       %.2ff\n', bestYaw.ki);
fprintf('#define YAW_KD       %.2ff\n', bestYaw.kd);
fprintf('#define YAW_INT_LIM  %.2ff\n',YAW_INT_LIM);
fprintf('#define YAW_OUT_MAX  %.2ff\n\n',YAW_OUT_MAX);

%% ========================================================================
%  辅助函数：YAW 综合评分（非线性时域仿真，跟踪 + 抗扰）
% ========================================================================
function score = yawScoreFunc(kp, ki, kd, a, b, nb, bestWz, ts, nYaw, distMag, distStep, targets, ...
                               wzIntLim, wzOutMax, yawIntLim, yawOutMax)
    score = 0;
    penalty = 0;
    if ki > 0.01; penalty = penalty + ki*5; end
    if kd > 0.01; penalty = penalty + kd*2; end

    for ti = 1:length(targets)
        target = targets(ti);

        yawBuf = zeros(nYaw, 1);
        wzBuf  = zeros(nYaw, 1);
        dBuf   = zeros(nYaw, 1);
        wzRef  = 0;
        wzInteg = 0;
        yawInteg = 0;

        for k = 5:nYaw
            % --- 航向角环 PID（50ms = 每5步 @10ms） ---
            if mod(k-1, 5) == 0
                yawErr = target - yawBuf(k-1);
                if yawErr > 180; yawErr = yawErr - 360; end
                if yawErr < -180; yawErr = yawErr + 360; end

                yawInteg = yawInteg + yawErr * ki * 5 * ts;
                intY = ki * yawInteg;
                if ki > 0
                    if intY > yawIntLim; yawInteg = yawIntLim / ki; end
                    if intY < -yawIntLim; yawInteg = -yawIntLim / ki; end
                end

                wzRef = kp * yawErr + ki * yawInteg - kd * wzBuf(k-1);
                if wzRef > yawOutMax; wzRef = yawOutMax; end
                if wzRef < -yawOutMax; wzRef = -yawOutMax; end
            end

            % --- 角速度环 PI（10ms） ---
            wzErr = wzRef - wzBuf(k-1);
            wzInteg = wzInteg + wzErr * ts;
            intW = bestWz.ki * wzInteg;
            if bestWz.ki > 0
                if intW > wzIntLim; wzInteg = wzIntLim / bestWz.ki; end
                if intW < -wzIntLim; wzInteg = -wzIntLim / bestWz.ki; end
            end
            dOut = bestWz.kp * wzErr + bestWz.ki * wzInteg;
            if dOut > wzOutMax; dOut = wzOutMax; end
            if dOut < -wzOutMax; dOut = -wzOutMax; end
            dBuf(k) = dOut;

            % ARX 模型: diffSpd → wz
            wzBuf(k) = -a(1)*wzBuf(k-1) - a(2)*wzBuf(k-2) - a(3)*wzBuf(k-3) ...
                       + b(1)*dBuf(k-1)+b(2)*dBuf(k-2)+b(3)*dBuf(k-3)+b(4)*dBuf(k-4);
            if ~isfinite(wzBuf(k)) || abs(wzBuf(k)) > 1e4; wzBuf(k) = 0; end
            yawBuf(k) = yawBuf(k-1) + wzBuf(k) * ts;

            % 扰动注入
            if k == distStep; yawBuf(k) = yawBuf(k) + distMag; end
        end

        if any(~isfinite(yawBuf)) || max(abs(yawBuf)) > 1e4
            score = score + 1e6; continue;
        end

        yT = yawBuf(1:500);
        osPct = max(0, max(yT)-target)/target*100;
        r10 = find(yT>=target*0.1,1); r90 = find(yT>=target*0.9,1);
        riseT = 99;
        if ~isempty(r10)&&~isempty(r90); riseT=(r90-r10)*ts; end
        if riseT > 2.0; riseT = 2.0 + (riseT-2.0)*10; end
        sIdx = find(abs(yT-target)/target>0.05,1,'last');
        setT = 0; if ~isempty(sIdx); setT=sIdx*ts; end

        yA  = yawBuf(distStep+30:min(distStep+700,length(yawBuf)));
        maxDev = max(abs(yA-target));
        recIdx = find(abs(yA-target)<target*0.05,1,'first');
        recT = 5; if ~isempty(recIdx); recT=recIdx*ts; end

        score = score + riseT*3 + setT*2 + osPct^2 + (maxDev/target)*30 + recT*2;
    end
    score = score + penalty;
end

%% ========================================================================
%  辅助函数：WZ 内环 PI 评分（非线性时域仿真）
% ========================================================================
function [score, os, rt, st] = wzScoreFunc(kp, ki, a, b, ts, nSim, target, wzIntLim, wzOutMax)
    wzS = zeros(nSim,1); dS = zeros(nSim,1); integ = 0;
    for k = 5:nSim
        e = target - wzS(k-1);
        integ = integ + e*ts;
        intVal = ki * integ;
        if ki > 0
            if intVal > wzIntLim; integ = wzIntLim / ki; end
            if intVal < -wzIntLim; integ = -wzIntLim / ki; end
        end
        dS(k) = kp*e + ki*integ;
        if dS(k) > wzOutMax; dS(k) = wzOutMax; end
        if dS(k) < -wzOutMax; dS(k) = -wzOutMax; end
        wzS(k) = -a(1)*wzS(k-1) - a(2)*wzS(k-2) - a(3)*wzS(k-3) ...
                 + b(1)*dS(k-1)+b(2)*dS(k-2)+b(3)*dS(k-3)+b(4)*dS(k-4);
        if ~isfinite(wzS(k)) || abs(wzS(k)) > 1e4; wzS(k) = 0; end
    end
    if max(abs(wzS(101:end))) > 1e3; score = 1e5; return; end
    yS = wzS(101:end);
    os = max(0, max(yS)-target)/target*100;
    r10 = find(yS>=target*0.1,1); r90 = find(yS>=target*0.9,1);
    rt = 99; if ~isempty(r10)&&~isempty(r90); rt=(r90-r10)*ts; end
    si = find(abs(yS-target)/target>0.05,1,'last'); st = 0;
    if ~isempty(si); st=si*ts; end
    score = rt*3 + st*2 + os*0.5;
end
