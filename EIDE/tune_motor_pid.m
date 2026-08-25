%% 电机级联 PID 整定 — 闭环传递函数法
% 内环: 速度 PI (speedErr -> CCR)  外环: 位置 PID (posErr -> speedRef)
% 含抗扰测试: 灵敏度函数 S=1/(1+CP), 8s 注入 200 脉冲扰动
clear; clc; close all;

load('identified_model.mat');

ts = 0.01;
na = 3; nb = 2;
a  = -best_theta_b(1:na)';
b  = best_theta_b(na+1:end)';
fullSpd  = 96;
maxCcr   = 3599;
deadZone = 800;
ccrMean  = 1800;
spdMean  = 47;

fprintf('=== 电机模型 ===  G(z) = CCR -> speed,  拟合度 %.1f%%\n\n', fit_b);
fprintf('满速: %d 脉冲/10ms (CCR=%d), 死区: %d CCR\n', fullSpd, maxCcr, deadZone);

%% 阶跃响应确认
figure('Name', '电机模型确认');
subplot(2,1,1);
impulseResp = filter(b, [1 a], [1; zeros(99,1)]);
stem(0:99, impulseResp); xlabel('k (10ms)'); ylabel('speed (脉冲/10ms)');
title('脉冲响应 (CCR=1 -> speed)'); grid on;

subplot(2,1,2);
stepResp = filter(b, [1 a], ones(200,1));
plot(0:199, stepResp, 'b-', 'LineWidth', 1.5);
xlabel('k (10ms)'); ylabel('speed (脉冲/10ms)');
title(sprintf('阶跃响应 (CCR=1 -> speed). DC增益=%.4f', sum(b)/(1+sum(a)))); grid on;

%% ========================================================================
%  STEP 1: 速度环 PI 扫参（穷举扫描，2参数比PSO更可靠）
% ========================================================================
fprintf('=== 速度环 PI 扫参 ===\n');

nSim      = 2000;
spdTarget = 65;
spdKpList = 5:5:60;     % 12 个
spdKiList = 5:5:60;     % 12 个 → 144 组合

bestSpd = struct('kp',0,'ki',0,'os',0,'rise',99,'settle',99);
bestSpdScore = inf;

for kp = spdKpList
    for ki = spdKiList
        [score, os, rt, st] = spdScoreFunc(kp, ki, a, b, ts, nSim, spdTarget, ...
                                             fullSpd, maxCcr, deadZone, ccrMean, spdMean);
        if isfinite(score) && score < bestSpdScore
            bestSpdScore = score;
            bestSpd.kp = kp; bestSpd.ki = ki;
            bestSpd.os = os; bestSpd.rise = rt; bestSpd.settle = st;
        end
    end
end
fprintf('最优: Kp=%d, Ki=%d  过冲=%.1f%%  上升=%.2fs  调节=%.2fs\n\n', ...
    bestSpd.kp, bestSpd.ki, bestSpd.os, bestSpd.rise, bestSpd.settle);

%% ========================================================================
%  STEP 2: 构建速度环闭环 + 位置被控对象
% ========================================================================
numC = [bestSpd.kp + bestSpd.ki*ts,  -bestSpd.kp];
denC = [1, -1];
numG = [zeros(1, nb), b];
denG = [1, -a(1), -a(2), -a(3)];

numCG = conv(numC, numG);
denCG = conv(denC, denG);

len = max(length(denCG), length(numCG));
numPad = [zeros(1, len-length(numCG)), numCG];
denPad = [zeros(1, len-length(denCG)), denCG];
denCL = denPad + numPad;
numCL = numCG;

numInt = [0, ts];
denInt = [1, -1];
numP = conv(numCL, numInt);
denP = conv(denCL, denInt);

spdCls = tf(numCL, denCL, ts);
posPln = tf(numP, denP, ts);

fprintf('=== 速度环闭环建立 ===  极点: ');
fprintf('%.3f ', pole(spdCls)); fprintf('\n\n');

%% ========================================================================
%  STEP 3: 位置环 PID 扫参（含抗扰测试）
%  跟踪: step() 仿真 1000/2000 脉冲
%  抗扰: 灵敏度 S=1/(1+CP)，t=8s 注入 200 脉冲扰动
%  综合评分 = 跟踪分(上升+过冲+调节) + 抗扰分(最大偏差+恢复时间)
% ========================================================================
fprintf('=== 位置环 PID 扫参（含抗扰测试） ===\n');

posKpList = 0.1:0.2:3.0;
nPos      = 2500;
distMag   = 200;
distStep  = 800;

posObjFunc = @(kp, ki, kd) posScoreFunc(kp, ki, kd, posPln, ts, ...
                                         nPos, distMag, distStep, [1000, 2000]);

opts.swarmSize = 40;
opts.maxIter   = 80;
opts.w         = 0.9;
opts.wDamp     = 0.97;
opts.c1        = 1.8;
opts.c2        = 1.8;
opts.display   = true;

tic;
bestPos = pso_pid(posObjFunc, 3, [0.05, 0, 0], [3, 0.2, 2], opts);
fprintf('\nPSO 搜索完成: %.1fs\n\n', toc);

%% ========================================================================
%  STEP 4: 验证并出图（含抗扰响应）
% ========================================================================
pNum = [bestPos.kp + bestPos.ki*ts + bestPos.kd/ts,  -(bestPos.kp + 2*bestPos.kd/ts),  bestPos.kd/ts];
pDen = [1, -1, 0];
Cp = tf(pNum, pDen, ts);
clSys = feedback(Cp * posPln, 1);
Sens  = 1 / (1 + Cp * posPln);

fprintf('=== 多目标性能（含抗扰） ===\n');
figure('Name', '位置环阶跃响应（含抗扰）');
targets = [1000, 2000];

for ti = 1:length(targets)
    target = targets(ti);
    [yResp, tResp] = step(clSys * target, (0:nPos-1)*ts);
    yResp = yResp(:);

    [yDist, ~] = step(Sens * distMag, (0:nPos-distStep-1)*ts);
    yDist = yDist(:);
    yFull = yResp;
    nL = min(length(yDist), length(yFull)-distStep);
    yFull(distStep+1:distStep+nL) = yResp(distStep+1:distStep+nL) + yDist(1:nL);

    subplot(2,2,ti);
    plot(tResp, target*ones(size(tResp)), 'k--', 'LineWidth', 1); hold on;
    plot(tResp, yFull, 'b-', 'LineWidth', 1.5);
    xline(8, 'r--', '扰动'); hold off;
    xlabel('时间 (s)'); ylabel('位置 (脉冲)'); grid on;
    title(sprintf('目标 %d (含扰动)', target)); legend('目标', '响应', '扰动');

    osPct = max(0, max(yFull(1:500))-target)/target*100;
    r10 = find(yFull(1:500)>=target*0.1,1); r90 = find(yFull(1:500)>=target*0.9,1);
    rt = 99; if ~isempty(r10)&&~isempty(r90); rt=(r90-r10)*ts; end
    si = find(abs(yFull(1:500)-target)/target>0.05,1,'last'); st=0;
    if ~isempty(si); st=si*ts; end
    yA = yFull(distStep+50:min(distStep+1200,length(yFull)));
    maxDev = max(abs(yA-target));
    recIdx = find(abs(yA-target)<target*0.05,1,'first');
    recT = 5; if ~isempty(recIdx); recT=recIdx*ts; end
    fprintf('目标=%4d: 过冲=%.0f%% 上升=%.2fs  |  扰动后: 最大偏差=%4.0f  恢复=%.2fs\n', ...
        target, osPct, rt, maxDev, recT);
end

sgtitle(sprintf('位置环 Kp=%.2f Ki=%.2f Kd=%.2f', bestPos.kp, bestPos.ki, bestPos.kd));

%% 速度环阶跃响应图
figure('Name', '速度环阶跃响应');
spdV = zeros(nSim,1); ccrV = zeros(nSim,1); integV = 0;
spdIntLim = 2800;
for k = 5:nSim
    eV = spdTarget - spdV(k-1);
    integV = integV + eV*ts;
    intVal = bestSpd.ki * integV;
    if bestSpd.ki > 0
        if intVal > spdIntLim; integV = spdIntLim / bestSpd.ki; end
        if intVal < -spdIntLim; integV = -spdIntLim / bestSpd.ki; end
    end
    ccrV(k) = bestSpd.kp*eV + bestSpd.ki*integV;
    if ccrV(k) > maxCcr; ccrV(k)=maxCcr; end
    if ccrV(k) < 0; ccrV(k)=0; end
    if ccrV(k) > 0 && ccrV(k) < deadZone; ccrV(k)=deadZone; end
    yNorm = a(1)*(spdV(k-1)-spdMean)+a(2)*(spdV(k-2)-spdMean)+a(3)*(spdV(k-3)-spdMean) ...
            + b(1)*(ccrV(k-1)-ccrMean)+b(2)*(ccrV(k-2)-ccrMean);
    spdV(k) = max(0, min(fullSpd, yNorm+spdMean));
end
t = (0:nSim-1)*ts;
subplot(2,1,1);
plot(t, spdTarget*ones(nSim,1),'k--',t,spdV,'b-','LineWidth',1.5);
ylabel('speed (脉冲/10ms)'); grid on;
title(sprintf('速度环 Kp=%d Ki=%d', bestSpd.kp, bestSpd.ki));
legend('目标','响应');
subplot(2,1,2);
plot(t, ccrV, 'r-'); ylabel('CCR'); xlabel('时间 (s)'); grid on;


%% 输出固件参数
fprintf('\n========================================\n');
fprintf('=== 推荐固件参数（复制到 Ctrl.c） ===\n');
fprintf('========================================\n\n');
fprintf('/* ===== 速度环 PI（内环 10ms） ===== */\n');
fprintf('#define SPD_KP       %.0f\n', bestSpd.kp);
fprintf('#define SPD_KI       %.0f\n', bestSpd.ki);
fprintf('#define SPD_TS       0.01f\n');
fprintf('#define SPD_INT_LIM  %.2ff\n',spdIntLim);
fprintf('#define DEAD_ZONE_MIN %d\n\n', deadZone);
fprintf('/* ===== 位置环 PID（外环 50ms） ===== */\n');
fprintf('#define POS_KP       %.2ff\n', bestPos.kp);
fprintf('#define POS_KI       %.2ff\n', bestPos.ki);
fprintf('#define POS_KD       %.2ff\n', bestPos.kd);
fprintf('#define POS_INT_LIM  30.0f\n');
fprintf('#define POS_SPD_MAX  60\n\n');

%% ========================================================================
%  辅助函数：位置环综合评分（跟踪 + 抗扰）
% ========================================================================
function score = posScoreFunc(kp, ki, kd, posPln, ts, nPos, distMag, distStep, targets)
    score = 0;
    penalty = 0;
    if ki > 0.01; penalty = penalty + ki*10; end
    for ti = 1:length(targets)
        target = targets(ti);
        cNum = [kp + ki*ts + kd/ts,  -(kp + 2*kd/ts),  kd/ts];
        cDen = [1, -1, 0];
        Cp = tf(cNum, cDen, ts);
        try
            clSys = feedback(Cp * posPln, 1);
            [yResp, ~] = step(clSys * target, (0:nPos-1)*ts);
            yResp = yResp(:);
            S = 1 / (1 + Cp * posPln);
            [yDist, ~] = step(S * distMag, (0:nPos-distStep-1)*ts);
            yDist = yDist(:);
            yFull = yResp;
            nL = min(length(yDist), length(yFull)-distStep);
            yFull(distStep+1:distStep+nL) = yResp(distStep+1:distStep+nL) + yDist(1:nL);
        catch
            score = score + 1e6; continue;
        end
        if any(~isfinite(yFull)) || max(abs(yFull)) > 1e6
            score = score + 1e5; continue;
        end
        yT = yFull(1:500);
        os = max(0, max(yT)-target)/target*100;
        r10 = find(yT>=target*0.1,1); r90 = find(yT>=target*0.9,1);
        rt = 99; if ~isempty(r10)&&~isempty(r90); rt=(r90-r10)*ts; end
        if rt > 3.0; rt = 3.0 + (rt-3.0)*10; end
        si = find(abs(yT-target)/target>0.05,1,'last');
        st = 0; if ~isempty(si); st=si*ts; end
        yA = yFull(distStep+50:min(distStep+1200,length(yFull)));
        maxDev = max(abs(yA-target));
        recIdx = find(abs(yA-target)<target*0.05,1,'first');
        recT = 5; if ~isempty(recIdx); recT=recIdx*ts; end
        score = score + rt*3 + st*2 + os^1.5 + (maxDev/target)*20 + recT*2;
    end
    score = score + penalty;
end

%% ========================================================================
%  辅助函数：速度环评分（时域 ARX 仿真，含积分限幅）
% ========================================================================
function [score, os, rt, st] = spdScoreFunc(kp, ki, a, b, ts, nSim, target, ...
                                              fullSpd, maxCcr, deadZone, ccrMean, spdMean)
    spdIntLim = 2800;
    spdS = zeros(nSim,1); ccrS = zeros(nSim,1); integ = 0;
    for k = 5:nSim
        e = target - spdS(k-1);
        integ = integ + e * ts;
        intVal = ki * integ;
        if ki > 0
            if intVal > spdIntLim; integ = spdIntLim / ki; end
            if intVal < -spdIntLim; integ = -spdIntLim / ki; end
        end
        ccrS(k) = kp*e + ki*integ;
        if ccrS(k) > maxCcr; ccrS(k) = maxCcr; end
        if ccrS(k) < 0; ccrS(k) = 0; end
        if ccrS(k) > 0 && ccrS(k) < deadZone; ccrS(k) = deadZone; end
        yNorm = a(1)*(spdS(k-1)-spdMean) + a(2)*(spdS(k-2)-spdMean) ...
              + a(3)*(spdS(k-3)-spdMean) ...
              + b(1)*(ccrS(k-1)-ccrMean) + b(2)*(ccrS(k-2)-ccrMean);
        spdS(k) = max(0, min(fullSpd, yNorm + spdMean));
    end
    if max(abs(spdS(101:end))) > 200; score = 1e5; return; end
    yS = spdS(101:end);
    os = max(0, max(yS)-target)/target*100;
    r10 = find(yS>=target*0.1,1); r90 = find(yS>=target*0.9,1);
    rt = 99; if ~isempty(r10)&&~isempty(r90); rt = (r90-r10)*ts; end
    si = find(abs(yS-target)/target>0.05,1,'last'); st = 0;
    if ~isempty(si); st = si*ts; end
    score = rt*3 + st*2 + os*0.5;
end
