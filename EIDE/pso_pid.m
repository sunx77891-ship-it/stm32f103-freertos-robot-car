function [best, history] = pso_pid(objFunc, nVar, varMin, varMax, opts)
    % PSO_PID 粒子群算法优化PID参数
    %   [best, history] = pso_pid(objFunc, nVar, varMin, varMax, opts)
    %
    %   输入:
    %       objFunc - 目标函数句柄 @(kp,ki,kd) -> score
    %       nVar    - 变量数 (通常 3: Kp, Ki, Kd)
    %       varMin  - 下界 [Kp_min, Ki_min, Kd_min]
    %       varMax  - 上界 [Kp_max, Ki_max, Kd_max]
    %       opts    - 可选结构体:
    %           swarmSize    - 粒子数 (默认 30)
    %           maxIter      - 最大迭代 (默认 50)
    %           w            - 惯性权重 (默认 0.7)
    %           wDamp        - 惯性衰减系数 (默认 0.98)
    %           c1           - 个体学习因子 (默认 1.5)
    %           c2           - 社会学习因子 (默认 2.0)
    %           display      - 是否显示进度 (默认 true)
    %
    %   输出:
    %       best    - 最优参数结构体: .kp .ki .kd .score
    %       history - 每代最优得分数组

    %% 默认参数
    if nargin < 5; opts = struct(); end
    swarmSize = getOpt(opts, 'swarmSize', 30);
    maxIter   = getOpt(opts, 'maxIter', 50);
    w         = getOpt(opts, 'w', 0.7);
    wDamp     = getOpt(opts, 'wDamp', 0.98);
    c1        = getOpt(opts, 'c1', 1.5);
    c2        = getOpt(opts, 'c2', 2.0);
    display   = getOpt(opts, 'display', true);

    %% 初始化粒子群
    swarm.pos     = zeros(swarmSize, nVar);   % 当前位置
    swarm.vel     = zeros(swarmSize, nVar);   % 当前速度
    swarm.bestPos = zeros(swarmSize, nVar);   % 个体最优位置
    swarm.bestScore = inf(swarmSize, 1);      % 个体最优得分

    globalBestPos   = zeros(1, nVar);
    globalBestScore = inf;

    % 随机初始化位置和速度
    for i = 1:swarmSize
        for j = 1:nVar
            swarm.pos(i,j) = varMin(j) + rand() * (varMax(j) - varMin(j));
            swarm.vel(i,j) = -(varMax(j)-varMin(j))/2 + rand() * (varMax(j)-varMin(j));
        end
        % 计算初始适应度
        if nVar == 2
            score = objFunc(swarm.pos(i,1), swarm.pos(i,2));
        else
            score = objFunc(swarm.pos(i,1), swarm.pos(i,2), swarm.pos(i,3));
        end
        swarm.bestPos(i,:) = swarm.pos(i,:);
        swarm.bestScore(i) = score;

        if score < globalBestScore
            globalBestScore = score;
            globalBestPos = swarm.pos(i,:);
        end
    end

    %% PSO 主循环
    history = zeros(maxIter, 1);
    best = struct('kp',0,'ki',0,'kd',0,'score',inf);

    for iter = 1:maxIter
        for i = 1:swarmSize
            % 更新速度
            r1 = rand(1, nVar);
            r2 = rand(1, nVar);
            swarm.vel(i,:) = w * swarm.vel(i,:) ...
                + c1 * r1 .* (swarm.bestPos(i,:) - swarm.pos(i,:)) ...
                + c2 * r2 .* (globalBestPos - swarm.pos(i,:));

            % 速度限幅 (最大速度 = 变量范围的 20%)
            vMax = 0.2 * (varMax - varMin);
            swarm.vel(i,:) = max(-vMax, min(vMax, swarm.vel(i,:)));

            % 更新位置
            swarm.pos(i,:) = swarm.pos(i,:) + swarm.vel(i,:);

            % 边界处理 (反弹)
            for j = 1:nVar
                if swarm.pos(i,j) < varMin(j)
                    swarm.pos(i,j) = varMin(j);
                    swarm.vel(i,j) = -0.5 * swarm.vel(i,j);
                elseif swarm.pos(i,j) > varMax(j)
                    swarm.pos(i,j) = varMax(j);
                    swarm.vel(i,j) = -0.5 * swarm.vel(i,j);
                end
            end

            % 评估（支持 nVar=2 或 3）
            if nVar == 2
                score = objFunc(swarm.pos(i,1), swarm.pos(i,2));
            else
                score = objFunc(swarm.pos(i,1), swarm.pos(i,2), swarm.pos(i,3));
            end

            % 更新个体最优
            if score < swarm.bestScore(i)
                swarm.bestScore(i) = score;
                swarm.bestPos(i,:) = swarm.pos(i,:);
            end

            % 更新全局最优
            if score < globalBestScore
                globalBestScore = score;
                globalBestPos = swarm.pos(i,:);
            end
        end

        % 惯性衰减
        w = w * wDamp;

        % 记录历史
        history(iter) = globalBestScore;
        best.kp = globalBestPos(1);
        best.ki = globalBestPos(2);
        if nVar >= 3; best.kd = globalBestPos(3); else; best.kd = 0; end
        best.score = globalBestScore;

        if display
            fprintf('  PSO iter=%2d/%d  bestScore=%.2f  Kp=%.2f Ki=%.2f Kd=%.2f\n', ...
                iter, maxIter, globalBestScore, best.kp, best.ki, best.kd);
        end
    end
end

function val = getOpt(opts, field, default)
    if isfield(opts, field) && ~isempty(opts.(field))
        val = opts.(field);
    else
        val = default;
    end
end
