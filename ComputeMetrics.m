function [rmse_t, compA_t, compG_t, time] = ComputeMetrics(trajAStar, trajGreedy, refAStar, refGreedy, rover)
%#codegen
% Time-varying RMSE between A* and Greedy paths at matched along-track
% progress, and completion % for each algorithm based on its own reference.

%% Match time vectors
N = min(size(trajAStar,1), size(trajGreedy,1));
trajAStar  = trajAStar(1:N,:);   % [x y theta]
trajGreedy = trajGreedy(1:N,:);
time = (0:N-1)' * rover.sampleTime_dynamics;

%% Precompute arclength parameterization for both reference paths
sRefA = cumulativeArcLength(refAStar);   % length M_A
sRefG = cumulativeArcLength(refGreedy);  % length M_G
LA = sRefA(end);
LG = sRefG(end);

%% Per-time along-track progress by projecting dynamic poses onto refs
sA = zeros(N,1);
sG = zeros(N,1);
prevSA = 0.0; prevSG = 0.0;
for k = 1:N
    pA = trajAStar(k,1:2);
    pG = trajGreedy(k,1:2);

    sA(k) = max(prevSA, projectProgressOnRef(refAStar, sRefA, pA));
    sG(k) = max(prevSG, projectProgressOnRef(refGreedy, sRefG, pG));

    prevSA = sA(k);
    prevSG = sG(k);
end

%% Completion % over time (clamped to [0,100])
if LA <= 1e-6
    compA_t = 100*ones(N,1);
else
    compA_t = 100 * min(sA/LA, 1.0);
end

if LG <= 1e-6
    compG_t = 100*ones(N,1);
else
    compG_t = 100 * min(sG/LG, 1.0);
end

%% Time-varying RMSE at matched progress
% Compare the two reference paths at the SAME progress sCommon(t).
% This yields 0 initially if both refs share the same corridor.
sCommon = zeros(N,1);
for k = 1:N
    % Use the lesser progress so both are defined
    sCommon(k) = min(sA(k), sG(k));
end

rmse_t = zeros(N,1);
for k = 1:N
    % Interpolate point on each reference at sCommon(k)
    PA = interpRefAtS(refAStar, sRefA, sCommon(k));
    PG = interpRefAtS(refGreedy, sRefG, sCommon(k));
    dx = PA(1) - PG(1);
    dy = PA(2) - PG(2);
    rmse_t(k) = sqrt(dx*dx + dy*dy);
end
end

%% Helpers

function s = cumulativeArcLength(xy)
%#codegen
n = size(xy,1);
s = zeros(n,1);
for i = 2:n
    dx = xy(i,1)-xy(i-1,1);
    dy = xy(i,2)-xy(i-1,2);
    s(i) = s(i-1) + sqrt(dx*dx + dy*dy);
end
end

function sProj = projectProgressOnRef(refXY, sRef, p)
%#codegen
% Project point p onto the broken line refXY and return arclength s at projection.
% Linear search segment-by-segment
M = size(refXY,1);
if M < 2
    sProj = 0.0; return;
end

bestDist2 = realmax;
bestS     = 0.0;

for i = 1:M-1
    a = refXY(i,1:2); b = refXY(i+1,1:2);
    abx = b(1)-a(1); aby = b(2)-a(2);
    apx = p(1)-a(1); apy = p(2)-a(2);
    L2 = abx*abx + aby*aby;
    if L2 <= 1e-12
        t = 0.0;
    else
        t = (apx*abx + apy*aby)/L2;
        if t < 0.0, t = 0.0; elseif t > 1.0, t = 1.0; end
    end
    projx = a(1) + t*abx; projy = a(2) + t*aby;
    dx = p(1)-projx; dy = p(2)-projy;
    d2 = dx*dx + dy*dy;

    if d2 < bestDist2
        bestDist2 = d2;
        segLen = sqrt(L2);
        bestS = sRef(i) + t*segLen;
    end
end
sProj = bestS;
end

function P = interpRefAtS(refXY, sRef, sQ)
%#codegen
% Linear interpolation along arclength grid to get point at sQ.
M = length(sRef);
if sQ <= 0 || M == 0
    P = refXY(1,1:2);
    return;
end
L = sRef(end);
if sQ >= L
    P = refXY(end,1:2);
    return;
end

% Find segment such that sRef(i) <= sQ <= sRef(i+1)
i = 1;
while i < M && sRef(i+1) < sQ
    i = i + 1;
end
s0 = sRef(i); s1 = sRef(i+1);
if s1 <= s0
    alpha = 0.0;
else
    alpha = (sQ - s0) / (s1 - s0);
end
P = (1-alpha)*refXY(i,1:2) + alpha*refXY(i+1,1:2);
end
