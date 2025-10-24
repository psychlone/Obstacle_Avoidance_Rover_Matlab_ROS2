function [pathXY, reachedGoal] = SimulateReactiveRover(env, rover, simOpts, senseOpts)
%#codegen
% Runs a simple sense-plan-step loop until goal or max steps

% State (x,y,theta in radians)
x     = env.startPose(1);
y     = env.startPose(2);
theta = deg2rad(env.startPose(3));

gx = env.goalPose(1); gy = env.goalPose(2);

% Preallocate a reasonable path capacity
maxN = simOpts.maxSteps + 1;
pathXY = zeros(maxN,2);
pathXY(1,:) = [x,y];
k = 1;

reachedGoal = false;

for step = 1:simOpts.maxSteps

    % 1) Sense
    [hitPts, freeEnds] = SensorModel_RangeRays(env.map, [x,y,theta], senseOpts);

    % 2) Plan next heading (greedy-to-goal with sidesteps)
    nextHeading = GreedyStepPlanner(x, y, theta, [gx, gy], env.map, ...
        simOpts.stepLen, simOpts.forwardCorrHalfW, simOpts.forwardMargin, ...
        simOpts.sidestepDeg);

    % If no feasible heading found, we’re stuck
    if isnan(nextHeading)
        break;
    end

    % 3) Step
    theta = nextHeading;
    x = x + simOpts.stepLen * cos(theta);
    y = y + simOpts.stepLen * sin(theta);

    % Clamp inside map bounds slightly (avoid going out of world limits)
    x = min(max(x, env.map.XWorldLimits(1)+1e-3), env.map.XWorldLimits(2)-1e-3);
    y = min(max(y, env.map.YWorldLimits(1)+1e-3), env.map.YWorldLimits(2)-1e-3);

    % Record
    k = k + 1;
    pathXY(k,:) = [x,y];

    % Goal check
    dx = x - gx; dy = y - gy;
    if sqrt(dx*dx + dy*dy) <= simOpts.goalTol
        reachedGoal = true;
        break;
    end
end

pathXY = pathXY(1:k,:);
end
