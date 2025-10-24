clc; clear; close all;
%% Environment Creation
%#codegen
env = Environment(); % load environment
%% Rover Parameters
rover = RoverParams();
%% Visualize environment and rover start configuration
visualize(env,rover);
%% A* Planner & Visualize
[x_ref, y_ref, theta_ref] = SimpleAStarPlanner(env.startPose, env.goalPose, env.map);

figure;
show(env.map); hold on;
plot(x_ref, y_ref, 'b-', 'LineWidth', 2);
plot(env.startPose(1), env.startPose(2), 'go', 'MarkerFaceColor','g');
plot(env.goalPose(1), env.goalPose(2), 'ro', 'MarkerFaceColor','r');
title('Simple A* Planned Path');
legend('Planned Path','Start','Goal');
axis equal;
%% Sim Setup
simOpts.stepLen          = 0.6;     % meters per step
simOpts.goalTol          = 0.5;     % stop when within this distance of goal (m)
simOpts.maxSteps         = 400;     % hard cap to avoid infinite loops
simOpts.forwardCorrHalfW = rover.bodyWidth*0.5 + 0.08;  % corridor half-width
simOpts.forwardMargin    = 0.10;    % extend collision check a bit beyond step
simOpts.sidestepDeg      = [0, 20, -20, 35, -35, 50, -50]; % try in this order

% Sensor Setup
senseOpts.FOVdeg = rover.camera.FOV_horizontal;
senseOpts.range  = rover.camera.range;
senseOpts.nRays  = 51;       % number of rays across FOV
senseOpts.ds     = 1/env.resolution; % step in meters along a ray

[pathXY, reachedGoal] = SimulateReactiveRover(env, rover, simOpts, senseOpts);

figure; show(env.map); hold on;
plot(pathXY(:,1), pathXY(:,2), 'm-', 'LineWidth', 2);
plot(env.startPose(1), env.startPose(2), 'go', 'MarkerFaceColor','g');
plot(env.goalPose(1), env.goalPose(2), 'ro', 'MarkerFaceColor','r');
title(sprintf('Reactive Run (reached=%d, steps=%d)', reachedGoal, size(pathXY,1)));
legend('Driven Path','Start','Goal'); axis equal; grid on;
%% Not Used
% % Define sample time (e.g., 0.1 s)
% Ts = 0.1;
% t = (0:length(x_ref)-1)' * Ts;
% 
% % Convert to timeseries
% x_ref_signal = timeseries(x_ref, t);
% y_ref_signal = timeseries(y_ref, t);
% theta_ref_signal = timeseries(theta_ref, t);
% 
% % save to workspace for Simulink
% assignin('base','x_ref_signal',x_ref_signal);
% assignin('base','y_ref_signal',y_ref_signal);
% assignin('base','theta_ref_signal',theta_ref_signal);

%% Simulink
% RoverSim