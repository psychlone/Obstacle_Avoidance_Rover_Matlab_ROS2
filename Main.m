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
refAStar  = [x_ref, y_ref];


figure;
show(env.map); hold on;
plot(x_ref, y_ref, 'b-', 'LineWidth', 2);
plot(env.startPose(1), env.startPose(2), 'go', 'MarkerFaceColor','g');
plot(env.goalPose(1), env.goalPose(2), 'ro', 'MarkerFaceColor','r');
title('Simple A* Planned Path');
legend('Planned Path','Start','Goal');
axis equal;
%% Sim Setup
simOpts.stepLen          = 0.6;
simOpts.goalTol          = 0.5;
simOpts.maxSteps         = 400;
simOpts.forwardCorrHalfW = rover.bodyWidth*0.5 + 0.08;
simOpts.forwardMargin    = 0.10;
simOpts.sidestepDeg      = [0, 20, -20, 35, -35, 50, -50];
senseOpts.FOVdeg = rover.camera.FOV_horizontal;
senseOpts.range  = rover.camera.range;
senseOpts.nRays  = 51;
senseOpts.ds     = 1/env.resolution;

% Greedy Sim Call
[refGreedy, reachedGoal] = SimulateReactiveRover(env, rover, simOpts, senseOpts);
disp(['Greedy reached goal: ', num2str(reachedGoal)]);


figure; show(env.map); hold on;
plot(refGreedy(:,1), refGreedy(:,2), 'm-', 'LineWidth', 2);
plot(env.startPose(1), env.startPose(2), 'go', 'MarkerFaceColor','g');
plot(env.goalPose(1), env.goalPose(2), 'ro', 'MarkerFaceColor','r');
title(sprintf('Greedy Run (reached=%d, steps=%d)', reachedGoal, size(refGreedy,1)));
legend('Driven Path','Start','Goal'); axis equal; grid on;

%% Simulate dynamics
totalTime = 160; % seconds

[t1, trajAStar]  = SimulateRoverDynamics(refAStar,  rover, totalTime);
[t2, trajGreedy] = SimulateRoverDynamics(refGreedy, rover, totalTime);

%% Compute metrics (time-varying)
[rmse_t, compA_t, compG_t, time] = ComputeMetrics(trajAStar, trajGreedy, refAStar, refGreedy, rover);

%% Plot Time-varying RMSE
figure('Name','Time-Varying RMSE (Progress-Aligned)');
plot(time, rmse_t, 'r-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('RMSE (m)');
title('RMSE vs Time (A* vs Greedy, matched along-track progress)');
grid on;
%% Plot Completion Percentage
figure('Name','Completion Comparison');
plot(time, compA_t, 'b-', 'LineWidth', 2, 'DisplayName', 'Greedy');
hold on;
plot(time, compG_t, 'm-', 'LineWidth', 2, 'DisplayName', 'A*');
xlabel('Time (s)'); ylabel('Completion (%)');
legend('Location','southeast');
title('Completion % vs Time (projection onto own reference)');
grid on;
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