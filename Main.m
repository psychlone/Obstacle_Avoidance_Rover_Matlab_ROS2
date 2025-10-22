clc; clear; close all;
%% Environment Creation
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

% Define sample time (e.g., 0.1 s)
Ts = 0.1;
t = (0:length(x_ref)-1)' * Ts;

% Convert to timeseries
x_ref_signal = timeseries(x_ref, t);
y_ref_signal = timeseries(y_ref, t);
theta_ref_signal = timeseries(theta_ref, t);

% Optional: save to workspace for Simulink
assignin('base','x_ref_signal',x_ref_signal);
assignin('base','y_ref_signal',y_ref_signal);
assignin('base','theta_ref_signal',theta_ref_signal);

%% Simulink
RoverSim