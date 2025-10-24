function [x_ref, y_ref, theta_ref] = SimpleAStarPlanner(startPose, goalPose, map)
%#codegen

% Simple A* planner for occupancyMap
% Outputs: x_ref, y_ref, theta_ref

%% Clamp poses inside map bounds
startXY = startPose(1:2);
goalXY  = goalPose(1:2);
worldLimits = [map.XWorldLimits; map.YWorldLimits];
startXY = max(min(startXY, worldLimits(:,2)'), worldLimits(:,1)');
goalXY  = max(min(goalXY, worldLimits(:,2)'), worldLimits(:,1)');

%% Create planner
planner = plannerAStarGrid(map);

%% Convert world -> grid indices
startIdx = world2grid(map, startXY);  % returns 1x2: [row, col]
goalIdx  = world2grid(map, goalXY);

%% Plan
[pathIdx, ~] = plan(planner, startIdx, goalIdx);

%% Handle failure
if isempty(pathIdx)
    warning('A* failed - using straight line fallback.');
    x_ref = linspace(startXY(1), goalXY(1), 50)';
    y_ref = linspace(startXY(2), goalXY(2), 50)';
else
    xy_ref = grid2world(map, pathIdx);  % Nx2 array of [x, y]
    x_ref = xy_ref(:,1);
    y_ref = xy_ref(:,2);
end

%% Compute orientation
dx = [diff(x_ref); 0];
dy = [diff(y_ref); 0];
theta_ref = atan2(dy, dx);

%% Smooth for controller
windowSize = 5;
x_ref = smoothdata(x_ref, 'movmean', windowSize);
y_ref = smoothdata(y_ref, 'movmean', windowSize);
theta_ref = smoothdata(theta_ref, 'movmean', windowSize);

end





% function [x_ref, y_ref] = SimpleAStarPlanner(startPose, goalPose, map)
% 
% % Create planner object
% planner = plannerAStarGrid(map);
% 
% % Convert start and goal positions to Nx2 array
% startXY = startPose(1:2);  % ensure 1x2
% goalXY  = goalPose(1:2);
% 
% % Convert from world coordinates to grid indices
% startIdx = map.world2grid(startXY);  % Nx2
% goalIdx  = map.world2grid(goalXY);
% 
% % Plan path (grid indices)
% [pathIdx, ~] = plan(planner, startIdx, goalIdx);
% 
% if isempty(pathIdx)
%     warning('A* could not find a path! Using straight line fallback.');
%     x_ref = linspace(startXY(1), goalXY(1), 50)';
%     y_ref = linspace(startXY(2), goalXY(2), 50)';
% else
%     % Convert grid indices back to world coordinates
%     xy_ref = map.grid2world(pathIdx);  % Nx2
%     x_ref = xy_ref(:,1);
%     y_ref = xy_ref(:,2);
% end
% 
% end