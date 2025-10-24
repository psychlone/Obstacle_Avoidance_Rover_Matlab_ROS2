function env = Environment()
%#codegen
%% Map parameters
env.mapWidth  = 30;     % meters
env.mapHeight = 30;     % meters
env.resolution = 30;    % resolution 30 cells/m

% Create an empty occupancy map
env.map = occupancyMap(env.mapWidth, env.mapHeight, env.resolution);
setOccupancy(env.map, [0 0], 0);   % Initialize

%% Rover spawn / start and goal positions
% [x, y, theta(deg)]
env.startPose = [3, 3, 90];           % Start near bottom-left
env.goalPose  = [28, 28, 0];         % Goal near top-right

%% Generate Random Obstacles
numObstacles = 8;  % Number of obstacles to generate
minObstacleSize = 1.5;  % Minimum width/height (m)
maxObstacleSize = 3.0;  % Maximum width/height (m)
safetyBuffer = 3.0;     % Minimum distance from start/goal (m)
inflationRadius = 0.15; % Inflation amount (m)

% Total clearance needed around start/goal
minClearance = safetyBuffer + inflationRadius;

env.obstacles = obstacles(numObstacles, ...
    env.mapWidth, env.mapHeight, ...
    env.startPose, env.goalPose, ...
    minObstacleSize, maxObstacleSize, ...
    minClearance);

%% Populate occupancy map
for i = 1:size(env.obstacles,1)
    obs = env.obstacles(i,:);
    xRange = obs(1)-obs(3)/2 : 1/env.resolution : obs(1)+obs(3)/2;
    yRange = obs(2)-obs(4)/2 : 1/env.resolution : obs(2)+obs(4)/2;
    [xGrid, yGrid] = meshgrid(xRange, yRange);
    setOccupancy(env.map, [xGrid(:), yGrid(:)], 1);
end

inflate(env.map, inflationRadius); % Inflate obstacles for rover radius (safety margin)

%% Check Start and End Positions
% if checkOccupancy(env.map, [env.startPose(1), env.startPose(2)])
%     warning('Start position is occupied!');
% end
% if checkOccupancy(env.map, [env.goalPose(1), env.goalPose(2)])
%     warning('Goal position is occupied!');
% end

%% Visualization
figure('Name','Simulation Environment'); 
show(env.map);
hold on;
plot(env.startPose(1), env.startPose(2),'go','MarkerFaceColor','g','MarkerSize',10,'DisplayName','Start');
plot(env.goalPose(1),  env.goalPose(2),'ro','MarkerFaceColor','r','MarkerSize',10,'DisplayName','Goal');
legend;
title('Rover Environment Map with Random Obstacles');
axis equal;
grid on;
hold off;

%% Numeric Grid Data
env.mapMatrix = occupancyMatrix(env.map);
env.mapResolution = env.resolution;
env.mapLimits = [env.map.XWorldLimits; env.map.YWorldLimits];

%% Return environment structure
disp('Environment created successfully.');
fprintf('Generated %d obstacles\n', size(env.obstacles,1));
end
