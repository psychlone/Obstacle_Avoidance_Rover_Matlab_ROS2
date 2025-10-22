%% Helper function to generate random obstacles
function obstacles = obstacles(numObstacles, mapWidth, mapHeight, ...
    startPose, goalPose, minSize, maxSize, minClearance)
    
    obstacles = zeros(numObstacles, 4);  % [x_center, y_center, width, height]
    
    % Define map boundaries with margin
    mapMargin = 2.0;  % Stay away from edges
    
    for i = 1:numObstacles
        validPosition = false;
        attempts = 0;
        maxAttempts = 100;
        
        while ~validPosition && attempts < maxAttempts
            attempts = attempts + 1;
            
            % Generate random obstacle properties
            x = mapMargin + rand() * (mapWidth - 2*mapMargin);
            y = mapMargin + rand() * (mapHeight - 2*mapMargin);
            width = minSize + rand() * (maxSize - minSize);
            height = minSize + rand() * (maxSize - minSize);
            
            % Check distance from start position
            distToStart = sqrt((x - startPose(1))^2 + (y - startPose(2))^2);
            
            % Check distance from goal position
            distToGoal = sqrt((x - goalPose(1))^2 + (y - goalPose(2))^2);
            
            % Check if obstacle overlaps with existing obstacles
            overlaps = false;
            for j = 1:i-1
                if j > 0 && size(obstacles,1) >= j
                    existingObs = obstacles(j,:);
                    % Check for overlap (with some spacing)
                    spacing = 1.0;  % Minimum space between obstacles
                    if abs(x - existingObs(1)) < (width + existingObs(3))/2 + spacing && ...
                       abs(y - existingObs(2)) < (height + existingObs(4))/2 + spacing
                        overlaps = true;
                        break;
                    end
                end
            end
            
            % Validate position
            if distToStart > minClearance && distToGoal > minClearance && ~overlaps
                validPosition = true;
                obstacles(i,:) = [x, y, width, height];
            end
        end
        
        if ~validPosition
            warning('Could not place obstacle %d after %d attempts', i, maxAttempts);
            % Use a safe default position
            obstacles(i,:) = [mapWidth/2, mapHeight/2, minSize, minSize];
        end
    end
end