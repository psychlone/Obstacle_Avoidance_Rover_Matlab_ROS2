function visualize = visualize(env,rover)
figure('Name','Rover Environment Visualization'); 
show(env.map);
hold on;

% Plot start & goal
plot(env.startPose(1), env.startPose(2), 'go', 'MarkerFaceColor','g', 'DisplayName','Start');
plot(env.goalPose(1),  env.goalPose(2),  'ro', 'MarkerFaceColor','r', 'DisplayName','Goal');

% Plot rover body footprint
bodyL = rover.bodyLength;
bodyW = rover.bodyWidth;
x0 = env.startPose(1);
y0 = env.startPose(2);
theta0 = deg2rad(env.startPose(3));
corners = [ bodyL/2  bodyW/2;
            bodyL/2 -bodyW/2;
           -bodyL/2 -bodyW/2;
           -bodyL/2  bodyW/2;
            bodyL/2  bodyW/2]';
R = [cos(theta0) -sin(theta0); sin(theta0) cos(theta0)];
bodyXY = R * corners + [x0; y0];
plot(bodyXY(1,:), bodyXY(2,:), 'b-', 'LineWidth', 1.5, 'DisplayName','Rover Body');

% Plot camera field of view (FOV)
fov = rover.camera.FOV_horizontal;
range = rover.camera.range;
fovX = [0 range*cosd(-fov/2) range*cosd(fov/2)];
fovY = [0 range*sind(-fov/2) range*sind(fov/2)];
fovRot = R * [fovX; fovY] + [x0; y0];
fill([x0 fovRot(1,:)],[y0 fovRot(2,:)],'c','FaceAlpha',0.1,'EdgeColor','c','DisplayName','Camera FOV');

legend;
axis equal;
grid on;
xlabel('X (m)');
ylabel('Y (m)');
title('Rover Start Pose and Environment');

disp('Visualization complete.');
end