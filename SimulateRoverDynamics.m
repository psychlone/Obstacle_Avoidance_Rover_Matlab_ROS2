function [t, traj] = SimulateRoverDynamics(refPath, rover, totalTime)
%#codegen
% dynamic simulation (unicycle model)
% refPath: [x_ref, y_ref] waypoints (m)
% rover: structure from RoverParams
% totalTime: seconds to simulate
% Returns: time vector and trajectory [x, y, theta]

dt = rover.sampleTime_dynamics;
N  = round(totalTime/dt);
t  = (0:N-1)'*dt;

% PID gains (tuned small)
Kp_lin = 1.0;
Kp_ang = 2.0;

x = refPath(1,1);
y = refPath(1,2);
theta = 0;
traj = zeros(N,3);
traj(1,:) = [x,y,theta];

% Target path step index
idx = 1;
nRef = size(refPath,1);

for k = 2:N
    % If reached end, stop
    if idx >= nRef
        traj(k:end,:) = repmat(traj(k-1,:), N-k+1, 1);
        break;
    end

    % Current target waypoint
    goal = refPath(idx,:);
    dx = goal(1) - x;
    dy = goal(2) - y;
    dist = sqrt(dx^2 + dy^2);
    goalAngle = atan2(dy, dx);

    % Heading error
    angErr = wrapToPi(goalAngle - theta);

    % Simple control law
    v = Kp_lin * dist;                 % forward speed
    w = Kp_ang * angErr;               % angular speed
    v = min(v, rover.maxSpeed);
    w = min(max(w, -rover.maxAngularSpeed), rover.maxAngularSpeed);

    % Dynamics (unicycle)
    x = x + v*cos(theta)*dt;
    y = y + v*sin(theta)*dt;
    theta = wrapToPi(theta + w*dt);

    traj(k,:) = [x, y, theta];

    % If close to target waypoint, advance
    if dist < 0.2
        idx = idx + 1;
    end
end
end
