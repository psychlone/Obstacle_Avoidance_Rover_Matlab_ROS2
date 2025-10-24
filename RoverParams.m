function rover = RoverParams()
%#codegen
%% Physical parameters
rover.mass             = 15;         % kg
rover.wheelRadius      = 0.10;       % m
rover.wheelBase        = 0.25;       % m  % distance between front and rear axles
rover.trackWidth       = 0.25;       % m  % distance between left and right wheels
rover.bodyLength       = 0.3;       % m
rover.bodyWidth        = 0.50;       % m

%% Dynamics parameters
rover.wheelInertia     = 0.02;       % kg*m^2 per wheel
rover.frictionCoeff    = 0.8;        % Ground friction coefficient (μ)
rover.rollingResistance= 0.015;      % Rolling resistance coefficient
rover.motorMaxTorque   = 3.0;        % N*m
rover.maxSpeed         = 2.0;        % m/s
rover.maxSteerAngle    = 30*pi/180;  % rad (for Ackermann-type steering)
rover.minTurningRadius = 1;

%% Control sample times
rover.sampleTime_control = 0.05;    % s (20 Hz)
rover.sampleTime_dynamics = 0.01;   % s (100 Hz)
rover.sampleTime_sensor   = 0.1;    % s (10 Hz)

% %% Traction model
% rover.tractionModel = 'simplified'; 


%% Camera parameters
rover.camera.FOV_horizontal = 120;   % degrees
rover.camera.FOV_vertical   = 60;    % degrees (typical)
rover.camera.mountHeight    = 0.25;  % m above ground
rover.camera.mountPitch     = -10;   % degrees (slightly downward)
rover.camera.range          = 10;    % m (detection range)
rover.camera.resolution     = [640, 480]; % pixels [width, height]

%% Derived parameters
rover.bodyDiagonal = sqrt(rover.bodyLength^2 + rover.bodyWidth^2);
rover.maxAngularSpeed = rover.maxSpeed / rover.minTurningRadius;

disp('Rover parameters loaded successfully.');

end