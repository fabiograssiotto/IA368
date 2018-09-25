function [ vu, omega ] = calculateControlOutput( robotPose, goalPose, parameters )
%CALCULATECONTROLOUTPUT This function computes the motor velocities for a differential driven robot

% current robot position and orientation
x = robotPose(1);
y = robotPose(2);
theta = robotPose(3);

% goal position and orientation
xg = goalPose(1);
yg = goalPose(2);
thetag = goalPose(3);

% compute control quantities
rho = sqrt((xg-x)^2+(yg-y)^2);  % pythagoras theorem, sqrt(dx^2 + dy^2)
lambda = atan2(yg-y, xg-x);     % angle of the vector pointing from the robot to the goal in the inertial frame
alpha = lambda - theta;         % angle of the vector pointing from the robot to the goal in the robot frame
alpha = normalizeAngle(alpha);

% calculate beta
beta = - theta - alpha;

% the following paramerters should be used:
% Task 3:
% parameters.Kalpha, parameters.Kbeta, parameters.Krho: controller tuning parameters
% Task 4:
% parameters.backwardAllowed: This boolean variable should switch the between the two controllers
% parameters.useConstantSpeed: Turn on constant speed option
% parameters.constantSpeed: The speed used when constant speed option is on

% backwards velocities allowed:
if (parameters.backwardAllowed == true)
    if ( alpha <= pi/2 && alpha >= -pi/2) 
        % Alpha is normalized between [-pi,pi].
        % If the angle of the vector pointing from the robot to the goal in
        % the robot frame is between -90 and 90 degrees, a positive velocity is a faster
        % alternative for the controller. Otherwise, use a negative one.
        vel_factor = 1.0;
    else
        vel_factor = -1.0;
    end
end

% Now calculate velocity and omega.
if (parameters.useConstantSpeed == false)
    % Constant speed setting is off.
    vu = vel_factor*parameters.Krho*rho;
    omega = parameters.Kalpha*alpha + parameters.Kbeta*beta;
else
    % Constant speed setting is on.
    vu_pre = parameters.Krho*rho;
    omega_pre = parameters.Kalpha*alpha + parameters.Kbeta*beta;
    
    % Now scale vu and omega to keep vu constant.
    % Using the fact that the quotient between v/w is kept constant.
    vu = vel_factor*parameters.constantSpeed;
    omega = omega_pre * (vu/vu_pre);
end

