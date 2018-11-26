function [h, H_x] = measurementFunction(x, m)
% [h, H_x] = measurementFunction(x, m) returns the predicted measurement
% given a state x and a single map entry m. H_x denotes the Jacobian of the
% measurement function with respect to the state evaluated at the state
% provided.
% Map entry and state are defined according to "Introduction to Autonomous Mobile Robots" pp. 337

%STARTRM

% First we need to compute the the transformation from a line expressed in the world coordinate
% frame into the body coordinate frame of the robot, z^t, given by the
% return parameter h. m(1) and m(2) are map entries.
% This formula is taken from the reference (1), 5.94.
xX = x(1);
xY = x(2);
xTheta = x(3);
Map1 = m(1);
Map2 = m(2);

h = [Map1 - xTheta; 
     Map2 - (xX*cos(Map1) + xY*sin(Map1))];

% Jacobian of h above. (5.95).
H_x = [0          0          -1; 
       -cos(Map1) -sin(Map1)  0];
%ENDRM

[h(1), h(2), isRNegated] = normalizeLineParameters(h(1), h(2));

if isRNegated 
    H_x(2, :) = - H_x(2, :);
end

