function [f, F_x, F_u] = transitionFunction(x,u, l)
% [f, F_x, F_u] = transitionFunction(x,u,l) predicts the state x at time t given
% the state at time t-1 and the input u at time t. F_x denotes the Jacobian
% of the state transition function with respect to the state evaluated at
% the state and input provided. F_u denotes the Jacobian of the state
% transition function with respect to the input evaluated at the state and
% input provided.
% State and input are defined according to "Introduction to Autonomous Mobile Robots", pp. 337

%STARTRM

syms xX xY xTheta uDeltaSl uDeltaSr;

% f is the estimate for the state x at T t.
f_Sym = [xX;xY;xTheta] + [ (uDeltaSr + uDeltaSl)/2*cos(xTheta + (uDeltaSr-uDeltaSl)/2*l);
          (uDeltaSr + uDeltaSl)/2*sin(xTheta + (uDeltaSr-uDeltaSl)/2*l);
          (uDeltaSl - uDeltaSr)/l];

% Jacobians
J_x = jacobian(f_Sym, [xX;xY;xTheta]);
J_u = jacobian(f_Sym, [uDeltaSl;uDeltaSr]);

% Evaluation 
f = eval(subs(f_Sym, [xX;xY;xTheta;uDeltaSl;uDeltaSr], [x(1);x(2);x(3);u(1);u(2)]));
F_x = eval(subs(J_x, [xX;xY;xTheta;uDeltaSl;uDeltaSr], [x(1);x(2);x(3);u(1);u(2)]));
F_u = eval(subs(J_u, [xX;xY;xTheta;uDeltaSl;uDeltaSr], [x(1);x(2);x(3);u(1);u(2)]));

%ENDRM
