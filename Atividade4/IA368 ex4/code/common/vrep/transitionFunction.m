function [f, F_x, F_u] = transitionFunction(x,u, l)
% [f, F_x, F_u] = transitionFunction(x,u,l) predicts the state x at time t given
% the state at time t-1 and the input u at time t. F_x denotes the Jacobian
% of the state transition function with respect to the state evaluated at
% the state and input provided. F_u denotes the Jacobian of the state
% transition function with respect to the input evaluated at the state and
% input provided.
% State and input are defined according to "Introduction to Autonomous Mobile Robots", pp. 337

%STARTRM
%syms xX xY xTheta uDeltaSr uDeltaSl;
% Jacobians - Symbolic.
%J_x = jacobian(f_Sym, [xX;xY;xTheta]);
%J_u = jacobian(f_Sym, [uDeltaSl;uDeltaSr]);

% Evaluation 
%f =  eval(subs(f_Sym, [xX xY xTheta uDeltaSr uDeltaSl], [x(1) x(2) x(3) u(2) u(1)]));
%F_x = eval(subs(J_x, [xX xY xTheta uDeltaSr uDeltaSl], [x(1) x(2) x(3) u(2) u(1)]));
%F_u = eval(subs(J_u, [xTheta uDeltaSr uDeltaSl], [x(3) u(2) u(1)]));
% Symbolic solution does not seem to work at all.

xX = x(1);
xY = x(2);
xTheta = x(3);
uDeltaSl = u(1);
uDeltaSr = u(2);

% f is the estimate for the state x at T t.
f = [xX;xY;xTheta] + [ ((uDeltaSr + uDeltaSl)/2)*cos(xTheta + ((uDeltaSr-uDeltaSl)/2*l));
          ((uDeltaSl + uDeltaSr)/2)*sin(xTheta + ((uDeltaSr-uDeltaSl)/2*l));
          (uDeltaSr - uDeltaSl)/l];

% Jacobians - formulas by hand.
F_x = [ 1 0 -(((uDeltaSr + uDeltaSl)/2)*sin(xTheta+(((uDeltaSr - uDeltaSl)/l)/2))); 
        0 1 (((uDeltaSr + uDeltaSl)/2)*cos(xTheta+(((uDeltaSr - uDeltaSl)/l)/2)));
        0 0 1];
        
F_u = [cos(xTheta+(((uDeltaSr - uDeltaSl)/l)/2))/2 + (((uDeltaSr + uDeltaSl)/2)/(2*l))*sin(xTheta+(((uDeltaSr - uDeltaSl)/l)/2))...
          cos(xTheta+(((uDeltaSr - uDeltaSl)/l)/2))/2 - (((uDeltaSr + uDeltaSl)/2)/(2*l))*sin(xTheta+(((uDeltaSr - uDeltaSl)/l)/2));
       sin(xTheta+(((uDeltaSr - uDeltaSl)/l)/2))/2 - (((uDeltaSr + uDeltaSl)/2)/(2*l))*cos(xTheta+(((uDeltaSr - uDeltaSl)/l)/2))... 
          sin(xTheta+(((uDeltaSr - uDeltaSl)/l)/2))/2 + (((uDeltaSr + uDeltaSl)/2)/(2*l))*cos(xTheta+(((uDeltaSr - uDeltaSl)/l)/2));
       (-1/(l))...
          (1/(l))] ;


%ENDRM
 