function [x_posteriori, P_posteriori] = filterStep(x, P, u, Z, R, M, k, g, l)
% [x_posteriori, P_posteriori] = filterStep(x, P, u, z, R, M, k, g, l)
% returns an a posteriori estimate of the state and its covariance

%STARTRM

uDeltaSl = u(1);
uDeltaSr = u(2);

% Q below is the Covariance of the noise associated to the motion model.
% propagate the state (p. 338) , here kr=kl=k

Q = [k*abs(uDeltaSr) 0; 
     0 k*abs(uDeltaSl)]; 

% From 1st activity
[x_priori, F_x, F_u] = transitionFunction(x,u,l);
P_priori = (F_x*P*(F_x')) + (F_u*Q*(F_u'));

if size(Z,2) == 0
    x_posteriori = x_priori;
    P_posteriori = P_priori;
    return;
end
    
[v, H, R] = associateMeasurements(x_priori, P_priori, Z, R, M, g);

y = reshape(v, [], 1);
H = reshape(permute(H, [1,3,2]), [], 3);
R = blockDiagonal(R);

% update state estimates (pp. 335)
S = (H*P_priori*(H'))+R;
K = P_priori*(H')*(inv(S));

x_posteriori = x_priori + K * y;
P_posteriori = (eye(size(P_priori)) - K*H) * P_priori;

%ENDRM
