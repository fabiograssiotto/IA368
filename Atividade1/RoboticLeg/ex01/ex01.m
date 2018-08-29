clc; clear all;

syms alpha beta gamma real

% write down the rotation matrices using the symbolic parameters alpha, beta, gamma
R_B1 = [ 1 0 0; 0 cos(alpha) -sin(alpha); 0 sin(alpha) cos(alpha)]; % x axis
R_12 = [ cos(beta) 0 sin(beta); 0 1 0; -sin(beta) 0 cos(beta)]; % y axis
R_23 = [ cos(gamma) 0 sin(gamma); 0 1 0; -sin(gamma) 0 cos(gamma)]; % y axis again

% answers
valid

