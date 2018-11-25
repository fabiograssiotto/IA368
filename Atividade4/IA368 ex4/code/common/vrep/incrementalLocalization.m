function [x_posterori, P_posterori] = incrementalLocalization(x, P, u, S, M, params, k, g, l)
% [x_posterori, P_posterori] = incrementalLocalization(x, P, u, S, R, M,
% k, l, g) returns the a posterori estimate of the state and its covariance,
% given the previous state estimate, control inputs, laser measurements and
% the map

C_TR = diag([repmat(0.1^2, 1, size(S, 2)) repmat(0.1^2, 1, size(S, 2))]);
[z, R, ans] = extractLinesPolar(S(1,:), S(2,:), C_TR, params);


%STARTRM
figure(2), cla, hold on;

%compute z_prior
z_prior=[];
% Now we need to loop through the map.
nM = size(M, 2); % number of map entries.

% Loop through all Map Entries
for i=1:nM
    % Call measurement function for each set of map entries.
    [h, ~]= measurementFunction(x, M(:,i));
    z_prior = [z_prior; h(1) h(2)];
end
% Just rearrange data at the end for plotting data.
z_prior = z_prior';

plot(z(1,:), z(2,:),'bo');
plot(z_prior(1,:), z_prior(2,:),'rx');
xlabel('angle [rad]'); ylabel('distance [m]')
legend('measurement','prior')
drawnow

% estimate robot pose
% Use the function from the previos step here.
[x_posterori, P_posterori] = filterStep(x, P, u, z, R, M, k, g, l);

%ENDRM
