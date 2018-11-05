%---------------------------------------------------------------------
% This function computes the parameters (r, alpha) of a line passing
% through input points that minimize the total-least-square error.
%
% Input:   XY - [2,N] : Input points
%
% Output:  alpha, r: paramters of the fitted line

function [alpha, r] = fitLine(XY)
% Compute the centroid of the point set (xmw, ymw) considering that
% the centroid of a finite set of points can be computed as
% the arithmetic mean of each coordinate of the points.

% XY(1,:) contains x position of the points
% XY(2,:) contains y position of the points


    X = XY(1,:);
    Y = XY(2,:);
    
    xc = mean(X);
    yc = mean(Y);
    
    % compute parameter alpha (see exercise pages)
    s = 0;
    for i = 1:length(X)
        s = s + (X(i) - xc)*(Y(i) - yc);
    end
    nom   = -2*s;
    
    denom = 0;
    for i = 1:length(X)
        denom = denom + ((Y(i) - yc)^2 - (X(i) - xc)^2); 
    end
    
    alpha = atan2(nom,denom)/2;

    % compute parameter r (see exercise pages)
    % (xc,yc) must lie in the line we are fitting.
    r = xc*cos(alpha) + yc*sin(alpha);


% Eliminate negative radii
if r < 0,
    alpha = alpha + pi;
    if alpha > pi, alpha = alpha - 2 * pi; end
    r = -r;
end

end
