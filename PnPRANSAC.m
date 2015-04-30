function [Cnew, Rnew] = PnPRANSAC(X, x, K)
%PnPRANSAC
%we assume that X and x are 3D and 2D correspondences one from the
%structure the other from a new image. 

% is the X specific matches to the points in x?
if size(x,2) == 2
    x = [x, ones(size(x,1),1)];
end
if size(X,2) == 3
    X = [X, ones(size(X,1),1)];
end

numpts = length(x);
maxiters = 100;
eps = 0.01;
max_inliers= 0;

for i=1:maxiters
    rinds = ceil(rand(8,1)*numpts);
    [R,C] = LinearPNP(X(rinds, :), x(rinds, :), K);
    P = [R,C];
    error = zeros(numpts, 1);
    for j=1:numpts
        %calculate reprojection error
        error(i) = abs((x(j,1) - (P(1,:)*X')/(P(3,:)*X') )^2 + (x(j,2) - (P(2,:)*X')/(P(3,:)*X') )^2); %may not need to transpose X
    end
    mask = error < eps;
    if sum(mask) > max_inliers
        max_inliers = sum(mask);
        inliers = x(mask, :); %the points that match with the 3D points
    end
end
[Cnew,Rnew] = LinearPNP(X(mask, :), inliers, K);

end

