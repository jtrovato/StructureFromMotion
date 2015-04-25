function [Cnew, Rnew] = PnPRANSAC(X, x, K)
%PnPRANSAC
%we assume that X and x are 3D and 2D correspondences one from the
%structure the other from a new image. 

% is the X specific matches to the points in x?

numpts = length(x);
maxiters = 100;
eps = 0.01;
max_inliers= 0;

for i=1:maxiters
    rinds = ceil(rand(8,1)*num_matches);
    [R,C] = LinearPNP(X(rinds, :), x(rinds, :), K);
    P = [R,C];
    error = zeros(numpts, 1);
    for j=1:numpts
        %calculate reprojection error
        error(i) = abs((x(j,1) - (P(1,:)*X')/ )^2 + ) %may not need to transpose X
    end
    mask = error < eps;
    if sum(mask) > max_inliers
        max_inliers = sum(mask);
        inliers = x(mask, :); %the points that match with the 3D points
    end
end
[Cnew,Rnew] = LinearPnP(X, inliers, K);

end

