function [Cnew, Rnew] = PnPRANSAC(X, x, K, I)
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
maxiters = 1000;
eps = 5;
max_inliers= 0;
figure();
for i=1:maxiters
    rinds = ceil(rand(8,1)*numpts);
    [C,R] = LinearPNP(X(rinds, :), x(rinds, :), K);
    P = K*R*[eye(3),-C];
    %calculate reprojection error
    proj = bsxfun(@rdivide, P(1:2, :)*X', P(3,:)*X'); %[2xN] / [1x4]x[4xN]
    error = sum((x(:, 1:2) - proj').^2, 2);
%     imshow(I); hold on;
%     plot(proj(1,:), proj(2,:), 'rx');
%     plot(x(:,1), x(:,2), 'g+');
%     pause
    mask = error < eps;
    %fprintf('num inliers %d \n', sum(mask));
    if sum(mask) > max_inliers
        best_mask = mask;
        max_inliers = sum(mask)
        inliers = x(mask, :); %the points that match with the 3D points
    end
end
[Cnew,Rnew] = LinearPNP(X(best_mask, :), x(best_mask, :), K);

end

