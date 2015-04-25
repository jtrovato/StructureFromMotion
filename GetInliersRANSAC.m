function [inliers1, inliers2, idx] = GetInliersRANSAC(points1, points2)
%use ransac to get inlier point between two sets of matched features

maxiters = 100;
num_matches = length(points1);
max_inliers = 0;
eps = 0.01;

% augment the pixel points
points1 = [points1 , ones(num_matches, 1)];
points2 = [points2 , ones(num_matches, 1)];

for i=1:maxiters
    %choose 8 feature matches
    rinds = ceil(rand(8,1)*num_matches);
    %rinds = randsample(1:num_matches,8);
    F = EstimateFundamentalMatrix(points1(rinds, :), points2(rinds, :));
    comp = zeros(num_matches, 1);
    for j=1:num_matches
        comp(j) = abs(points2(j,:)*F*points1(j,:)');
    end
    mask = comp < eps;
    if sum(mask) > max_inliers
        max_inliers = sum(mask);
        inliers1 = points1(mask, :);
        inliers2 = points2(mask, :);
        idx= find(mask);
    end
end


end

