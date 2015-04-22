function [inliers1, inliers2, idx] = GetInliersRANSAC(points1, points2)
%use ransac to get inlier point between two sets of matched features

maxiters = 1000;
num_matches = length(points1);
max_inliers = 0;
eps = 0.5;

points1 = [points1 , ones(size(points1, 1), 1)];
points2 = [points2 , ones(size(points2, 1), 1)];

for i=1:maxiters
    %choose 8 feature matches
    rinds = ceil(rand(8,1)*num_matches);    
    F = EstimateFundamentalMatrix(points1(rinds, :), points2(rinds, :));
    idx = find(sum((points2'.*(F*points1')).^2) < eps);
    if length(idx) > max_inliers
        max_inliers = length(idx);
        inliers1 = points1(idx, :);
        inliers2 = points2(idx, :);
    end
end


end

