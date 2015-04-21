function [inliers1, inliers2, idx] = GetInliersRANSAC(points1, points2)
%use ransac to get inlier point between two sets of matched features

maxiters = 10000;
num_matches = length(points1);
max_inliers = 0;
eps = 1e-5;

for i=1:maxiters
    %choose 8 feature matches
    rinds = ceil(rand(8,1)*num_matches);    
    F = EstimateFundamentalMatrix(points1(rinds, :), points2(rinds, :));
    idx = find(sum((points2'.*F*points1').^2) < eps);
    if length(idx) > max_inliers
        max_inliers = length(idx);
        inliers1 = points1(idx, :);
        inliers2 = points2(idx, :);
    end
end


end

