function F = EstimateFundamentalMatrix(points1 , points2)
% Calculate the fundamental matrix

points1 = [points1 , ones(size(points1, 1), 1)];
points2 = [points2 , ones(size(points2, 1), 1)];
M = kron(points2, [1,1,1]).*[points1, points1, points1];
[~,~,V] = svd(M);
F = reshape(V(:,9), 3, 3)';
[FU, FD, FV] = svd(F);
FD(3,3) = 0;
F = FU*FD*FV;

end


