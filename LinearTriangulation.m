function X = LinearTriangulation(K, C1, R1, C2, R2, points1, points2)
%triangulates points from two images in 3D space
P1 = K*R1*[eye(3), -C1];
P2 = K*R2*[eye(3), -C2];
vec2skew = @(v) [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
numpts = length(points1);

points1 = [points1 , ones(size(points1, 1), 1)];
points2 = [points2 , ones(size(points2, 1), 1)];

X = zeros(numpts, 3);
for i=1:numpts
    skew1 = vec2skew(points1(i,:)); 
    skew2 = vec2skew(points2(i,:));

    A = [skew1*P1; skew2*P2];
    [~,~,V] = svd(A);
    xrow = V(:,end)/V(end,end);
    X(i, :) = xrow(1:3)';
end


end

