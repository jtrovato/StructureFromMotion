function X = LinearTriangulation(K, C1, R1, C2, R2, points1, points2)
%triangulates points from two images in 3D space
P1 = K*R1*[eye(3), C1];
P2 = K*R2*[eye(3), C2];
vec2skew = @(v) [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];

X = zeros(3, numpts);
for i=1:num
    skew1 = vec2skew(points1(i,:)); 
    skew2 = vec2skew(points2(i,:));

    A = [skew1*P1; skew2*P2];
    [~,~,V] = svd(A);
    X(:, i) = V(:,end)/V(end,end);
end


end

