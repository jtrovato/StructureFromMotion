function [C,R] = LinearPNP(X, x, K)
%Perspective n point
numpts = length(X);
X = [X, ones(numpts, 1)];
vec2skew = @(v) [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
Xdiag = zeros(3,12);
Xdiag(1, 1:4) = X(1,:);
Xdiag(2, 5:8) = X(2,:);
Xdiag(3, 9:12) = X(3,:);

A = [];
for i=1:numpts
    skew = vec2skew(x(i,:));
    A = [A; skew*Xdiag];

end
[~,~,V] = svd(A, 0);
Pvec = V(:,end)/V(end,end);
P = [Pvec(1:4)';Pvec(5:8)';Pvec(9:12)'];

%P = K*[R -C]
RC = K\P;
R = RC(:,1:3);
C = RC(:,4);
end

