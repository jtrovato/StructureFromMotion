function [C,R] = LinearPNP(X, x, K)
%Perspective n point
numpts = length(x);
if isempty(X) || isempty(x)
    disp('problem');
end
if size(X, 2) == 3
    X = [X, ones(numpts, 1)];
end
vec2skew = @(v) [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];

Xdiag = zeros(3,12);
A = [];
for i=1:numpts
    Xdiag(1, 1:4) = X(i,:);
    Xdiag(2, 5:8) = X(i,:);
    Xdiag(3, 9:12) = X(i,:);
    skew = vec2skew(x(i,:));
    A = [A; skew*Xdiag];

end

%least squares to solve for P
[~,~,V] = svd(A);
Pvec = V(:,end)/V(end,end);
P = [Pvec(1:4)';Pvec(5:8)';Pvec(9:12)'];

RC = K\P;
[U,~,V] = svd(RC(:,1:3));
R = U*V'*sign(det(U*V'));
C = -R'*RC(:,4);
end

