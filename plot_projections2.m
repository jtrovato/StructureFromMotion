function plot_projections2(I, R, C, K, X1, X2, x)
if size(x,2) == 2
    x = [x, ones(size(x,1),1)];
end
if size(X1,2) == 3
    X1 = [X1, ones(size(X1,1),1)];
end
if size(X2,2) == 3
    X2 = [X2, ones(size(X2,1),1)];
end
P = K*R*[eye(3),-C];
%calculate reprojection error
proj1 = bsxfun(@rdivide, P(1:2, :)*X1', P(3,:)*X1'); %[2xN] / [1x4]x[4xN]
proj2 = bsxfun(@rdivide, P(1:2, :)*X2', P(3,:)*X2'); %[2xN] / [1x4]x[4xN]

imshow(I); hold on;
plot(proj1(1,:), proj1(2,:), 'bx');
plot(proj2(1,:), proj2(2,:), 'rx');
plot(x(:,1), x(:,2), 'g+');
legend('linear tri projection', 'nonlinear tri projection', 'image features');
pause(0.025);

end