function plot_projections3(I, R1, C1, R2, C2, K, X, x)
P1 = K*R1*[eye(3),-C1];
P2 = K*R2*[eye(3),-C2];

%calculate reprojection error
proj1 = bsxfun(@rdivide, P1(1:2, :)*X', P1(3,:)*X'); %[2xN] / [1x4]x[4xN]
proj2 = bsxfun(@rdivide, P2(1:2, :)*X', P2(3,:)*X'); %[2xN] / [1x4]x[4xN]

imshow(I); hold on;
plot(proj1(1,:), proj1(2,:), 'bx');
plot(proj2(1,:), proj2(2,:), 'rx');
plot(x(:,1), x(:,2), 'g+');
legend('linear pnp projection', 'nonlinear pnp projection', 'image features');
pause(0.025);

end