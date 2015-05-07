function plot_projections(I, R, C, K, X, x)
P = K*R*[eye(3),-C];
%calculate reprojection error
proj = bsxfun(@rdivide, P(1:2, :)*X', P(3,:)*X'); %[2xN] / [1x4]x[4xN]
imshow(I); hold on;
plot(proj(1,:), proj(2,:), 'rx');
plot(x(:,1), x(:,2), 'g+');
legend('projection', 'image features');
title('Reprojection error after Linear PNP');
pause(0.025);

end

