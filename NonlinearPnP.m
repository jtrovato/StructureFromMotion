function [C, R] = NonlinearPnP(X, x, K, C0, R0, I)
% refine choices of C and R usinf nonlinear optimization
    if size(x,2) == 2
        x = [x, ones(size(x,1),1)];
    end
    if size(X,2) == 3
        X = [X, ones(size(X,1),1)];
    end
    % convert rotation matrix to quaternions
    q0 = R2q(R0);
    qC0 = [q0;C0];
    %nonlinear optimization
    opts = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', 'Display', 'off');
    
    qC = lsqnonlin(@repro_error, qC0, [], [], opts, X, x, K);
    q = qC(1:4);
    R = q2R(q);
    C = qC(5:7);
    
    Pnew = K*R*[eye(3),-C];
    %calculate reprojection error
%     figure();
%     proj = bsxfun(@rdivide, Pnew(1:2, :)*X', Pnew(3,:)*X'); %[2xN] / [1x4]x[4xN]
%     imshow(I); hold on;
%     plot(proj(1,:), proj(2,:), 'rx');
%     plot(x(:,1), x(:,2), 'g+');
%     title('Reprojection error after Nonlinear PNP');
%     pause(0.05);

end

function J = repro_error(qC, X, x, K)
    %cost function to be minimized
    numpts = length(x);
    qcur = qC(1:4);
    Ccur = qC(5:7);
    Rcur = q2R(qcur);
    P = K*Rcur*[eye(3),-Ccur];
    
    projection = bsxfun(@rdivide, P(1:2, :)*X', P(3,:)*X');
    J = (x(:, 1:2)' - projection)';
end

