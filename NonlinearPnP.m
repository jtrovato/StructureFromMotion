function [C R] = NonlinearPnP(X, x, K, C0, R0)
% refine choices of C and R usinf nonlinear optimization
    if size(x,2) == 2
        x = [x, ones(size(x,1),1)];
    end
    if size(X,2) == 3
        X = [X, ones(size(X,1),1)];
    end
    % convert rotation matrix to quaternions

    %nonlinear optimization
    opts = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', 'TolX', 1e-5, 'TolFun', 1e-5, 'MaxFunEvals', 1e64, 'MaxIter', 1e64, 'Display', 'off');
    

    function J = repro_error(q, C, X, x, K)
    %cost function to be minimized
    P = K*[q,t];
    Xaug = [X, ones(size(X,1),1)];
    
    proj1 = P1(1:2,:)*Xaug'; %[2x4]x[4xN] = [2xN]
    proj1 = bsxfun(@rdivide, proj1, P1(3,:)*Xaug'); %[2xN] / [1x4]x[4xN]
    J1 = x1(:, 1:2) - proj1';
    
    proj2 = P2(1:2,:)*Xaug'; %[2x4]x[4xN] = [2xN]
    proj2 = bsxfun(@rdivide, proj2, P2(3,:)*Xaug'); %[2xN] / [1x4]x[4xN]
    J2 = x2(:, 1:2)- proj2';
    
    J = [J1,J2];
    end

end

