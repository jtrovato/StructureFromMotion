function X = NonlinearTriangulation(K, C1, R1, C2, R2, x1, x2, X0)
%refine 3D triangulation points. itial guess is X0. uses nonlinear
%optimization to minimize reprojection error.
    if size(x1,2) == 2 || size(x2,2)==2
        x1 = [x1, ones(size(x1,1),1)];
        x2 = [x2, ones(size(x2,1),1)];
    end
    numpts = length(X0);
    P1 = K*R1*[eye(3), -C1];
    P2 = K*R2*[eye(3), -C2];
    %options
    opts = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', 'TolX', 1e-64, 'TolFun', 1e-64, 'MaxFunEvals', 1e64, 'MaxIter', 1e64, 'Display', 'off');
    
    X = zeros(size(X0));
    %each 3D point is a different problem
    for i = 1:numpts
        X(i,:) = lsqnonlin(@repro_error, X0(i,:), [], [], opts, P1, P2, x1(i,:), x2(i,:));
    end
    

    function J = repro_error(X, P1, P2, x1, x2)
    %cost function to be minimized
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

