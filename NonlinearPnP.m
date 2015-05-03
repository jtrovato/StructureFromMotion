function [C, R] = NonlinearPnP(X, x, K, C0, R0)
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
    opts = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', 'TolX', 1e-5, 'TolFun', 1e-5, 'MaxFunEvals', 1e64, 'MaxIter', 1e64, 'Display', 'off');
    
    qC = lsqnonlin(@repro_error, qC0, [], [], opts, X, x, K);
    q = qC(1:4);
    R = q2R(q);
    C = qC(5:7);
    

    function J = repro_error(qC, X, x, K)
    %cost function to be minimized
    numpts = length(x);
    qcur = qC(1:4);
    Ccur = qC(5:7);
    Rcur = q2R(qcur);
    P = K*Rcur*[eye(3),-Ccur];
    
    J = 0;
    for j=1:numpts
        %calculate reprojection error
        J = J + abs((x(j,1) - (P(1,:)*X')/(P(3,:)*X') )^2 + (x(j,2) - (P(2,:)*X')/(P(3,:)*X') )^2); %may not need to transpose X
    end
    
    end

end

