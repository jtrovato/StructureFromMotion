function X = NonlinearTriangulation(K, C1, R1, C2, R2, x1, x2, X0)
%refine 3D triangulation points. itial guess is X0. uses nonlinear
%optimization to minimize reprojection error.

    %options
    opts = optimoptions('lsqnonlin', 'Display', 'iter');
    X = lsqnonlin(@objective, X0, opts);


    function J = objective(P1, P2, X)
    %cost function to be minimized
    Xaug = [X, ones(1, size(X,1))];
    N11 = bsxfun(P1(1,:), Xaug);
    N12 = bsxfun(P1(2,:), Xaug);
    D1 = bsxfun(P1(3,:),Xaug);
    
    uproj1 = N11/D1;
    vproj1 = N12/D1;
    J1 = sum(sum((X(:, 1:2) - [uproj1, vproj1]).^2));
    
    N21 = bsxfun(P2(1,:), Xaug);
    N22 = bsxfun(P2(2,:), Xaug);
    D2 = bsxfun(P2(3,:),Xaug);
    
    uproj2 = N21/D2;
    vproj2 = N22/D2;
    J2 = sum(sum((X(:, 1:2) - [uproj2, vproj2]).^2));
    
    J = J1+J2;
    end

end

