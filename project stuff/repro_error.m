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