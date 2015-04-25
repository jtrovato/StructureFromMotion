function F = EstimateFundamentalMatrix(x1 , x2)
% Calculate the fundamental matrix
M = kron(x2, [1,1,1]).*[x1, x1, x1];
[~,~,V] = svd(M);
    
F = reshape(V(:,9), 3, 3)';
[FU, FD, FV] = svd(F);
FD(3,3) = 0;
F = FU*FD*FV';

end


