function F = EstimateFundamentalMatrix(x1 , x2)
% Calculate the fundamental matrix

if size(x1,2) == 2 || size(x2,2)==2
    x1 = [x1, ones(size(x1,1),1)];
    x2 = [x2, ones(size(x2,1),1)];
end

M = kron(x2, [1,1,1]).*[x1, x1, x1];
[~,~,V] = svd(M);
    
F = reshape(V(:,9), 3, 3)';
[FU, FD, FV] = svd(F);
FD(3,3) = 0;
F = FU*FD*FV';

end


