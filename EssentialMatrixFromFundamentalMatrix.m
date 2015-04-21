function E = EssentialMatrixFromFundamentalMatrix(F, K)
E_noisy = K'*F*K;
[U, D, V] = svd(E_Noisy);
D = eye(3);
D(3,3) = 0;
E = U*D*V';

end

