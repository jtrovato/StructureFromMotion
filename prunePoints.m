function ptmask = prunePoints(X)
%X and idx need to be the same size
ptmask = X(:,1) == X(:,1);
%remove point far away
distmask = abs(X(:,1)) > 50 | abs(X(:,2)) > 50  | abs(X(:,3)) > 50 ;
ptmask(distmask) = 0;
%remove points behind camera
cammask = X(:,3) < 0;
ptmask(cammask) = 0;

end

