function ptmask = prunePoints(X)
%X and idx need to be the same size
ptmask = X(:,1) == X(:,1);
%remove point far away
distmask = abs(X(:,1)) > 70 | abs(X(:,2)) > 70  | abs(X(:,3)) > 70 ;
ptmask(distmask) = 0;
%remove points behind camera
cammask = X(:,3) < 0;
ptmask(cammask) = 0;

end

