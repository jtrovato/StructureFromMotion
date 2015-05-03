function [C, R, X0] = DisambiguateCameraPose(Cset, Rset, Xset)
%given four camera poses and triangulated points for each, find the actual
%camera pose by checking the cheirality condition
%X should be 3xN

max_count = 0;


for i=1:4
    Ccur = Cset{i};
    Rcur = Rset{i};
    Xcur = Xset{i};
    
    xmc = bsxfun(@minus, Xcur', Ccur); 
    cheiral = Rcur(:,3)'*xmc;
    count = sum(cheiral > 0);
    
    if count > max_count
        ind = i;
        max_count = count;
        C = Ccur;
        R = Rcur;
        X0 = Xcur;
    end
    
end
fprintf('choosing camera pose %d \n', ind);    
    
    
end

