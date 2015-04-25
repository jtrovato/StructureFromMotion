function [C, R, X0] = DisambiguateCameraPose(Cset, Rset, Xset)
%given four camera poses and triangulated points for each, find the actual
%camera pose by checking the cheirality condition
%X should be 3xN

max_count = 0;


for i=1:4
    Ccur = Cset{i};
    Rcur = Rset{i};
    Xcur = Xset{i};
    
    numpts = size(Xcur, 1);
    count = 0;
    for j=1:numpts
        if Rcur(3,:)*(Xcur(j,:)'-Ccur) > 0 %cheirality condition %TODO debg is it a row or column of R and is it transposed
            count = count +1;
        end
    end
    
    if count > max_count
        max_count = count;
        C = Ccur;
        R = Rcur;
        X0 = Xcur;
        i
        max_count
    end
end
        
    
    
end

