function [C, R] = DisambiguateCameraPose(Cset, Rset, Xset)
%given four camera poses and triangulated points for each, find the actual
%camera pose by checking the cheirality condition
%X should be 3xN

max_count = 0;


for i=1:4
    Ccur = Cset(:,:,i);
    Rcur = Rset(:,:,i);
    Xcur = Xcur(:,:,i);
    
    numpts = size(Xcur, 2);
    count = 0;
    for j=1:numpts
        if R(3,:)'*(Xcur(:,j)-Ccur)
            count = count +1;
        end
    end
    
    if count > max_count
        max_count = count;
        C = Ccur;
        R = Rcur;
    end
end
        
    
    
end

