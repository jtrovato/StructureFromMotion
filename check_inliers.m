function  check_inliers(M,Mx,My, im)
N = size(M,2);
for i = 1:N
    for j=i+1:N
        
        idx1 = find(M(:,i)==1);
        idx2 = find(M(:,j)==1);
        idx = intersect(idx1, idx2); %indexes of inliers
        if ~isempty(idx)
            x1 = [Mx(idx,i) My(idx,i)];
            x2 = [Mx(idx,j) My(idx,j)];
            showMatchedFeatures(im{i}, im{j}, x1, x2);
            title(['images ' num2str(i) ' and ' num2str(j)]);
            pause
        end
    end
end

end

