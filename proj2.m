%Structure from motion

%% Parse the Data

matches = cell(5,6);

mdir = dir('./matching');
for i=3:length(mdir)
    fprintf([mdir(i).name, '\n']);
    data = dlmread(['./matching/', mdir(i).name], ' ', 1 ,0);
    for j=1:length(data)
        for k=1:data(j, 1)-1
           matches{i-2, data(j,7+(k-1)*3)} = [matches{i-2, data(j,7+(k-1)*3)} ; [data(j,5:6), data(j, (7+(k-1)*3+1):(7+(k-1)*3+2))]];
        end
    end
end






%% Reject Outliers with RANSAC

numimages = length(mdir)-2;
for i=1:numimages
    for j = i+1:numimages
        cur_m = matches{i, j};
        [inliers1 , inliers2, inds] = GetInliersRANSAC(cur_m(:,1:2), cur_m(:,3:4))
    end
end

