%Structure from motion

%% Parse the Data
K = [568.996140852 0 643.21055941;
     0 568.988362396 477.982801038;
     0 0 1];

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

inliers = cell(5,6);
numimages = length(mdir)-2;
for i=1:numimages
    for j = i+1:numimages
        if ~isempty(matches{i, j})
            cur_m = matches{i, j};
            [inliers1 , inliers2, inds] = GetInliersRANSAC(cur_m(:,1:2), cur_m(:,3:4));
            inliers{i,j} = [inliers1, inliers2];
        end
    end
end

%% Estimate initial C and R
inliers12 = inliers{1,2};
x1 = [inliers12(:,1:2) , ones(size(inliers12, 1), 1)];
x2 = [inliers12(:,3:4) , ones(size(inliers12, 1), 1)];
F = EstimateFundamentalMatrix(x1, x2);
E = EssentialMatrixFromFundamentalMatrix(F, K);
[Cset, Rset] = ExtractCameraPose(E);

Xset = cell(4,1);
for i=1:4
    Xset{i} = LinearTriangulation(K, zeros(3,1), eye(3), Cset{i}, Rset{i}, inliers12(:,1:2), inliers12(:,3:4));
end

[C,R] = DisambiguateCameraPose(Cset, Rset, Xset);
points = NonlinearTriangulation(K, zeros(3,1), eye(3), C, inliers12(:,1:2), inliers12(:,3:4));
Cset = cell(C);
Rset = cell(R);

%% Register Cameras and 3D  Points from Other Images
for i=1:3
    
end


