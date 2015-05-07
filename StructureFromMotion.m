function StructureFromMotion

%%
close all; 
K = [568.996140852 0 643.21055941;
     0 568.988362396 477.982801038;
     0 0 1];
nImages = 6;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
% Load images
disp('Loading Data');
for iImage = 1 : nImages
    str = sprintf('data/image%07d.bmp', iImage);
    im{iImage} = imread(str);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
% Load matching
Mx = []; My = []; M = []; colors = [];
for iImage = 1 : nImages-1;
    str = sprintf('matching/matching%d.txt', iImage);
    [mx, my, m c] = LoadMatching(str, iImage, nImages);
    Mx = [Mx;mx];
    My = [My;my];
    M = [M;m];
    colors = [colors; c];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
disp('Initializing with Two Images');
% Initialize 3D points, reconstruction index, and visibility matrix
X3D = zeros(size(M,1), 3);
ReconX = zeros(size(M,1),1);
V = zeros(size(M,1), nImages);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
% Exclude outliers using F matrix
for iImage1 = 1 : nImages-1
    for iImage2 = iImage1+1 : nImages
        idx1 = find(M(:,iImage1)==1);
        idx2 = find(M(:,iImage2)==1);
        idx = intersect(idx1, idx2);
        
        x1 = [Mx(idx,iImage1) My(idx,iImage1)];
        x2 = [Mx(idx,iImage2) My(idx,iImage2)];
        if size(x1,1) < 8
            continue;
        end
        [x1, x2, inlier] = GetInliersRANSAC(x1, x2); %inlier is a mask of where the inlies are. 
        M(idx(~inlier),iImage1) = 0; %set all non-inliers to 0
        fprintf('images %d and %d. Percent inliers = %f \n', iImage1, iImage2, sum(inlier)/length(inlier));
    end
end
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
% Set initial two frames
initialframe1 = 1;
initialframe2 = 4;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
% Get point index for two frames
idx1 = find(M(:,initialframe1)==1);
idx2 = find(M(:,initialframe2)==1);
idx = intersect(idx1, idx2); %indexes of inliers

x1 = [Mx(idx,initialframe1) My(idx,initialframe1)];
x2 = [Mx(idx,initialframe2) My(idx,initialframe2)];
%showMatchedFeatures(im{initialframe1}, im{initialframe2}, x1, x2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
% Get fundamental matrix and essential mtraix
F = EstimateFundamentalMatrix(x1, x2)
E = EssentialMatrixFromFundamentalMatrix(F,K);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
% Camera pose estimation
[Cset, Rset] = ExtractCameraPose(E);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
% Triangulation and pose disambiguation
figure();
for i = 1 : 4
    Xset{i} = LinearTriangulation(K, zeros(3,1), eye(3), Cset{i}, Rset{i}, x1, x2); 
    subplot(2,2,i);
    cs = repmat([0 0 1], length(Xset{i}), 1);
    visualizeStructure(Xset{i}, {[0 0 0]', Cset{i}}, {eye(3), Rset{i}}, cs);
end
pause(0.025);
[C, R, X] = DisambiguateCameraPose(Cset, Rset, Xset);
ptmask = prunePoints(X);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
% Nonlinear triangulation
disp('Nonlinear Triangulation');
X = NonlinearTriangulation(K, zeros(3,1), eye(3), C, R, x1(ptmask,:), x2(ptmask, :), X(ptmask, :));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
% Set reconstructed frame
r_idx = [initialframe1, initialframe2];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
% Set camera pose
Cr_set = cell(2,1);
Rr_set = cell(2,1);
Cr_set{1} = zeros(3,1);
Rr_set{1} = eye(3,3);
Cr_set{2} = C;
Rr_set{2} = R;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
% Set points and visibility matrix
X3D(idx(ptmask),:) = X;
ReconX(idx(ptmask)) = 1;
V(idx(ptmask), initialframe1) = 1;
V(idx(ptmask), initialframe2) = 1;

figure()
visualizeStructure(X, Cr_set, Rr_set, colors(ReconX == 1, :));
pause(0.025);
%pause

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
% Add images
fprintf('Adding the other images \n');
for iImage = 1 : nImages
    if any(r_idx==iImage)
        continue; 
    else
        fprintf(['adding image ' num2str(iImage) '\n']);
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
    % Get 2D-3D correspondences
    idx1 = find(ReconX==1); %points in the strucure 
    idx2 = find(M(:,iImage)==1); %points in the new image
    idx = intersect(idx1, idx2); %the points in both
    PNPpoints = length(idx)
    if length(idx) < 6
        continue;
    end
    
    X = X3D(idx,:);
    x = [Mx(idx,iImage) My(idx,iImage)];
    %figure();
    %visualizeStructure(X, Cr_set, Rr_set, colors(idx));
    %pause
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
    % Run PnP
    fprintf(' \tLinear PnP \n');
    [C, R, inds] = PnPRANSAC(X, x, K, im{iImage});
    
    fprintf('\tNonlinear PnP\n');
    [C, R] = NonlinearPnP(X(inds,:), x(inds,:), K, C, R, im{iImage});
    %figure()
    %visualizeStructure(X(mask,:), {C}, {R}, colors(idx(mask)));
    %pause
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
    % Set camera poses and reconstructed frame index
    Cr_set{end+1} = C;
    Rr_set{end+1} = R;
    r_idx(end+1) = iImage;
    V(idx, iImage) = 1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
    % Triangulation for additional points
    fprintf('\tAdding more points\n');
    for iImage1 = 1 : length(r_idx)-1
        idx1 = find(ReconX~=1);
        idx2 = find(M(:,r_idx(iImage1))==1);
        idx3 = find(M(:,iImage)==1);
        idx = intersect(idx1, idx2);
        idx = intersect(idx, idx3);
        x1 = [Mx(idx,r_idx(iImage1)) My(idx,r_idx(iImage1))];
        x2 = [Mx(idx,iImage) My(idx,iImage)];
        X = LinearTriangulation(K, Cr_set{iImage1}, Rr_set{iImage1}, C, R, x1, x2);
        fprintf('\tNonlinear Triangulation\n');
        ptmask = prunePoints(X);
        X = NonlinearTriangulation(K, Cr_set{iImage1}, Rr_set{iImage1}, C, R,  x1(ptmask,:), x2(ptmask, :), X(ptmask, :));
        
        X3D(idx(ptmask),:) = X;
        ReconX(idx(ptmask)) = 1;
        V(idx(ptmask), initialframe1) = 1;
        V(idx(ptmask), initialframe2) = 1;
        
    end    
    figure();
    visualizeStructure(X3D(ReconX == 1, :), Cr_set, Rr_set, colors(ReconX==1));
    pause
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
    % Set visibiltiy and measurements for bundle adjustment
    V_bundle = V(:,r_idx);
    Mx_bundle = Mx(:,r_idx);
    My_bundle = My(:,r_idx);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
    % Run bundle adjustment
    %disp('Bundle adjustment');
    %[Cset, Rset, X] = BundleAdjustment(K, Cr_set, Rr_set, X3D, ReconX, V_bundle, Mx_bundle, My_bundle);
end