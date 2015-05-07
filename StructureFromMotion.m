function StructureFromMotion
addpath('sba-1.6/matlab');
addpath('SBA_example');

report = 1;
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
% Initialize 3D points, reconstruction index, and visibility matrix
X3D = zeros(size(M,1), 3);
ReconX = zeros(size(M,1),1);
V = zeros(size(M,1), nImages);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
fprintf('Rejecting outliers with RANSAC\n');
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
disp('Initializing with Two Images');
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
if report
    figure(1);
    title(['RANSAC matches. Images ', num2str(initialframe1), ' and ', num2str(initialframe1)]);
    showMatchedFeatures(im{initialframe1}, im{initialframe2}, x1, x2);
    figure(2);
    title(['Another view of RANSAC matches. Images ', num2str(initialframe1), ' and ', num2str(initialframe1)]);
    showMatchedFeatures(im{initialframe1}, im{initialframe2}, x1, x2, 'montage');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
% Get fundamental matrix and essential mtraix
F = EstimateFundamentalMatrix(x1, x2);
if report
    fprintf('The Fundamental Matrix between images %d and %d: ', initialframe1, initialframe2);
    F
end
E = EssentialMatrixFromFundamentalMatrix(F,K);
if report
    fprintf('The Essential Matrix between images %d and %d: ', initialframe1, initialframe2);
    E
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
% Camera pose estimation
[Cset, Rset] = ExtractCameraPose(E);
if report
    fprintf('Possible Camera Poses\n');
    for i=1:length(Cset)
        sprintf('Pose %d \n:, i');
        Cset{i}
        Rset{i}
    end
end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
% Triangulation and pose disambiguation
if report
    figure(3); %just setting up the figure
end
for i = 1 : 4
    Xset{i} = LinearTriangulation(K, zeros(3,1), eye(3), Cset{i}, Rset{i}, x1, x2); 
    if report
        subplot(2,2,i);
        cs = repmat([0 0 1], length(Xset{i}), 1);
        title(['Camera Configuration ', num2str(i)]);
        visualizeStructure(Xset{i}, {[0 0 0]', Cset{i}}, {eye(3), Rset{i}}, cs);
    end
end
pause(0.025);
[C, R, X] = DisambiguateCameraPose(Cset, Rset, Xset);
Xlintri = X;
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

fprintf('Using %d initial points in the reconstruction\n\n', sum(idx(ptmask) > 0));
camera_colors = colormap(jet(nImages));

%plot the non-linear and linear triangulation in differnt colors
if report
    %3D
    figure(4);
    visualizeStructure(Xlintri, {Cr_set{1}}, {Rr_set{1}}, repmat([1 0 0], length(Xlintri), 1));
    hold on;
    visualizeStructure(X, {Cr_set{2}}, {Rr_set{2}}, repmat([0 0 1], length(X), 1));
    hold off;
    title('Linear vs Nonlinear Triangualtion');
    legend('Linear Triangulation', 'Nonlinear Triangulation');
    %projections
    figure(5);
    plot_projections2(im{1}, Rr_set{1}, Cr_set{1}, K, Xlintri, X, x1);
    title('Reprojection of linear and nonlinear triangluation');
    
end

if report
    figure(8)
    visualizeStructure(X, Cr_set, Rr_set, repmat(camera_colors(1, :), length(X), 1));
    hold on;
    pause(0.025);
end
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
    if report
        fprintf('Camera pose between camera %d \n', iImage);
        Clin = C
        Rlin = R
        fprintf('Kept %d inliers of %d', length(inds), length(X));
    end
    
    fprintf('\tNonlinear PnP\n');
    [C, R] = NonlinearPnP(X(inds,:), x(inds,:), K, C, R, im{iImage});

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
    % Set camera poses and reconstructed frame index
    Cr_set{end+1} = C;
    Rr_set{end+1} = R;
    r_idx(end+1) = iImage;
    V(idx, iImage) = 1;
    
    %plot the output of PNP
    if report && iImage == 2
        %3D
        figure(6);
        title(['Adding Camera ', num2str(iImage), ]);
        visualizeStructure(X3D(ReconX == 1, :), Cr_set, Rr_set, colors(ReconX==1, :));
        pause(0.25);
        %projection
        figure(7);
        plot_projections3(im{1}, Rlin, Clin, R, C, K, X(inds,:), x);
        title('Reprojection of linear and nonlinear pnp points'); 
    end
    
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
        
        ptmask = prunePoints(X);
        X3D(idx(ptmask),:) = X(ptmask, :);
        ReconX(idx(ptmask)) = 1;
        V(idx(ptmask), initialframe1) = 1;
        V(idx(ptmask), initialframe2) = 1;
        fprintf('\tAdding %d points\n', sum(idx(ptmask)>0));
        
    end    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
    % Set visibiltiy and measurements for bundle adjustment
    V_bundle = V(:,r_idx);
    Mx_bundle = Mx(:,r_idx);
    My_bundle = My(:,r_idx);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
    % Run bundle adjustment
    %disp('Bundle adjustment');
    %[Cset, Rset, X] = BundleAdjustment(K, Cr_set, Rr_set, X3D, ReconX, V_bundle, Mx_bundle, My_bundle);
    
    
    if report
        %plot each iteration in a different color
        figure(8);
        hold on;
        inds = V(:,iImage) == 1;
        visualizeStructure(X3D(inds,:), {C}, {R}, repmat(camera_colors(iImage, :), length(inds), 1));
        legend('cam1 and cam4', 'cam2', 'cam3', 'cam5', 'cam6');
    else
        figure();
        visualizeStructure(X3D(ReconX == 1, :), Cr_set, Rr_set, colors(ReconX==1, :));
        %pause
    end
end
%output final bundle
if report
    figure;
    visualizeStructure(X3D(ReconX == 1, :), Cr_set, Rr_set, colors(ReconX==1, :));
    title('The Final Reconstruction');
    
    for i = 1:6
        figure();
        plot_projections(im{i}, Rr_set{i}, Cr_set{i}, K, X3D(ReconX ==1, :), [Mx(ReconX ==1,i) My(ReconX ==1,i)]);
    end
    
    sprintf('Reconstructed %d points!\n', sum(ReconX == 1));
    
end
