%Structure from motion

%% Parse the Data
fprintf('Loading the Data\n');
K = [568.996140852 0 643.21055941;
     0 568.988362396 477.982801038;
     0 0 1];

matches = cell(5,6);

mdir = dir('./matching');
numims = length(mdir)-2;
for i=3:length(mdir)
    %fprintf([mdir(i).name, '\n']);
    data = dlmread(['./matching/', mdir(i).name], ' ', 1 ,0);
    for j=1:length(data)
        for k=1:data(j, 1)-1
           matches{i-2, data(j,7+(k-1)*3)} = [matches{i-2, data(j,7+(k-1)*3)} ; [data(j,5:6), data(j, (7+(k-1)*3+1):(7+(k-1)*3+2))]];
        end
    end
end

% %% Check Matches
% figure();
% Ii = imread('image0000002.bmp');
% Ij = imread('image0000003.bmp');
% m = matches{2,3};
% for i=1:length(m)
%     subplot(1,2,1);
%     imshow(Ii); hold on;
%     plot(m(i,1), m(i,2), 'rx'); hold off;
%     subplot(1,2,2);
%     imshow(Ij); hold on;
%     plot(m(i,3), m(i,4), 'rx'); hold off;
%     pause
% end

%% Reject Outliers with RANSAC
fprintf('RANSAC\n');
inliers = cell(5,6);
numimages = 6;
for i=1:numimages
    for j = i+1:numimages
        if ~isempty(matches{i, j})
            cur_m = matches{i, j};
            [inliers1 , inliers2, inds] = GetInliersRANSAC(cur_m(:,1:2), cur_m(:,3:4));
            inliers{i,j} = [inliers1, inliers2];
        end
    end
end

%% Visualize RANSAC
verbose = 1;
if verbose
    for i=1:numimages-1
        Ii = imread(['data/image000000', num2str(i), '.bmp']);
        for j = i+1:numimages

            if ~isempty(inliers{i, j})
                Ij = imread(['data/image000000', num2str(j), '.bmp']);
                cur_inliers = inliers{i,j};
                cur_matches = matches{i,j};
%                 subplot(2,2,1);
%                  imshow(Ii); hold on;
%                 plot(cur_matches(:,1), cur_matches(:,2), 'rx');
%                 title(['image ', num2str(i)]);
%                 subplot(2,2,2);
%                 imshow(Ij); hold on;
%                 plot(cur_matches(:,3), cur_matches(:,4), 'gx');
%                 title(['image ', num2str(j)]);
%                 subplot(2,2,3);
%                 imshow(Ii); hold on;
%                 plot(cur_inliers(:,1), cur_inliers(:,2), 'rx');
%                 subplot(2,2,4);
%                 imshow(Ij); hold on;
%                 plot(cur_inliers(:,4), cur_inliers(:,5), 'gx');
                showMatchedFeatures(Ii, Ij, cur_inliers(:,1:2), cur_inliers(:,4:5))
                title(['percent inliers = ' num2str(length(cur_inliers)/length(cur_matches)*100), '%']);
                
%                 subplot(2,1,1);
%                 showMatchedFeatures(Ii, Ij, cur_matches(:,1:2), cur_matches(:,3:4), 'montage');
%                 subplot(2,1,2);
%                 showMatchedFeatures(Ii, Ij, cur_inliers(:,1:2), cur_inliers(:,4:5), 'montage');
                pause
            end
        end
    end
end

%% Estimate initial C and R between first two images
fprintf('Calculating Initial Guess\n');
Ii = imread('image0000002.bmp');
Ij = imread('image0000003.bmp');
inliers12 = inliers{2,3};
x1 = inliers12(:,1:3);
x2 = inliers12(:,4:6);
F = EstimateFundamentalMatrix(x1, x2);
E = EssentialMatrixFromFundamentalMatrix(F, K);
[Cset, Rset] = ExtractCameraPose(E);

Xset = cell(4,1);
for i=1:4
    %start the transformation with the identity and build from there
    Xset{i} = LinearTriangulation(K, [0;0;0], eye(3), Cset{i}, Rset{i}, x1, x2);
    verbose = 1;
    if verbose
        points = Xset{i};
        subplot(2,2,i);
        showPointCloud(points(:,1), points(:,2), points(:,3));
    end
end
fprintf('Refine Initial Guess\n');
[C0,R0,X0] = DisambiguateCameraPose(Cset, Rset, Xset); %initial structure from first two images is in X0. initial transform is [R, -C]
%X = NonlinearTriangulation(K, zeros(3,1), eye(3), C, R, x1, x2, X0);
Cset = {C0};
Rset = {R0};

%% Plot the 3D points
figure();
R = [0 0 1; -1 0 0; 0 -1 0];
points_r = (R*points')';
showPointCloud(points_r(:,1), points_r(:,2), points_r(:,3));
hold on;
x = xlim;
y = ylim;
[gx,gy] = meshgrid(x(1):x(2), y(1):y(2));
%mesh(gx, gy, zeros(size(gy)));

%% Register Cameras and 3D  Points from Other Images
% after the first two images, start with the third and registers the rest
for i=3:numims 
    [Cnew, Rnew] = PnPRANSAC(X,x,k);
    [Cnew, Rnew] = NonLinearPnP(X,x,k,Cnew,Rnew);
    %take the union of the new tranaltion and rotations ---> isn't it
    %pretty unlikely that the same C or R will be in the set
    
    Cset{end+1} = Cnew;
    Rset{end+1} = Rnew;
    X0 = LinearTriangulation(K, C0, R0, Cnew, Rnew, x1, x2);
    Xnew = NonlinearTriangulation(K, C0, R0, Cnew, Rnew, x1, x2, X0); 
    
    X = [X; Xnew];
    %create traj
    
    V = BuildVisibilityMatrix(traj);
    [Cset, Rset, X] = BundleAdjustment(Cset, Rset, X, K, traj, V);
end


