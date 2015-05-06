function [] = visualizeStructure(points, Cset, Rset, I, x)
%figure();
R = [0 0 1; -1 0 0; 0 -1 0];
%R = eye(3);
%points(points(:,3) < 0, :) = [];
%x(abs(points(:,3)) > 50, :) = [];
%points(abs(points(:,3)) > 50, :) = [];
N = length(points);
points_r = (R*points')';

sizex = size(x)
sizepts = size(points)


%create colors for 3D points
colors = zeros(N, 3);
Ir = I(:,:,1);
colors(:,1) = Ir(sub2ind(size(Ir), round(x(:,2)), round(x(:,1))));
Ig = I(:,:,2);
colors(:,2) = Ig(sub2ind(size(Ig), round(x(:,2)), round(x(:,1))));
Ib = I(:,:,3);
colors(:,3) = Ib(sub2ind(size(Ib), round(x(:,2)), round(x(:,1))));
colors = double(colors)/255;

showPointCloud(points_r(:,1), points_r(:,2), points_r(:,3), colors);
hold on;
for i=1:length(Cset)
    unit = [0 0 5]';
    unit_r = Rset{i}*R*unit;
    quiver3(Cset{i}(1), Cset{i}(2), Cset{i}(3), unit_r(1), unit_r(2), unit_r(3));
    
end
hold off;
end

