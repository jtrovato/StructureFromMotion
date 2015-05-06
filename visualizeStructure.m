function [] = visualizeStructure(points, Cset, Rset, colors)
%figure();
R = [0 0 1; -1 0 0; 0 -1 0];
%R = eye(3);
%points(points(:,3) < 0, :) = [];
%x(abs(points(:,3)) > 50, :) = [];
%points(abs(points(:,3)) > 50, :) = [];
N = length(points);
points_r = (R*points')';

showPointCloud(points_r(:,1), points_r(:,2), points_r(:,3), colors);
hold on;
for i=1:length(Cset)
    unit = [0 0 5]';
    unit_r = Rset{i}'*R*unit;
    t = -Rset{i}*Cset{i};
    quiver3(Cset{i}(1), Cset{i}(2), Cset{i}(3), unit_r(1), unit_r(2), unit_r(3), 'MaxHeadSize', 0.5, 'LineWidth', 1);
    
end
hold off;
end

