function [] = visualizeStructure(points, Cset, Rset, colors)
%figure();
R = [0 0 1; -1 0 0; 0 -1 0];
%R = eye(3);
ptmask = prunePoints(points);
points= points(ptmask, :);
colors = colors(ptmask, :);
N = length(points);
points_r = (R*points')';

showPointCloud(points_r(:,1), points_r(:,2), points_r(:,3), colors);
hold on;
for i=1:length(Cset)
    unit = [0 0 2.5]';
    unit_r = R*Rset{i}'*unit;
    t = -Rset{i}*Cset{i};
    Cr = R*Cset{i};
    quiver3(Cr(1), Cr(2), Cr(3), unit_r(1), unit_r(2), unit_r(3), 'MaxHeadSize', 0.5, 'LineWidth', 2);
    
end
hold off;
end

