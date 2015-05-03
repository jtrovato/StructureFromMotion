function [] = visualizeStructure(points, Cset, Rset)
%figure();
R = [0 0 1; -1 0 0; 0 -1 0];
%R = eye(3);
%points(points(:,3) < 0, :) = [];
points(abs(points(:,3)) > 50, :) = [];
points_r = (R*points')';


showPointCloud(points_r(:,1), points_r(:,2), points_r(:,3));
hold on;
for i=1:length(Cset)
    unit = [0 0 5]';
    unit_r = Rset{i}*R*unit;
    quiver3(Cset{i}(1), Cset{i}(2), Cset{i}(3), unit_r(1), unit_r(2), unit_r(3));
    
end
hold off;
end

