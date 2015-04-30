function [] = visualizeStructure(points)
%figure();
R = [0 0 1; -1 0 0; 0 -1 0];
points_r = (R*points')';
%points_r(norm(points_r) > 30) = [];
showPointCloud(points_r(:,1), points_r(:,2), points_r(:,3));

end

