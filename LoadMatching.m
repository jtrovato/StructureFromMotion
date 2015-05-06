function [Mx My V, colors] = LoadMatching(filename, image_idx, nImages)

fid = fopen(filename);
fscanf(fid, '%s', 1);
n = fscanf(fid, '%d', 1);
Mx = zeros(n, nImages);
My = zeros(n, nImages);
V = zeros(n, nImages);
colors = uint8(zeros(n ,3));
for i = 1 : n
    m = fscanf(fid, '%d', 1);
    
    colors(i, :) = fscanf(fid, '%d', 3);
    Mx(i, image_idx) = fscanf(fid, '%f', 1);
    My(i, image_idx) = fscanf(fid, '%f', 1);
    V(i, image_idx) = 1;
    for k = 1 : m-1
        j = fscanf(fid, '%d', 1);
        Mx(i, j) = fscanf(fid, '%f', 1);
        My(i, j) = fscanf(fid, '%f', 1);
        V(i, j) = 1;
    end
end
fclose(fid);