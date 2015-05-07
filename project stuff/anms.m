function [y, x,rmax] = anms(C, N)
% This function takes and input image I and the maximum number of corners
% to find and outputs corner locations (y,x) and the maximum radius used to
% dteremine most relavant corners. 
verbose =1;

[h,w] = size(C);
if verbose
    figure();
    imagesc(C);
end
% only use pixels that above a certain threshold
thresh = 0.01*max(max(C)); %mean2(C) - 8*std2(C); %may want to make this smarter
C(C < thresh) = 0;


% if verbose
%     size(find(C))
%     figure();
%     imagesc(C);
% end
%find local maxima
for ii = 2:size(C,1)-1
    for jj = 2:size(C,2)-1
        val = C(ii,jj);
        surronding_pixels = [C(ii-1, jj-1),C(ii-1, jj),C(ii-1, jj+1),...
                             C(ii, jj-1), C(ii, jj+1)...
                             C(ii+1, jj-1),C(ii+1, jj),C(ii+1, jj+1)];
        if(any(any(surronding_pixels > val)))
            C(ii,jj) = 0;
        end
    end
end

% if verbose
%     figure();
%     imagesc(C);
% end

% find radius in which each max is considered a corner

[corner_i, corner_j, corner_values] = find(C);
inds = ~logical(corner_i <= 20 + corner_i >= h-20 + corner_j <= 20 + corner_j >= w-20);
corner_i = corner_i(inds);
corner_j = corner_j(inds);
corner_values = corner_values(inds);

corner_radii = zeros(size(corner_values));

for ii=1:size(corner_values, 1)
    cur_val = corner_values(ii);
    cur_i = corner_i(ii);
    cur_j = corner_j(ii);
    better_corners = corner_values(corner_values>cur_val);
    better_i = corner_i(corner_values>cur_val);
    better_j = corner_j(corner_values>cur_val);
    
    if isempty(better_corners)
        cur_r = Inf;
    else
        better_dists = sqrt((double(better_j) - double(cur_j)).^2 + (double(better_i)-double(cur_i)).^2);
        [cur_r, ~] = min(better_dists);
    end
    corner_radii(ii) = cur_r;
end

%corner_radii
[sorted_radii, inds] = sort(corner_radii,  'descend');
corner_i = corner_i(inds);
corner_j = corner_j(inds);

x = corner_j(1:N);
y = corner_i(1:N);
rmax = sorted_radii(N);






end