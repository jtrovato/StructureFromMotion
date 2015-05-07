function [Cset, Rset, X] = BundleAdjustment(K, Cr_set, Rr_set, X3D, ReconX, V_bundle, Mx_bundle, My_bundle)

num_cams = length(Cr_set);
P_set = cell(num_cams, 1);
for i =1:num_cams
    P_set{i} = K*Rr_set{i}*[eye(3), -Cr_set{i}];
end

N = sum(ReconX);
meas = zeros(2*N, num_cams);
for i=1:num_cams
    meas(:, i) = reshape([Mx_bundle(ReconX == 1, i), My_bundle(ReconX == 1, i)]', 2*N, 1);
end
[Pset, X] = sba_wrapper(meas, P_set, X3D(ReconX==1, :), K);
%recover Cset and Rset from Pset
Cset = cell(num_cams, 1);
Rset = cell(num_cams, 1);
for i =1:num_cams
    H = inv(K)*Pset{i};
    Rset{i} = H(1:3, 1:3); %H(1:3, 1:3) is R
    Cset{i} = -Rset{i}'*H(:,4); %H(:,4) is t
end


end

