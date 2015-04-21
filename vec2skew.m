function skew = vec2skew(v)
%convert correspondence point vectors to skew matrices
numpts = size(v,1);
skew = zeros(3,3,numpts);
v = [v, ones(numpts, 1)];

for i=1:numpts
    skew(:,:,i) = [0, -v(i,3), v(i,2); v(i,3), 0, -v(i,1); -v(i,2), v(i,1), 0];
end

end

