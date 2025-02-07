% Utility function to extract the displacement vector from a homogeneous
% transformation matrix
function d = dispFromTransform(H)
d = H(1:3,4);
end

