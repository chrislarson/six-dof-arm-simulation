% Utility function to extract rotation matrix from homogeneous
% transformation matrix
function R = rotFromTransform(H)
R = H(1:3, 1:3);
end

