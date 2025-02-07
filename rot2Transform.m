% Utility function to convert a rotation matrix into a homogeneous
% transformation matrix
function H = rot2Transform(R)
H = [
    R, zeros(3,1);
    0, 0, 0, 1;
];
end
