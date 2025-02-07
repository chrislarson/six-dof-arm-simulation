% Utility function to convert a displacement vector into a homogeneous
% transformation matrix
function H = disp2Transform(d)

H = [
    eye(3),  d;
    0, 0, 0, 1;
];
end
