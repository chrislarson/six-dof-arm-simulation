% cpMap Returns the matrix packing of the cross product operator.
%
% X = cpMap(w)
%
% Outputs:
% X = matrix representation of the cross product operator for w
%
% Inputs:
% w = 3-dimensional vector to be converted to cross product packed matrix
%
% Chris Larson
% Robot Mechanics
% 2023-09-02

function X = cpMap(w)
X = [
    0, -w(3),  w(2);
    w(3),     0, -w(1);
    -w(2),  w(1),     0;
    ];
end
