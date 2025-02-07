% rot2AngleAxis Returns the product of angle and axis (theta * k) corresponding to a rotation matrix.
%
% [Omega] = rot2AngleAxis(R)
%
% Outputs:
% Omega = The rotation encoded as theta * k
%
% Inputs:
% R = 3x3 rotation matrix to convert to angle axis
%
% Chris Larson
% Robot Mechanics
% 2023-09-02

function Omega = rot2AngleAxis(R)
u = [
    R(3,2)-R(2,3);
    R(1,3)-R(3,1);
    R(2,1)-R(1,2);
    ];

theta = atan2(0.5 * norm(u), 0.5*(trace(R)-1));

% Special case 1: pi rotation
if abs(sin(theta)) < eps
    kx = sqrt((R(1,1) + 1)/2);
    ky = sqrt((R(2,2) + 1)/2);
    kz = sqrt((R(3,3) + 1)/2);
    if kx > max([ky, kz])
        ky = R(1,2) / (2*kx);
        kz = R(1,3) / (2*kx);
    elseif ky > max([kx,kz])
        kx = R(1,2) / (2*ky);
        kz = R(2,3) / (2*ky);
    else
        kx = R(1,3) / (2*kz);
        ky = R(2,3) / (2*kz);
    end
    k = [kx; ky; kz];
    % Special case 2: small angle
elseif abs(theta - 0) < eps
    k = [0; 0; 0];
    % General case:
else
    k = u / (2*sin(theta));
end

Omega = k * theta;
end