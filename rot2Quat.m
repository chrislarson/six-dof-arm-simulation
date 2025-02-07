% rot2Quat Returns the quaternion [qo;q_vec] that corresponds to the rotation matrix.
%
% Q = rot2Quat(R)
%
% Outputs:
% Q = the quaternion encoding the supplied rotation
%
% Inputs:
% R = 3x3 rotation matrix to be converted to a quaternion
%
% Chris Larson
% Robot Mechanics
% 2023-09-02

function Q = rot2Quat(R)
% Hollerbach 3.10.3

offDiag = [
    R(3,2)-R(2,3);
    R(1,3)-R(3,1);
    R(2,1)-R(1,2);
    ];
theta = atan2(0.5*norm(offDiag), (trace(R)-1)/2);

if abs(theta - pi) < 0.00001
    % Step 1: Find magnitude of each quaternion component
    q0=sqrt((1+R(1,1)+R(2,2)+R(3,3))/4);
    q1=sqrt((1+R(1,1)-R(2,2)-R(3,3))/4);
    q2=sqrt((1-R(1,1)+R(2,2)-R(3,3))/4);
    q3=sqrt((1-R(1,1)-R(2,2)+R(3,3))/4);
    
    % Step 2: Find largest quaternion component and resolve signs
    if abs(q0) >= max(abs([q1,q2,q3]))
        q1 = (R(3,2)-R(2,3))/(4*q0);
        q2 = (R(1,3)-R(3,1))/(4*q0);
        q3 = (R(2,1)-R(1,2))/(4*q0);
    elseif abs(q1) >= max(abs([q0,q2,q3]))
        q0 = (R(3,2)-R(2,3))/(4*q1);
        q2 = (R(1,2)+R(2,1))/(4*q1);
        q3 = (R(1,3)+R(3,1))/(4*q1);
    elseif abs(q2) >= max(abs([q0,q1,q3]))
        q0 = (R(1,3)-R(3,1))/(4*q2);
        q1 = (R(1,2)+R(2,1))/(4*q2);
        q3 = (R(2,3)+R(3,2))/(4*q2);
    else
        q0 = (R(2,1)-R(1,2))/(4*q3);
        q1 = (R(1,3)+R(3,1))/(4*q3);
        q2 = (R(2,3)+R(3,2))/(4*q3);
    end
    
else
    q0 = abs(0.5*sqrt(1+R(1,1)+R(2,2)+R(3,3)));
    q1 = (R(3,2)-R(2,3))/(4*q0);
    q2 = (R(1,3)-R(3,1))/(4*q0);
    q3 = (R(2,1)-R(1,2))/(4*q0);
end


Q = [q0; q1; q2; q3];
end
