% velocityJacobian Returns the velocity jacobian of the manipulator given an
% array of links created by the createLink function and the current joint variables.
%
% [Jv, JvDot] = velocityJacobian(linkList, paramList, paramRateList)
%
% Outputs:
% Jv = the velocity jacobian of the manipulator
% JvDot = the time derivative of the velocity jacobian of the manipulator
%
% Inputs:
% linkList = the list of links created by the createLink function
% paramList = the current theta or d values for the joints, according to the encoders (Nx1)
% paramRateList = the current theta_dot or d_dot values for the joins (Nx1)
%
% Chris Larson
% Robot Mechanics
% 2023-10-23

function [Jv, JvDot] = velocityJacobian(linkList, paramList, paramRateList)

N = length(linkList);
pidx = 1;

T0i = eye(4);

z0is = zeros(3,N+1);
z0is(:, 1) = [0; 0; 1];
d0is = zeros(3,N+1);
w0is = zeros(3,N+1);
v0is = zeros(3,N+1);

Jv = zeros(6,N);
if exist('paramRateList', 'var')
    JvDot = zeros(6,N);
else
    JvDot = [];
end

for i=2:N+1
    li = i-1;
    link_i = linkList(li);
    if link_i.isRotary == 1
        Tin = dhTransform(link_i.a,link_i.d,link_i.alpha,paramList(pidx) - link_i.offset);
        T0i = T0i * Tin;
        d0i = dispFromTransform(T0i);
        if exist('paramRateList', 'var')
            w0i = w0is(:,li) + paramRateList(pidx)*z0is(:,li);
            w0is(:,i) = w0i;
            v0i = v0is(:,li) + cross(w0i, d0i - d0is(:,li));
            v0is(:,i) = v0i;
        end
        pidx = pidx + 1;
    elseif link_i.isRotary == 0
        Tin = dhTransform(link_i.a,paramList(pidx)- link_i.offset,link_i.alpha,link_i.theta);
        T0i = T0i * Tin;
        d0i = dispFromTransform(T0i);
        if exist('paramRateList', 'var')
            w0i = w0is(:,li);
            w0is(:, i) = w0i;
            v0i = v0is(:,li) + cross(w0i, d0i - d0is(:,li)) + paramRateList(pidx)*z0is(:,li);
            v0is(:,i) = v0i;
        end
        pidx = pidx + 1;
    else
        Tin = dhTransform(link_i.a,link_i.d,link_i.alpha,link_i.theta);
        T0i = T0i * Tin;
        d0i = dispFromTransform(T0i);
        if exist('paramRateList', 'var')
            w0i = w0is(:,li);
            w0is(:, i) = w0i;
            v0i = v0is(:,li) + cross(w0i, d0i - d0is(:,li));
            v0is(:,i) = v0i;
        end
        pidx = pidx + 1;
    end
    
    d0n = dispFromTransform(T0i);
    R0n = rotFromTransform(T0i);
    z0n = R0n(1:3,3);
    d0is(:,i) = d0n;
    z0is(:,i) = z0n;
    
end

for i=2:N+1
    li = i-1;
    link_i = linkList(li);
    d0in = d0is(:,N+1) - d0is(:, li);
    z0in = z0is(:, li);
    
    if link_i.isRotary == 1
        Jv(1:3, li) = cross(z0in, d0in);
        Jv(4:6, li) = z0in;
        if exist('paramRateList', 'var')
            JvDot(1:3, li) = cross(cross(w0is(:,i), z0is(:,li)), d0in) + cross(z0is(:,li), v0is(:,N+1) - v0is(:,li));
            JvDot(4:6, li) = cross(w0is(:,i), z0is(:,li));
        end
    elseif link_i.isRotary == 0
        Jv(1:3, li) = z0in;
        Jv(4:6, li) = [0; 0; 0];
        if exist('paramRateList', 'var')
            JvDot(1:3, li) = cross(w0is(:,i), z0is(:,li));
            JvDot(4:6, li) = zeros(3,1);
        end
    else
        Jv(1:6, li) = zeros(6,1);
        if exist('paramRateList', 'var')
            JvDot(1:3, li) = cross(w0is(:,i), z0is(:,li));
            JvDot(4:6, li) = zeros(3,1);
        end
    end
    
end
end
