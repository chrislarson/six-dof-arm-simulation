% constAccelInterp Returns the position (p), velocity (v), and
% acceleration (a) at time t for a trajectory interpolated using the
% constant acceleration approach.
%
% [p,v,a] = constAccelInterp(t,trajectory,transPercent)
%
% Outputs:
% p = position per interpolated trajectory at time t
% v = velocity per interpolated trajectory at time t
% a = acceleration per interpolated trajectory at time t
%
% Inputs:
% t = time to determine position, velocity, acceleration
% trajectory = the trajectory to be interpolated
% transPercent = transition percent for constant velocity and acceleration
% segments
%
% Chris Larson
% Robot Mechanics
% 2023-10-17

function [p,v,a] = constAccelInterp(t,trajectory,transPercent)

L = size(trajectory, 2);

% Special case: time is negative
if t < 0
    p = zeros(1, L-1);
    v = zeros(1, L-1);
    a = zeros(1, L-1);
    return;
end




N = length(trajectory);
transitions = zeros([N 2]);

for i=2:N-1
    % Previous, current, next times.
    pt = trajectory(i-1,1);
    ct = trajectory(i,1);
    nt = trajectory(i+1,1);
    
    % Previous, current, next positions.
    pp = trajectory(i-1,2:end);
    cp = trajectory(i,2:end);
    np = trajectory(i+1, 2:end);
    
    % Transitions.
    tau_seg = transPercent * min(nt-ct, ct-pt);
    prev_trans = ct - tau_seg;
    next_trans = ct + tau_seg;
    transitions(i, 1) = prev_trans;
    transitions(i, 2) = next_trans;
    seg_prev = transitions(i-1,:);
    seg_curr = transitions(i,:);
    t1 = seg_prev(2);
    t2 = seg_curr(1);
    t3 = seg_curr(2);
    
    % Segment velocities, accelerations, positions.
    V_ab = (cp - pp) / (ct - pt);
    V_bc = (np - cp) / (nt - ct);
    a_seg = (V_bc - V_ab) / (2 * tau_seg);
    p_seg = cp - V_ab*tau_seg;
    
    if t >= t1 && t < t2
        % Constant velocity segment
        p = pp + V_ab*(t - pt);
        v = V_ab;
        a = zeros(1, L-1);
        break;
    elseif t >= t2 && t <= t3
        % Constant acceleration segment
        p = p_seg + V_ab*(t-t2) + 0.5*a_seg*(t-t2)^2;
        v = V_ab + a_seg*(t-t2);
        a = a_seg;
        break;
    elseif (i == N-1 && t > t3)
        % End segment
        p = np;
        v = zeros(1,L-1);
        a = zeros(1,L-1);
        break;
    elseif (i == 2 && t < t2)
        % Start segment
        p = pp;
        v = zeros(1,L-1);
        a = zeros(1,L-1);
        break;
    end
end

end
