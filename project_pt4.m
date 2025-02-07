clear all; clc;

load 'points3D';

[times, quats, poss] = getQuatTrajectory(points3D);
traj = [times, quats, poss]

N = length(times);

for i=1:N 

    % [p, v, a] = constAccelInterp(times(i), traj, 0.4);
    % 
    % quat_p = p(1:4);
    % disp_p = p(5:7);
    % quat_p_norm = quat_p/norm(quat_p);
    % 
    % R_des = quat2Rot(quat_p_norm');
    % d_des = disp_p';
    % T_des = transformFromRotDisp(R_des,d_des);
    % 
    % % dQ/dt
    % quat_v = v(1:4)';
    % 
    % 
    % q0 = quat_p_norm(1);
    % q_vec = quat_p_norm(2:4)';
    % 
    % quat_cross = cpMap(q_vec);
    % 
    % first_cols = -q_vec;
    % second_cols_p1 = q0*eye(3);
    % second_cols_p2 = quat_cross;
    % second_cols = second_cols_p1 + second_cols_p2;
    % 
    % second_term = [first_cols, second_cols];
    % 
    % quat_w = 2*second_term*quat_v;
    % 
    % twist_des = [quat_v;quat_w];
    % 
    % disp(twist_des);
    [twist_des, T_des] = getTwistDesired(traj, i, 0.2);
end










