function [twist_des, T_des, accel_des] = getTwistDesired(trajectory,time, transPercent)
    [p, v, a] = constAccelInterp(time, trajectory, transPercent);

    quat_p = p(1:4);
    quat_p_norm = quat_p/norm(quat_p);
    disp_p = p(5:7);

    R_des = quat2Rot(quat_p_norm');
    d_des = disp_p';
    T_des = transformFromRotDisp(R_des,d_des);
    
    % dQ/dt
    quat_v = v(1:4)';
    % d2Q/dt2
    quat_a = a(1:4)';

    disp_v = v(5:7)';
    disp_a = a(5:7)';
    
    q0 = quat_p_norm(1);
    q_vec = quat_p_norm(2:4)';

    quat_w = 2*[-q_vec, q0*eye(3) + cpMap(q_vec)]*quat_v;
    quat_alpha = 2*[-q_vec, q0*eye(3) + cpMap(q_vec)]*quat_a;

    twist_des = [disp_v;quat_w];
    accel_des = [disp_a;quat_alpha];
end