
function [times, quats, poss] = getQuatTrajectory(points, time_step) 
    % Initialization
    N = length(points);
    times_ = zeros(N,1);
    quats_ = zeros(N, 4);
    poss_ = zeros(N,3);
    
    as = [0; 0.270; 0.070; 0; 0; 0];
    ds = [0.290; 0; 0; 0.302; 0; 0.072];
    alphas = [-pi/2; 0; -pi/2; pi/2; -pi/2; 0];
    thetas = [0; 0; 0; 0; 0; 0];

    T01 = dhTransform(as(1), ds(1), alphas(1), thetas(1));
    T12 = dhTransform(as(2), ds(2), alphas(2), thetas(2));
    T23 = dhTransform(as(3), ds(3), alphas(3), thetas(3));
    T34 = dhTransform(as(4), ds(4), alphas(4), thetas(4));
    T45 = dhTransform(as(5), ds(5), alphas(5), thetas(5));
    T56 = dhTransform(as(6), ds(6), alphas(6), thetas(6));
    T06 = T01 * T12 * T23 * T34 * T45 * T56;
    base_rot = rotFromTransform(T06);
    base_quat = rot2Quat(base_rot);
    base_disp = dispFromTransform(T06);

    T01_e = dhTransform(as(1), ds(1), alphas(1), thetas(1));
    T12_e = dhTransform(as(2), ds(2), alphas(2), thetas(2)-pi/2);
    T23_e = dhTransform(as(3), ds(3), alphas(3), thetas(3));
    T34_e = dhTransform(as(4), ds(4), alphas(4), thetas(4));
    T45_e = dhTransform(as(5), ds(5), alphas(5), thetas(5));
    T56_e = dhTransform(as(6), ds(6), alphas(6), thetas(6));
    T06_e = T01_e * T12_e * T23_e * T34_e * T45_e * T56_e;
    end_rot = rotFromTransform(T06_e);
    end_quat = rot2Quat(end_rot);
    end_disp = dispFromTransform(T06_e);

    % Configuration
    time = 1.25;
    time_inc = time_step;
    time_stop = 0.2;

    for i = 1:N
        % Handle special case (final position on M)
        if i == N 
            point = points(i-1,:);
            point_next = points(i,:);
        else
            point = points(i,:);
            point_next = points(i+1,:);
        end

        % Rotation axes
        z_vec_unit = [0; -1; 0];
        x_vec = (point_next-point)';
        x_vec_unit = x_vec/norm(x_vec);
        % x_vec_unit = normalize(point_next - point)';
        y_vec = cross(z_vec_unit, x_vec_unit);
        y_vec_unit = y_vec/norm(y_vec);
        point = points(i,:);

        % Rotation and transformation
        R_des = [x_vec_unit, y_vec_unit, z_vec_unit];
        T_des = transformFromRotDisp(R_des, [point(1);point(2);point(3);]);

        % Twist targets
        disp_des = dispFromTransform(T_des);
        quat_des = rot2Quat(R_des);
        quat_des = quat_des / norm(quat_des);

        times_(i) = time;
        quats_(i,:) = quat_des';
        poss_(i,:) = disp_des';
        time = time + time_inc;
    end

    % Add duplicate points to start, top of C, bottom of M, and end
    times = [
        % Zero position
        0;
        time_stop;
        % CSM
        times_(1) - time_stop;
        times_(1:26);
        times_(26) + time_stop; 
        % Zero position
        times_(26) + 1 + time_stop;
        times_(26) + 1 + time_stop + time_stop;
        ];

    quats = [
        % Zero position
        base_quat';
        base_quat';
        % CSM
        quats_(1,:);
        quats_(1:26,:);
        quats_(26,:);
        % Zero position
        end_quat';
        end_quat';
    ];

    poss = [
        % Zero position
        base_disp';
        base_disp';
        % CSM
        poss_(1,:);
        poss_(1:26,:);
        poss_(26,:);
        % Zero position
        end_disp';
        end_disp';
        ];
end

