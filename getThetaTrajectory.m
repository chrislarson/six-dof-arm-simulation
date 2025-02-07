
function [times, thetas] = getThetaTrajectory(points) 
    % Initialization
    N = length(points);
    thetas_ = zeros(N, 6);
    times_ = zeros(N,1);
    base_config = [0; pi/2; 0; 0; 0; 0];
    th_last = base_config;
   
    % Configuration
    time = 1.5;
    time_inc = 0.25;

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
        x_vec_unit = normalize(point_next - point)';
        y_vec = cross(z_vec_unit, x_vec_unit);
        y_vec_unit = y_vec/norm(y_vec);
        point = points(i,:);

        % Rotation and transformation
        R_des = [x_vec_unit, y_vec_unit, z_vec_unit];
        T_des = transformFromRotDisp(R_des, [point(1);point(2);point(3);]);

        % Theta targets
        [th1, th2, th3, th4, th5, th6, ~] = abbInvKine(T_des, th_last);
        times_(i) = time;
        th_last = [real(th1(1)), real(th2(1)), real(th3(1)), real(th4(1)), real(th5(1)), real(th6(1))];
        thetas_(i,:) = th_last;
        th_last = th_last';
        time = time + time_inc;
    end

    % Add duplicate points to start, top of C, bottom of M, and end
    times = [
        % Zero position
        0;
        0.25;
        % CSM
        times_(1) - 0.5;
        times_(1:26);
        times_(26) + 1; 
        % Zero position
        times_(26) + 1 + 1;
        times_(26) + 1 + 1 + 0.25;
    ];

    thetas = [
        % Zero position
        base_config';
        base_config';
        % CSM
        thetas_(1,:);
        thetas_(1:26,:);
        thetas_(26,:);
        % Zero position
        base_config';
        base_config';
    ];
end

