

function plot_points()
clc; clear;

load 'points2D.mat' points_all;

L = length(points_all);
points = [points_all(:,1), zeros(L,1), points_all(:,2)];

H_scale = [
    0.02, 0, 0,    0;
    0,    1, 0,    0;
    0,    0, 0.02, 0;
    0,    0, 0,    1;
];
H_tx = disp2Transform([-0.06, 0,0]');
H_ty = disp2Transform([0, -0.3, 0]');
H_tz = disp2Transform([0,0,0.4]');

points_w = zeros(L, 3);
xhat = zeros(L,3);
yhat = zeros(L,3);
zhat = zeros(L,3);

for i = 1:L-1
    point = points(i,:);
    H_point = [point, 1]';
    point_w = eye(4) * H_tx * H_ty * H_tz * H_scale * H_point;

    point_next = points(i+1,:);
    H_point_next = [point_next, 1]';
    point_w_next = eye(4) * H_tx * H_ty * H_tz * H_scale * H_point_next;

    x_vec = point_w_next(1:3) - point_w(1:3);
    x_vec_unit = x_vec/norm(x_vec);
    z_vec_unit = [0; -1; 0];
    y_vec = cross(z_vec_unit,x_vec_unit);
    y_vec_unit = y_vec/norm(y_vec);

    % 
    % X_axes = R[:,1];
    % Y = R[:,2];
    % Z = R[:,3];

    xhat(i,:) = x_vec_unit;
    yhat(i,:) = y_vec_unit;
    zhat(i,:) = z_vec_unit;
    points_w(i,:) = point_w(1:3);
    points_w(i+1,:) = point_w_next(1:3);
end

xhat(L,:) = xhat(L-1,:);
yhat(L,:) = yhat(L-1,:);
zhat(L,:) = zhat(L-1,:);

figure(1);
quiver3(points_w(:,1), points_w(:,2), points_w(:,3), xhat(:,1), xhat(:,2), xhat(:,3), 0.2, 'r');
hold on;
quiver3(points_w(:,1), points_w(:,2), points_w(:,3), yhat(:,1), yhat(:,2), yhat(:,3), 0.2, 'g');
hold on;
quiver3(points_w(:,1), points_w(:,2), points_w(:,3), zhat(:,1), zhat(:,2), zhat(:,3), 0.03, 'b');
hold on;
plot3(points_w(:,1), points_w(:,2), points_w(:,3), '-o', 'DisplayName','Path');
hold on;
axis([-0.07, 0.16, -0.31, -0.29, 0.39, 0.49]);
hold off;


points3D = points_w;

save('points3D.mat', 'points3D');

xlabel('world x [m]');
ylabel('world y [m]');
zlabel('world z [m]');
legend({'x-axis','y-axis', 'z-axis', 'Path'});

end