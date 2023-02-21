% ログをロード
logs = readmatrix('log.csv');

logs_deviding_idx = find(logs(:, 1) == 9999);

logs_first_half = logs(1:logs_deviding_idx-1, :);
logs_second_half = logs(logs_deviding_idx+1:end, :);


distance_log = logs_first_half(:, 1);
theta_log = logs_first_half(:, 2);
target_velocity_log = logs_second_half(:, 1);
current_velocity_log = logs_second_half(:, 2);

% 角度に０があったら限りなく0に近い値に書き換える
theta_log(theta_log == 0) = 1e-5;


% 半径を計算
radius = abs(distance_log ./ theta_log);
radius(radius >= 1000) = 1000;

% プロット
x_first_half = 1 : length(logs_first_half);
subplot(2, 1, 1)
plot(x_first_half, radius);
title("半径")

subplot(2, 1, 2)
x_second_half = 1 : length(logs_second_half);

plot(x_second_half, target_velocity_log);
hold on
plot(x_second_half, current_velocity_log);
hold off
legend("目標速度", "現在速度")
title("速度追従")