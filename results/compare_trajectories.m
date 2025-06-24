clear all;
close all;
clc;
function q_fixed = fix_quaternion_signs(q)
    q_fixed = q;
    for i = 2:size(q,1)
        if dot(q_fixed(i-1,:), q_fixed(i,:)) < 0
            q_fixed(i,:) = -q_fixed(i,:);
        end
    end
end
% === Parametri ===
num_joints = 7;
t_start = 1.3;  % tempo di inizio asse X per le posizioni giunti
t_end = 7.5;    % tempo di fine asse X per le posizioni giunti
% === Funzione di caricamento e interpolazione ===
function [time_fine, joint_pos_interp, joint_vel_interp, eef_pos_interp, eef_quat_interp, eul_angles] = process_csv(filename, num_joints)
    % Leggi CSV
    data = readmatrix(filename);
    time = data(:,1);

    % Estrai posizioni e velocità giunti
    joint_pos = zeros(length(time), num_joints);
    joint_vel = zeros(length(time), num_joints);
    for j = 1:num_joints
        joint_pos(:,j) = data(:, 2*(j-1)+2);
        joint_vel(:,j) = data(:, 2*(j-1)+3);
    end

    % Posizione e quaternione end-effector
    idx_eef_pos_start = 2*num_joints + 2;
    eef_pos = data(:, idx_eef_pos_start : idx_eef_pos_start+2);
    eef_quat = data(:, idx_eef_pos_start+3 : idx_eef_pos_start+6);

    % Pulisci timestamp
    [time_sorted, ia, ~] = unique(time, 'stable');
    if length(time_sorted) < length(time)
        warning('Timestamps duplicati trovati e rimossi.');
        joint_pos = joint_pos(ia,:);
        joint_vel = joint_vel(ia,:);
        eef_pos = eef_pos(ia,:);
        eef_quat = eef_quat(ia,:);
        time = time_sorted;
    end
    if any(diff(time) <= 0)
        error('Errore: il vettore time deve essere strettamente crescente.');
    end

    % Interpolazione
    time_fine = linspace(time(1), time(end), 1000)';

    joint_pos_interp = zeros(length(time_fine), num_joints);
    joint_vel_interp = zeros(length(time_fine), num_joints);
    for j = 1:num_joints
        joint_pos_interp(:,j) = interp1(time, joint_pos(:,j), time_fine, 'pchip');
        joint_vel_interp(:,j) = interp1(time, joint_vel(:,j), time_fine, 'pchip');
    end

    eef_pos_interp = zeros(length(time_fine), 3);
    for i = 1:3
        eef_pos_interp(:,i) = interp1(time, eef_pos(:,i), time_fine, 'pchip');
    end

    % Interpolazione quaternioni
    eef_quat = fix_quaternion_signs(eef_quat);
    eef_quat_interp = zeros(length(time_fine),4);
    for i = 1:4
        eef_quat_interp(:,i) = interp1(time, eef_quat(:,i), time_fine, 'pchip');
    end
    norms = sqrt(sum(eef_quat_interp.^2,2));
    eef_quat_interp = eef_quat_interp ./ norms;

    eul_angles = quat2eul(eef_quat_interp, 'ZYX');
end

% === Processa desired.csv ===
[time_fine_des, joint_pos_des, joint_vel_des, eef_pos_des, quat_des, eul_des] = process_csv('planned.csv', num_joints);
% === Processa executed.csv ===
[time_fine_exe, joint_pos_exe, joint_vel_exe, eef_pos_exe, quat_exe, eul_exe] = process_csv('executed.csv', num_joints);

% === Plot desired ===
figure('Name','Joint Positions Comparison');

subplot(2,1,1);
plot(time_fine_des, joint_pos_des);
title('PLANNED Joint Positions');
xlabel('Time [s]');
ylabel('Position [rad]');
xlim([0 time_fine_des(end)]);
legend(arrayfun(@(j) sprintf('Joint %d', j), 1:num_joints, 'UniformOutput', false));

subplot(2,1,2);
plot(time_fine_exe, joint_pos_exe);
title('EXECUTED Joint Positions');
xlabel('Time [s]');
ylabel('Position [rad]');
xlim([t_start t_end]);
figure('Name','Joint Velocities Comparison');

subplot(2,1,1);
plot(time_fine_des, joint_vel_des);
title('PLANNED Joint Velocities');
xlabel('Time [s]');
ylabel('Velocity [rad/s]');
xlim([0 time_fine_des(end)]);

subplot(2,1,2);
plot(time_fine_exe, joint_vel_exe);
title('EXECUTED Joint Velocities');
xlabel('Time [s]');
ylabel('Velocity [rad/s]');
xlim([t_start t_end]);
figure('Name','End-Effector Position Comparison');

subplot(2,1,1);
plot(time_fine_des, eef_pos_des);
title('PLANNED End-Effector Position');
xlabel('Time [s]');
ylabel('Position [m]');
legend({'X','Y','Z'});
xlim([0 time_fine_des(end)]);

subplot(2,1,2);
plot(time_fine_exe, eef_pos_exe);
title('EXECUTED End-Effector Position');
xlabel('Time [s]');
ylabel('Position [m]');
legend({'X','Y','Z'});
xlim([t_start t_end]);
figure('Name','End-Effector Orientation Comparison');

subplot(2,1,1);
plot(time_fine_des, rad2deg(eul_des));
title('PLANNED End-Effector Orientation (Euler ZYX)');
xlabel('Time [s]');
ylabel('Degrees');
legend({'Yaw (Z)','Pitch (Y)','Roll (X)'});
xlim([0 time_fine_des(end)]);

subplot(2,1,2);
plot(time_fine_exe, rad2deg(eul_exe));
title('EXECUTED End-Effector Orientation (Euler ZYX)');
xlabel('Time [s]');
ylabel('Degrees');
legend({'Yaw (Z)','Pitch (Y)','Roll (X)'});
xlim([t_start t_end]);

figure('Name','Quaternion Components');
subplot(2,2,1);
plot(time_fine_des, quat_des(:,1), '--', 'DisplayName','des');
hold on;
plot(time_fine_exe, quat_exe(:,1), '-', 'DisplayName','exe');
title('q_x'); xlabel('Time [s]'); ylabel('Component');
legend; grid on;

subplot(2,2,2);
plot(time_fine_des, quat_des(:,2), '--', 'DisplayName','des');
hold on;
plot(time_fine_exe, quat_exe(:,2), '-', 'DisplayName','exe');
title('q_y'); xlabel('Time [s]'); ylabel('Component');
legend; grid on;

subplot(2,2,3);
plot(time_fine_des, quat_des(:,3), '--', 'DisplayName','des');
hold on;
plot(time_fine_exe, quat_exe(:,3), '-', 'DisplayName','exe');
title('q_z'); xlabel('Time [s]'); ylabel('Component');
legend; grid on;

subplot(2,2,4);
plot(time_fine_des, quat_des(:,4), '--', 'DisplayName','des');
hold on;
plot(time_fine_exe, quat_exe(:,4), '-', 'DisplayName','exe');
title('q_w'); xlabel('Time [s]'); ylabel('Component');
legend; grid on;
