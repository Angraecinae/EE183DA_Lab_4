function [ array ] = imu( rot_left, rot_right, interval, turn )
%  Input: rot_left, double [rad/s]
%         rot_right, double [rad/s]
%         interval, double [ms]
%         turn, integer [0 to 2]
% Output: array, double 1 x 3 array, {accel_x, accel_y, rot_z}
%         accel_x, [cm/s^2]
%         accel_y, [cm/s^2]
%         rot_z, [rad]
    wheel_radius = 0.5; % [cm]
    car_width = 0.45; % [cm]
    interval = interval / 1000; % [ms] -> [s]
    % Case 1: pure forward movement
    if (turn == 0)
        % average sensed angular velocity from left and right wheels
        accel_x = ((rot_left * wheel_radius + ...
        rot_right * wheel_radius) / 2) / interval;
        accel_x = accel_x + normrnd(accel_x, (accel_x / 5));
        accel_y = 0;
        rot_z = 0;
    % Case 2: pure rotational movement
    elseif (turn == 1) 
        accel_x = 0;
        accel_y = 0;
        rot_z = (wheel_radius / car_width) * rot_left * interval;
        rot_z = rot_z * (1 + norm(0, (rot_z / 2.5)));
    else
        accel_x = 0;
        accel_y = 0;
        rot_z = - (wheel_radius / car_width) * rot_left * interval;
        rot_z = rot_z * (1 + norm(0, (rot_z / 2.5)));        
    end
    array(1, 1) = accel_x;
    array(1, 2) = accel_y;
    array(1, 3) = rot_z;
end

