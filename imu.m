function [ array ] = imu( rotational_left, rotational_right, interval )
%  Input: rotational_left, double [rad/s]
%         rotational_right, double [rad/s]
%         interval, double [ms]
% Output: array, double 1 x 3 array, {accel_x, accel_y, rot_z}
%         accel_x, [cm/s^2]
%         accel_y, [cm/s^2]
%         rot_z, [rad]
    wheel_radius = 0.5; % [cm]
    interval = interval / 1000; % [ms] -> [s]
    % Case 1: pure forward movement
    if ((rotational_left < 0 && rotational_right < 0) || ...
            (rotational_left >= 0 && rotational_right >= 0))
        % average sensed angular velocity from left and right wheels
        accel_x = ((rotational_left * wheel_radius + ...
        rotational_right * wheel_radius) / 2) / interval;
        accel_x = accel_x + normrnd(accel_x, (accel_x / 5));
        accel_y = 0;
        rot_z = 0;
    % Case 2: pure rotational movement
    else 
        accel_x = 0;
        accel_y = 0;
        % 1 to 1 ratio between rotation in wheel and rotation around z-axis
        rot_z = (rotational_left * wheel_radius) * interval;
        rot_z = rot_z + norm(rot_z, (rot_z / 5));
    end
    array(1, 1) = accel_x;
    array(1, 2) = accel_y;
    array(1, 3) = rot_z;
end

