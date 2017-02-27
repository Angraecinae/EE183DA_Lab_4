function [ position ] = simLoc( position, rot_left, rot_right, ...
    interval, turn )
%  Input: position, double 1 x 3 array, {x, y, theta} 
%         rotational_left, double [rad/s] (actual)
%         rotational_right, [rad/s] (actual)
%         interval, double [ms]
%         turn, integer [0 to 2]
% Output: position, double 1 x 3 array, {x, y, theta}
    wheel_radius = 0.5; % [cm]
    car_width = 0.45; % [cm]
    interval = interval / 1000; % [s]
    if (turn == 0) 
        position(1, 1) = position(1, 1) + ...
            (((rot_left + rot_right) / 2) * wheel_radius ...
            * interval * cos(position(1, 3)));
        position(1, 2) = position(1, 2) + ...
            (((rot_left + rot_right) / 2) * wheel_radius ...
            * interval * sin(position(1, 3)));
    elseif (turn == 1)
        position(1, 3) = position(1, 3) + ((wheel_radius / car_width) ...
            * rot_left * interval);
    else 
         position(1, 3) = position(1, 3) - ((wheel_radius / car_width) ...
            * rot_left * interval);       
    end
end

