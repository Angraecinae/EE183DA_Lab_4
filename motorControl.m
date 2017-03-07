function [ array ] = motorControl( magnitude, theta_1, theta_2 )
%  Input: magnitude, double [cm] 
%         theta_1, double [rad] (direction of forward movement)
%         theta_2, double [rad] (angle to final rotational state) 
% Output: array, double 1 x 4 array, {pwm_left, pwm_right, interval, turn}
%         pwm_left, double [0 to 100] 
%         pwm_right, double [0 to 100]
%         interval, double [ms]
%         turn, integer [0 to 2]
    interval = timingInterval(25); % [ms]
    if (abs(magnitude) <= 0.1) % within 0.1cm of goal 
        if (abs(theta_2) <= 0.0001) % within < 0.5 degrees of goal
            % reached goal, stop movement
            turn = 0;
            pwm_left = 0;
            pwm_right = 0;
        elseif (theta_2 < 0) 
            % must rotate right to reach desired rotational state
            turn = 2;
            pwm_left = 50;
            pwm_right = 50;            
        else
            % must rotate left to reach desired rotational state
            turn = 1;
            pwm_left = 50;
            pwm_right = 50;
        end
    else
        if (abs(theta_1) <= 0.0001) % within < 0.5 degrees of goal
            % forward movement
            turn = 0;
            pwm_left = 70;
            pwm_right = 70;
        elseif (theta_1 < 0)
            % rotational movement towards the right
            turn = 2;
            pwm_left = 50;
            pwm_right = 50;
        else 
            % rotational movement towards the left
            turn = 1;
            pwm_left = 50;
            pwm_right = 50;
        end
    end
    array(1, 1) = pwm_left;
    array(1, 2) = pwm_right;
    array(1, 3) = interval;
    array(1, 4) = turn;
end