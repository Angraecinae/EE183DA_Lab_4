function [ array ] = irSensor( pwm_left, pwm_right, cycle )
%  Input: cycle, integer
% Output: array, double 1 x 3 array
%         rotational_left double [rad/s]
%         rotational_right [rad/s]
%         cycle, integer
    if (pwm_left < 25) 
        rotational_left = 0;
    else
        output = motorOutput(pwm_left, pwm_right, cycle);
        while(output(1, 1) ~= 1) 
            cycle = cycle + 1;
            output = motorOutput(pwm_left, pwm_right, cycle);
        end
        cycle = cycle + 1;
        output = motorOutput(pwm_left, pwm_right, cycle);
        duration_left = cycle;
        while(output(1, 1) ~= 1)
            cycle = cycle + 1;
            output = motorOutput(pwm_left, pwm_right, cycle);
        end
        duration_left = abs(duration_left - cycle) + 1; 
        rotational_left = 2 * pi *(9 - duration_left) / 70;
    end
    if (pwm_right < 25)
        rotational_right = 0;
    else
        output = motorOutput(pwm_left, pwm_right, cycle);
        while(output(1, 2) ~= 1) 
            cycle = cycle + 1;
            output = motorOutput(pwm_left, pwm_right, cycle);
        end
        cycle = cycle + 1;
        output = motorOutput(pwm_left, pwm_right, cycle);
        duration_right = cycle;
        while(output(1, 2) ~= 1)
            cycle = cycle + 1;
            output = motorOutput(pwm_left, pwm_right, cycle);
        end    
        duration_right = abs(duration_right - cycle) + 1; 
        rotational_right = 2 * pi * (9 - duration_right) / 70;
    end
    array(1, 1) = rotational_left;
    array(1, 2) = rotational_right;
    array(1, 3) = cycle;
end

