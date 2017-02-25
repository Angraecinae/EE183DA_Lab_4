function [ ir ] = motorOutput( pwm_left, pwm_right, cycle )
%  Input: pwm_left, integer [0 to 100]
%         pwm_right, integer [0 to 100]
%         cycle, integer 
% Output: ir, boolean 1 x 2 array, {ir_left, ir_right}
%   PWM -> IR Sensor will read 1 at...
%   100 -> every 2 cycles
%    90 -> every 3 cycles
%    80 -> every 4 cycles
%    70 -> every 5 cycles
%    60 -> every 6 cycles
%    50 -> every 7 cycles
%    40 -> every 8 cycles
%    30 -> every 9 cycles
    if (pwm_left < 25)
        ir(1, 1) = 0;
    else 
        if (mod(cycle, floor((100 - pwm_left) / 10) + 2) == 0)
            ir(1, 1) = 1;
        else
            ir(1, 1) = 0;
        end
    end
    if (pwm_right < 25)
        ir(1, 2) = 0;
    else
        if (mod(cycle, floor((100 - pwm_right) / 10) + 2) == 0)
            ir(1, 2) = 1;
        else
            ir(1, 2) = 0;
        end 
    end  
end

