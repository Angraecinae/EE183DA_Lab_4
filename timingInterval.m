function [ duration ] = timingInterval( duration )
%  Input: duration, double [ms]
% Output: duration, double [ms]
    duration = duration + normrnd(duration, (duration / 10));
end

