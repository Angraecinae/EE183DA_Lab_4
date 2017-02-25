function [ vector ] = createVec( position, coordinates )
%  Input: position, double 1 x 2 array, {x, y} 
%         coordinates, double 1 x 2 array, {x, y} 
% Output: vector, double 1 x 2 array, {magnitude [cm], theta [rad]}
    vector(1, 1) = sqrt((coordinates(2) - position(2)).^2 + ...
        (coordinates(1) - position(1)).^2);
    vector(1, 2) = atan((coordinates(2) - position(2)) / ...
        (coordinates(1) - position(1)));
end

