function [ vector ] = createVec( position, coordinates )
%  Input: position, double 1 x 3 array, {x, y, theta} 
%         coordinates, double 1 x 3 array, {x, y, theta} 
% Output: vector, double 1 x 3 array, 
%         {magnitude [cm], theta1 [rad], theta2 [rad]}
%         theta1 is the rotation for vector movement
%         theta2 is the rotation to achieve final rotational state
    vector(1, 1) = sqrt((coordinates(2) - position(2)).^2 + ...
        (coordinates(1) - position(1)).^2);
    vector(1, 2) = atan((coordinates(2) - position(2)) / ...
        (coordinates(1) - position(1)));
    vector(1, 3) = coordinates(1, 3) - position(1, 3) + vector(1, 2);
end

