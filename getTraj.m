function [ waypoints ] = getTraj( obstacles, start, goal )
%  Input: obstacles, double 1 x k cell array
%         start, double 1 x 3 array {x, y, theta}
%         goal, double 1 x 3 array {x, y, theta}
% Output: waypoints, double 1 x k cell array
    waypoints{1, 1} = inputCoor(start);
    for i = 1:length(obstacles)
        waypoints{1, i + 1} = inputCoor(obstacles{1, i});
    end
    waypoints{1, length(waypoints) + 1} = inputCoor(goal);
end

