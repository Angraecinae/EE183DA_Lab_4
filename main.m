clear all;
close all;
% create offline map with predefined boundaries
map = zeros(80, 160);
% initialize map boundaries
for i = 1:80
    map(i, 1) = 1;
    map(i, 160) = 1;
end

for i = 1:160
    map(1, i) = 1;
    map(80, i) = 1;
end
% initialize obstacle bypass waypoints
obstacles{1, 1} = [20, 100, 0];
obstacles{1, 2} = [30, 60, 0];
obstacles{1, 3} = [40, 100, 0];
obstacles{1, 4} = [50, 60, 0];
obstacles{1, 5} = [60, 100, 0];
% initialize obstacles in the map
for i = 1:length(obstacles)
    coor = obstacles{1, i};
    if (coor(1, 2) == 100)
        for j = 1:coor(1, 2) - 15
            map(coor(1, 1), j) = 1;
        end
    else
        for j = coor(1, 2) + 15:160
            map(coor(1, 1), j) = 1;
        end        
    end
end
% initialize initial positions
position = [10, 10, 0];
position_real = [10, 10, 0];
position_sim = [10, 10, 0];
% initialize desired end state
coordinate_list = getTraj(obstacles, position, [70, 150, 0]);
cycle = 0;
counter = 0;
% initialize state and covariance matrices
state = [position(1, 1), position(1, 2), 0, 0, 0, 0];
cov = [0 0 0 0 0 0; ...
       0 0 0 0 0 0; ...
       0 0 0 0 0 0; ...
       0 0 0 0 0 0; ...
       0 0 0 0 0 0; ...
       0 0 0 0 0 0];
covflag = 0;
for i = 1:length(coordinate_list)
    vector = createVec(position, coordinate_list{1, i});
    motor_array = motorControl(vector(1, 1), vector(1, 2), vector(1, 3));
    ir_array = irSensor(motor_array(1, 1), motor_array(1, 2), cycle);
    imu_array = imu(ir_array(2, 1), ir_array(2, 2), ...
        motor_array(1, 3), motor_array(1, 4));
    position_real = realLoc(position_real, ir_array(2, 1), ir_array(2, 2), ...
        motor_array(1, 3), motor_array(1, 4));
    [state, covflag, cov] = kalmanFilter(state, covflag, cov, ...
        mean(ir_array(1, 1:2)), position_real(1, 3), imu_array(1, 3), ...
        motor_array(1, 3), motor_array(1, 4));
    position(1, 1) = state(1, 1);
    position(1, 2) = state(1, 2);
    position(1, 3) = position_real(1, 3);
    position_sim = simLoc(position_sim, ir_array(1, 1), ir_array(1, 2), ...
    motor_array(1, 3), motor_array(1, 4));
    while (motor_array(1, 1) ~= 0 && ...
           motor_array(1, 1) ~= 0 && ...
           counter < 1000000)
        counter = counter + 1;
        vector = createVec(position, coordinate_list{1, i});
        disp('Vector: ');
        disp(vector);
        motor_array = motorControl(vector(1, 1), vector(1, 2), vector(1, 3));
        ir_array = irSensor(motor_array(1, 1), motor_array(1, 2), cycle);
        imu_array = imu(ir_array(2, 1), ir_array(2, 2), ...
        motor_array(1, 3), motor_array(1, 4));
        position_real = realLoc(position_real, ir_array(2, 1), ir_array(2, 2), ...
            motor_array(1, 3), motor_array(1, 4));
        [state, covflag, cov] = kalmanFilter(state, covflag, cov, ...
            mean(ir_array(1, 1:2)), position_real(1, 3), imu_array(1, 3), ...
            motor_array(1, 3), motor_array(1, 4));
        position(1, 1) = state(1, 1);
        position(1, 2) = state(1, 2);
        position(1, 3) = position_real(1, 3);
        position_sim = simLoc(position_sim, ir_array(1, 1), ir_array(1, 2), ...
            motor_array(1, 3), motor_array(1, 4));
        % for troubleshooting
        disp('Current Identified Position: ');
        disp(position);
        disp('Current Actual Position: ');
        disp(position_real);
        % save coordinates
        x_position_kal(1, counter) = position(1, 1); % x-position from filter
        y_position_kal(1, counter) = position(1, 2); % y-position from filter
        x_position_real(1, counter) = position_real(1, 1); % real x-position
        y_position_real(1, counter) = position_real(1, 2); % real y-position
        x_position_sim(1, counter) = position_sim(1, 1); % unfiltered 
        y_position_sim(1, counter) = position_sim(1, 2); % unfiltered
    end
end
% graph results
figure, plot(x_position_sim, y_position_sim, 'b-', ...
             x_position_real, y_position_real, 'r-', ...
             x_position_kal, y_position_kal, 'g-');
% graph boundaries and obstacles
hold on;
for a = 1:80
    for b = 1:160
        if (map(a, b) == 1)
            scatter(a, b, 'y');
        end
    end
end
hold off;