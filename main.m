clear all;
close all;
% initialize initial positions
position = [0, 0, 0];
position_real = [0, 0, 0];
position_sim = [0, 0, 0];
% initialize desired end state
coordinate_list{1, 1} = inputCoor([10, 10, 0]);
coordinate_list{1, 2} = inputCoor([15, 5, 0]);
coordinate_list{1, 3} = inputCoor([20, 10, 0]);
coordinate_list{1, 4} = inputCoor([25, 5, 0]);
coordinate_list{1, 5} = inputCoor([30, 10, 0]);
coordinate_list{1, 6} = inputCoor([35, 5, 0]);
cycle = 0;
counter = 0;
% initialize state and covariance matrices
state = [0, 0, 0, 0, 0, 0];
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
figure, plot(x_position_sim, y_position_sim, 'b-', ...
             x_position_real, y_position_real, 'r-', ...
             x_position_kal, y_position_kal, 'g-'); 