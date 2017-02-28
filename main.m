clear all;
close all;
% initialize initial positions
position = [0, 0, 0];
position_real = [0, 0, 0];
position_sim = [0, 0, 0];
% initialize desired end state
coordinates = inputCoor([40, 20, 0]);
cycle = 0;
vector = createVec(position, coordinates);
motor_array = motorControl(vector(1, 1), vector(1, 2), vector(1, 3));
ir_array = irSensor(motor_array(1, 1), motor_array(1, 2), cycle);
imu_array = imu(ir_array(2, 1), ir_array(2, 2), ...
    motor_array(1, 3), motor_array(1, 4));
% initialize state and covariance matrices
state = [0, 0, 0, 0, 0, 0];
cov = [0 0 0 0 0 0; ...
       0 0 0 0 0 0; ...
       0 0 0 0 0 0; ...
       0 0 0 0 0 0; ...
       0 0 0 0 0 0; ...
       0 0 0 0 0 0];
position_real = realLoc(position_real, ir_array(2, 1), ir_array(2, 2), ...
    motor_array(1, 3), motor_array(1, 4));
[state, covflag, cov] = kalmanFilter(state, 0, cov, ...
    mean(ir_array(1, 1:2)), position_real(1, 3), imu_array(1, 3), ...
    motor_array(1, 3), motor_array(1, 4));
position(1, 1) = state(1, 1);
position(1, 2) = state(1, 2);
position(1, 3) = position_real(1, 3);
position_sim = simLoc(position_sim, ir_array(1, 1), ir_array(1, 2), ...
    motor_array(1, 3), motor_array(1, 4));
counter = 0;
while (motor_array(1, 1) ~= 0 && ...
       motor_array(1, 1) ~= 0 && ...
       counter < 1000000)
    counter = counter + 1;
    vector = createVec(position, coordinates);
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
figure, plot(x_position_sim, y_position_sim, 'b-', ...
             x_position_real, y_position_real, 'r-', ...
             x_position_kal, x_position_kal, 'g-'); 