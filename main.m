position = [0, 0, 0];
position_real = [0, 0, 0];
coordinates = inputCoor([20, 10, 0]);
cycle = 0;
vector = createVec(position, coordinates);
motor_array = motorControl(vector(1, 1), vector(1, 2), vector(1, 3));
ir_array = irSensor(motor_array(1, 1), motor_array(1, 2), cycle);
imu_array = imu(ir_array(2, 1), ir_array(2, 2), ...
    motor_array(1, 3), motor_array(1, 4));
state = [0, 0, 0, 0, 0, 0];
cov = [0 0 0 0 0 0; ...
       0 0 0 0 0 0; ...
       0 0 0 0 0 0; ...
       0 0 0 0 0 0; ...
       0 0 0 0 0 0; ...
       0 0 0 0 0 0];
[state, covflag, cov] = kalmanFilter(state, 0, cov, ...
    mean(ir_array(1, 1:2)), imu_array(1, 1), imu_array(1, 3), ...
    motor_array(1, 3), motor_array(1, 4));
%position = realLoc(position, ir_array(2, 1), ir_array(2, 2), ...
%    motor_array(1, 3), motor_array(1, 4));
counter = 0;
while (motor_array(1, 1) ~= 0 && ...
       motor_array(1, 1) ~= 0 && ...
       counter < 1000000)
    counter = counter + 1;
    position(1, 1) = state(1, 1);
    position(1, 2) = state(1, 2);
    position(1, 3) = state(1, 5);
    vector = createVec(position, coordinates);
    disp('Current Vector: ');
    disp(vector);
    motor_array = motorControl(vector(1, 1), vector(1, 2), vector(1, 3));
    ir_array = irSensor(motor_array(1, 1), motor_array(1, 2), cycle);
    imu_array = imu(ir_array(2, 1), ir_array(2, 2), ...
    motor_array(1, 3), motor_array(1, 4));
    [state, covflag, cov] = kalmanFilter(state, covflag, cov, ...
        mean(ir_array(1, 1:2)), imu_array(1, 1), imu_array(1, 3), ...
        motor_array(1, 3), motor_array(1, 4));
    %position = realLoc(position, ir_array(2, 1), ir_array(2, 2), ...
    %    motor_array(1, 3), motor_array(1, 4));
    disp('Current Identified Position: ');
    disp(position);
    %disp('Current Actual Position: ');
    %disp(position_real);
    x_position(1, counter) = position(1, 1);
    y_position(1, counter) = position(1, 2);
end

figure, plot(x_position, y_position);