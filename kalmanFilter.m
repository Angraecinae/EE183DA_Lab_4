function [ state, covflag, cov ] = kalmanFilter( state, covflag, cov, ...
    dx, theta, accel, t, turn )
%  Input: state double 1 x 6 array {x, y, dx, dy, theta, dtheta}
%         x, double [cm]         
%         y, double [cm]
%         dx, double [cm/s]
%         dy, double [cm/s]
%         theta, [rad]
%         dtheta, [rad/s]
%         covflag
%         cov
%         dx, double [rad/s] from IR sensor (in x-direction)
%         theta, double [rad] from IMU
%         accel, double [cm/s^2] from IMU (in x-direction)
%         t, double [ms] from Clock
% Output: state, double 1 x 6 array {x, y, dx, dy, theta, dtheta}
%         x, double [cm] updated x position relative to origin       
%         y, double [cm] updated y position relative to origin
%         dx, double [cm/s] 
%         dy, double [cm/s]
%         theta, [rad]
%         dtheta, [rad/s]
%         covflag, boolean [0 or 1] 
%         cov, matrix
    state(1, 1) = state(1, 1) / 10; % [m]
    state(1, 2) = state(1, 2) / 10; % [m]
    theta_actual = theta;
    wheel_radius = 0.05; % [m]
    car_width = 0.045; % [m]
    accel = accel / 100; % [m/s^2]
    vel_mag = dx;
    if (turn == 0) 
        dx = (vel_mag * wheel_radius) * cos(theta_actual); % [m/s]
        dy = (vel_mag * wheel_radius) * sin(theta_actual); % [m/s]
        dtheta = 0;
    elseif (turn == 1)
        dx = 0;
        dy = 0;
        dtheta = (wheel_radius / car_width) * dx;
    else
        dx = 0;
        dy = 0;
        dtheta = - (wheel_radius / car_width) * dx;
    end
    t = t / 1000; % [s]
    state = state';
    % from sensor data
    measurements = [dx; dy; theta; dtheta];
    % define state space model 
    A = [1 0 t 0 0 0; ...
         0 1 0 t 0 0; ...
         0 0 1 0 0 0; ...
         0 0 0 1 0 0; ...
         0 0 0 0 1 t; ...
         0 0 0 0 0 1]; 
    B = [cos(theta_actual) * ((t.^2) / 2); ...
         sin(theta_actual) * ((t.^2) / 2); ...
         cos(theta_actual) * t; ...
         sin(theta_actual) * t;
         0; ...
         0];
    C = [0 0 1 0 0 0; ...
         0 0 0 1 0 0; ...
         0 0 0 0 1 0; ...
         0 0 0 0 0 1]; 
    % define covariances
    process_noise = 0.1; % process noise (stdv of acceleration: [cm/s^2])
    m_n_dx = 10;  % measurement noise in the x axis
    m_n_dy = 10;  % measurement noise in the y axis
    m_n_dtheta = 10;  % measurement noise in the z axis.
    m_n_theta = 10;  % measurement noise in theta.
    % measurement error
    m_error = [m_n_dx 0 0 0; ...
               0 m_n_dy 0 0; ...
               0 0 m_n_theta 0; ...
               0 0 0 m_n_dtheta];
    % process error
    p_error = [(t^4)/4 0 (t^3)/2 0 0 0; ...
               0 t^4/4 0 (t^3)/2 0 0; ...
               (t^3)/2 0 t^2 0 0 0; ...
               0 (t^3)/2 0 t^2 0 0; ...
               0 0 0 0 0 0; ...
               0 0 0 0 0 0];
    p_error = p_error.*process_noise^2;
    if (covflag == 0)             
        cov = p_error; % initialize variance
        covflag = 1;
    end
    % kalman filter
    state = A * state + B * accel; % predict next state
    cov = A * cov * A' + p_error; % predict next covariance
    K = cov * C' * inv(C * cov * C' + m_error); % kalman gain
    state = state + K * (measurements - C * state); % update estimation
    cov = (eye(6) - K * C) * cov; % update covariance
    state = state';
    state(1, 1) = state(1, 1) * 10;
    state(1, 2) = state(1, 2) * 10;
end
