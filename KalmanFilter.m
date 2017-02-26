%% KALMAN FILTER IMPLEMENTATION

function [x_out,y_out,velx_out, vely_out,theta_out, velt_out, covflag, Covariance] = KalmanFilter(state, covflag, Covariance, velx_measure, vely_measure, theta_measure, velt_measure, acc, dt)

%% Initialize

%state = [x_in; y_in; velx_in; vely_in; theta_in; velt_in];  %initial state estimate(what we are updating)
measurements = [velx_measure; vely_measure; theta_measure; velt_measure]; %measurements from sensors

%% Define state space model (Coefficent matrices)

A = [1 0 dt 0 0 0; ...
    0 1 0 dt 0 0; ...
    0 0 1 0 0 0; ...
    0 0 0 1 0 0; ...
    0 0 0 0 1 dt; ...
    0 0 0 0 0 1]; 

B = [(dt^2/2); ...
    (dt^2/2); ...
    dt; ...
    dt;
    0; ...
    0];

C = [0 0 1 0 0 0; ...
    0 0 0 1 0 0; ...
    0 0 0 0 1 0; ...
    0 0 0 0 0 1];  
    
%% Define Covariances

process_noise = .1; %process noise (stdv of acceleration: meters/sec^2)
measurement_noise_velx = 1;  %measurement noise in the horizontal direction (x axis).
measurement_noise_vely = 1;  %measurement noise in the horizontal direction (y axis).
measurement_noise_velt = 1;  %measurement noise in the angular rotation (z axis).
measurement_noise_theta = 1;  %measurement noise in the angular measurement (theta).

%assuming all noises are independent
Err_measure = [measurement_noise_velx 0 0 0; ...
                0 measurement_noise_vely 0 0; ...
                0 0 measurement_noise_theta 0; ...
                0 0 0 measurement_noise_velt];
            
Err_process = [dt^4/4 0 dt^3/2 0 0 0; ...
                 0 dt^4/4 0 dt^3/2 0 0; ...
                 dt^3/2 0 dt^2 0 0 0; ...
                 0 dt^3/2 0 dt^2 0 0; ...
                 0 0 0 0 0 0; ...
                 0 0 0 0 0 0].*process_noise^2; % convert the process noise (stdv) into covariance matrix
if covflag == 0             
Covariance = Err_process; % initialize variance (covariance matrix)
covflag = 1;
end
%% Kalman Filter 
    
    % Predict next state
    state = A * state + B * acc;
    
    %predict next covariance
    Covariance = A * Covariance * A' + Err_process;
    
    % Kalman Gain
    K = Covariance*C'*inv(C*Covariance*C' + Err_measure);
    
    % Update the state estimate.
    state = state + K * (measurements - C * state);
    
    % update covariance estimation.
    Covariance = (eye(6)-K*C)*Covariance;
    
    x_out = state(1);
    y_out = state(2);
    velx_out = state(3);
    vely_out = state(4);
    theta_out = state(5);
    velt_out = state(6);
end