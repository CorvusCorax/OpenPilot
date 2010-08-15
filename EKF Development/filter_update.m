function [ q_attitude ] = filter_update( time, w, a, mag, init_heading, variance )
%FILTER_UPDATE EKF Filter Testing
%   Detailed explanation goes here

if nargin < 6,
    error('There are not enought arguments for filter_update')
end

% Merge the measurements into matricies for easier handling
% a_x = a(1); a_y = a(2); a_z = a(3);       % Accelerometer Measurements
w_x = w(1); w_y = w(2); w_z = w(3);       % Gyroscop Angular Rates Measurements
% m_x = mag(1); m_y = mag(2); m_z = mag(3); % Magnetometer Measurements


%% Static Persistent Variales
persistent state_x;   % State contains all vairables tracked over time
                      % it has quaternion attitude and gyro biases
persistent P;         % State Covariance
% Contains the time of the last update. This is used to calulate dt.                    
persistent last_time;


%% Intialize Initial Value
if isempty(state_x)
    disp('Setting Initial Values for Kalman Filter');
    % (4) Estimated Orientation Quaternion elements with intial conditions  
    % (3) and Gyroscope Bias errors
    state_x = [1 0 0 0, 0 0 0]'; % [quaternion_attitude[4], gyro_bias[3]]
    state_x(1:4) = euler2quat(init_heading,0,0); % Initialize the Yaw Heading
    
    last_time = 0;       % Time used only to figure out dt and is not a state
end

if isempty(P)
    P = zeros(length(state_x),length(state_x));
end

%% System Constants
m = 4;               % Number of measurements in the measurement vector.
n = length(state_x); % Number of states being tracked by the Kalman Filter

g = 9.80665;         % Earth Gravity Magnitude in m/s^2
g = 10.35071665353;  % Gravity for Sparkfun 6dof IMU

% Kalman Filter Tuning Coefficients
% TODO: Figure out how to calculate these
Q = eye(n) * variance.gyro/4; % TODO: The bias covariance will be changed once we determine what the equations are that are updating the gyro bias states.
R = [variance.accel * eye(m-1,m) ; [0 0 0 variance.heading]];

%% Main Body of EKF Filter

% Calculate the time in between samples. TODO: This could also be a constant
deltat = time - last_time;
last_time = time;

% Setup some easy to reference values
q = state_x(1:4);
q = qnormalize(q); % Normalize the Quaternion and save it back
state_x(1:4) = q;
biasx = state_x(5);
biasy = state_x(6);
biasz = state_x(7);

% Calculate the measurement of heading from the magnetometer values. This
% could also be taken from the GPS if no Magnetometer is available.
% [yawe pitche rolle] = quat2euler(q);
% euler = angle_wrap([yawe pitche rolle],'pitch');
% heading = 180/pi*magnetometer_compass(mag, euler(2), euler(3)); % this is also not working right
% heading2 = 180/pi*get_heading(mag, q); % TODO: Not Working right
% mod(sqrt((heading - heading2)^2),360)
heading = get_heading(mag, q);
%     heading = 0        ;
% Determine the state tranistion matrix
% Calculate the matrix for quaternion attitude with new angular rates
% q_dot = 1/2*omega*q
omega = [    0      -(w_x-biasx) -(w_y-biasy) -(w_z-biasz);
        (w_x-biasx)       0       (w_z-biasz) -(w_y-biasy);
        (w_y-biasy) -(w_z-biasz)       0       (w_x-biasx);
        (w_z-biasz)  (w_y-biasy) -(w_x-biasx)       0    ];
    
 % Matrix of reordered conversion expression. In this version one
 % multiplies the matrix of quaternion entries with the angular rates.
 % q_k = [kappa_4x3]*[w_rates_3x1]
 kappa = [-q(2) -q(3) -q(4);
           q(1) -q(4)  q(3);
           q(4)  q(1) -q(2);
          -q(3)  q(2)  q(1)];

% For the nonlinear state transition matrix we must add zeros to the sides
% and bottom of the omega matrix to take care of the gyro bias terms.
% f = [(eye(4)-0.5*omega)*q ; zeros(3,1)];
%f = [(0.5*omega)*q ; zeros(3,1)]; % Alternative method using omega matrix
% TODO: this is quicker than the below calculation 0.28s < 1.32s
f = [0.5*qmult(q,[0 w])' ; zeros(3,1)];

% Calculate the Jacobian of the state transition and measurement transition
F = [0.5*omega , -0.5*kappa]; F = [F ; zeros(n-4,n)];

% Relates the state to the measurement. Used in estimation of current measurement.
% z = [a_x a_y a_z azimuth]' = h(x)
h = [2*g*(q(2)*q(4)-q(1)*q(3));
     2*g*(q(3)*q(4)+q(1)*q(2));
     g*(q(1)^2-q(2)^2-q(3)^2-q(4)^2);
     atan2(2*(q(1)*q(4)+q(2)*q(3)), 1 - 2*(q(3)^2+q(4)^2))];
 
temp1 = q(1)^2+q(2)^2-q(3)^2-q(4)^2;
temp2 = q(2)*q(3)-q(1)*q(4);
temp3 = 4*(temp2/temp1)^2 + 1;
H(1,:) = 2*g*[-q(3)  q(4) -q(1) q(2), 0, 0, 0];
H(2,:) = 2*g*[ q(2)  q(1)  q(4) q(3), 0, 0, 0];
H(3,:) = 2*g*[ q(1) -q(2) -q(3) q(4), 0, 0, 0];
H(4,:) = [(2*q(4)/temp1+4*q(1)*temp2/temp1^2)/temp3,...
          (2*q(3)/temp1+4*q(2)*temp2/temp1^2)/temp3,...
          (2*q(2)/temp1+4*q(3)*temp2/temp1^2)/temp3,...
          (2*q(1)/temp1+4*q(4)*temp2/temp1^2)/temp3,...
           0, 0, 0];
 H(4,:) = zeros(1,n); % TODO: TESTING Yaw now works much better
% H(4,:) = 1e-8*[1 1 1 1 0 0 0]; % TODO: TESTING Yaw now works much better
% H = H % Just for printing

% The measurement for the attitude observer is the accelerometer. This
% corrects the pitch and yaw. The magnetometer measurements are used for
% the correction of yaw.
% z = h(x_k, v_k)
z = [a heading]';

% Time Update - Predictor Equations
% state_x = exp(f*deltat);
% state_x = state_x + (f*deltat)
state_x = (eye(n) + F*deltat) * state_x;

P = F*P*F' + Q;

% Measurement Update - Corrector Equations
S = H*P*H' + R;
% K = P*H'*inv(S); % b*INV(a) = b/a , much faster
K = P*H'/S; % b*INV(a) = b/a , much faster
state_x = state_x + K*(z - h);
P = (eye(n) - K*H)*P;

% Calculate the Gyroscope Error based on gradient of attitude q_dot
% q_dot = F*deltat*state_x % Same as above
q_err = qmult(q, qconj(state_x(1:4)))


q_attitude = state_x(1:4)';

end
