function [ fitness ] = ekf_ahrs_sim( gyro_var, accel_var, mag_var, imu_data )
%ekf_ahrs_sim Summary of this function goes here
%   Detailed explanation goes here
% Flightgear IMU data Fitltering Comparison
% Author: Patrick Glass
% Date Modified: June 23, 2010
% Current Version: 1.0
[path] = fileparts(mfilename('fullpath'));
addpath(fullfile(path,'ST Quaternion Lib'));
disp(' ')
disp('Starting AHRS Kalman Filter Simulation')

global M

%% Simulation Configurations
use_no_drift_gyro    = 'yes';  % ['yes' | 'no']

if nargin < 3,
    gyro_var = 1e-3;
    accel_var = 1e-9;
    mag_var = 1e-7;
end

% Used for genetic algorith in the variance gains
variance.gyro = gyro_var;
variance.accel = accel_var;
variance.mag = mag_var;

if nargin < 4,
    imu_data = M
    %% Here we load in the log data file from Flightgear. The custom
    % swisstech_imu.xml protocol was used for the logging.
    % [A,B,C] = textread(flight.out,format)
    % Flightgear Uses the body coordinate frame with x along right wing, y
    % towards the nose, and z vertically up.
    % The Frame used the these calculations and general aviatino is
    % North-East-Down (NED) Coordianate frame. This is x being positive to the
    % nose. Y positive to the right wing. And Z positive downwards.
    disp('Importing Flight Log IMU Data')
    M(:,1)
    imu_data.time       = M(:,1)
    imu_data.roll       = M(:,2);
    imu_data.pitch      = M(:,3);
    imu_data.yaw        = M(:,4);
    imu_data.accel = [-M(:,5), M(:,6), -M(:,7)];% Coordinate frame Transformation neccessary
    switch( use_no_drift_gyro ),
        case 'no', % JSBSIM Model
            % First sensor is the raw values read from the jsbsim model
            % /fdm/jsbsim/velocities/pi-rad_sec
            imu_data.gyro = [M(:,8), M(:,9), M(:,10)];
        case 'yes', % General Accurate Orientation Data
            % Gyro Rate Sensor 2 is much more precise and does not model 
            % reality as well since there is no drift. Would be ok to use
            % for general algorithm testing if we inject more noise. Sensor
            % used has values from /orientation/roll-rate-degps
            imu_data.gyro = [M(:,11), M(:,12), M(:,13)];
    end
    imu_data.gps = [M(:,14), M(:,15), M(:,16)];
    
    % Generate Simulated Magnetometer Readings from heading (yaw)
    imu_data.mag = zeros(length(imu_data.time),3);
    for i=1:length(imu_data.time),
        imu_data.mag(i,:) = sim_magnetometer(imu_data.yaw(i), imu_data.pitch(i), imu_data.roll(i)); % Uses Actual Attitude to estimate Mxyz
    end
end

%% Sensor and Flight Models used in calculations below.
disp('Extended Kalman Filter Sensor and Flight Model Calculations')

% This section describes the model that was used to describe the system below.
% It should give some insight to the conventions used and naming scheme.
output.attitude = zeros(length(imu_data.time),4);
output.yaw   = zeros(length(imu_data.time),1);
output.pitch = zeros(length(imu_data.time),1);
output.roll  = zeros(length(imu_data.time),1);
for i=1:length(imu_data.time),
    output.attitude(i,:) = filter_update(imu_data.time(i), ...
                           imu_data.gyro(i,:), ...
                           imu_data.accel(i,:), ...
                           imu_data.mag(i,:), imu_data.yaw(1), ...
                           variance);
    
    % Convert the Quaternion attitude to euler angles for displaying
    [output.yaw(i) output.pitch(i) output.roll(i)] = quat2euler( output.attitude(i,:) );
end


%% Calculate the error in the Kalman Filter Estimate to True Attitude
disp('Calculate the Fitness of this EKF Estimate')
error.yaw     = sum(sqrt((yaw   - output.yaw).^2));
error.pitch   = sum(sqrt((pitch - output.pitch).^2));
error.roll    = sum(sqrt((roll  - output.roll).^2));
% Calculate a fitness scalar value. Less emphasis on yaw accuracy.
error.fitness = 0.5*error.yaw + error.pitch + error.roll;
fitness = error.fitness;

disp('Completed Simulation Sucessfully');
end

