% function [ imu ] = imu_load_putty( filename, file_mode )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%% Sparkfun 9DOM IMU v4 Interface
% Author: Patrick Glass
% Date Modified: Aug 2, 2010
% Current Version: 1.0
disp(' ')
disp('Starting Putty IMU Log Data Import')
[path] = fileparts(mfilename('fullpath'));
addpath(fullfile(path,'ST Quaternion Lib'));
colordef black
format compact

R2D = 180/pi;
D2R = pi/180;

% Sample Frequency
f_sample = 50; % 100 Hz sample frequency

%% Configuration
% if nargin < 1 ,
    filename = 'imu_capture_roll_pitch_yaw.bin';
    %      filename = 'imu_capture_static_imu.bin';
    %      filename = 'imu_capture_static_15.25min.bin';
    filename = 'imu_capture_withVideo1.bin';
% end

if nargin < 2 || strcmp( file_mode, 'binary'),
    in_binary = 'yes';
    f_sample = 100; % 100 Hz sample frequency for bianry mode
elseif strcmp( file_mode, 'ascii')
    in_binary = 'no';
else
    error('file_mode was not set properly. Must be [NULL ascii binary]');
end

fid = fopen(filename);

%% Read in some samples
i=1; fail = 0;
data = fread(fid, [22 1], 'uchar');
while ~isempty(data),
    if strcmp(in_binary, 'no')
%         % ASCII Modebr.ReadByte()
%         if(data(1) == 65 && data(12) == 90)
%             imu_samples(i,:) = data';
%             success = success + 1;
%         else
%             errors = errors + 1;
%         end
    else
        % Binary Mode
        if( numel(data) == 22 && data(1) == 65 && data(22) == 90 )
%             imu_samples(i,1) = data(1); % 'A'
            imu_samples(i,2) = double(i*1/f_sample); % Count
            imu_samples(i,3) = 256*data(4)+data(5); % M_x
            imu_samples(i,4) = 256*data(6)+data(7); % M_y
            imu_samples(i,5) = 256*data(8)+data(9); % M_z
            imu_samples(i,6) = 256*data(10)+data(11); % A_x
            imu_samples(i,7) = 256*data(12)+data(13); % A_y
            imu_samples(i,8) = 256*data(14)+data(15); % A_z
            imu_samples(i,9) = 256*data(16)+data(17); % w_x
            imu_samples(i,10) = 256*data(18)+data(19); % w_y
            imu_samples(i,11) = 256*data(20)+data(21); % w_z
%             imu_samples(i,12) = data(22); % 'Z'
        else
            fail = fail + 1
            data'
        end
        
    end
    
    % Get some more data to parse
    data = fread(fid, [22 1], 'uchar');
    i = i + 1;
end

%% Stop and close the file
fclose(fid);

%% Extract the Sensor Parameters
imu.data  = imu_samples;
imu.time  = imu_samples(:,2);
imu.gyro  = imu_samples(:,9:11);
imu.accel = imu_samples(:,6:8);
imu.mag   = imu_samples(:,3:5);

% Known Sensor Constants for IMU 6DOF v4
adc_resolution = 2^10;
adc_Vref = 3.3*1000; % mV
gyro_sensitivity = 2/D2R; % mV/deg/sec for 500 dps output
gyro_scale = adc_Vref / adc_resolution / gyro_sensitivity;
accel_sensitivity = 600; %mV/g : [800,600,300,200] for [1.5g,2g,4g,6g]
accel_scale = adc_Vref / adc_resolution / accel_sensitivity * 9.81;

% Now we must convert the imu samples tp proper units
% Correct for Gyro Bias and axis mounting
gyro_bias = mean(imu.gyro(1:800,:))'
imu.gyro(:,1) = (imu.gyro(:,1) - gyro_bias(1))*(gyro_scale);
imu.gyro(:,2) = (imu.gyro(:,2) - gyro_bias(2))*(-gyro_scale);
imu.gyro(:,3) = (imu.gyro(:,3) - gyro_bias(3))*(-gyro_scale);

% Correct for Accelerometer Bias and axis mounting
accel_bias = mean(imu.accel(1:800,:))'
imu.accel(:,1) = (imu.accel(:,1) - accel_bias(1))*(accel_scale);
imu.accel(:,2) = (imu.accel(:,2) - accel_bias(2))*(accel_scale);
imu.accel(:,3) = (imu.accel(:,3) - mean(accel_bias(1:2)))*(accel_scale);

% Swap Sensor values to mounting frame is corrected to body frame
temp = imu.accel(:,1); % Swap X and Y axis
imu.accel(:,1) = imu.accel(:,2);
imu.accel(:,2) = temp;

temp = imu.gyro(:,1); % Swap X and Y axis
imu.gyro(:,1) = imu.gyro(:,2);
imu.gyro(:,2) = temp;

imu.mag = imu.mag - 546;

% These values were calculated with the file
% 'determine_covariance_from_static.m'
clear imu.variance;
imu.variance.gyro  = mean([0.018758574406617   0.031347219830686   0.018758574406617]);     %0.022955;
imu.variance.accel = mean([0.006660244872792   0.004873801290534   0.005356609169103]);    %0.005630;
imu.variance.heading   = 0.002759143085507; % Calcualted from Magnetomerter Compass Heading Variance


%% Run the Extended Kalman Filter on the gathered data

output = run_filter(imu);


%% Estimate of attitude given raw sensor data
disp('Estimate Attitude Orientations from Raw Sensor Values')
% What are the raw attitude estimates based off each sensor

% Get the pitch and roll estimates from raw accelerometer readings
est_pitch_accel = zeros(size(imu.time));
est_roll_accel = zeros(size(imu.time));
for i=1:length(imu.time),
    [est_pitch_accel(i), est_roll_accel(i)] = accel2attitude(imu.accel(i,1), imu.accel(i,2),imu.accel(i,3));
end

% Do the same now using the raw gyroscope values only by integrating
est_yaw_gyro   = zeros(size(imu.time));
est_pitch_gyro = zeros(size(imu.time));
est_roll_gyro  = zeros(size(imu.time));
for i=2:length(imu.time),
    dt = (imu.time(i)-imu.time(i-1)) / f_sample;
    est_yaw_gyro(i)   = est_yaw_gyro(i-1)+imu.gyro(i,1)*dt;
    est_pitch_gyro(i) = est_pitch_gyro(i-1)+imu.gyro(i,2)*dt;
    est_roll_gyro(i)  = est_roll_gyro(i-1)+imu.gyro(i,3)*dt;
end

% Compute the heading from the magnetometer readings as well as from
% acceleromter data.
est_heading = zeros(size(imu.time));
for i=1:length(imu.time),
    est_heading(i) = magnetometer_compass(imu.mag(i,:), est_pitch_accel(i), est_roll_accel(i));
end

gyro_euler = [est_yaw_gyro est_pitch_gyro est_roll_gyro];
euler_accel = [est_heading est_pitch_accel est_roll_accel];
%% Plot raw sampled data and correct for biases ans such

% Plot the raw sensor values to check for proper signs and coordinate frame
subplot 511, plot(imu.time, angle_wrap(output.euler321,'pitch','rad','deg')) % KALMAN FILTER
legend('yaw','pitch','roll'); grid on;
% subplot 512, plot(imu.time, angle_wrap(gyro_euler,'pitch','rad','deg'))      % GYRO ESTIMATE
subplot 512, plot(imu.time, angle_wrap(euler_accel,'pitch','rad','deg'))     % ACCEL/MAG ESTIMATE
legend('yaw','pitch','roll'); grid on;
subplot 513, plot(imu.time, imu.accel)
legend('A_x','A_y','A_z'); grid on;
subplot 514, plot(imu.time, imu.gyro)
legend('w_x','w_y','w_z'); grid on;
subplot 515, plot(imu.time, imu.mag)
legend('Mag_x','Mag_y','Mag_z'); grid on;

disp('Completed Simulation Sucessfully');

% end

