%% Determine the variance of the imu Sensors

% Author: Patrick Glass
% Date Modified: June 23, 2010
% Current Version: 1.0
disp(' ')
disp('Starting to gather the variance from Sensors.')

%% Configuration
filename = 'imu_capture_static_imu.bin';
%filename = 'imu_capture_static_15.25min.bin';
filename = 'imu_capture_withVideo1.bin';

f_sample = 100; % 100 Hz

g_start = 10;
g_stop = 2800;


%% Read in some samples
fid = fopen(filename);
i=1; fail = 0;
data = fread(fid, [22 1], 'uchar');
while ~isempty(data),
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

    % Get some more data to parse
    data = fread(fid, [22 1], 'uchar');
    i = i + 1;
end
% Stop and close the file
fclose(fid);


%% Extract the Sensor Parameters
imu.time  = imu_samples(:,2);
imu.gyro  = imu_samples(:,9:11);
imu.accel = imu_samples(:,6:8);
imu.mag   = imu_samples(:,3:5);

% Known Sensor Constants for IMU 6DOF v4
adc_resolution = 2^10;
adc_Vref = 3.3*1000; % mV
gyro_sensitivity = 9.1; % mV/deg/sec for 110 dps output
gyro_scale = adc_Vref / adc_resolution / gyro_sensitivity;
accel_sensitivity = 600; %mV/g : [800,600,300,200] for [1.5g,2g,4g,6g]
accel_scale = adc_Vref / adc_resolution / accel_sensitivity * 9.81;

% Now we must convert the imu samples tp proper units
% Correct for Gyro Bias and axis mounting
gyro_bias = mean(imu.gyro(:,:))'
imu.gyro(:,1) = (imu.gyro(:,1) - gyro_bias(1))*(gyro_scale);
imu.gyro(:,2) = (imu.gyro(:,2) - gyro_bias(2))*(-gyro_scale);
imu.gyro(:,3) = (imu.gyro(:,3) - gyro_bias(3))*(-gyro_scale);

% Correct for Accelerometer Bias and axis mounting
accel_bias = mean(imu.accel(:,:))'
imu.accel(:,1) = (imu.accel(:,1) - accel_bias(1))*(-accel_scale);
imu.accel(:,2) = (imu.accel(:,2) - accel_bias(2))*(accel_scale);
imu.accel(:,3) = (imu.accel(:,3) - mean(accel_bias(1:2)))*(accel_scale);
gravity_estimate = mean(imu.accel(g_start:g_stop,3))

mag_bias = 512;
imu.mag = imu.mag - mag_bias;

v_gx = variance(imu.gyro(:,1), g_start, g_stop)
v_gy = variance(imu.gyro(:,2), g_start, g_stop)
v_gz = variance(imu.gyro(:,3), g_start, g_stop)

v_ax = variance(imu.accel(:,1), g_start, g_stop)
v_ay = variance(imu.accel(:,2), g_start, g_stop)
v_az = variance(imu.accel(:,3), g_start, g_stop)

v_mx = variance(imu.mag(:,1), g_start, g_stop)
v_my = variance(imu.mag(:,2), g_start, g_stop)
v_mz = variance(imu.mag(:,3), g_start, g_stop)

imu.variance.gyro  = ([v_gx v_gy v_gz]);
imu.variance.accel = ([v_ax v_ay v_az]);
imu.variance.mag   = ([v_mx v_my v_mz]);


%% Patrick Testing with real Mesurement Variance Calculator
pitch_accel = zeros(length(imu.accel),1);
roll_accel = zeros(length(imu.accel),1);
yaw = zeros(length(imu.accel),1);
for i=1:length(imu.mag)
    [pitch_accel(i), roll_accel(i)] = accel2attitude(imu.accel(i,1),imu.accel(i,2),imu.accel(i,3));
    yaw(i) = magnetometer_compass(imu.mag(i,:),pitch_accel(i),roll_accel(i));
end
var_yaw   = variance(yaw,g_start,g_stop)
var_pitch = variance(pitch_accel,g_start,g_stop)
var_roll  = variance(roll_accel,g_start,g_stop)

imu.variance.R = ([v_ax v_ay v_az var_yaw]);


%% Calculating the covariance Matrix Q. Process Noise Covariance Q;
% we use the noise inherent to the quaternion and attempt to see the noise
% that gets passed through.
% Since we have small angles when we are calibrarting we can ignore the
% euler to quaternion equation and just evaulate the q itsself
v_q1 = 1;
v_q2 = var_roll/4
v_q2 = var_pitch/4
v_q2 = var_yaw/4


%%

% Sparkfun 6dof IMU v4
gyro_variance = imu.variance.gyro
accel_variance = imu.variance.accel
mag_variance = imu.variance.mag
gyro_variance = mean(imu.variance.gyro)
accel_variance = mean(imu.variance.accel)
mag_variance = mean(imu.variance.mag)
yaw_variance = var_yaw
% imu.variance.gyro  = 2; 
% imu.variance.accel = 1;
% imu.variance.mag   = 10;

%%
subplot 311, plot(imu.time, imu.accel)
legend('A_x','A_y','A_z'); grid on;
subplot 312, plot(imu.time, imu.gyro)
legend('w_x','w_y','w_z'); grid on;
subplot 313, plot(imu.time, imu.mag)
legend('Mag_x','Mag_y','Mag_z'); grid on;

disp('Completed Simulation Sucessfully');









