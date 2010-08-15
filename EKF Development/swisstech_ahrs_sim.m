% function fitness = swisstech_ahrs_sim(gyro_var, accel_var, mag_var)
%% Flightgear IMU data Fitltering Comparison
% Author: Patrick Glass
% Date Modified: June 23, 2010
% Current Version: 1.0

% Future Development
%    Rev 1.0: Basic Flight Model for static testing with Gyro, Accel, Mag
%    Rev 1.1: Include interface for forward direction to compensate for
%             centripital acceleration.
% clear, clc
disp(' ')
disp('Starting AHRS Kalman Filter Simulation')
colordef black
format compact
R2D = 180/pi;
D2R = pi/180;


%% Simulation Configurations
save_random_seed     = 'yes'; % ['yes' | 'no']
always_reload_flight = 'yes'; % ['yes' | 'no']
run_library_tests    = 'no';  % ['yes' | 'no']
use_no_drift_gyro    = 'yes'; % ['yes' | 'no']
insert_noise         = 'yes'; % ['yes' | 'no']
compute_variance     = 'yes'; % ['yes' | 'no']
% Noise Parameters of Sesnor
gyro_sd  = 5*D2R;       % Gyroscope Noise Standard Deviation (Rad)
accel_sd = 9.81*0.01;   % Accelerometer Noise Standard Deviation (g)
mag_sd   = 5;           % Magnetometor Noise Standard Deviation (Percent)


%% SIMULATION
% Save the random stream so we have repeatable results.
if strcmp(save_random_seed,'yes'),
    defaultStream = RandStream.getDefaultStream;
    savedState    = defaultStream.State;
else
    disp('Setting new Random Seed')
    RandStream.setDefaultStream(RandStream('mt19937ar','seed',sum(100*clock)));
end

[path] = fileparts(mfilename('fullpath'));
addpath(fullfile(path,'ST Quaternion Lib'));
% addpath(fullfile(path,'aero_quaternion_matlab'));

% Only load in the data if it has not been done before.
%  If flight.out is change one must call 'clear M'
if ~exist('imu','var') || strcmp(always_reload_flight, 'yes'),
    %% Here we load in the log data file from Flightgear. The custom
    % swisstech_imu.xml protocol was used for the logging.
    % [A,B,C] = textread(flight.out,format)
    % Flightgear Uses the body coordinate frame with x along right wing, y
    % towards the nose, and z vertically up.
    % The Frame used the these calculations and general aviatino is
    % North-East-Down (NED) Coordianate frame. This is x being positive to the
    % nose. Y positive to the right wing. And Z positive downwards.
    disp('Importing Flight Log IMU Data')
    M = importdata('flight.out', '	');
    imu.time       = M(:,1);
    imu.roll       = M(:,2);
    imu.pitch      = M(:,3);
    imu.yaw        = M(:,4);
    imu.euler321   = [imu.yaw imu.pitch imu.roll];
    imu.accel      = [-M(:,5), M(:,6), -M(:,7)]; % Coordinate frame Transformation neccessary
    switch( use_no_drift_gyro ),
        case 'no', % JSBSIM Model
            % First sensor is the raw values read from the jsbsim model
            % /fdm/jsbsim/velocities/pi-rad_sec
            imu.gyro   = [M(:,8), M(:,9), M(:,10)];
        case 'yes', % General Accurate Orientation Data
            % Gyro Rate Sensor 2 is much more precise and does not model 
            % reality as well since there is no drift. Would be ok to use
            % for general algorithm testing if we inject more noise. Sensor
            % used has values from /orientation/roll-rate-degps
            imu.gyro   = [M(:,11), M(:,12), M(:,13)];
    end
    imu.latitude   = M(:,14);
    imu.longitude  = M(:,15);
    imu.elevation  = M(:,16);
    
    % Generate Simulated Magnetometer Readings from heading (yaw)
    imu.mag = zeros(length(imu.time),3);
    for i=1:length(imu.time),
        imu.mag(i,:) = sim_magnetometer(imu.yaw(i), imu.pitch(i), imu.roll(i)); % Uses Actual Attitude to estimate Mxyz
    end
    
    figure(2)
    % Plot the raw sensor values to check for proper signs and coordinate frame
    subplot 411, plot(imu.time,angle_wrap(imu.euler321,'pitch','rad','deg'))
    legend('yaw','pitch','roll'); grid on;
    subplot 412, plot(imu.time, imu.accel)
    legend('A_x','A_y','A_z'); grid on;
    subplot 413, plot(imu.time,angle_wrap(imu.gyro,'pitch','rad','deg'))
    legend('w_x','w_y','w_z'); grid on;
    subplot 414, plot(imu.time, imu.mag)
    legend('Mag_x','Mag_y','Mag_z'); grid on;

    % Do we want to insert noise into system sensors
    if strcmp(insert_noise, 'yes'),
        imu.accel(:,1) = imu.accel(:,1) + accel_sd*randn(length(imu.time),1);
        imu.accel(:,2) = imu.accel(:,2) + accel_sd*randn(length(imu.time),1);
        imu.accel(:,3) = imu.accel(:,3) + accel_sd*randn(length(imu.time),1);
        imu.gyro(:,1)  = imu.gyro(:,1)  + gyro_sd*randn(length(imu.time),1);
        imu.gyro(:,2)  = imu.gyro(:,2)  + gyro_sd*randn(length(imu.time),1);
        imu.gyro(:,3)  = imu.gyro(:,3)  + gyro_sd*randn(length(imu.time),1);
        imu.mag         = imu.mag.*(ones(length(imu.time),3)+mag_sd/100*randn(length(imu.time),3));
    end
end

%% Calculate the variance of the sensor inputs
disp('Calculate or Set the Variance of each measurement sensor')
if strcmp(compute_variance, 'yes')
    disp('Calculating the Variance Based off first static samples')
    
    cal_start = 5;
    cal_stop = 75;
    v_gx = variance(imu.gyro(:,1), cal_start, cal_stop)
    v_gy = variance(imu.gyro(:,2), cal_start, cal_stop)
    v_gz = variance(imu.gyro(:,3), cal_start, cal_stop)

    v_ax = variance(imu.accel(:,1), cal_start, cal_stop)
    v_ay = variance(imu.accel(:,2), cal_start, cal_stop)
    v_az = variance(imu.accel(:,3), cal_start, cal_stop)
    
    v_mx =variance(imu.mag(:,1), cal_start, cal_stop)
    v_my = variance(imu.mag(:,2), cal_start, cal_stop)
    v_mz = variance(imu.mag(:,3), cal_start, cal_stop)
    
    for i=1:length(imu.mag)
        [est_pitch_accel, est_roll_accel] = accel2attitude(imu.accel(i,1), imu.accel(i,2), imu.accel(i,3));
        yaw(i) = magnetometer_compass(imu.mag(i,:), est_pitch_accel, est_roll_accel);
    end
    var_yaw = variance(yaw,m_start,m_stop)
    
    imu.variance.gyro  = mean([v_gx v_gy v_gz]); 
    imu.variance.accel = mean([v_ax v_ay v_az]);
    imu.variance.heading   = var_yaw;
else
    disp('Variance was set to constant configuration values.')
    clear imu.variance;
    %imu.variance.gyro    = 0.01;
    %imu.variance.accel   = 0.015;
    %imu.variance.heading = 0.002567094491980;
end

%% Sensor and Flight Models used in calculations below.
disp('Extended Kalman Filter Sensor and Flight Model Calculations')
% kf_out = run_filter(imu, imu.yaw(1));
kf_out = run_filter(imu);

%% Calculate the error in the Kalman Filter Estimate to True Attitude
disp('Calculate the Fitness of this EKF Estimate')
error_yaw     = sum(sqrt((imu.yaw   - kf_out.yaw).^2));
error_pitch   = sum(sqrt((imu.pitch - kf_out.pitch).^2));
error_roll    = sum(sqrt((imu.roll  - kf_out.roll).^2));
% Calculate a fitness scalar value. Less emphasis on yaw accuracy.
fitness = 0.5*error_yaw + error_pitch + error_roll;

%% Estimate of attitude given raw sensor data
disp('Estimate Attitude Orientations from Raw Sensor Values')
% What are the raw attitude estimates based off each sensor

% Get the pitch and roll estimates from raw accelerometer readings
est_pitch_accel = zeros(size(imu.time));
est_roll_accel = zeros(size(imu.time));
for i=1:length(imu.time),
    [est_pitch_accel(i), est_roll_accel(i)] = accel2attitude(imu.accel(i,1),imu.accel(i,2),imu.accel(i,3));
end

% Do the same now using the raw gyroscope values only by integrating
est_yaw_gyro   = zeros(size(imu.time));
est_pitch_gyro = zeros(size(imu.time));
est_roll_gyro  = zeros(size(imu.time));
% Initialize First Elements
est_yaw_gyro(1)   = imu.yaw(1);
est_pitch_gyro(1) = imu.pitch(1);
est_roll_gyro(1)  = imu.roll(1);
for i=2:length(imu.time),
    dt = imu.time(i) - imu.time(i-1);
    est_yaw_gyro(i)   = est_yaw_gyro(i-1)   + imu.gyro(i,3)*dt;
    est_pitch_gyro(i) = est_pitch_gyro(i-1) + imu.gyro(i,2)*dt;
    est_roll_gyro(i)  = est_roll_gyro(i-1)  + imu.gyro(i,1)*dt;
end

% Compute the heading from the magnetometer readings as well as from
% acceleromter data.
est_heading = zeros(length(imu.time),1);
for i=1:length(imu.time),
    est_heading(i) = magnetometer_compass(imu.mag(i,:), imu.pitch(i), imu.roll(i));
end


%% Plot the actual attitude of the Airframe
disp('Plot the actual attitude of the Airframe over time')
figure(1)
% x_plot_start = 110;
% x_plot_end   = 230;
subplot(3,1,1),
plot(imu.time,angle_wrap(imu.roll,'pitch','rad','deg'),'r',...
     imu.time,angle_wrap(kf_out.roll,'pitch','rad','deg'),'b',...
     imu.time,angle_wrap(est_roll_accel,'pitch','rad','deg'),'y',...
     imu.time,angle_wrap(est_roll_gyro,'pitch','rad','deg'),'g'); % Kalman Filtered Roll
title('Absolute Roll Angle');
xlabel('time (s)');
ylabel('degrees');
legend('Actual','EKF','Accelerometer Raw', 'Gyro Raw');
grid on
% xlim([x_plot_start x_plot_end])
subplot(3,1,2),
plot(imu.time,angle_wrap(imu.pitch,'pitch','rad','deg'),'r',...
     imu.time,angle_wrap(kf_out.pitch,'pitch','rad','deg'),'b',...
     imu.time,angle_wrap(est_pitch_accel,'pitch','rad','deg'),'y',...
     imu.time,angle_wrap(est_pitch_gyro,'pitch','rad','deg'),'g'); % Kalman Filtered Pitch
title('Absolute Pitch Angle');
xlabel('time (s)');
ylabel('degrees');
legend('Actual','EKF','Accelerometer Raw', 'Gyro Raw');
grid on
% xlim([x_plot_start x_plot_end])
subplot(3,1,3),
plot(imu.time,angle_wrap(imu.yaw,'yaw','rad','deg'),'r',...
     imu.time,angle_wrap(kf_out.yaw,'yaw','rad','deg'),'b',...
     imu.time,angle_wrap(est_heading,'yaw','rad','deg'),'y',...
     imu.time,angle_wrap(est_yaw_gyro,'yaw','rad','deg'),'g'); % Kalman Filtered Yaw
title('Absolute Yaw Angle');
xlabel('time (s)');
ylabel('degrees');
legend('Actual','EKF', 'Magnetometer', 'Gyro Raw');
grid on
% xlim([x_plot_start x_plot_end])


%% Quaternion Library Test
if strcmp(run_library_tests, 'yes'),
    disp('Quaternion Math and Utility Routine Test Functions')
    % Test conversion to and from euler to quaternions
    yaw_test = 5*D2R
    pitch_test = 19.555*D2R
    roll_test = 20*D2R
    
    q = euler2quat(yaw_test, pitch_test, roll_test) % Patricks Implementation
    euler = quat2euler(q);
    euler = euler.*R2D
    
    q1 = qmult([1 2 3 4], [1 2 4 8])
    
    q = qnorm([1 2 4 8]) % should equal 9.2195 = sqrt(85)
    
    for i=1:length(imu.yaw),
        Mxyz(i,:) = sim_magnetometer(imu.yaw(i), imu.pitch(i), imu.roll(i)); % Uses Actual Attitude to estimate Mxyz
        h1(i) = magnetometer_compass(Mxyz(i,:), imu.pitch(i), imu.roll(i)); % Theses should be estimated values
        h2(i) = imu.yaw(i);
    end
    
    % Find the bias in the first measurement
    h1 = h1 - (h1(1) - imu.yaw(1));
    
    figure(3);
    plot(imu.time,h1*R2D,'r',imu.time, imu.yaw*R2D, 'b')
    
    figure(4);
    plot(imu.time, h1*R2D,'g');
    
    %% Test Euler2Quat and Quat2Euler functions over all values
    start = -pi;
    stop = pi;
    incr = 11*D2R;
    
    success = 0;
    fail = 0;
    for i = start:incr:stop,
        for j = start:incr:stop,
            for k = start:incr:stop,
                q = euler2quat(i, j, k); % Patricks Implementation
%                 q = angle2quat(i, j, k, 'ZYX');
                [yawe pitche rolle] = quat2euler(q);
%                 [yawe pitche rolle] = quat2angle(q, 'ZYX');
                ang1 = angle_wrap(real([i j k]),'yaw','rad','deg');
                ang2 = angle_wrap(real([yawe pitche rolle]),'yaw','rad','deg');
                
                % Calculate the error
                error = sqrt(sum((ang1 - ang2).^2));
                if( error > 1 ), % Limit to 1 degree error
                    ang1;
                    ang2;
                    fail = fail + 1;
                else
                    success = success + 1;
                end
            end
        end
    end
    success
    fail
    percent_pass = 100*success/fail
    
    
    % Test the diffirent quaternion to euler representations and see if
    % they are computationally the same
    q = rand(5,4)
    %yaw measurements
    y1 = q(:,1).^2 + q(:,2).^2 - q(:,3).^2 - q(:,4).^2
    y2 = 2*q(:,1).^2 + 2*q(:,2).^2 - 1
    y3 = 1 - 2 .* (q(:,3).^2 + q(:,4).^2)
    diff_y12 = sum((y1-y2).^2)
    diff_y13 = sum((y1-y3).^2)
    diff_y23 = sum((y2-y3).^2)
    
end


%% Restore the Random Stream for repeatable results
if strcmp(save_random_seed,'yes'),
    defaultStream.State = savedState;
end
disp('Completed Simulation Sucessfully');