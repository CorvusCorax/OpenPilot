%% Sparkfun 9DOM IMU v4 Interface
% Author: Patrick Glass
% Date Modified: June 23, 2010
% Current Version: 1.0

% Future Development
%    Rev 1.0: Basic Flight Model for static testing with Gyro, Accel, Mag
%    Rev 1.1: Include interface for forward direction to compensate for
%             centripital acceleration.
% clear, clc
disp(' ')
disp('Starting 6DOF IMU Data Sampling')
colordef black
format compact
R2D = 180/pi;
D2R = pi/180;


% Sample Frequency
f_sample = 50; % 100 Hz sample frequency

%% Configuration
serial_port = 'COM12';
sample_duration = 240;   % Seconds
in_binary = 'no';


% Get the serial port
s = serial(serial_port,'BAUD',115200,'timeout',45);
fopen(s);
if strcmp(s.Status, 'closed')
    error('Serial Port could not be opened!')
else
    disp('Serial Port conected succefully');
end
    
% Ensure that we are sampling (sometimes autostart does not work)
if strcmp(in_binary,'no')
    fprintf(s, '99999');
else
    fprintf(s, ' &*#9'); % 2g, 100Hz, Start
end


%% Read in some samples
error = 0; success = 0;
disp('Samples remainging:')
total = f_sample*sample_duration;
for i=1:total,
    remaining = total-i
    if strcmp(in_binary, 'no')
        % ASCII Mode
        temp = fscanf(s,'%s %d %d %d %d %d %d %d %d %d %d %s',[12 1])
        
        try
            if(temp(1) == 65 && temp(12) == 90)
                data(i,:) = temp';
                success = success + 1;
            end
            error = error + 1;
        catch exception
            error = error + 1;
        end
    else
        % Binary Mode
        temp = fscanf(s,'%s %d %d %d %d %d %d %d %d %d %d %s',[12 1])
        
        try
            if(temp(1) == 65 && temp(12) == 90)
                data(i,:) = temp';
                success = success + 1;
            end
            error = error + 1;
        catch exception
            error = error + 1;
        end
    end
end


%% Stop the sampling
fprintf(s, '     ');
% data(i) = fscanf(s,'%d',[12 1])


%%
fclose(s);
fclose(s);
fclose(s);
% delete(s);
% clear s;


%% Plot raw sampled data and correct for biases ans such

data = imu

% Run linear correction equations on raw measurements
% Correct for biases
plot( imu(:, 3:11) )

% Extract the Sensor Parameters
time = imu(:,2);
mag = imu(:,3:5);
a = imu(:,6:8);
w = imu(:,9:11);

% Correct for orientation frame
% The sparkfun IMU defines its gyro roll as y axis and pitch at x axis the
% extended kalman filter designed uses the convention of x-roll, y-pitch,
% and z-yaw.

% Plot the raw sensor values to check for proper signs and coordinate frame
% subplot 411, %%plot(time,angle_wrap([roll pitch yaw],'pitch','rad','deg'))
% legend('roll','pitch','yaw'); grid on;
subplot 311, plot(time,a)
legend('A_x','A_y','A_z'); grid on;
%     subplot 413, plot(time,angle_wrap([roll_dot pitch_dot yaw_dot],'pitch','rad','deg'))
subplot 312, plot(time, w)
legend('w_x','w_y','w_z'); grid on;
subplot 313, plot(time, mag)
legend('Mag_x','Mag_y','Mag_z'); grid on;





%% Run the extended Kalman Filter on Data





disp('Completed Simulation Sucessfully');