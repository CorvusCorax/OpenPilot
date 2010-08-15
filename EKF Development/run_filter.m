function [ out ] = run_filter( imu_data, init_heading )
%run_filter Summary of this function goes here
%   Detailed explanation goes here

if nargin < 2,
    init_heading = 0;
end

% This section describes the model that was used to describe the system below.
% It should give some insight to the conventions used and naming scheme.
out.attitude = zeros(length(imu_data.time),4);
out.yaw   = zeros(length(imu_data.time),1);
out.pitch = zeros(length(imu_data.time),1);
out.roll  = zeros(length(imu_data.time),1);
for i=1:length(imu_data.time),
    out.attitude(i,:) = filter_update(imu_data.time(i), ...
                           imu_data.gyro(i,:), ...
                           imu_data.accel(i,:), ...
                           imu_data.mag(i,:), init_heading, ...
                           imu_data.variance);
    
    % Convert the Quaternion attitude to euler angles for displaying
    [out.yaw(i) out.pitch(i) out.roll(i)] = quat2euler( out.attitude(i,:) );
end

out.euler321 = [out.yaw out.pitch out.roll];
end

