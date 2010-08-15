function [ var ] = variance( array, start, stop )
%VARIANCE Calculate the variance of an given array.
%   Usage: variance(imu.accel(:,1), a_start, a_stop)

var = mean((array(start:stop) - mean(array(start:stop))).^2);

end

