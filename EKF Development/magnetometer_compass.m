function [ azimuth ] = magnetometer_compass( mag_xyz, pitch, roll )
%MAGNETOMETER_COMPASS Takes in sensor readings from a magnetometer and
%generates a estimated heading based off of this information and the pitch
%and roll from the attitude. The azimuth is returned in units of radians.
%Pitch and roll are in units of radians. 10^4 gauss = 1 Tesla.
%   see also: ACCEL2ATTITUDE
%
%   Author: Patrick Glass
%   Date:   June 16, 2010
%
%   Usage 
%      heading = magnetometer_compass([Mx My Mz], accel_pitch, accel_roll)

Mx = mag_xyz(1);
My = mag_xyz(2);
Mz = mag_xyz(3);

X = Mx*cos(pitch) + My*sin(roll)*sin(pitch) + Mz*cos(roll)*sin(pitch);
Y = My*cos(roll) - Mz*sin(roll); % TODO: Some papers say this should be a pos sign

azimuth = angle_wrap(atan2(Y, X),'yaw');

end

