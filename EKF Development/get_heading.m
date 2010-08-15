function [ azimuth ] = get_heading( mag_xyz, q_body )
%GET_HEADING Takes in sensor readings from a magnetometer and
%generates a estimated heading based off of this information and the pitch
%and roll from the attitude. The azimuth is returned in units of radians.
%Pitch and roll are in units of radians. 10^4 gauss = 1 Tesla.
%   see also: ACCEL2ATTITUDE
%
%   Author: Patrick Glass
%   Date:   Aug 2, 2010
%
%   Usage 
%      heading = magnetometer_compass([Mx My Mz], q)

% warning('get_heading: This function does not work. Please dont use it until fixed');
mag_earth = qvqc(q_body, mag_xyz);
azimuth = angle_wrap( atan2( mag_earth( 2 ), mag_earth( 1 ) ), 'yaw' );

end
