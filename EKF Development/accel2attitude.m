function [ pitch, roll ] = accel2attitude( Ax, Ay, Az )
%ACCEL2ATTITUDE Takes in a reading of gravity vector on a static body and
%calculated the pitch and roll. This does not take into account centripital
%acceleration. Pitch and roll are returned in units of radians.
%
%   see also: MAGNETOMETER_COMPASS
%
%   Author: Patrick Glass
%   Date:   May 27, 2010
%
%   Usage
%      Ax = 0; Ay = 0; Az = 9.8;
%      [pitch, roll] = accel2attitude(Ax, Ay, Az)
%      % Should return a pitch of 0 and roll of 0

roll  =  atan(Ay/Az);
pitch = -atan(Ax/sqrt(Ay^2+Az^2));
%pitch = asin(-Ax/sqrt(Ax^2+Ay^2+Az^2)); % Master Thesis Mathieu same as
%above expressino but uses sin instead

end

