function [ b ] = sim_magnetometer( yaw, pitch, roll )
%sim_magnetometer Simulate a Magnetometer sensor given the current
%orientation of the body. There is in some situations multiple solutions
%for a given input which yields some issues. Further testing is required.
%   Detailed explanation goes here

% Constants of Magnetic Field Strength in NED Coordinate Frame
% http://www.ngdc.noaa.gov/geomag/magfield.shtml
% http://www.ngdc.noaa.gov/geomagmodels/struts/calcIGRFWMM
% Values from: Canada, Vancouver : 0m Elevation
% MAG = [17587.2, 5556.7, 51839.9]; % nT North, East, Down Components
% 1000m Elevation
% MAG = [17578.5, 5552.8, 51814.2]; % nT North, East, Down Components
MAG = [17587.2, 5556.7, 51839.9]; % nT North, East, Down Components

% http://contentdm.lib.byu.edu/ETD/image/etd1527.pdf
bxyz = [cos(pitch) .* cos(yaw), ...
        cos(pitch) .* sin(yaw), ...
        -sin(pitch); ...
       (sin(roll) .* sin(pitch) .* cos(yaw)) - (cos(roll) .* sin(yaw)), ...
       (sin(roll) .* sin(pitch) .* sin(yaw)) + (cos(roll) .* cos(yaw)), ...
        sin(roll) .* cos(pitch); ...
       (cos(roll) .* sin(pitch) .* cos(yaw)) + (sin(roll) .* sin(yaw)), ...
       (cos(roll) .* sin(pitch) .* sin(yaw)) - (sin(roll) .* cos(yaw)), ...
        cos(roll) .* cos(pitch);];
   
% Compute the Magnetic Field Intensity of Each Body Frame Axis NED
b = bxyz * MAG';

end

