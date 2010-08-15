function [ angle ] = angle_wrap( angle, type, unit_in, unit_out )
%angle_wrap Summary of this function goes here
%   Detailed explanation goes here

if nargin < 2
    type = 'yaw';
end
if nargin < 3
    unit_in = 'rad';
end
if nargin < 4
    unit_out = unit_in;
end

if strcmp(unit_in,'deg'),
    angle = angle.*pi./180;
end

[lenx leny] = size(angle);

for i=1:lenx,
    for j=1:leny,
        switch(lower(type)),
            case 'yaw',
                if angle(i,j) >= 2*pi,
                    angle(i,j) = angle(i,j) - 2*pi;
                elseif angle(i,j) < 0,
                    angle(i,j) = angle(i,j) + 2*pi;
                end
            otherwise % 'pitch'
                if angle(i,j) > pi,
                    angle(i,j) = angle(i,j) - 2*pi;
                elseif angle(i,j) < -pi,
                    angle(i,j) = angle(i,j) + 2*pi;
                end
        end
    end
end

if strcmp(unit_out,'deg'),
    angle = angle.*180./pi;
end

end

