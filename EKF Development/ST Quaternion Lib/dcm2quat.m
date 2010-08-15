function [ q ] = dcm2quat( C )
%dcm2quat directional cosine matrix to a quaternion
% Author: Patrick Glass
% Date Modified: May 26, 2010

q3 = sqrt( C(1,1) + C(2,2) + C(3,3) + 1 );
q0 = (C(2,3) - C(3,2)) / 4*q3;
q1 = (C(3,1) - C(1,3)) / 4*q3;
q2 = (C(1,2) - C(2,1)) / 4*q3;

q = [q0 q1 q2 q3];

end

