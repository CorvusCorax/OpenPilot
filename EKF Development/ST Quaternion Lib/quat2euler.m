function [ yaw pitch roll ] = quat2euler( q )
%quat2euler Here is how you would go about converting the quaternion into
% an Euler 321 sequence (yaw, pitch, roll).
% Author: Patrick Glass
% Date Modified: July 21, 2010
% http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
          
q = qnormalize( q );

yaw = atan2(2.*(q(1).*q(4) + q(2).*q(3)), 1 - 2*(q(3).^2 + q(4).^2));
pitch = asin( 2.*(q(1).*q(3) - q(4).*q(2)) );
roll = atan2(2.*(q(1).*q(2) + q(3).*q(4)),1 - 2*(q(2).^2 + q(3).^2));

end

