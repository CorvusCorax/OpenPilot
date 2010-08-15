function [ q ] = qmult( a, b )
%qmult Summary of this function goes here
%   Detailed explanation goes here
% Author: Patrick Glass
% Date Modified: June 7, 2010

           
 % Calculate vector portion of quaternion product
% vec = s1*v2 + s2*v1 + cross(v1,v2)
% vec = [a(1).*b(2), a(1).*b(3), a(1).*b(4)] + ...
%       [b(1).*a(2), b(1).*a(3), b(1).*a(4)] + ...
%       [a(3).*b(4)-a(4).*b(3), ...
%        a(4).*b(2)-a(2).*b(4), ...
%        a(2).*b(3)-a(3).*b(2)];
% 
% % Calculate scalar portion of quaternion product
% % scalar = s1*s2 - dot(v1,v2)
% scalar = a(1).*b(1) - a(2).*b(2) - ...
%          a(3).*b(3) - a(4).*b(4);
%     
% qout = [scalar  vec]; 

% % Q0 and Q1 shoud be in this order
% % Q0=[w0 x0 y0 z0] % w0 is scalar part, x0,y0,z0 are vector part
% % Q1=[w1 x1 y1 z1] % w1 is scalar part, x1,y1,z1 are vector part
% % Multiplication is not commutative in that the products Q0Q1 and Q1Q0 are
% % not necessarily equal.
% w0=Q0(1); x0=Q0(2); y0=Q0(3); z0=Q0(4); 
% w1=Q1(1); x1=Q1(2); y1=Q1(3); z1=Q1(4); 
% 
% wr=(w0.*w1 - x0.*x1 - y0.*y1 - z0.*z1);
% xr=(w0.*x1 + x0.*w1 + y0.*z1 - z0.*y1);
% yr=(w0.*y1 - x0.*z1 + y0.*w1 + z0.*x1);
% zr=(w0.*z1 + x0.*y1 - y0.*x1 + z0.*w1);
% 
% QMR=[wr xr yr zr]; % wr is scalar part, xr, yr, zr are vector part

% http://en.wikipedia.org/wiki/Quaternion           
q = [a(1)*b(1) - a(2)*b(2) - a(3)*b(3) - a(4)*b(4), ...
     a(1)*b(2) + a(2)*b(1) + a(3)*b(4) - a(4)*b(3),...
     a(1)*b(3) - a(2)*b(4) + a(3)*b(1) + a(4)*b(2),...
     a(1)*b(4) + a(2)*b(3) - a(3)*b(2) + a(4)*b(1)];

end

