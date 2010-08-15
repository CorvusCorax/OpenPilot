function [ q ] = euler2quat( yaw, pitch, roll )
%euler2quat NED coordinate system
%   Detailed explanation goes here
% http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/index.htm
% heading = pitch;
% attitude = yaw;
% bank = roll;
% 
% c1 = cos(heading/2);
% s1 = sin(heading/2);
% c2 = cos(attitude/2);
% s2 = sin(attitude/2);
% c3 = cos(bank/2);
% s3 = sin(bank/2);
% 
% c1c2 = c1*c2;
% s1s2 = s1*s2;
% 
% w = c1c2*c3 - s1s2*s3;
% x = c1c2*s3 + s1s2*c3;
% y = s1*c2*c3 + c1*s2*s3;
% z = c1*s2*c3 - s1*c2*s3;
% 
% q2 = [w x y z];

% % Second Method
% Qx = [cos(roll/2)   sin(roll/2)  0             0			];
% Qy = [cos(pitch/2)  0            sin(pitch/2)  0			];
% Qz = [cos(yaw/2) 	  0            0             sin(yaw/2)	];
% 
% % Rotation in order x->y->z
% Q1 = qmult(Qy,Qz);
% Q2 = qmult(Qx,Q1);
% 
% q2 = [Q2(1,1) Q2(1,2) Q2(1,3) Q2(1,4)];
% 
% 

% Aero Toolpack Method
angles = [yaw pitch roll];
cang = cos( angles/2 );
sang = sin( angles/2 );

% Could be wrong the the code right below.
% q = [ cang(:,1).*cang(:,2).*cang(:,3) + sang(:,1).*sang(:,2).*sang(:,3), ...
%     cang(:,1).*cang(:,2).*sang(:,3) - sang(:,1).*sang(:,2).*cang(:,3), ...
%     cang(:,1).*sang(:,2).*cang(:,3) + sang(:,1).*cang(:,2).*sang(:,3), ...
%     sang(:,1).*cang(:,2).*cang(:,3) - cang(:,1).*sang(:,2).*sang(:,3)];

warning('euler2quat: is not properly tested and seems to have errors')
%http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/index.htm
q = [ cang(:,1).*cang(:,2).*cang(:,3) - sang(:,1).*sang(:,2).*sang(:,3), ...
    cang(:,1).*cang(:,2).*sang(:,3) + sang(:,1).*sang(:,2).*cang(:,3), ...
    sang(:,1).*cang(:,2).*cang(:,3) + cang(:,1).*sang(:,2).*sang(:,3), ...
    cang(:,1).*sang(:,2).*cang(:,3) - sang(:,1).*cang(:,2).*sang(:,3)]

% Aero Tookkit Functin angle2eulerZYX
% q = [ sang(:,1).*cang(:,2).*cang(:,3) - cang(:,1).*sang(:,2).*sang(:,3),...
%       cang(:,1).*cang(:,2).*cang(:,3) + sang(:,1).*sang(:,2).*sang(:,3), ...
%       cang(:,1).*cang(:,2).*sang(:,3) - sang(:,1).*sang(:,2).*cang(:,3), ...
%       cang(:,1).*sang(:,2).*cang(:,3) + sang(:,1).*cang(:,2).*sang(:,3), ...
%     ];

% % Alternate Method Base Quaternion Library
% tic
% Qx = [cos(roll/2)  sin(roll/2) 	 0            0	    ];
% Qy = [cos(pitch/2)      0      sin(pitch/2)   0     ];
% Qz = [cos(yaw/2)  	  0          0        sin(yaw/2)];
% Q1 = qmult(Qy,Qx);
% q  = qmult(Qz,Q1);
% toc

end

