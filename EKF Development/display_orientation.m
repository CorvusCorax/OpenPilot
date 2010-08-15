function [ q_disp ] = display_orientation( q )
%display_orientation Takes the Quaternion orientation estimate and displays
%it in 3D.
%   Detailed explanation goes here

% Constants describing look of chart
% axisLength = 1
% axisWidth = 1

q_disp = qconj(q);
q_disp = qmult(q_disp, q_EB);

% Compute the Euler Angles
[yaw pitch roll] = quat2euler(q_disp);

% Compute the Rotation Matrix


% Display the coordinate frame
% % line([0 1],[0 0],[0 0],'Color','r','LineWidth',4)
% % line([0 0],[0 -1],[0 0],'Color','b','LineWidth',4)
% % line([0 0],[0 0],[0 -1],'Color','g','LineWidth',4)
% % 
% % % line([0 cos(pitch)*cos(yaw)],[0 0],[0 0],'Color','r','LineWidth',2)
% % % line([0 0],[0 -cos(pitch)*cos(yaw)],[0 0],'Color','r','LineWidth',2)
% % % line([0 0],[0 0],[0 -cos(pitch)*cos(roll)],'Color','r','LineWidth',2)
% % 
% % % Define Rotation Matrix (DCM) Vehicle Reference (VCS) Frame to GCS
% % R = [cos(roll)*cos(pitch) cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw) cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw);
% %      sin(roll)*cos(pitch) sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw) sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw);
% %      -sin(pitch)          cos(pitch)*sin(yaw)                             cos(pitch)*cos(yaw)]
% % 
% % x = [1 0 0]; y = [0 -1 0]; z = [0 0 -1];
% % xl = R*x'
% % yl = R*y'
% % zl = R*z'
% % % xl = x*R
% % % yl = y*R
% % % zl = z*R
% % 
% % data = eye(3)
% % res = data*R
% % 
% % line([0 xl(1)],[0 xl(2)],[0 xl(3)],'Color','r','LineWidth',2)
% % line([0 yl(1)],[0 yl(2)],[0 yl(3)],'Color','b','LineWidth',2)
% % line([0 zl(1)],[0 zl(1)],[0 zl(3)],'Color','g','LineWidth',2)
% % 
% % grid on;
% % view(-70,20)



% Plot 3D Euler Angle Rotation
% figure(2); clf
% disp('3D Rotation Testing')
% yaw = 90*pi/180
% pitch = 0*pi/180
% roll = 0*pi/180
% 
% line([0 1],[0 0],[0 0],'Color','r','LineWidth',4)
% line([0 0],[0 -1],[0 0],'Color','b','LineWidth',4)
% line([0 0],[0 0],[0 -1],'Color','g','LineWidth',4)
% 
% % line([0 cos(pitch)*cos(yaw)],[0 0],[0 0],'Color','r','LineWidth',2)
% % line([0 0],[0 -cos(pitch)*cos(yaw)],[0 0],'Color','r','LineWidth',2)
% % line([0 0],[0 0],[0 -cos(pitch)*cos(roll)],'Color','r','LineWidth',2)
% 
% % Define Rotation Matrix (DCM) Vehicle Reference (VCS) Frame to GCS
% R = [cos(roll)*cos(pitch) cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw) cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw);
%      sin(roll)*cos(pitch) sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw) sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw);
%      -sin(pitch)          cos(pitch)*sin(yaw)                             cos(pitch)*cos(yaw)]
% 
% x = [1 0 0]; y = [0 -1 0]; z = [0 0 -1];
% xl = R*x'
% yl = R*y'
% zl = R*z'
% % xl = x*R
% % yl = y*R
% % zl = z*R
% 
% data = eye(3)
% res = data*R
% 
% line([0 xl(1)],[0 xl(2)],[0 xl(3)],'Color','r','LineWidth',2)
% line([0 yl(1)],[0 yl(2)],[0 yl(3)],'Color','b','LineWidth',2)
% line([0 zl(1)],[0 zl(1)],[0 zl(3)],'Color','g','LineWidth',2)
% 
% grid on;
% view(-70,20)


figure
y = 1* exp(1j * yaw );
subplot 131; compass(y);
p = 1* exp(1j * pitch );
subplot 132; compass(p);
r = 1* exp(1j * roll );
subplot 133; compass(r);

end

