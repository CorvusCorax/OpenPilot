function qout = qrotate( q, r )
%  QROTATE Rotate a vector by a quaternion.
%   N = QUATROTATE( Q, R ) calculates the rotated vector, N, for a
%   quaternion, Q, and a vector, R.  Q is either a M-by-4 matrix
%   containing M quaternions or a single 1-by4 quaternion.  R
%   is either a M-by-3 matrix or a single 1-by-3 vector.  N returns an
%   M-by-3 matrix of rotated vectors.  Each element of Q and R must be a
%   real number.  Additionally, Q has its scalar number as the first column.
%
%   Examples:
%
%      q = [1 0 1 0];
%      r = [1 1 1];
%      n = qrotate( q, r ) 
%
%      q = [1 0 1 0; 1 0.5 0.3 0.1];
%      r = [1 1 1];
%      n = qrotate( q, r ) 
%
%      q = [1 0 1 0];
%      r = [1 1 1; 2 3 4];
%      n = qrotate( q, r ) 
%
%      q = [1 0 1 0; 1 0.5 0.3 0.1];
%      r = [1 1 1; 2 3 4];
%      n = qrotate( q, r ) 
%

dcm = quat2dcm(q);
qout = (dcm*r')';
end