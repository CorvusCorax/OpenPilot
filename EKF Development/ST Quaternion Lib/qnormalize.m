function q = qnormalize( q )
%  QNORMALIZE Normalize a quaternion.
%   N = QUATNORMALIZE( Q ) calculates the normalized quaternion, N, for a
%   given quaternion, Q.  Input Q is an M-by-4 matrix containing M
%   quaternions.  N returns an M-by-4 matrix of normalized quaternions.
%   Each element of Q must be a real number.  Additionally, Q has its
%   scalar number as the first column.
%
%   Examples:
%
%   Normalize q = [1 0 1 0]:
%      normal = qnormalize([1 0 1 0])

q = q./qnorm(q);
