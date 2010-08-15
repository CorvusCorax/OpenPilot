function n = qnorm( q )
%  QNORM Calculate the norm of a quaternion.
%   N = QORM( Q ) calculates the norm, N, for a given quaternion, Q.  Input
%   Q is an M-by-4 matrix containing M quaternions.  N returns a column vector
%   of M norms.  Each element of Q must be a real number.  Additionally, Q has
%   its scalar number as the first column.
%
%   Examples:
%
%   Determine the norm of q = [1 0 0 0]:
%      norm = qnorm([1 0 0 0])
%
%   Copyright 2010 SwissTech Consulting
%   Revision: 1.0     Date: June 7, 2010

n = sqrt(sum(q.^2));