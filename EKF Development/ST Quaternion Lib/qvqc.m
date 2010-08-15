function [ vec ] = qvqc( quat, vector )
%QVQC Rotation on Vector defined by Quaternion Rotation
%   Detailed explanation goes here


q = qnormalize(quat);
q_vec = [0 vector];

v_new = qmult(qmult(q, q_vec), qconj(q));
vec = [v_new(2) v_new(3) v_new(4) ];

end

