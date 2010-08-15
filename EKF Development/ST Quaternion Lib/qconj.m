function [ q ] = qconj( q )
%qconj Summary of this function goes here
%   
% Author: Patrick Glass
% Date Modified: Aug 2, 2010

q = [ q(1)  -q(2:4)' ];

end

