function [ n, theta ] = q2aa(q)
%q2aa converts a quaternion to [n,theta] where n is a unit vector and theta
%is a rotation around that vector. n is of the form [x;y;z]. Assume q is of
%the form of a row vector and uses the quirk convention of 'scalar last'
%   Detailed explanation goes here
theta = 2*acos(q(4));
if norm(q(1:3)) == 0
    n = [1;0;0];
else
    n = q(1:3)'/norm(q(1:3));
end

end

