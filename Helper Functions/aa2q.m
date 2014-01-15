function [ q ] = aa2q( v, theta )
%vA2Q takes a vector and an angle to rotate around that vector and converts
%it into a QuARK form quaternion
[n,m] = size(v);
if (n+m ~= 4 || mod(n,2) ~= 1 || mod(m,2) ~= 1 )
    error('invalid argument vector');
end
if n == 3
    v = v';
end
vNorm = v/norm(v);

q = [vNorm*sin(theta/2),cos(theta/2)];

end

