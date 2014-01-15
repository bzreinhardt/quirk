function [ zVec ] = att2zvec( att )
%att2zvec converts the attitude of a cylinder body in quark to a unit
%vector that points along the z axis of the cylinder

matrix = quat2dcm(qq2mq(att));
qmatrix = inv(matrix);
zVec = qmatrix*[0;0;1];


end

