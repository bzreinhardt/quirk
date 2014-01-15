function [ out ] = att2vec( att, varargin )
%att2vec converts the attitude of a qurik body in quark to a unit
%vector that points along body axes of that body

matrix = quat2dcm(qq2mq(att));
qmatrix = inv(matrix);

xVec = qmatrix*[1;0;0];
yVec = qmatrix*[0;1;0];
zVec = qmatrix*[0;0;1];

if numel(varargin) == 0
    out = [xVec, yVec, zVec];
elseif strcmp(varargin,'x') == 1
    out = xVec;
elseif strcmp(varargin,'y') == 1
    out = yVec;
elseif strcmp(varargin,'z') == 1
    out = zVec;
end


end

