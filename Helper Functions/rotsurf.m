%function [X2,Y2,Z2] = rotsurf(X,Y,Z,rotation);
%
% Rotates surface triples (from Mason, modified by Joe)
%
% rotation  3x3 matrix      rotation matrix to apply
%           3x1 mat         axis and angle (angle in rad = norm of matrix)
%           4x1 mat         quaternion to apply
%

% Originally written by Mason Peck, Cornell University
% Included as a supporting function for QuIRK
function [X2,Y2,Z2]=rotsurf(X,Y,Z,rot)

if numel(rot) == 4
    Q = q2d(rot);
elseif numel(rot) == 3
    Q = expm(-crs(rot));
elseif numel(rot) == 9
    Q = rot;
else
    Q = eye(3);
end

[n,m]=size(X);
XR=X(1:end);
YR=Y(1:end);
ZR=Z(1:end);

XYZ=Q*[XR; YR; ZR];

X2=XYZ(1,:);
Y2=XYZ(2,:);
Z2=XYZ(3,:);

X2=reshape(X2,n,m);
Y2=reshape(Y2,n,m);
Z2=reshape(Z2,n,m);


