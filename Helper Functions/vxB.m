function [ vxB_out ] = vxB(X,xMag,mMag,vMag,varargin)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
%inputs:
%x,y,z are the positions in the current frame to find vxB
%mMag_bod is the magnet's dipole vector (magnitude and direction)
%xMag_bod is the magnet's position in the body frame
%omMag_bod is the magnet's angular velocity in the body frame
%vMag_bod is the magnet's linear velocity in the body frame
%axis is a scalar indicating which component we're interested in

%Assumption: v is a matrix of the velocities of the magnetic field at each
%point in X


%vel = vMag_bod - om_x*r;

if size(varargin) ~= 0
    axis = varargin;
end
if size(vMag,2) == 1 && size(X,2) ~=1
    vMag = vMag*ones(1,size(X,2));
end
% omegaX = [0 -omega(3) omega(2); 
%           omega(3) 0 -omega(1);
%           -omega(2) omega(1) 0];

B = magFlux(X,xMag,mMag);

%v_x = [0 -velz vely; velz 0 -velx; -vely velx 0];
%vCrossB = v_x*B;
vxB_out = cross(vMag,B,1);



if exist('axis','var')
    switch axis
        case 'x'
            vxB_out = vxB_out(1,:);
        case 'y'
             vxB_out = vxB_out(2,:);
        case 'z'
             vxB_out = vxB_out(3,:);
    end
end

