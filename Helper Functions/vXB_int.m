function [ vxBds ] = vXB_int(theta, r, coord, mMag_bod, xMag_bod, omMag_bod,vMag_bod, axis)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


%YZ plane 
if strcmp(axis,'x') == 1;
    x_bod = coord;
    y_bod = r.*cos(theta);
    z_bod = r.*sin(theta);
    ds_y = -sin(theta);
    ds_z = cos(theta);
    yflux = vxB(x_bod,y_bod,z_bod,mMag_bod,xMag_bod,omMag_bod,vMag_bod,'y');
    zflux = vxB(x_bod,y_bod,z_bod,mMag_bod,xMag_bod,omMag_bod,vMag_bod,'z');
    vxBds = ds_y.*yflux+ds_z.*zflux;
    

%XZ plane 
elseif strcmp(axis,'y') == 1;
    x_bod = r.*cos(theta);
    y_bod = coord;
    z_bod = r.*sin(theta);
    ds_x = -sin(theta);
    ds_z = cos(theta);
    xflux = vxB(x_bod,y_bod,z_bod,mMag_bod,xMag_bod,omMag_bod,vMag_bod,'x');
    zflux = vxB(x_bod,y_bod,z_bod,mMag_bod,xMag_bod,omMag_bod,vMag_bod,'z');
    vxBds = ds_x.*xflux+ds_z.*zflux;

%XY plane 
elseif strcmp(axis,'z') == 1;
    x_bod = r.*cos(theta);
    y_bod = r.*sin(theta);
    z_bod = coord;
    ds_x = -sin(theta);
    ds_y = cos(theta);
    xflux = vxB(x_bod,y_bod,z_bod,mMag_bod,xMag_bod,omMag_bod,vMag_bod,'x');
    yflux = vxB(x_bod,y_bod,z_bod,mMag_bod,xMag_bod,omMag_bod,vMag_bod,'y');
    vxBds = ds_x.*xflux+ds_y.*yflux;
else
    error('incorrect axis inputs');
end


end

