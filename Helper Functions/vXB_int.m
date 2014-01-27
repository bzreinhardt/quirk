function [ vxBds ] = vXB_int(theta, r, coord, mMag_bod, xMag_bod, omMag_bod,vMag_bod, axis)
%vXB_int finds the cross product of velocity and magnetic field dotted with
%a point on a circular path
%   Inputs: 
%   theta: position on circle in radians
%   r: radius of the circle
%   coord: position in body frame along the axis perp to the circle
%   mMag_bod - magnet dipole vector in body coordinates
%   xMag_bod - magnet position vector in body coordinates
%   omMag_bod - magnet angular velocity in body coordinates
%   vMag_bod - magnet linear velocity in body coordinates
%   axis - string indicated which body axis is perpendicular to the circle

%make sure theta is a row vector
if size(theta,2) == 1
    theta = theta';
end

perpCoord = coord*ones(1,size(theta,2)); %coordinate perpendicular to the plane is constant
h_coord = r.*cos(theta); %coordinate on the 'horizontal axis'
v_coord = r.*sin(theta); %coordinate on the 'vertical axis'
h_ds = -sin(theta);
v_ds = cos(theta);
perp_ds = zeros(size(perpCoord));

%YZ plane 
if strcmp(axis,'x') == 1;
    
    %     x_bod = perpCoord;
    %     y_bod = h_coord;
    %     ds_y = h_ds;
    %     z_bod = v_coord;
    %     ds_z = v_ds;
    
    %     yflux = vxB(x_bod,y_bod,z_bod,mMag_bod,xMag_bod,omMag_bod,vMag_bod,'y');
    %     zflux = vxB(x_bod,y_bod,z_bod,mMag_bod,xMag_bod,omMag_bod,vMag_bod,'z');
    X = [perpCoord; h_coord; v_coord];
    ds = [perp_ds; h_ds; v_ds];
    


%XZ plane 
elseif strcmp(axis,'y') == 1;
    %      y_bod = perpCoord;
    %
    %     x_bod = v_coord;
    %     ds_x = v_ds;
    %
    %     z_bod = h_coord;
    %     ds_z = h_ds;
    %
    %     xflux = vxB(x_bod,y_bod,z_bod,mMag_bod,xMag_bod,omMag_bod,vMag_bod,'x');
    %     zflux = vxB(x_bod,y_bod,z_bod,mMag_bod,xMag_bod,omMag_bod,vMag_bod,'z');
    %     vxBds = ds_x.*xflux+ds_z.*zflux;
    X = [v_coord; perpCoord; h_coord];
    ds = [ v_ds; perp_ds; h_ds];
    

%XY plane 
elseif strcmp(axis,'z') == 1;
    %     x_bod = h_coord;
    %     ds_x = h_ds;
    %     y_bod = v_coord;
    %      ds_y = v_ds;
    %     z_bod = perpCoord;
    %
    %
    %     xflux = vxB(x_bod,y_bod,z_bod,mMag_bod,xMag_bod,omMag_bod,vMag_bod,'x');
    %     yflux = vxB(x_bod,y_bod,z_bod,mMag_bod,xMag_bod,omMag_bod,vMag_bod,'y');
    %     vxBds = ds_x.*xflux+ds_y.*yflux;
    X = [h_coord;  v_coord; perpCoord];
    ds = [ h_ds; v_ds; perp_ds];
else
    error('incorrect axis inputs');
end
cross = vxB(X,xMag_bod,mMag_bod,vMag_bod);
    vxBds = dot(cross,ds,1);

end

