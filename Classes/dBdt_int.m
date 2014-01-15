function [dBdt] = dBdt_int( theta,r, constCoord, mag_dBdt, magM, magX, axis)
%out is the dB_dt perpendicular to the indicated plane assume all inputs
%are already in body coordinates

%YZ plane 
if strcmp(axis,'x') == 1;
    x_bod = constCoord;
    y_bod = r.*cos(theta);
    z_bod = r.*sin(theta);
    flux = magFlux(x_bod,y_bod,z_bod,magX,magM,axis);
    dBdt = mag_dBdt*flux.*r;
    

%XZ plane 
elseif strcmp(axis,'y') == 1;
    x_bod = r.*cos(theta);
    y_bod = constCoord;
    z_bod = r.*sin(theta);
    flux = magFlux(x_bod,y_bod,z_bod,magX,magM,axis);
    dBdt = mag_dBdt*flux.*r;
    


%XY plane 
elseif strcmp(axis,'z') == 1;
    x_bod = r.*cos(theta);
    y_bod = r.*sin(theta);
    z_bod = constCoord;
    flux = magFlux(x_bod,y_bod,z_bod,magX,magM,axis);
    dBdt = mag_dBdt*flux.*r;
else
    error('incorrect axis inputs');
end

%pos = [x;y;z];


% x component of dBdt perpendicular to the YZ plane 
%if plane == 1
%    dBdt = dBdt(1);
%    % y component of dBdt perpendicular to the XZ plane 
%elseif plane == 2
%    dBdt = dBdt(2);
%    % z component of dBdt perpendicular to the XY plane 
%elseif plane == 3
%    dBdt = dBdt(3);


end
