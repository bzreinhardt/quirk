function [dBdt] = vec_dBdt_int( theta,r, constCoord,  mag_dBdt, magM, magX, axis)
%out is the dB_dt perpendicular to the indicated plane assume all inputs
%are already in body coordinates

%YZ plane 
if strcmp(axis,'x') == 1;
    x_bod = constCoord*ones(size(theta));
    y_bod = r.*cos(theta);
    z_bod = r.*sin(theta);
    
%XZ plane 
elseif strcmp(axis,'y') == 1;
    x_bod = r.*cos(theta);
    y_bod = constCoord*ones(size(theta));
    z_bod = r.*sin(theta);

%XY plane 
elseif strcmp(axis,'z') == 1;
    x_bod = r.*cos(theta);
    y_bod = r.*sin(theta);
    z_bod = constCoord*ones(size(theta));

else
    error('incorrect axis inputs');
end

X_bod = cat(3,x_bod,y_bod,z_bod);
flux = magFlux(X_bod,magX, magM, axis );
dBdt = mag_dBdt*flux.*r;


end
