function [dBdt] = vec_dBdt_int( theta,r, constCoord,  mag_dBdt, magM, magX, axis)
%out is the dB_dt perpendicular to the indicated plane assume all inputs
%are already in body coordinates
x_m = ones(size(theta))*magX(1);
y_m = ones(size(theta))*magX(2);
z_m = ones(size(theta))*magX(3);

m_x = ones(size(theta))*magM(1);
m_y = ones(size(theta))*magM(2);
m_z = ones(size(theta))*magM(3);
%YZ plane 
if strcmp(axis,'x') == 1;
    x_bod = constCoord*ones(size(theta));
    y_bod = r.*cos(theta);
    z_bod = r.*sin(theta);
    flux = vecMagFlux(x_bod,y_bod,z_bod,x_m,y_m,z_m, m_x,m_y,m_z, axis );
    dBdt = mag_dBdt*flux.*r;
    

%XZ plane 
elseif strcmp(axis,'y') == 1;
    x_bod = r.*cos(theta);
    y_bod = constCoord*ones(size(theta));
    z_bod = r.*sin(theta);
    flux = vecMagFlux(x_bod,y_bod,z_bod,x_m,y_m,z_m, m_x,m_y,m_z, axis );
    dBdt = mag_dBdt*flux.*r;
    


%XY plane 
elseif strcmp(axis,'z') == 1;
    x_bod = r.*cos(theta);
    y_bod = r.*sin(theta);
    z_bod = constCoord*ones(size(theta));
    flux = vecMagFlux(x_bod,y_bod,z_bod,x_m,y_m,z_m, m_x,m_y,m_z, axis );
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
