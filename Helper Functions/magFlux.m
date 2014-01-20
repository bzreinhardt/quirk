function [B] = magFlux(x,y,z,Xmag, m, axis )
%magFlux finds the vector magnetic field at point x created by a dipole
%vector m centered at point Xmag

% B = 3*u/r^3*(sin(theta)cos(theta)cos(phi)xhat +
% sin(theta)cos(theta)sin(phi)yhat + (cos^2(theta)-1/3)zhat
%not actually 1

 
%u0 = 1;
%m(abs(m)<1E-10) = 0; %added 10/11/2013 to see effect
u0 = 4*pi*10^-7;

%r = [x;y;z] - Xmag;

r_x = x - Xmag(1);
r_y = y - Xmag(2);
r_z = z - Xmag(3);

r_abs = (r_x.^2+r_y.^2+r_z.^2).^.5;
%phi = atan2(r_y,r_x);
%theta = atan2((r_x.^2+r_y.^2)^0.5,r_z);

mDotR = m(1).*r_x + m(2).*r_y + m(3).*r_z;



switch axis
    case 'x'
        B = u0/(4*pi) * (3*r_x.*mDotR./r_abs.^5 - m(1)./r_abs.^3); 
    case 'y'
        B = u0/(4*pi) * (3*r_y.*mDotR./r_abs.^5 - m(2)./r_abs.^3);
    case 'z'
        B = u0/(4*pi) * (3*r_z.*mDotR./r_abs.^5 - m(3)./r_abs.^3);
    case 'all'
        B = [u0/(4*pi) * (3*r_x.*mDotR./r_abs.^5 - m(1)./r_abs.^3); ...
            u0/(4*pi) * (3*r_y.*mDotR./r_abs.^5 - m(2)./r_abs.^3); ...
            u0/(4*pi) * (3*r_z.*mDotR./r_abs.^5 - m(3)./r_abs.^3)];
end

%B = u0/(4*pi)*(3*r*dot(m,r)/norm(r)^5-m/norm(r)^3);
%B = u0*m/(4 pi r^3)*[3cos(theta)sin(theta)cos(ph);
%                    3cos(theta)sin(theta)cos(phi);
%                    2cos(theta)^2-sin(theta)^2]
 

end

