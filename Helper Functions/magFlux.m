function [B] = magFlux(X,Xmag, m, varargin )

if size(varargin) ~= 0
    axis = varargin;
end
%magFlux finds the vector magnetic field at point x created by a dipole
%vector m centered at point Xmag

%1/27 make X always in the form [[x1;y1;z1],[x2;y2;z2]]

% B = 3*u/r^3*(sin(theta)cos(theta)cos(phi)xhat +
% sin(theta)cos(theta)sin(phi)yhat + (cos^2(theta)-1/3)zhat
%not actually 1
%Xmag and m are vectors in the current frame
%x,y,z are the positions to calculate the field
%axis is a string 'x','y' or 'z' that determines the component of the
%magnetic field output

 
%u0 = 1;
%m(abs(m)<1E-10) = 0; %added 10/11/2013 to see effect
u0 = 4*pi*10^-7;

%r = [x;y;z] - Xmag;
[mm,n] = size(X);
r = X-Xmag*ones(1,n);
r_abs = sum(r.^2).^.5;
% r_x = X(:,1) - Xmag(1);
% r_y = X(:,2) - Xmag(2);
% r_z = X(:,3) - Xmag(3);
%r_abs = (r_x.^2+r_y.^2+r_z.^2).^.5;
% mDotR = m(1).*r_x + m(2).*r_y + m(3).*r_z;
mDotR = m'*r;
%phi = atan2(r_y,r_x);
%theta = atan2((r_x.^2+r_y.^2)^0.5,r_z);
m_div_r = m*(1./r_abs.^3);

mDotR_div_r = mDotR./r_abs.^5;
r_mDotR_div_r = bsxfun(@times,r,mDotR_div_r);

B = u0/(4*pi)*(3*r_mDotR_div_r - m_div_r);

if exist('axis','var')
    switch axis
        case 'x'
            B = B(1,:);
        case 'y'
             B = B(2,:);
        case 'z'
             B = B(3,:);
    end
end

% switch axis
%     case 'x'
%         B = u0/(4*pi) * (3*r_x.*mDotR./r_abs.^5 - m(1)./r_abs.^3); 
%     case 'y'
%         B = u0/(4*pi) * (3*r_y.*mDotR./r_abs.^5 - m(2)./r_abs.^3);
%     case 'z'
%         B = u0/(4*pi) * (3*r_z.*mDotR./r_abs.^5 - m(3)./r_abs.^3);
%     case 'all'
%         B = [u0/(4*pi) * (3*r_x.*mDotR./r_abs.^5 - m(1)./r_abs.^3); ...
%             u0/(4*pi) * (3*r_y.*mDotR./r_abs.^5 - m(2)./r_abs.^3); ...
%             u0/(4*pi) * (3*r_z.*mDotR./r_abs.^5 - m(3)./r_abs.^3)];
% end

%B = u0/(4*pi)*(3*r*dot(m,r)/norm(r)^5-m/norm(r)^3);
%B = u0*m/(4 pi r^3)*[3cos(theta)sin(theta)cos(ph);
%                    3cos(theta)sin(theta)cos(phi);
%                    2cos(theta)^2-sin(theta)^2]
 

end

