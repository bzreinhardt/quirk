function [B] = magFlux(X,Xmag, m, varargin )

if size(varargin) ~= 0
    axis = varargin{1};
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
[mm,n,p] = size(X);
if p == 1
    %for a 1D array of points
    if mm ~= 3 && n == 3
        %make sure position matrix is in the right orientation
        X = X';
        n = mm;
    end
    %for a 3xn grouping of points
    r = X-Xmag*ones(1,n);
    r_abs = sum(r.^2).^.5;
    
    mDotR = m'*r;
    
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
elseif p ==3 
    %X is a 2D array of points
    r_x = X(:,:,1) - Xmag(1);
    r_y = X(:,:,2) - Xmag(2);
    r_z = X(:,:,2) - Xmag(3);
    r_abs = (r_x.^2+r_y.^2+r_z.^2).^.5;
    
    mDotR = m(1)*r_x + m(2)*r_y + m(3)*r_z;
    
    if exist('axis','var')
        switch axis
            case 'x'
                B = u0/(4*pi) * (3*r_x.*mDotR./r_abs.^5 - m(1)./r_abs.^3);
            case 'y'
                B = u0/(4*pi) * (3*r_y.*mDotR./r_abs.^5 - m(2)./r_abs.^3);
            case 'z'
                B = u0/(4*pi) * (3*r_z.*mDotR./r_abs.^5 - m(3)./r_abs.^3);
        end
    else
        B = zeros(mm,n,p);
        B(:,:,1) = u0/(4*pi) * (3*r_x.*mDotR./r_abs.^5 - m(1)./r_abs.^3);
        B(:,:,2) = u0/(4*pi) * (3*r_y.*mDotR./r_abs.^5 - m(2)./r_abs.^3);
        B(:,:,3) = u0/(4*pi) * (3*r_z.*mDotR./r_abs.^5 - m(3)./r_abs.^3);
    end
end


end

