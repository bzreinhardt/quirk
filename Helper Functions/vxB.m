function [ component ] = vxB(x,y,z, mMag_bod, xMag_bod, omMag_bod,vMag_bod,axis)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
%r = [x;y;z];
%om_x = [0 -omMag_bod(3) omMag_bod(2); omMag_bod(3) 0 -omMag_bod(1); -omMag_bod(2) omMag_bod(1) 0];
%vel = vMag_bod - cross(omMag_bod,r);
%vel = vMag_bod - om_x*r;
velx = vMag_bod(1) - (-omMag_bod(3)*y+omMag_bod(2)*z);
vely = vMag_bod(2) - (omMag_bod(3)*x - omMag_bod(1)*z);
velz = vMag_bod(3) - (-omMag_bod(2)*x + omMag_bod(1)*y);
B = magFlux(x,y,z,xMag_bod,mMag_bod);

%v_x = [0 -velz vely; velz 0 -velx; -vely velx 0];
%vCrossB = v_x*B;
vCrossBx = -velz*B(2)+vely*B(3);
vCrossBy = velz*B(1) -velx*B(3);
vCrossBz = -vely*B(1) + velx*B(2);



switch axis
    case 'x'
        component = vCrossBx;
    case 'y'
        component = vCrossBy;
    case 'z'
        component = vCrossBz;

end

