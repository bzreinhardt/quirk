 r = 0.5;
 coord = 0;
 mMag_bod = [-1;0;0];
 xMag_bod = [1; 0 ; 0];
 omMag_bod = [0;1;0];
 vMag_bod = 0;
 axis = 'z';
 sigma = 1;
 
 
 testPt = vXB_int(pi, r, coord, mMag_bod, xMag_bod, omMag_bod,vMag_bod,axis);

 int = @(theta) vXB_int(theta, r, coord, mMag_bod, xMag_bod, omMag_bod,vMag_bod,axis);
 cur = -1*sigma*integral(int, 0, 2*pi,'ArrayValued',true);