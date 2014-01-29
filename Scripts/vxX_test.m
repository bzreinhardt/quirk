n = 100;
thetas = 0:2*pi/n:2*pi*(1-1/n);
thetas = thetas';
r_x = 0.01;
r_y = 0.01;
r_z = 0.01;
coord = 0;
mMag = [-1;0;0];
xMag = [.04;0;0];
omMag_bod = [0;0;0];
vMag_bod = [0;1;0];


intX = vXB_int(thetas, r_x, coord, mMag, xMag, omMag_bod,vMag_bod,'x');
intY = vXB_int(thetas, r_y, coord, mMag, xMag, omMag_bod,vMag_bod,'y');
intZ = vXB_int(thetas, r_z, coord, mMag, xMag, omMag_bod,vMag_bod,'z');
intX2 = @(theta) vXB_int(theta, r_x, coord, mMag, xMag, omMag_bod,vMag_bod,'x');
intY2 = @(theta) vXB_int(theta, r_y, coord, mMag, xMag, omMag_bod,vMag_bod,'y');
intZ2 = @(theta) vXB_int(theta, r_z, coord, mMag, xMag, omMag_bod,vMag_bod,'z');

curX = sum(intX);
curY = sum(intY);
curZ = sum(intZ);
curX2 = -1*integral(intX2,0,2*pi);
curY2 = -1*integral(intY2,0,2*pi);
curZ2 = -1*integral(intZ2,0,2*pi);

%yz plane (perp x axis)
x_x = zeros(n,1);
z_x = r_x*sin(thetas);
y_x = r_x*cos(thetas);

pts_x = zeros(size(pts_y));
X_x = [x_x,y_x,z_x]';
B_x = magFlux(X_x,xMag,mMag);
% B_x_x = magFlux(x_x,y_x,z_x,xMag,mMag,'x');
% B_y_x = magFlux(x_x,y_x,z_x,xMag,mMag,'y');
% B_z_x = magFlux(x_x,y_x,z_x,xMag,mMag,'z');

vxB_x = vxB(X_x,xMag,mMag,vMag_bod);
% vxB_z_x = vxB(x_x,y_x,z_x,mMag,xMag,omMag_bod,vMag_bod,'z');

figure(3);clf;
subplot(211)
quiver(y_x,z_x,B_x(2,:)',B_x(3,:)');
xlabel('y');ylabel('z');
subplot(212)
quiver(y_x,z_x,vxB_x(2,:)',vxB_x(3,:)');
xlabel('y');ylabel('z');

%xz plane (perp y axis) 
y_y = zeros(n,1);
x_y = r_y*sin(thetas);
z_y = r_y*cos(thetas);

X_y = [x_y';y_y';z_y'];

pts_y = zeros(size(pts_y));
% B_x_y = magFlux(x_y,y_y,z_y,xMag,mMag,'x');
% B_y_y = magFlux(x_y,y_y,z_y,xMag,mMag,'y');
% B_z_y = magFlux(x_y,y_y,z_y,xMag,mMag,'z');
B_y = magFlux(X_y,xMag,mMag);

% vxB_x_y = vxB(x_y,y_y,z_y,mMag,xMag,omMag_bod,vMag_bod,'x');
% vxB_z_y = vxB(x_y,y_y,z_y,mMag,xMag,omMag_bod,vMag_bod,'z');
vxB_y = vxB(X_y,xMag,mMag,vMag_bod);

figure(4);clf;
subplot(211)
quiver(z_y,x_y,B_y(3,:)',B_y(1,:)');
xlabel('z');ylabel('x');
subplot(212)
quiver(z_y,x_y,vxB_y(3,:)',vxB_y(1,:)');
xlabel('z');ylabel('x');


