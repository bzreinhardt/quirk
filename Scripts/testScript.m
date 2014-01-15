% %%[X,Y,Z] = cylinder([0 ones(1,30) 0.98 ones(1,10) 0], 20);
% %figure(1);clf; subplot(211)
% h = surf(X,Y,Z);
% xlabel('x');
% 
% hold on
% quiver3(0,0,0,0,0,2)
% 
% angle = -pi/2;
% axis = [0;1;0];
% [x2,y2,z2] = rotsurf(X,Y,Z,-angle*axis);
% figure(2);clf; subplot(211);
% h = surf(x2,y2,z2);
% xlabel('x');
% 
% B = body([0;0;0],aa2q(axis,angle),'shape','cylinder');
% subplot(212);
% draw(B);
% vector = att2vec(B.att,'z');
% hold on
% quiver3(0,0,0,vector(1),vector(2),vector(3));
% 
% C = plate([0;0;0], aa2q([0;1;0],pi/4));
% matrix = quat2dcm(qq2mq(C.att));
% figure(3);
% draw(C);
% testVec = [-1/2^.5;0;-1/2^.5];
% plateZ = att2vec(C.att,'z');
% hold on;
% %quiver3(0,0,0,testVec(1),testVec(2),testVec(3),'color','r');
% %quiver3(0,0,0,plateZ(1),plateZ(2),plateZ(3),'color','g');

%%
% Plan of attack for qurik implementation
% Done make loops on plate with single EM (we know this works)
% Done change properties of magnet (position) test loop update
% Done repeat the above two steps with multiple EM's
% Done test magforce 

% 

% Done generate loops with moving permanent magnet
% Done implement calling loop update in the magforce function - it should
% go through all the magnets in the mBody
% Done test the generated force vector 
% Done generate loops with a spinning permanent magnet 
% Done test the generated force vector

%TODO test force as magnet spins

theta = -pi/2;
vec = [0;1;0];
theta2 = linspace(0,2*pi,3);
freq = 60;
phase1 = 0;
phase2 = 2*pi;

m1_x = [0.2;0.1;0];
m2_x = [0.2;-0.1;0];
% m3_x = [0; 0.1; 0];

m1_m = aa2q(vec,theta);
m2_m = aa2q(vec,theta);
% m3_m = aa2q([1;0;0],pi/2);
% 
% P1 = plate([0;0;0], [0,0,0,1]);
% set(P1,'sy',0.1);
% set(P1,'sz',0.1);
% M1 = magnet(1,0,freq,m1_x, m1_m);
% M2 = magnet(1,phase1,freq,m2_x, m2_m);
% M3 = magnet(1,phase1,freq,m3_x, m3_m);

P1 = plate([0;0;0], [0,0,0,1]);
M1 = magnet(1,0,0,m1_x, m1_m);
M2 = magnet(1,0,0,m2_x, m2_m);
m1_om = [0;0;1];
M1.om = m1_om;
m2_om = -1*[0;0;1];
M2.om = m2_om;
MB1 = magmBody();
MB1.add(P1,M1,M2);

rotq1 = aa2q([0;0;1],pi/4);
rotq2 = aa2q([0;0;1],-pi/4);
atts1 = zeros(8,4);
atts2 = zeros(8,4);
vec1 = zeros(3,8);
vec2 = zeros(3,8);
F1 = zeros(3,8);
F2 = zeros(3,8);
for i = 1:8
    atts1(i,:) = M1.att;
    atts2(i,:) = M2.att;
    vec1(:,i) = att2vec(M1.att,'z');
    vec2(:,i) = att2vec(M2.att,'z');
    F1(:,i) = P1.plateForce(P1,M1,0);
    F2(:,i) = P1.plateForce(P1,M2,0);
    M1.att = mq2qq(quatmultiply(qq2mq(rotq1),qq2mq(atts1(i,:))));
    M2.att = mq2qq(quatmultiply(qq2mq(rotq2),qq2mq(atts2(i,:))));
end
F_net = F1 + F2;
% v1 = 0.05 * att2vec(M1.att,'z');
% 
% figure(1);clf
% draw(MB1);
% hold on
%     quiver3(m1_x(1),m1_x(2),m1_x(3),v1(1),v1(2),v1(3),'color','r');
%     quiver3(m1_x(1),m1_x(2),m1_x(3),0.05*m1_om(1),0.05*m1_om(2),0.05*m1_om(3),'color','g');

% P2 = plate([0;0;0], [0,0,0,1]);
% M2 = magnet(1,0,0,m1_x, m1_m);
% M2.om = m1_om;
% MB2 = magmBody();
% MB2.add(P2,M2);
% 
% P3 = plate([0;0;0], [0,0,0,1]);
% M3 = magnet(1,0,0,m1_x, m1_m);
% M3.om = m1_om;
% MB3 = magmBody();
% MB3.add(P3,M3);
% % M22 = magnet(1,phase2,freq,m2_x, m2_m);
% % M32 = magnet(1,phase2,freq,m3_x, m3_m);
% 
% 
% %cur1 = P2.loops{1}.cur
% F = zeros(3,3);
% 
% for i = 1:3
%     
% %v2 = 0.05 * m1_om;
%     v1 = 0.05 * att2vec(M12.att,'z');
%     figure(i);clf
%     draw(MB);
%     hold on
%     quiver3(m1_x(1),m1_x(2),m1_x(3),v1(1),v1(2),v1(3),'color','r');
%     quiver3(m1_x(1),m1_x(2),m1_x(3),0.05*m1_om(1),0.05*m1_om(2),0.05*m1_om(3),'color','g');
%     F(:,i) = P2.plateForce(P2,M12,0);
%     
%     mMag = att2vec(M12.att,'z');
%     rotq = aa2q([0;0;1],theta2(i));
%     rotM = quat2dcm(qq2mq(rotq));
%     mMag_new = rotM*mMag;%magnet moment in body coordinates
%     magAtt_new = aa2q(mMag_new,0);
%     M12.att = magAtt_new;
%     
% 
%     
% end
% 
% 
% %quiver3(m1_x(1),m1_x(2),m1_x(3),v2(1),v2(2),v2(3),'color','g');

% M12.om = [0; 1;0];
% F2 = P2.plateForce(P2,M12,0)
% M12.om = [0; 2;0];
% F3 = P2.plateForce(P2,M12,0)
% MB.add(M2);
% cur2 = P1.loops{5}.cur
% MB.add(M3);
% cur3 = P1.loops{8}.cur

% M1.pos = [0.2;0;0];
% M2.pos = [0;-0.2;0];
% M3.pos = [0; 0.2; 0];
% P1.updateLoops();
% cur4 = P1.loops{1}.cur
% cur5 = P1.loops{5}.cur
% cur6 = P1.loops{8}.cur

% F1 = P1.plateForce(P1,M1,0);
% F2 = P1.plateForce(P1,M2,0);
% F3 = P1.plateForce(P1,M3,0);
% MB = magmBody(P1,M1,M2,M3);
%  v1 = 0.05*att2vec(M1.att,'z');
% v2 = 0.04*att2vec(M2.att,'z');
% v3 = 0.04*att2vec(M3.att,'z');

% figure(1);clf;
% draw(MB);
% hold on
% quiver3(m1_x(1),m1_x(2),m1_x(3),v1(1),v1(2),v1(3),'color','r');
% quiver3(m2_x(1),m2_x(2),m2_x(3),v2(1),v2(2),v2(3),'color','g');
% quiver3(m3_x(1),m3_x(2),m3_x(3),v3(1),v3(2),v3(3),'color','b');



% P1 = genLoops(P1,M2);
% P1 = genLoops(P1,M3);
% P2 = genLoops(P2,M12);
% P2 = genLoops(P2,M22);
% P2 = genLoops(P2,M32);
% 
% F1 = plate.plateForce(P1,M1,0);
% F2 = plate.plateForce(P1,M2,0);
% F3 = plate.plateForce(P1,M3,0);
%  
% F_net1 = F1+F2+F3;
% 
% F12 = plate.plateForce(P2,M12,0);
% F22 = plate.plateForce(P2,M22,0);
% F32 = plate.plateForce(P2,M32,0);
%  
% F_net2 = F12+F22+F32;
% 
% 
% i = 0;
% F_nets = zeros(3,10);
% F1s = zeros(3,10);
% F2s = zeros(3,10);
% F3s = zeros(3,10);
% phaseSpace = linspace(0,2*pi,40);
% for phases = phaseSpace
%     i = i +1;
%     M2.phase = phases;
%     M3.phase = phases;
%     P1.loops = {};
%     P1 = genLoops(P1,M1);
%     P1 = genLoops(P1,M2);
%     P1 = genLoops(P1,M3);


%  r1 = 0.25; r2 = 0.006; r3 = 0.006;
%  L1 = iLoop(P,M, r1, 'x',0);
%  L2 = iLoop(P,M, r2, 'y',0);
%  L3 = iLoop(P,M, r3, 'z',0);
%  current = [L1.cur,L2.cur,L3.cur];
%  
%  F1 = iLoop.loopForce(L1,M,120);
%     F1 = plate.plateForce(P1,M1,0);
%     F2 = plate.plateForce(P1,M2,0);
%     F3 = plate.plateForce(P1,M3,0);
%  
%     F_net = F1+F2+F3;
%     F1s(:,i) = F1;
%     F2s(:,i) = F2;
%     F3s(:,i) = F3;
%     F_nets(:,i) = F_net;
% 
%     
% end
%     figure(2); clf;
%     plot(phaseSpace*180/pi,F_nets(1,:)*1E-5); set(gca, 'YTick', [0])
%     xlabel('Phase Offset (degrees)');ylabel('Force on Plate'); xlim([0 360]);
%     set(gca,'fontsize',16);xlabel('Phase Offset (degrees)','fontsize',24);ylabel('Force on Plate','fontsize',24);

%%
% sigma = 1;
% r = 0.25;
% coord =0;
% mag_dBdt = 1;
% mMag_bod = vector;
% 
% [X,Z] = meshgrid(-.3:.01:.3);
% Y = zeros(size(X));
% [m,n] = size(X);
% Bx = zeros(m,n); By = zeros(m,n); Bz = zeros(m,n);
% for i = 1:m
%     for j = 1:m
%         x = X(i,j); y = Y(i,j); z = Z(i,j);
%         Bx(i,j) = magFlux(x,y,z,xMag_bod,mMag_bod,'x');
%         By(i,j) = magFlux(x,y,z,xMag_bod,mMag_bod,'y');
%         Bz(i,j) = magFlux(x,y,z,xMag_bod,mMag_bod,'z');
%     end
% end

%figure(2);clf;
%surf(X,Z,By);
%zlabel('y');xlabel('x');ylabel('z');

%figure(2);clf;
%quiver3(xMag_bod(1),xMag_bod(2),xMag_bod(3),mMag_bod(1),mMag_bod(2),mMag_bod(3));
%hold on
%quiver3(X,Y,Z,Bx,By,Bz,0.5);
        
%axis = 'y';
%int = @(theta,r)dBdt_int( theta, r,coord, mag_dBdt, mMag_bod, xMag_bod,axis);
%cur = -1*sigma*integral2 (int,0,2*pi,0,r); 

