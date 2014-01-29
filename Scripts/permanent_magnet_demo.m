%% This demo shows how to use permanent magnets in the QuIRKE environment


% %% One Spinning Permanent Magnet
% 
theta = -pi/2;
vec = [0;1;0];

% m1_x = [0.2;0;0];
% m1_m = aa2q(vec,theta);
% m1_om = [0;0;1];
% 
% P1 = plate([0;0;0], [0,0,0,1]);
% M1 = magnet(1,0,0,m1_x, m1_m);
% MB1 = magmBody();
% MB1.add(P1,M1);
% 
% v1 = att2vec(M1.att,'z');
% M1.om = m1_om;
% P1.updateLoops(); %whenever the properties of an existing magnet is changed
%                     %the current loops won't notice it until you run updateLoops. 
%                     %plateForce automatically runs updateLoops
% 
% figure(5);clf;
% draw(MB1);
% hold on
% %draw the dipole, the angular velocity and the net force
% quiver3(m1_x(1),m1_x(2),m1_x(3),0.05*v1(1),0.05*v1(2),0.05*v1(3),'color','r');
% quiver3(m1_x(1),m1_x(2),m1_x(3),0.05*m1_om(1),0.05*m1_om(2),0.05*m1_om(3),'color','g');
% 
% force = P1.plateForce(P1,M1,0);
% 
% quiver3(0,0,0, 10^9*force(1),10^9*force(2),10^9*force(3));

% %% Two counterspinning Permanent Magnets
% 
% 
% m2_x = [0.2;0.1;0];
% m3_x = [0.2;-0.1;0];
% 
% m2_m = aa2q(vec,theta);
% m3_m = aa2q(vec,theta);
% 
% P2 = plate([0;0;0], [0,0,0,1]);
% M2 = magnet(1,0,0,m2_x, m2_m);
% M3 = magnet(1,0,0,m3_x, m3_m);
% MB2 = magmBody();
% MB2.add(P2,M2,M3);
% 
% m2_om = [0;0;1];
% M2.om = m2_om;
% m3_om = -1*[0;0;1];
% M3.om = m3_om;
% 
% % rotate the magnets through an entire revolution and record the forces
% rotq2 = aa2q([0;0;1],pi/4);
% rotq3 = aa2q([0;0;1],-pi/4);
% atts2 = zeros(8,4);
% atts3 = zeros(8,4);
% vec2 = zeros(3,8);
% vec3 = zeros(3,8);
% F2 = zeros(3,8);
% F3 = zeros(3,8);
% for i = 1:8
%     atts2(i,:) = M2.att;
%     atts3(i,:) = M3.att;
%     vec2(:,i) = att2vec(M2.att,'z');
%     vec3(:,i) = att2vec(M3.att,'z');
%     F2(:,i) = P2.plateForce(P2,M2,0);
%     F3(:,i) = P2.plateForce(P2,M3,0);
%     M2.att = mq2qq(quatmultiply(qq2mq(rotq2),qq2mq(atts2(i,:))));
%     M3.att = mq2qq(quatmultiply(qq2mq(rotq3),qq2mq(atts3(i,:))));
% end
% 
% v2 = att2vec(M2.att,'z');
% v3 = att2vec(M3.att,'z');
% 
% 
% F_net = F2 + F3;
% cycle_force = sum(F_net,2); %net force on plate over the cycle
% 
% figure(6); clf;
% draw(MB2);
% hold on
% %draw the angular velocities, the dipoles and the net force
% quiver3(m2_x(1),m2_x(2),m2_x(3),0.05*v2(1),0.05*v2(2),0.05*v2(3),'color','r');
% quiver3(m2_x(1),m2_x(2),m2_x(3),0.05*m2_om(1),0.05*m2_om(2),0.05*m2_om(3),'color','r');
% quiver3(m3_x(1),m3_x(2),m3_x(3),0.05*v3(1),0.05*v3(2),0.05*v3(3),'color','g');
% quiver3(m3_x(1),m3_x(2),m3_x(3),0.05*m3_om(1),0.05*m3_om(2),0.05*m3_om(3),'color','g');
% quiver3(0,0,0, 0.1*cycle_force(1), 0.1*cycle_force(2), 0.1*cycle_force(3));

%% Linearly Translating Permanent Magnet

m4_x = [0.2;0;0];
m4_m = aa2q(vec,theta);
m4_vel = [0;1;0]; %swiping magnet across face of plate

P3 = plate([0;0;0], [0,0,0,1]);
M4 = magnet(1,0,0,m4_x, m4_m);
MB3 = magmBody();
MB3.add(P3,M4);

v4 = att2vec(M4.att,'z');
M4.vel = m4_vel;
P3.updateLoops(); %whenever the properties of an existing magnet is changed
                    %the current loops won't notice it until you run updateLoops. 
                    %plateForce automatically runs updateLoops

figure(7);clf;
draw(MB3);
hold on
%draw the dipole, the linear velocity and the force
quiver3(m4_x(1),m4_x(2),m4_x(3),0.05*v4(1),0.05*v4(2),0.05*v4(3),'color','r');
quiver3(m4_x(1),m4_x(2),m4_x(3),0.05*m4_vel(1),0.05*m4_vel(2),0.05*m4_vel(3),'color','g');

force3 = P3.plateForce(P3,M4,0);

quiver3(0,0,0, force3(1),force3(2),force3(3));

