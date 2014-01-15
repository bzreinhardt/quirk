%% EC system with a single electromagnet
% This script shows how to create a plate, an electromagnet and an magmBody
% and find the resultant force on the plate

theta = -pi/2;
vec = [0;1;0]; %vector the magnet will be rotated around
freq = 60; %frequency of the EM in hz
phase = 0; %phase of the EM - all phases are relative
m1_x = [0.2; 0;0]; %position of the EM
m1_m = aa2q(vec,theta); %use aa2q to turn a vector and angle to rotate about it into a quaternion attitude for QuIRK
m1 = att2vec(m1_m,'z'); %find the unit vector pointing along the z axis of the magnet which is also the dipole vector

P1 = plate([0;0;0], [0,0,0,1]); %create a plate at the origin with body axes aligned with inertial axes
M1 = magnet(1,phase,freq,m1_x, m1_m); %create a magnet with dipole strength 1, pointing in the -x direction
MB1 = magmBody(P1); %create a magmBody that includes only the plate

MB1.add(M1); %add the magnet to the magmBody

%plot the magmBody that contains the plate and magnet, as well as the
%dipole vector of the magnet
figure(1);clf
draw(MB1);
hold on
quiver3(m1_x(1),m1_x(2),m1_x(3),0.05*m1(1),0.05*m1(2),0.05*m1(3),'color','r');

force_1 = P1.plateForce(P1,M1,0); %find the instantaneous force experienced by the plate from M1
force_1_mag = norm(force_1);
%plot the force
quiver3(0,0,0, 0.1/force_1_mag*force_1(1),0.1/force_1_mag*force_1(2),0.1/force_1_mag*force_1(3), 'color', 'g');

P3 = plate([0;0;0], [0,0,0,1]); %create a plate at the origin with body axes aligned with inertial axes
f = @(p,m,t)(P3.plateForce(p,m,t));
T = @(p,m,t)([0;0;0]);
grnd = joint(P3, 'ground');
M5 = magnet(1,phase,freq,m1_x/1.5, m1_m); 
MB3 = magmBody(P3,M5,grnd,'U', @(b)(98*b.mass*b.pos(1)));
F1 = force(P3,M5,f,T,0);
MB3.add(F1);
MB3.solve([0,0.2]);


%% Multiple Electromagnets

phase2 = pi/4; %phase of the new pair of magnets - relative to other magnet phases

%new magnet positions
m2_x = [0;-0.2;0]; 
m3_x = [0;0.2;0];

vec2 = [1;0;0];

%new magnets will be pointing down the y axis towards the plate
m2_m = aa2q(vec2, -pi/2);
m3_m = aa2q(vec2, pi/2);

%create the new magnets
M2 = magnet(1,phase2,freq, m2_x, m2_m);
M3 = magnet(1,phase2,freq, m3_x, m3_m);
M4 = magnet(1,phase,freq,m1_x, m1_m); 
%new plate
P2 = plate([0;0;0], [0,0,0,1]);

%extract the z vector of the new magnets for plotting purposes 
m2 = att2vec(M2.att,'z');
m3 = att2vec(M3.att,'z');

%create a new magmBody with the original magnet and two new magnets
MB2 = magmBody(P2,M4,M2,M3);

%plot the 3 magnet system
figure(2); clf;
draw(MB2);
hold on
quiver3(m1_x(1),m1_x(2),m1_x(3),0.05*m1(1),0.05*m1(2),0.05*m1(3),'color','r');
quiver3(m2_x(1),m2_x(2),m2_x(3),0.05*m2(1),0.05*m2(2),0.05*m2(3),'color','g');
quiver3(m3_x(1),m3_x(2),m3_x(3),0.05*m3(1),0.05*m3(2),0.05*m3(3),'color','b');

%find the new forces
force1 = P2.plateForce(P2, M4, 0);
force2 = P2.plateForce(P2, M2, 0);
force3 = P2.plateForce(P2, M3, 0);

net_force = force1 + force2 + force3;

quiver3(0,0,0, net_force(1), net_force(2),net_force(3));

%create the new magnets
M6 = magnet(1,phase2,freq, m2_x, m2_m);
M7 = magnet(1,phase2,freq, m3_x, m3_m);
M8 = magnet(1,phase,freq,m1_x, m1_m); 
%new plate
P4 = plate([-0.05;0;0], [0,0,0,1]);
f = @(p,m,t)(P4.plateForce(p,m,t));
T = @(p,m,t)([0;0;0]);

F6 = force(P4,M5,f,T,0);
F7 = force(P4,M6,f,T,0);
F8 = force(P4,M7,f,T,0);
G6 = joint(M6,'ground');
G7 = joint(M7,'ground');
G8 = joint(M8,'ground');
MB4 = magmBody(P4,M6,M7,M8,F6,F7,F8,G6,G7,G8);
MB4.solve([0,0.02]);


