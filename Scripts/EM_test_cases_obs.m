% replication tests
%%
% phase survey for changing side currents and constant back current
phase1 = 0; %phase of the EM - all phases are relative
phase2 = pi/4; %phase of the new pair of magnets - relative to other magnet phases
theta = -pi/2;
vec = [0;1;0]; %vector the magnet will be rotated around
vec2 = [1;0;0];
freq = 60; %frequency of the EM in hz

%new plate
P1 = plate([0;0;0], [0,0,0,1]);
w = 0.07;
l = 0.24;
t = 0.001;
%scale factor on the radii or the current loops
geoScaleFactor = 0.267472565225471;

%set the physical dimensions of the plate
P1.sx = t;P1.sy= w;P1.sz = l;
%set the radii of the current loops
P1.xr = geoScaleFactor*w;P1.yr = geoScaleFactor*t;P1.zr = geoScaleFactor*t;

%magnet positions - one on either side of plate on y axis and one down the
%x axis from the plate
m1_x = [0.002+t/2; 0;0]; %position of the EM
m2_x = [0;-0.002-w/2;0]; 
m3_x = [0;0.002+w/2;0];

%initial currents through the magnets
i_1 = 2;
i_23 = 1;

%create the new magnets
M1 = magnet(i_1,phase1,freq,m1_x, m1_m);
M2 = magnet(i_23,phase2,freq, m2_x, m2_m);
M3 = magnet(i_23,phase2,freq, m3_x, m3_m);


%put the magnets and plate into an mBody
MB1 = magmBody(P1,M1,M2,M3);
%set up the forces in the system
forceFun = @(p,m,t)P1.plateForce(p,m,t);
torqueFun = @(p,m,t) [0;0;0];

F1 = force(P1,M1,forceFun, torqueFun,0);
F2 = force(P1,M2,forceFun, torqueFun,0);
F3 = force(P1,M3,forceFun, torqueFun,0);

%set up test phases
phases = linspace(-pi,pi,25);
%test currents through the side magnets
i2 = [0,1,2,3,4];
fig7_rep = zeros(length(i2),length(phases));

%find the force on the plate for different phase difference and side
%currents
for i = 1:length(phases)
    M2.phase = phases(i);
    M3.phase = phases(i);
    for j = 1:length(i2)
        M2.mag = i2(j);
        M3.mag = i2(j);
        Fnet = P1.plateForce(P1,M1,0)+P1.plateForce(P1,M2,0)+P1.plateForce(P1,M3,0);
        fig7_rep(i,j) = Fnet(1);
    end
end

%plot the results
figure(1);clf; 
phases2 = phases*180/pi;
% subplot(211);
% for i = 1:length(i2)
%     hold on
%     plot(phases2, fig7_rep(:,i));
% end
plot(phases2, fig7_rep);
    legend('0','1','2','3','4');
title('Sim Fig7 Data - Force vs Phase Offset for different magnet currents');
xlabel('Phase difference');
ylabel('Force (+ is towards magnets)');
% subplot(212);MB1.draw();
%%
%Cycle through magnet currents at the phase difference with the maximum
%attract
%this section depends on running the previous section to find the maximum
%theta
%find the theta at which the force is maximized
[c,i] = max(fig7_rep(:,2));
theta_max = phases(i);
phase1 = 0;
phase2 = theta_max;
%
%create the new magnets
M1_8a = magnet(2,phase1,freq, m1_x, m1_m);
M2_8a = magnet(1,phase2,freq, m2_x, m2_m);
M3_8a = magnet(1,phase2,freq, m3_x, m3_m);
%new plate
P1_8a = plate([0;0;0], [0,0,0,1]);
P1_8a.sx = t;P1_8a.sy= w;P1_8a.sz = l;
P1_8a.xr = geoScaleFactor*w;P1_8a.yr = geoScaleFactor*t;P1_8a.zr = geoScaleFactor*t;
%set up forces
F1_8a = force(P1_8a,M1_8a,forceFun, torqueFun,0);
F2_8a = force(P1_8a,M2_8a,forceFun, torqueFun,0);
F3_8a = force(P1_8a,M3_8a,forceFun, torqueFun,0);
%set up mbody
MB_8a = magmBody(P1_8a, M1_8a, M2_8a,M3_8a);
%currents through the back and side magents
i1 = [1,2,3,4];
i2 = [0,1,2,3,4];
%initialize matrix for data
fig8a_rep = zeros(length(i1),length(i2));
for i = 1:length(i1)
    M1_8a.mag = i1(i);
    for j = 1:length(i2)
        M2_8a.mag = i2(j); M3_8a.mag = i2(j);
        Fnet = F1_8a.getForce(0)+ F2_8a.getForce(0) + F3_8a.getForce(0);
        fig8a_rep(i,j) = Fnet(1);
    end
end
figure(2);clf;
plot(i2,fig8a_rep);
legend('i1 = 1','i1 = 2','i1 = 3','i1 = 4'); title('Sim Fig8a Data - Force vs Side Magnet Current for Different Back Magnet Currents @ Theta_{max}');
xlabel('Current Through M2 and M3');ylabel('Force');

%%
%vary current in side and back magnets at phase difference that produces
%max repulsive current
[c,i] = min(fig7_rep(:,2));

theta_min = phases(i);
phase1_8b = 0;
phase2_8b = theta_min;

%create the new magnets
M1_8b = magnet(2,phase1_8b,freq, m1_x, m1_m);
M2_8b = magnet(1,phase2_8b,freq, m2_x, m2_m);
M3_8b = magnet(1,phase2_8b,freq, m3_x, m3_m);
%new plate
P1_8b = plate([0;0;0], [0,0,0,1]);
P1_8b.sx = t;P1_8b.sy= w;P1_8b.sz = l;
P1_8b.xr = geoScaleFactor*w;P1_8b.yr = geoScaleFactor*t;P1_8b.zr = geoScaleFactor*t;

F1_8b = force(P1_8b,M1_8b,forceFun, torqueFun,0);
F2_8b = force(P1_8b,M2_8b,forceFun, torqueFun,0);
F3_8b = force(P1_8b,M3_8b,forceFun, torqueFun,0);

MB_8b = magmBody(P1_8b, M1_8b, M2_8b,M3_8b);

i1 = [1,2,3,4];
i2 = [0,1,2,3,4];

fig8b_rep = zeros(length(i1),length(i2));

for i = 1:length(i1)
    M1_8b.mag = i1(i);
    for j = 1:length(i2)
        M2_8b.mag = i2(j); M3_8b.mag = i2(j);
        Fnet = F1_8b.getForce(0)+ F2_8b.getForce(0) + F3_8b.getForce(0);
        fig8b_rep(i,j) = Fnet(1);
    end
end


figure(3);clf;

plot(i2,fig8b_rep(1,:),i2,fig8b_rep(2,:),i2,fig8b_rep(3,:),i2,fig8b_rep(4,:));
legend('i1 = 1','i1 = 2','i1 = 3','i1 = 4'); title('Sim Fig8b Data - Force vs Side Magnet Current for Different Back Magnet Currents @ Theta_{min}');
xlabel('Current Through M2 and M3');ylabel('Force');

xlim([0,4]);

%%
%varying frequency while keeping the current constant (note that this is
%hard to do without current control because changing the frequency changes the 
% inductance of the IR circuit that the magnets are part of


freqs = [60 120 180 240 300 360];

%create the new magnets
M1_9 = magnet(1.5,phase1,freq, m1_x, m1_m);
M2_9 = magnet(3,phase2,freq, m2_x, m2_m);
M3_9 = magnet(3,phase2,freq, m3_x, m3_m);

%new plate

P1_9 = plate([0;0;0], [0,0,0,1]);
P1_9.sx = t;P1_9.sy= w;P1_9.sz = l;
P1_9.xr = geoScaleFactor* w;P1_9.yr = geoScaleFactor*t;P1_9.zr = geoScaleFactor*t;

MB1_9 = magmBody(P1_9,M1_9,M2_9,M3_9);

forceFun = @(p,m,t)P1.plateForce(p,m,t);
torqueFun = @(p,m,t) [0;0;0];

F1_9 = force(P1_9,M1_9,forceFun, torqueFun,0);
F2_9 = force(P1_9,M2_9,forceFun, torqueFun,0);
F3_9 = force(P1_9,M3_9,forceFun, torqueFun,0);

fig9_rep = zeros(length(freqs),length(phases));

for i = 1:length(phases)
    M2_9.phase = phases(i);
    M3_9.phase = phases(i);
    for j = 1:length(freqs)
        M1_9.freq = freqs(j);
        M2_9.freq = freqs(j);
        M3_9.freq = freqs(j);
        Fnet = P1.plateForce(P1_9,M1_9,0)+P1.plateForce(P1_9,M2_9,0)+P1.plateForce(P1_9,M3_9,0);
        fig9_rep(i,j) = Fnet(1);
    end
end

figure(4);clf;

plot(phases*180/pi, fig9_rep);
title('Sim Fig9 Data - Force vs. Phase Offset for Different Frequencies');
xlabel('Phase offset');ylabel('Force');
legend('60 hz','120 hz','180 hz','240 hz','300 hz','360 hz');

%%
% test the effect of the thickness of the material on the forces. Ideally,
% this wouldn't matter because the frequency would be high enough to
% mitigate the skin depth. 

depths = 0.001*[0.1 0.5 1 5];

%create the new magnets
M1_10 = magnet(2,phase1,freq, m1_x, m1_m);
M2_10 = magnet(4,phase2,freq, m2_x, m2_m);
M3_10 = magnet(4,phase2,freq, m3_x, m3_m);

%new plate

P1_10 = plate([0;0;0], [0,0,0,1]);
P1_10.sx = t;P1_10.sy= w;P1_10.sz = l;
P1_10.xr = geoScaleFactor* w;P1_10.yr = geoScaleFactor*t;P1_10.zr = geoScaleFactor*t;

MB1_10 = magmBody(P1_10,M1_10,M2_10,M3_10);

forceFun = @(p,m,t)P1.plateForce(p,m,t);
torqueFun = @(p,m,t) [0;0;0];

F1_10 = force(P1_10,M1_10,forceFun, torqueFun,0);
F2_10 = force(P1_10,M2_10,forceFun, torqueFun,0);
F3_10 = force(P1_10,M3_10,forceFun, torqueFun,0);

fig10_rep = zeros(length(depths),length(phases));

for i = 1:length(phases)
    M2_10.phase = phases(i);
    M3_10.phase = phases(i);
    for j = 1:length(depths)
        P1_10.sx = depths(j);
        P1_10.yr = geoScaleFactor*depths(j);P1_10.zr = geoScaleFactor*depths(j);
        M1_10.pos = [0.002+depths(j)/2; 0;0];
        Fnet = P1_10.plateForce(P1_10,M1_10,0)+P1.plateForce(P1_10,M2_10,0)+P1.plateForce(P1_10,M3_10,0);
        fig10_rep(i,j) = Fnet(1);
    end
end

figure(5);clf;

plot(phases*180/pi, fig10_rep);
title('Sim Fig10 Data - Force vs. Phase Offset for Different Plate Thickness @60 hz'); xlabel('Phase offset');ylabel('Force');legend('0.1 mm','0.5 mm','1 mm','5 mm');


