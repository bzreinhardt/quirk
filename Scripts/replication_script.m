%After optimizing c1-c6 and geoscale factor for fig 7,9, 10, and all 

%x0 =[1;0.350000000000000;1.00000000000000e-05;0.500000000000000;0.000155787926046400;0.500000000000000;0.255979912110779;]
%xall [0.823862538604085;0.212537379430452;3.87520711150364e-05;0.490335912316472;4.44120647175149e-05;0.474885079774660;0.267472565225471;]
%x10 [0.978726417894547;0.198006361201138;0.00306554265062272;0.476200135643139;0.000180758017513321;0.506432438662097;0.499999982047656;]
%x7 [0.903069223677262;0.130267113919195;4.70577720955209e-05;0.492953906398797;8.52943200309630e-06;0.741180746731916;0.384158176325482;]
%x9 [0.886461272161623;0.279324391468524;0.000206072930174052;0.137851588520475;5.85933055246332e-05;0.652912592831129;0.245160869056772;]

%% increasing amplitude sinusoid as you increase i_23/i_1 FIG 7
load('test data/fig7.mat');

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
%geoScaleFactor = 1/4.05;
%geoScaleFactor = 0.255998834721689; %from optimization
geoScaleFactor = 0.267472565225471;
P1.sx = t;P1.sy= w;P1.sz = l;
%10/13/2013 see what happens when the radius isn't the full radius of the
%plate
P1.xr = geoScaleFactor*w;P1.yr = geoScaleFactor*t;P1.zr = geoScaleFactor*t;


%new magnet positions
% m1_x = [0.2;0;0];
% m2_x = [0;-0.2;0];
% m3_x = [0;0.2;0];
m1_x = [0.002+t/2; 0;0]; %position of the EM
m2_x = [0;-0.002-w/2;0]; 
m3_x = [0;0.002+w/2;0];

%new magnets will be pointing down the y axis towards the plate
m1_m = aa2q(vec,theta);  
m2_m = aa2q(vec2, -pi/2);
m3_m = aa2q(vec2, pi/2);

%create the new magnets
M1 = magnet(2,phase1,freq,m1_x, m1_m);
M2 = magnet(1,phase2,freq, m2_x, m2_m);
M3 = magnet(1,phase2,freq, m3_x, m3_m);



MB1 = magmBody(P1,M1,M2,M3);

forceFun = @(p,m,t)P1.plateForce(p,m,t);
torqueFun = @(p,m,t) [0;0;0];

F1 = force(P1,M1,forceFun, torqueFun,0);
F2 = force(P1,M2,forceFun, torqueFun,0);
F3 = force(P1,M3,forceFun, torqueFun,0);

%set up amplitude Ratios 
%set up phases
phases = linspace(-pi,pi,25);
i2 = [1,2,3,4];
fig7_rep = zeros(length(i2),length(phases));

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

fig7_diff = zeros(size(fig7_data_x));
for j = 1:length(i2)
    M2.mag = i2(j);
    M3.mag = i2(j);
    
    for i = 1:length(fig7_data_x(:,i2(j)))
        if isnan(fig7_data_x(i,i2(j))) 
            fig7_diff(i,j) = NaN;
        else
        M2.phase = pi/180*fig7_data_x(i,i2(j));
        M3.phase = pi/180*fig7_data_x(i,i2(j));
        Fnet = P1.plateForce(P1,M1,0)+P1.plateForce(P1,M2,0)+P1.plateForce(P1,M3,0);
        fig7_diff(i,j) = Fnet(1);
        end
    end
end

figure(1);clf;
fig7_axes(1) =subplot(211);
 plot(phases*180/pi, fig7_rep(:,1),phases*180/pi, fig7_rep(:,2),phases*180/pi, fig7_rep(:,3),phases*180/pi, fig7_rep(:,4));legend('1','2','3','4');
title('Sim Fig7 Data - Force vs Phase Offset for different magnet currents');legend('i1 = 2,i2 = 1','i1 = 2,i2 = 2','i1 = 2,i2 = 3','i1 = 2,i2 = 4'); xlabel('Phase offset');ylabel('Force')
fig7_axes(2) = subplot(212);
 plot(fig7_data_x,fig7_data_y); title('Paper Fig7 Data');legend('i1 = 2,i2 = 1','i1 = 2,i2 = 2','i1 = 2,i2 = 3','i1 = 2,i2 = 4');
xlabel('Phase offset');ylabel('Force');
linkaxes(fig7_axes);

figure(6);clf;
fig7_axes2(1) = subplot(211); plot(fig7_data_x,fig7_diff);
fig7_axes2(2) =subplot (212); plot(fig7_data_x,(fig7_diff-fig7_data_y)./fig7_data_y);
linkaxes(fig7_axes2,'x');
%%
% TODO -as you increase i_23 at phase_max force (theta = -75), 
% force in z direction increases linearly with i_1, but with diminishing returns (d_slope/d_i1 < 0) FIG 8a
load('test data/fig8a.mat');
%right now theta_max = -15 - need to fix that eventually
[c,i] = max(fig7_rep(:,1));

theta_max = phases(i);
phase1 = 0;
phase2 = theta_max;

%create the new magnets
M1_8a = magnet(2,phase1,freq, m1_x, m1_m);
M2_8a = magnet(1,phase2,freq, m2_x, m2_m);
M3_8a = magnet(1,phase2,freq, m3_x, m3_m);
%new plate
P1_8a = plate([0;0;0], [0,0,0,1]);

P1_8a.sx = t;P1_8a.sy= w;P1_8a.sz = l;
P1_8a.xr = geoScaleFactor*w;P1_8a.yr = geoScaleFactor*t;P1_8a.zr = geoScaleFactor*t;

F1_8a = force(P1_8a,M1_8a,forceFun, torqueFun,0);
F2_8a = force(P1_8a,M2_8a,forceFun, torqueFun,0);
F3_8a = force(P1_8a,M3_8a,forceFun, torqueFun,0);

MB_8a = magmBody(P1_8a, M1_8a, M2_8a,M3_8a);

i1 = [1,2,3,4];
i2 = [0,1,2,3,4];

fig8a_rep = zeros(length(i1),length(i2));
fig8a_diff = zeros(size(fig8a_data_x));

for i = 1:length(i1)
    M1_8a.mag = i1(i);
    for j = 1:length(i2)
        M2_8a.mag = i2(j); M3_8a.mag = i2(j);
        Fnet = F1_8a.getForce(0)+ F2_8a.getForce(0) + F3_8a.getForce(0);
        fig8a_rep(i,j) = Fnet(1);
    end
end

for i = 1:length(i1)
    M1_8a.mag = i1(i);
    for j = 1:length(fig8a_data_x(:,i))
        if isnan(fig8a_data_x(j,i)) 
            fig8a_diff(j,i) = NaN;
        else
            M2_8a.mag = fig8a_data_x(j,i) ; M3_8a.mag = fig8a_data_x(j,i) ;
            Fnet = F1_8a.getForce(0)+ F2_8a.getForce(0) + F3_8a.getForce(0);
            fig8a_diff(j,i) = Fnet(1);
        end
    end
end



figure(2);clf;
fig8a_axes(1) = subplot(211);
plot(i2,fig8a_rep(1,:),i2,fig8a_rep(2,:),i2,fig8a_rep(3,:),i2,fig8a_rep(4,:));
legend('i1 = 1','i1 = 2','i1 = 3','i1 = 4'); title('Sim Fig8a Data - Force vs Side Magnet Current for Different Back Magnet Currents @ Theta_{max}');
xlabel('Current Through M2 and M3');ylabel('Force');
fig8a_axes(2) = subplot(212);
plot(fig8a_data_x,fig8a_data_y); title('Paper Fig8a Data')
legend('i1 = 1','i1 = 2','i1 = 3','i1 = 4');xlabel('Current Through M2 and M3');ylabel('Force');

linkaxes(fig8a_axes);

figure(7);clf;
fig8a_axes2(1) = subplot(211); plot(fig8a_data_x,fig8a_diff);
fig8a_axes2(2) =subplot (212); plot(fig8a_data_x,(fig8a_diff-fig8a_data_y)./fig8a_data_y);
linkaxes(fig8a_axes2,'x');
%%
% TODO -as you increase i_23 at phase_min force (theta = 105 degrees) 
% force decreases linearly with i_1 with increasing returns (d_slope/di_1 > 1) FIG 8b
% 
load('test data/fig8b.mat');
%for now it looks like the phase offset with the largest negative (min)
%force is ~160
[c,i] = min(fig7_rep(:,1));

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
fig8b_diff = zeros(size(fig8b_data_x));

for i = 1:length(i1)
    M1_8b.mag = i1(i);
    for j = 1:length(i2)
        M2_8b.mag = i2(j); M3_8b.mag = i2(j);
        Fnet = F1_8b.getForce(0)+ F2_8b.getForce(0) + F3_8b.getForce(0);
        fig8b_rep(i,j) = Fnet(1);
    end
end

for i = 1:length(i1)
    M1_8b.mag = i1(i);
    for j = 1:length(fig8b_data_x(:,i))
        if isnan(fig8b_data_x(j,i)) 
            fig8b_diff(j,i) = NaN;
        else
            M2_8b.mag = fig8b_data_x(j,i) ; M3_8b.mag = fig8b_data_x(j,i) ;
            Fnet = F1_8b.getForce(0)+ F2_8b.getForce(0) + F3_8b.getForce(0);
            fig8b_diff(j,i) = Fnet(1);
        end
    end
end

figure(3);clf;
fig8b_axes(1) = subplot(211);
plot(i2,fig8b_rep(1,:),i2,fig8b_rep(2,:),i2,fig8b_rep(3,:),i2,fig8b_rep(4,:));
legend('i1 = 1','i1 = 2','i1 = 3','i1 = 4'); title('Sim Fig8b Data - Force vs Side Magnet Current for Different Back Magnet Currents @ Theta_{min}');
xlabel('Current Through M2 and M3');ylabel('Force');
fig8b_axes(2) = subplot(212);
plot(fig8b_data_x,fig8b_data_y); title('Paper Fig8b Data')
xlabel('Current Through M2 and M3');ylabel('Force');legend('i1 = 1','i1 = 2','i1 = 3','i1 = 4');
xlim([0,4]);
linkaxes(fig8b_axes);

figure(8);clf;
fig8b_axes2(1) = subplot(211); plot(fig8b_data_x,fig8b_diff);
fig8b_axes2(2) =subplot (212); plot(fig8b_data_x,(fig8b_diff-fig8b_data_y)./fig8b_data_y);
linkaxes(fig8b_axes2,'x');

%%
% TODO -increasing excitation frequency increases amplitude of force, dd
% but decrease offset of the sine wave with phase difference FIG 9

%Need to modulate how force depends on frequency figure doesn't agree

load('test data/fig9.mat');

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
fig9_diff = zeros(size(fig9_data_x));

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

for i = 1:size(fig9_data_x,1)
    for j = 1:size(fig9_data_x,2)
        if isnan(fig9_data_x(i,j)) 
            fig9_diff(i,j) = NaN;
        else
        M2_9.phase = pi/180*fig9_data_x(i,j);
        M3_9.phase = pi/180*fig9_data_x(i,j);
        M1_9.freq = freqs(j);
        M2_9.freq = freqs(j);
        M3_9.freq = freqs(j);
        Fnet = P1.plateForce(P1_9,M1_9,0)+P1.plateForce(P1_9,M2_9,0)+P1.plateForce(P1_9,M3_9,0);
        fig9_diff(i,j) = Fnet(1);
        end
    end
end

figure(4);clf;
fig9_axes(1) = subplot(211);
plot(phases*180/pi, fig9_rep(:,1),phases*180/pi, fig9_rep(:,2),phases*180/pi, fig9_rep(:,3),phases*180/pi, fig9_rep(:,4),phases*180/pi, fig9_rep(:,5),phases*180/pi, fig9_rep(:,6));
title('Sim Fig9 Data - Force vs. Phase Offset for Different Frequencies');xlabel('Phase offset');ylabel('Force');legend('60 hz','120 hz','180 hz','240 hz','300 hz','360 hz');
fig9_axes(2) = subplot(212);
plot(fig9_data_x,fig9_data_y); title('Paper Fig9 Data');legend('60 hz','120 hz','180 hz','240 hz','300 hz','360 hz');
title('Paper Fig9 Data');xlabel('Phase offset');ylabel('Force');

linkaxes(fig9_axes);

figure(9);clf;
fig9_axes2(1) = subplot(211); plot(fig9_data_x,fig9_diff);
fig9_axes2(2) =subplot (212); plot(fig9_data_x,(fig9_diff-fig9_data_y)./fig9_data_y);
linkaxes(fig9_axes2,'x');

%%
% TODO -increasing plate thickness has the same effect as increasing excitation frequency
% - increasing amplitude of force with phase difference but decreaing offset FIG 10

load('test data/fig10.mat');

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
fig10_diff = zeros(size(fig10_data_x));

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

for i = 1:size(fig10_data_x,1)
    for j = 1:size(fig10_data_x,2)
        if isnan(fig10_data_x(i,j)) 
            fig10_diff(i,j) = NaN;
        else
        M2_10.phase = pi/180*fig10_data_x(i,j);
        M3_10.phase = pi/180*fig10_data_x(i,j);
        P1_10.sx = depths(j);
        P1_10.yr = geoScaleFactor*depths(j);P1_10.zr = geoScaleFactor*depths(j);
        M1_10.pos = [0.002+depths(j)/2; 0;0];
        Fnet = P1_10.plateForce(P1_10,M1_10,0)+P1.plateForce(P1_10,M2_10,0)+P1.plateForce(P1_10,M3_10,0);
        fig10_diff(i,j) = Fnet(1);
        end
    end
end

figure(5);clf;
fig10_axes(1) = subplot(211);
plot(phases*180/pi, fig10_rep(:,1),phases*180/pi, fig10_rep(:,2),phases*180/pi, fig10_rep(:,3),phases*180/pi, fig10_rep(:,4));
title('Sim Fig10 Data - Force vs. Phase Offset for Different Plate Thickness'); xlabel('Phase offset');ylabel('Force');legend('0.1 mm','0.5 mm','1 mm','5 mm');
fig10_axes(2) = subplot(212);plot(fig10_data_x,fig10_data_y); title('Paper Fig10 Data');
 xlabel('Phase offset');ylabel('Force');legend('0.1 mm','0.5 mm','1 mm','5 mm');
linkaxes(fig10_axes);

figure(10);clf;
fig10_axes2(1) = subplot(211); plot(fig10_data_x,fig10_diff);
fig10_axes2(2) =subplot (212); plot(fig10_data_x,(fig10_diff-fig10_data_y)./fig10_data_y);
linkaxes(fig10_axes2,'x');

diff_norms = [norm(fig7_diff,'fro');norm(fig8a_diff,'fro');norm(fig9_diff,'fro');norm(fig10_diff,'fro')];

%sendmail('bzreinhardt@gmail.com', 'Script Done', 'We live in the future!');