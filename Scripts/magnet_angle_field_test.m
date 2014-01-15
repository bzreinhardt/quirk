%set up points on a plate - much larger than magnet spacing

zgv = linspace(-10,10,20);
ygv = linspace(-10,10,20);
[Y,Z] = meshgrid(ygv,zgv);
%set up magnets
vec = [0;1;0]; %vector the magnet will be rotated around
freq = 60; %frequency of the EM in hz
phase = 0; %phase of the EM - all phases are relative
dist = linspace(0.01,1,10);
angles = linspace(0,pi/2,16);
rotq2 = aa2q([0;0;1],-pi/length(angles));
rotq3 = aa2q([0;0;1], pi/length(angles));
vec2 = [1;0;0];
m2_m = aa2q(vec2, -pi/2);
m3_m = aa2q(vec2, pi/2);

BperpVec = zeros(length(angles),length(dist));
BplaneVec = zeros(length(angles),length(dist));

perpMax = zeros(length(dist),1);
perpMaxI = zeros(length(dist),1);
planeMax = zeros(length(dist),1);
planeMaxI = zeros(length(dist),1);

for q = 1:length(dist)
    m2_x = [dist(q);-0.2;0];
    m3_x = [dist(q);0.2;0];
    
    
    M2 = magnet(1,phase,freq, m2_x, m2_m);
    M3 = magnet(1,phase,freq, m3_x, m3_m);
    
    MB = magmBody(M2,M3);
    %For angles
    
   
    atts2 = zeros(length(angles),4);
    atts3 = zeros(length(angles),4);
    vec2 = zeros(3,length(angles));
    vec3 = zeros(3,length(angles));
    Bx = zeros(length(ygv),length(zgv),length(angles));
    By = zeros(length(ygv),length(zgv),length(angles));
    Bz = zeros(length(ygv),length(zgv),length(angles));
    for i = 1:length(angles)
        atts2(i,:) = M2.att;
        atts3(i,:) = M3.att;
        vec2(:,i) = att2vec(M2.att,'z');
        vec3(:,i) = att2vec(M3.att,'z');
        for j = 1:length(zgv)
            z = zgv(j);
            for k = 1:length(ygv)
                y = ygv(k);
                Bx(k,j,i) = magFlux(0,y,z,M2.pos, att2vec(M2.att,'z'), 'x' )...
                    +magFlux(0,y,z,M2.pos, att2vec(M2.att,'z'), 'x' );
                By(k,j,i) = magFlux(0,y,z,M2.pos, att2vec(M2.att,'z'), 'y' )...
                    +magFlux(0,y,z,M2.pos, att2vec(M2.att,'z'), 'z' );
                Bz(k,j,i) = magFlux(0,y,z,M2.pos, att2vec(M2.att,'z'), 'z' )...
                    +magFlux(0,y,z,M2.pos, att2vec(M2.att,'z'), 'z' );
            end
        end
        M2.att = mq2qq(quatmultiply(qq2mq(rotq2),qq2mq(atts2(i,:))));
        M3.att = mq2qq(quatmultiply(qq2mq(rotq3),qq2mq(atts3(i,:))));
    end
    Bplane = (By.^2+Bz.^2).^.5;
    Bperp  = (Bx.^2).^.5;
    
    Bplane_net = sum(sum(Bplane));
    Bperp_net  = sum(sum(Bperp));
    
    for n = 1:length(angles)
        BperpVec(n,q) = Bperp_net(:,:,n);
        BplaneVec(n,q) = Bplane_net(:,:,n);
    end
    [planeC,planeI] = max(BplaneVec(:,q));
    [perpC,perpI] = max(BperpVec(:,q));
    perpMax(q) = perpC;
    perpMaxI(q) = perpI;
    planeMax(q) = planeC;
    planeMaxI(q) = planeI;
end

figure(1);clf; plot(angles,BperpVec,angles,BplaneVec); legend('perp','plane');
figure(2);clf;MB.draw();
hold on
v2 = att2vec(M2.att,'z');
v3 = att2vec(M3.att,'z');
quiver3(m2_x(1),m2_x(2),m2_x(3),0.1*v2(1),0.1*v2(2),0.1*v2(3),'color','r');
quiver3(m3_x(1),m3_x(2),m3_x(3),0.1*v3(1),0.1*v3(2),0.1*v3(3),'color','g');

for i = 1:length(planeMaxI) planeMaxA(i) = angles(planeMaxI(i)); end
for i = 1:length(perpMaxI) perpMaxA(i) = angles(perpMaxI(i)); end
perpMaxA=perpMaxA*180/pi;planeMaxA=planeMaxA*180/pi;
figure(3);clf; plot(dist,planeMaxA,dist,perpMaxA);legend('plane','perp');


