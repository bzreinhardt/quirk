classdef iLoop < hgsetget
    %iLoop keeps track of a current loop in a plate
    
    properties
        plate = {};
        phase = 0;
        radius = 0;
        perpAxis = 'x';
        perpCoord = 0;
        cur = 0;
        L_0 = 6.91083397316373e-05; %inductance of loop
        L = 6.91083397316373e-05;
     
        R = 0.0787191335859532;%resistance of loop
        alpha = pi/2; %default alpha for permanent magnets
        mag = {}; %the loops should remember what magnet made it - not strictly necessary but useful for bookeeping
        
        %these properties actually depend on the properties of the plate
        %and the size of the loop - TODO
        Z = 1.44; %impedence of the loop
        skinFactor = 1; %scaling factor to take into account skin effect
        delta = 0.01;
        
        SF_shift = 0; %parameter to adjust the position of the loop based on the skin effect - because the loop is 
        %lumped, the average position of the loop will move towards the
        %generating magnet as frequency increases
        %freq_pwr = 1; %parameter to just the effect of frequency on the force
        %freq_pwr = 0.722624499833053; %from optimization
        %freq_pwr = 0.505113090390219; %from optimization of high frequency data
        %freq_pwr = 0.25; % just a random test
        c1 = 0.823862538604085; %scaling factor on force increase from frequency
        c2 = 0.212537379430452; %power frequency is raised to 
        c3 = 3.87520711150364e-05;%scaling factor on skin effect position adjustment
        c4 = 0.490335912316472; %power on skin effect position adjustment
        c5 = 4.44120647175149e-05; %scaling factor on inductance based on the skin depth
        c6 = 0.474885079774660; %parameter to adjust the effect of the skin effect
        magSide = 0; % a variable +/- 1 depending on whether the magnet is on the positive
        %or negative side of the loop in body coordinates
        %inductance of a circular loop
        %u_0*u_r*R*ln(
        %inductance of a magnetic core solenoid
        % L = u_0*u_r*N^2*A/l
        % u_r = relative permiability, u_0 magnetic constant N - turns
        % A-xsectional area 
        %inductance of cicular loop L= u0*ur (ln(8*R/a)-2+1/4+O(a^2/R^2))
        %a - wire radius, R = loop radius
        %R = rho*l/A - l = length of a = cross sectional area
        
    end
    
    methods
        %% Constructor
        function loop = iLoop(plate, mag, r, axis, coord)
            %plate = plate that the loop lives on
            %mag = magnet generating the loop
            %r = radius of loop
            %axis - axis perpendicular to loop
            %coord - coordinate along that axis that the loop lives on
            loop.mag = mag;
            loop.plate = plate;
            
            loop.perpAxis = axis;
            loop.perpCoord = coord;
            loop.radius = r;
            
            omega = mag.freq*2*pi;
            if norm(mag.om) > 0 %ok, the model can't do spinning, frequency varying EMs right now
                omega = norm(mag.om);
            end
            %10/14 try lowering resistance of loop to see effect on alpha,
        %these numbers were for the MAGNETS from the paper, not the plate
            %loop.L = 11.7 *10^-4; %inductance of loop
            %loop.R = 1;%resistance of loop
            %the below values derived from optimizing the differences from
            %fig 7 in ohji
            %loop.L = 5.243472655280613e-04;
            %loop.R = 0.597055320645011;
            %after reoptimizing after changing the freq_pwr
            %loop.L = 0.000168416665606285;
            %loop.R = 0.191823466824657;
            if omega > 0
                delt = sqrt(2*plate.rho/(omega*plate.mu));
            end
            
            
            
            
            loop.delta = delt;
            
            %need to check whether loop goes outside the plate
            
             %convert vector from magnet to plate into body coordinates
            
            xMag_bod = mag.pos-plate.pos;%vector to center of plate from magenet in body coordinates
            vMag_bod = mag.vel-plate.vel; %relative velocity of the magnet
            mMag = att2vec(mag.att,'z');
            rotM = quat2dcm(qq2mq(plate.att)); %matrix to convert inertial-->plate coordinate
            rotMag2I = inv(quat2dcm(qq2mq(mag.att)));
            
            mMag_bod = rotM*mMag;%magnet moment in body coordinates
            
            omMag_bod = rotM*rotMag2I*mag.om;
            mag_dBdt = mag.mag;
            
            
            mMag_bod(abs(mMag_bod)<1E-10) = 0; %get rid of weird numerical stupid when doing rotations
            
            omMag_bod(abs(omMag_bod)<1E-10) = 0; %get rid of weird numerical stupid when doing rotations
            
            %find the closest face to the magnet
            x_dist = min(abs(xMag_bod(1) - plate.sx/2),abs(xMag_bod(1)+plate.sx/2));
            y_dist = min(abs(xMag_bod(2) - plate.sy/2),abs(xMag_bod(2)+plate.sy/2));
            z_dist = min(abs(xMag_bod(3) - plate.sz/2),abs(xMag_bod(3)+plate.sz/2));
            
            if x_dist <= y_dist && x_dist <= z_dist
                closeAxis = 'x';
            elseif y_dist < x_dist && y_dist < z_dist
                closeAxis = 'y';
            elseif z_dist < y_dist && z_dist < x_dist
                closeAxis = 'z';
            end
            
            switch axis
                case 'x'
                    %record which side of the loop the magnet is positioned
                    if strcmp('x',closeAxis) == 1
                        if xMag_bod(1) > 0
                            loop.magSide = 1;
                        else
                            loop.magSide = -1;
                        end
                        loop.perpCoord = loop.magSide*loop.c3*omega^loop.c4*plate.sx/2;
                    end
                    
                    if r > plate.sy/2 || r > plate.sz/2
                        warning('loop radius extends out of plate. changed to edge of plate.');
                        loop.radius = min([plate.sy, plate.sz]);
                    end
                    
                    if abs(loop.perpCoord) > plate.sx/2
                        warning('loop coordinate extends out of plate. changed to edge of plate.');
                        loop.perpCoord = loop.magSide*plate.sx/2;
                    end
                    
                    SF = (1-exp(-1*plate.sx/delt));
                    loop.skinFactor = SF;
                
                case 'y'
                    if strcmp('y',closeAxis) == 1
                        if xMag_bod(2) > 0
                            loop.magSide = 1;
                        else
                            loop.magSide = -1;
                        end
                        loop.perpCoord = loop.magSide*loop.c3*omega^loop.c4*plate.sy/2;
                    end
                    
                    if r > plate.sx/2 || r > plate.sz/2
                        warning('loop radius extends out of plate. changed to edge of plate.');
                        loop.radius = min([plate.sx, plate.sz]);
                    end
                    
                    if abs(loop.perpCoord) > plate.sy
                        warning('loop coordinate extends out of plate. changed to edge of plate.');
                        loop.perpCoord = loop.magSide*plate.sy/2;
                    end
                    SF = (1-exp(-1*plate.sy/delt));
                    loop.skinFactor = SF;
                    
                case 'z'
                    if strcmp('z',closeAxis) == 1
                        if xMag_bod(3) > 0
                            loop.magSide = 1;
                        else
                            loop.magSide = -1;
                        end
                        loop.perpCoord =  loop.magSide*loop.c3*omega^loop.c4*plate.sz/2;
                    end
                    
                    if r > plate.sy/2 || r > plate.sx/2
                        warning('loop radius extends out of plate. changed to edge of plate.');
                        loop.radius = min([plate.sx, plate.sy]);
                    end
                    
                    if abs(loop.perpCoord) > plate.sz
                        warning('loop coordinate extends out of plate. changed to edge of plate.');
                        loop.perpCoord = loop.magSide*plate.sz/2;
                    end
                    SF = (1-exp(-1*plate.sz/delt));
                    loop.skinFactor = SF;
            end
            
           %update inductance based on how deep it penetrates
           loop.L = loop.L_0 + loop.c5*loop.skinFactor;
           if omega > 0 
                loop.alpha = atan2(omega*loop.L,loop.R);
                loop.Z = (loop.R^2+omega^2*loop.L^2)^.5;
                
            else
                loop.alpha = pi/2; %alpha has no meaning if permanent magnet and thus no frequency
            end
            loop.phase = mag.phase+loop.alpha;
        
            
            %find loop from a sinusoidally driven electromagnet
            if mag.freq ~= 0;
                int = @(theta,rad)vec_dBdt_int( theta, rad,loop.perpCoord, mag_dBdt, mMag_bod, xMag_bod,axis);
                %int = @(theta,rad)dBdt_int( theta, rad,loop.perpCoord, mag_dBdt, mMag_bod, xMag_bod,axis);
                %disp(r);
                loop.cur = -1*SF^(loop.c6)*omega/loop.Z*integral2 (int,0,2*pi,0,r);
            
            %find a loop from a moving permanent magnet 
            
            %need to consider both linear velocity of the magnet and the
            %linear velocity from rotation at the point of the magnetic
            %field
            %10/14 - added skinFactor to account for penetration of current
            %into the plate
            elseif mag.freq == 0
                int = @(theta) vXB_int(theta, r, loop.perpCoord, mMag_bod, xMag_bod, omMag_bod,vMag_bod,axis);
                loop.cur = -1*SF^(loop.c6)*plate.sigma*integral(int, 0, 2*pi,'ArrayValued',true);
            %integrate vXB dotted with ds around the loop
            
            else
                error('magnet frequency not a number!');
            end
            
        end
        
        %% Update the existing loop
        function loop = loopUpdate(loop)
           
            
            omega = loop.mag.freq*2*pi;
            if norm(loop.mag.om) > 0 %ok, the model can't do spinning, frequency varying EMs right now
                omega = norm(loop.mag.om);
            end
            if omega > 0
                loop.delta = sqrt(2*loop.plate.rho/(omega*loop.plate.mu));
            end
            
            
            xMag_bod = loop.mag.pos-loop.plate.pos;%vector to center of plate from magenet in body coordinates
            vMag_bod = loop.mag.vel-loop.plate.vel; %relative velocity of the magnet
            
            mMag = att2vec(loop.mag.att,'z');
            rotM = quat2dcm(qq2mq(loop.plate.att));
            rotMag2I = inv(quat2dcm(qq2mq(loop.mag.att))); %convert mag_body--> intertial coords
            
            mMag_bod = rotM*mMag;%magnet moment in body coordinates
            mMag_bod(abs(mMag_bod)<1E-10) = 0; %get rid of weird numerical stupid when doing rotations
            omMag_bod = rotM*rotMag2I*loop.mag.om;
            omMag_bod(abs(omMag_bod)<1E-10) = 0; %get rid of weird numerical stupid when doing rotations
             %need to check whether loop goes outside the plate
            
             
            %find the closest face to the magnet
            x_dist = min(abs(xMag_bod(1) - loop.plate.sx/2),abs(xMag_bod(1)+loop.plate.sx/2));
            y_dist = min(abs(xMag_bod(2) - loop.plate.sy/2),abs(xMag_bod(2)+loop.plate.sy/2));
            z_dist = min(abs(xMag_bod(3) - loop.plate.sz/2),abs(xMag_bod(3)+loop.plate.sz/2));
            
            if x_dist <= y_dist && x_dist <= z_dist
                closeAxis = 'x';
            elseif y_dist < x_dist && y_dist < z_dist
                closeAxis = 'y';
            elseif z_dist < y_dist && z_dist < x_dist
                closeAxis = 'z';
            end
            
             switch loop.perpAxis
                case 'x'
                    if strcmp('x',closeAxis) == 1
                        if xMag_bod(1) > 0
                            loop.magSide = 1;
                        else
                            loop.magSide = -1;
                        end
                        loop.perpCoord = loop.magSide*loop.c3*omega^loop.c4*loop.plate.sx/2;
                    end
                    
                    loop.radius = loop.plate.xr;
                    if loop.radius > loop.plate.sy/2 || loop.radius > loop.plate.sz/2
                        warning('loop radius extends out of plate. changed to edge of plate.');
                        loop.radius = min([loop.plate.sy, loop.plate.sz]);
                    end
                    
                    if abs(loop.perpCoord) > loop.plate.sx/2
                        warning('loop coordinate extends out of plate. changed to edge of plate.');
                        loop.perpCoord = loop.magSide*loop.plate.sx/2;
                    end
                    loop.skinFactor = (1-exp(-1*loop.plate.sx/loop.delta));
                
                 case 'y'
                     if strcmp('y',closeAxis) == 1
                         if xMag_bod(2) > 0
                             loop.magSide = 1;
                         else
                             loop.magSide = -1;
                         end
                         loop.perpCoord =  loop.magSide*loop.c3*omega^loop.c4*loop.plate.sy/2;
                     end
                     
                    loop.radius = loop.plate.yr;
                    if loop.radius > loop.plate.sx/2 || loop.radius > loop.plate.sz/2
                        warning('loop radius extends out of plate. changed to edge of plate.');
                        loop.radius = min([loop.plate.sx, loop.plate.sz]);
                    end
                    
                    if abs(loop.perpCoord) > loop.plate.sy/2
                        warning('loop coordinate extends out of plate. changed to edge of plate.');
                        loop.perpCoord = loop.magSide*loop.plate.sy/2;
                    end
                    loop.skinFactor = (1-exp(-1*loop.plate.sy/loop.delta));
                    
                case 'z'
                    if strcmp('z',closeAxis) == 1
                        if xMag_bod(3) > 0
                            loop.magSide = 1;
                        else
                            loop.magSide = -1;
                        end
                        loop.perpCoord =  loop.magSide*loop.c3*omega^loop.c4*loop.plate.sz/2;
                    end
                    loop.radius = loop.plate.zr;
                    if loop.radius > loop.plate.sy/2 || loop.radius > loop.plate.sx/2
                        warning('loop radius extends out of plate. changed to edge of plate.');
                        loop.radius = min([loop.plate.sx/2, loop.plate.sy/2]);
                    end
                    
                    if abs(loop.perpCoord) > loop.plate.sz/2
                        warning('loop coordinate extends out of plate. changed to edge of plate.');
                        loop.perpCoord = loop.magSide*loop.plate.sz/2;
                    end
                    loop.skinFactor = (1-exp(-1*loop.plate.sz/loop.delta));
             end
             
             loop.L = loop.L_0 + loop.c5*loop.skinFactor;
            if omega > 0
                loop.alpha = atan2(omega*loop.L,loop.R);
                loop.Z = (loop.R^2+omega^2*loop.L^2)^.5;
                
            else
                loop.alpha = pi/2; %alpha has no meaning if permanent magnet and thus no frequency
            end
            loop.phase = loop.mag.phase+loop.alpha;
            
             
            
            mag_dBdt = loop.mag.mag;
            %10/14 - added skinFactor to account for penetration of current
            %into the plate
            %find loop from a sinusoidally driven electromagnet
            if omega ~= 0
                int = @(theta,rad)vec_dBdt_int( theta, rad,loop.perpCoord, mag_dBdt, mMag_bod, xMag_bod,loop.perpAxis);
                %int = @(theta,rad)dBdt_int( theta, rad,loop.perpCoord, mag_dBdt, mMag_bod, xMag_bod,loop.perpAxis);
                loop.cur = -1*loop.skinFactor^(loop.c6)*omega/loop.Z*integral2 (int,0,2*pi,0,loop.radius);
            
            %find a loop from a moving permanent magnet 
            
            %need to consider both linear velocity of the magnet and the
            %linear velocity from rotation at the point of the magnetic
            %field
            elseif loop.mag.freq == 0
                
                int = @(theta) vXB_int(theta, loop.radius, loop.perpCoord, mMag_bod, xMag_bod, omMag_bod,vMag_bod,loop.perpAxis);
                loop.cur = -1*(loop.skinFactor)^(loop.c6)*loop.plate.sigma*integral(int, 0, 2*pi);
            %integrate vXB dotted with ds around the loop
            else
                error('magnet frequency not a number!');
            end
            
        end
    end
        
    methods(Static)
        
        function [ F ] = loopForce(loop, mag, loop_pts)
            %Finds the force of a single magnet on a single current loop (in the loop body
            %coordinates)
            
            %set alpha as pi over 2 for now, change this later
            %alpha = pi/2;
            %these properties should go into the plate
            %  omega = 60*2*pi;
            %  L = 11.7 *10^-3;
            %  R = 1.44;
            %  alpha = atan2(omega*L,R);
            r = loop.radius;
            f_thetas = linspace(0,2*pi*(loop_pts -1)/loop_pts,loop_pts);
            F = zeros(3,1);
            
            xMag_bod = mag.pos-loop.plate.pos; %vector to center of plate from magenet in body coordinates
            mMag = att2vec(mag.att,'z');
            rotM = quat2dcm(qq2mq(loop.plate.att));
            mMag_bod = rotM*mMag; %magnet moment in body coordinates
            mMag_bod(abs(mMag_bod)<1E-10) = 0; %get rid of weird numerical stupid when doing rotations
            
            
            switch loop.perpAxis
                %current in YZ plane
                case 'x'
                    
                    x2 = loop.perpCoord*ones(size(f_thetas));
                    y2 = r*cos(f_thetas);
                    z2 = r*sin(f_thetas);
                    currentVec2(:,:,1) = zeros(size(f_thetas));
                    currentVec2(:,:,2) = loop.cur*-sin(f_thetas);
                    currentVec2(:,:,3) = loop.cur* cos(f_thetas);
                    m_x = mMag_bod(1)*ones(size(f_thetas));
                    m_y = mMag_bod(2)*ones(size(f_thetas));
                    m_z = mMag_bod(3)*ones(size(f_thetas));
                    x_m = xMag_bod(1)*ones(size(f_thetas));
                    y_m = xMag_bod(2)*ones(size(f_thetas));
                    z_m = xMag_bod(3)*ones(size(f_thetas));
                    B = vecMagFlux(x2,y2,z2,x_m,y_m,z_m, m_x,m_y,m_z, 'all');
                    F_pts = cross(currentVec2,B,3);
                    F_sum = sum(F_pts);
                    F = [F_sum(:,:,1);F_sum(:,:,2);F_sum(:,:,3)];
                   
                    
%                     F1 = zeros(3,loop_pts);
%                     B1 = zeros(3,loop_pts);
%                     x = loop.perpCoord;
%                     for j = 1:loop_pts
%                         
%                         y = r*cos(f_thetas(j));
%                         z = r*sin(f_thetas(j));
%                         currentVec = loop.cur*[0;-sin(f_thetas(j));cos(f_thetas(j))]; %create the current vector
%                         B1(:,j) = magFlux(x,y,z,xMag_bod,mMag_bod,'all');
%                         F1(:,j) = iCrossB(currentVec,x,y,z,mMag_bod,xMag_bod);
%                         F = F + iCrossB(currentVec,x,y,z,mMag_bod,xMag_bod); %sum up the forces on the current around the loop
%                     end
                    %Current in XZ plane
                case 'y'
                    
                    y = loop.perpCoord*ones(size(f_thetas));
                    x = r*cos(f_thetas);
                    z = r*sin(f_thetas);
                    currentVec(:,:,1) = loop.cur* -sin(f_thetas);
                    currentVec(:,:,2) =  zeros(size(f_thetas));
                    currentVec(:,:,3) = loop.cur* cos(f_thetas);
                    m_x = mMag_bod(1)*ones(size(f_thetas));
                    m_y = mMag_bod(2)*ones(size(f_thetas));
                    m_z = mMag_bod(3)*ones(size(f_thetas));
                    x_m = xMag_bod(1)*ones(size(f_thetas));
                    y_m = xMag_bod(2)*ones(size(f_thetas));
                    z_m = xMag_bod(3)*ones(size(f_thetas));
                    B = vecMagFlux(x,y,z,x_m,y_m,z_m, m_x,m_y,m_z, 'all');
                    F_pts = cross(currentVec,B,3);
                    F_sum = sum(F_pts);
                    F = [F_sum(:,:,1);F_sum(:,:,2);F_sum(:,:,3)];
                    
%                     y = loop.perpCoord;
%                     for j = 1:loop_pts
%                         
%                         x = r*cos(f_thetas(j));
%                         z = r*sin(f_thetas(j));
%                         currentVec = loop.cur*[-sin(f_thetas(j));0;cos(f_thetas(j))]; %create the current vector
%                         F = F + iCrossB(currentVec,x,y,z,mMag_bod,xMag_bod); %sum up the forces on the current around the loop
%                     end
                    %current in XY plane
                case 'z'
                    
                    x = r*cos(f_thetas);
                    y = r*sin(f_thetas);
                    z = loop.perpCoord*ones(size(f_thetas));
                    currentVec(:,:,1) = loop.cur* -sin(f_thetas);
                    currentVec(:,:,2) = loop.cur* cos(f_thetas);
                    currentVec(:,:,3) = zeros(size(f_thetas));
                    m_x = mMag_bod(1)*ones(size(f_thetas));
                    m_y = mMag_bod(2)*ones(size(f_thetas));
                    m_z = mMag_bod(3)*ones(size(f_thetas));
                    x_m = xMag_bod(1)*ones(size(f_thetas));
                    y_m = xMag_bod(2)*ones(size(f_thetas));
                    z_m = xMag_bod(3)*ones(size(f_thetas));
                    B = vecMagFlux(x,y,z,x_m,y_m,z_m, m_x,m_y,m_z, 'all');
                    F_pts = cross(currentVec,B,3);
                    F_sum = sum(F_pts);
                    F = [F_sum(:,:,1);F_sum(:,:,2);F_sum(:,:,3)];
                    
%                     z = loop.perpCoord;
%                     for j = 1:loop_pts
%                         
%                         x = r*cos(f_thetas(j));
%                         y = r*sin(f_thetas(j));
%                         currentVec = loop.cur*[-sin(f_thetas(j));cos(f_thetas(j));0]; %create the current vector
%                         F = F + iCrossB(currentVec,x,y,z,mMag_bod,xMag_bod); %sum up the forces on the current around the loop
%                     end
            end
            %multiplying by the frequency justified because we're assuming
            %that we're integrating the force across a cycle, so we need to
            %normalize for the fact that higher frequencies mean there are
            %more cycles in a given amount of time than a lower frequency
            %freq_pwr = loop.c1/(mag.freq/loop.c2*(2+sin(loop.phase-mag.phase)));
            
            if mag.freq > 0
                %F = mag.mag*mag.freq^loop.freq_pwr*sin(loop.phase - mag.phase)*F; %scale the forces by the phase difference between the current and the field over the entire cycle
                pwr = loop.c1*mag.freq^loop.c2;
                %e = exp(loop.c3*mag.freq*(sin(loop.phase - mag.phase) + loop.c4));
                F = mag.mag*pwr*sin(loop.phase - mag.phase)*F;
            else
                F = mag.mag*sin(loop.phase - mag.phase)*F;
            end
            %F = sin(loop.phase - mag.phase)*F;
        
    end
    
    end   
end

