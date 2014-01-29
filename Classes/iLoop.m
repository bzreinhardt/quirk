classdef iLoop < hgsetget
    %iLoop keeps track of a current loop in a plate
    
    properties
        plate = {}; %plate the magnet lives on
        mag = {}; %the loops should remember what magnet made it - not strictly necessary but useful for bookeeping
        phase = 0; %characteristic phase of the loop
        radius = 0; %radius of the loop
        perpAxis = 'x'; %body axis perpendicular to the loop
        perpCoord = 0; %location along the perpendicular axis (in body coords)
        cur = 0; %current through the loop
        omega = 0; %characteristic frequency of the loop
        magSide = 0; % a variable +/- 1 depending on whether the magnet is on the positive
        
        
        %these properties actually depend on the properties of the plate
        %and the size of the loop - TODO
        
        Z = 1.44; %impedence of the loop
        L_0 = 6.91083397316373e-05; %inductance of loop
        L = 6.91083397316373e-05;
        R = 0.0787191335859532;%resistance of loop
        alpha = pi/2; %default alpha for permanent magnets
        skinFactor = 1; %scaling factor to take into account skin effect
        delta = 0.01; %characteristic depth of the loop into the plate in m
        SF_shift = 0; %parameter to adjust the position of the loop based on the skin effect - because the loop is 
   % these scaling factors were determined by optimization to the ohji
   % paper
        c1 = 0.823862538604085; %scaling factor on force increase from frequency
        c2 = 0.212537379430452; %power frequency is raised to 
        c3 = 3.87520711150364e-05;%scaling factor on skin effect position adjustment
        c4 = 0.490335912316472; %power on skin effect position adjustment
        c5 = 4.44120647175149e-05; %scaling factor on inductance based on the skin depth
        c6 = 0.474885079774660; %parameter to adjust the effect of the skin effect
        
      
    end
    
    methods
        %% Constructor
        function loop = iLoop(plate, mag, r, axis, coord)
       %inputs:      %plate = plate that the loop lives on 
       %mag = magnet generating the loop
       %r = radius of loop
       %axis - axis perpendicular to loop
       %coord - coordinate along that axis that the loop lives on
       
            loop.mag = mag;
            loop.plate = plate;
            loop.perpAxis = axis;
            loop.perpCoord = coord;
            loop.radius = r;
            
             %set the characteristic frequency of the loop
            if mag.freq > 0
                loop.omega = mag.freq*2*pi;
            elseif norm(mag.om) > 0 %ok, the model can't do spinning, frequency varying EMs right now
                loop.omega = norm(mag.om);
            end
           
            %find the penetration depth of the loop
            if loop.omega > 0
                delt = sqrt(2*plate.rho/(loop.omega*plate.mu));
            elseif loop.omega == 0
                %in the case of a linear moving magnet, set skin depth equal to the minimum dimension of the conductor 
                delt = min([loop.plate.sx,loop.plate.sy,loop.plate.sz]);
            end
            loop.delta = delt;
            
            %find properties of the magnet in body coordinates
            [xMag_bod,vMag_bod,omMag_bod,mMag_bod] = i2b(loop.plate,loop.mag);
            
            % find which side of the plate the magnet is on, make sure the
            % loops don't go out of the plate (this could lead to
            % singularities) and account for skin effect issues
           loop.skinEffects();

           %update inductance based on how deep it penetrates
           loop.L = loop.L_0 + loop.c5*loop.skinFactor;
   
           %set alpha based on the characteristic frequency
           loop.setAlpha();
           %the loop phase is shifted by alpha from the magnet phase
            loop.phase = mag.phase+loop.alpha;
            %find loop from a sinusoidally driven electromagnet
            
            loop.cur = loop.findCurrent();
        end
        
        %% Update the existing loop
        function loopUpdate(loop)
        %update the properties of the loop based on its generating magnet
            
            omega = loop.mag.freq*2*pi;
            if norm(loop.mag.om) > 0 %ok, the model can't do spinning, frequency varying EMs right now
                omega = norm(loop.mag.om);
            end
            if omega > 0
                loop.delta = sqrt(2*loop.plate.rho/(omega*loop.plate.mu));
            end
            
            [xMag_bod,~,omMag_bod, mMag_bod] = i2b(loop.plate,loop.mag);
            
            
            %find the closest face to the magnet and skin effects
            
            loop.skinEffects();
              %update inductance based on how deep it penetrates
            loop.L = loop.L_0 + loop.c5*loop.skinFactor;
            %update alpha based on characteristic frequency
            loop.setAlpha();
            %update the phase based on the alpha
            loop.phase = loop.mag.phase+loop.alpha;

            loop.cur = loop.findCurrent();
            
        end
        %%
        function E = findEMF(obj)
            [xMag_bod,vMag_bod,omMag_bod, mMag_bod] = i2b(obj.plate,obj.mag);
            if obj.omega ~= 0
                int = @(theta,rad)vec_dBdt_int( theta, rad,obj.perpCoord, obj.mag.m, mMag_bod, xMag_bod,obj.perpAxis);
                E = -1*integral2 (int,0,2*pi,0,obj.radius);
            elseif obj.omega == 0
                int = @(theta) vXB_int(theta, obj.radius, obj.perpCoord, mMag_bod, xMag_bod, omMag_bod,vMag_bod,obj.perpAxis);
                E = integral(int, 0, 2*pi);
            else
                error('magnet frequency not a number!');
            end
            
        end
        
        function I = findCurrent(obj)
           %finds 
            
            E = findEMF(obj);
            I = (obj.skinFactor)^(obj.c6)*obj.plate.sigma*E;
        end

        function loopCur_PM(obj)
            %finds the current due to a moving magnetic field
            [xMag_bod,vMag_bod,omMag_bod, mMag_bod] = i2b(obj.plate,obj.mag);
            int = @(theta) vXB_int(theta, obj.radius, obj.perpCoord, mMag_bod, xMag_bod, omMag_bod,vMag_bod,obj.perpAxis);
            obj.cur = -1*(obj.skinFactor)^(obj.c6)*obj.plate.sigma*integral(int, 0, 2*pi);
        end
        
        function I = loopCur_EM(obj)
            %finds the current component from a time-varying magnetic field
            [xMag_bod,vMag_bod,omMag_bod, mMag_bod] = i2b(obj.plate,obj.mag);
            int = @(theta,rad)vec_dBdt_int( theta, rad,loop.perpCoord, mag_dBdt, mMag_bod, xMag_bod,loop.perpAxis);
            obj.cur = -1*loop.skinFactor^(loop.c6)*omega/loop.Z*integral2 (int,0,2*pi,0,loop.radius);
        end
        
        function face = closestFace(obj)
            %find the closest face of the loop's plate to the loop's magnet
            [xMag_bod,vMag_bod,omMag_bod, mMag_bod] = i2b(obj.plate,obj.mag);
            faces = [obj.plate.sx/2,0,0; -obj.plate.sx/2,0,0;...
                0,obj.plate.sy/2,0; 0,-obj.plate.sy/2,0; ...
                0,0,obj.plate.sz/2; 0,0,-obj.plate.sz/2];
            x_dist = min(pdist([faces(1,:);xMag_bod']),pdist([faces(2,:);xMag_bod']));
            y_dist = min(pdist([faces(3,:);xMag_bod']),pdist([faces(4,:);xMag_bod']));
            z_dist = min(pdist([faces(5,:);xMag_bod']),pdist([faces(6,:);xMag_bod']));
            
            if x_dist <= y_dist && x_dist <= z_dist
                face = 'x';
            elseif y_dist < x_dist && y_dist <= z_dist
                face = 'y';
            elseif z_dist < y_dist && z_dist < x_dist
                face = 'z';
            end
        end
        
        function skinEffects(obj)
            %find the effects of the skin effect on loop parameters
                %shift the loop towards the magnet based on frequency
        %set the skinFactor which will impact how 'strong' the current
        %is

            closeAxis = obj.closestFace();
            [xMag_bod,~,~,~] = i2b(obj.plate,obj.mag);
            switch obj.perpAxis
                case 'x'
                    magAxisCoord = xMag_bod(1);
                    onAxisPltSize = obj.plate.sx;
                    offAxisPltSize1 = obj.plate.sy;
                    offAxisPltSize2 = obj.plate.sz;
                case 'y'
                    magAxisCoord = xMag_bod(2);
                    onAxisPltSize = obj.plate.sy;
                    offAxisPltSize1 = obj.plate.sz;
                    offAxisPltSize2 = obj.plate.sx;
                case 'z'
                    magAxisCoord = xMag_bod(3);
                    onAxisPltSize = obj.plate.sz;
                    offAxisPltSize1 = obj.plate.sx;
                    offAxisPltSize2 = obj.plate.sy;
            end
            %find the side 
            if strcmp(axis,closeAxis) == 1
                if magAxisCoord > 0
                    obj.magSide = 1;
                else
                    obj.magSide = -1;
                end
                obj.perpCoord = obj.magSide*obj.c3*obj.omega^obj.c4*onAxisPltSize/2;
            end
            
            if obj.radius > offAxisPltSize1/2 || obj.radius > offAxisPltSize2/2
                warning('loop radius extends out of plate. changed to edge of plate.');
                obj.radius = min([offAxisPltSize1, offAxisPltSize2]);
            end
            
            if abs(obj.perpCoord) > onAxisPltSize/2
                warning('loop coordinate extends out of plate. changed to edge of plate.');
                obj.perpCoord = obj.magSide*onAxisPltSize/2;
            end
            
            SF = (1-exp(-1*onAxisPltSize/obj.delta));
            obj.skinFactor = SF;
        end
        
        function setAlpha(obj)
            %update the alpha value for the loop based on the
            %characteristic frequency
            if obj.omega > 0 
                obj.alpha = atan2(obj.omega*obj.L,obj.R);
                obj.Z = (obj.R^2+obj.omega^2*obj.L^2)^.5;
                
            else
                obj.alpha = pi/2; %alpha has no meaning if permanent magnet and thus no frequency
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
                F = mag.m*pwr*sin(loop.phase - mag.phase)*F;
            else
                F = mag.m*sin(loop.phase - mag.phase)*F;
            end
            %F = sin(loop.phase - mag.phase)*F;
        
    end
    
    end   
end

