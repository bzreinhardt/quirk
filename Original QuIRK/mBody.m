classdef mBody < hgsetget
    
    properties
        force;
        torque;
        U;
        
        thresh = 1e-2;
        damping = 0;
        data
    end
    
    properties (SetAccess = 'private')
        S = [];
        Sf = [];
        solved = false;
        
        bodies = {};
        joints = {};
        forces = {};
        sensors = {};
        
        bodyNames = {};
        jointNames = {};
        sensorNames = {};
        forceNames = {};
        
        numBodies = 0;
        numJoints = 0;
        numStates = 0;
        numConstraints = 0;
        numSensors = 0;
        numForces = 0;
        
        x = [];
        t = 0;
        
        currentE = 0;
    end
    
    properties (SetAccess = 'private', GetAccess = 'private')
        stopNextIteration = Inf;
    end %properties
    
    methods
        
        %% ADD
        function mbod = add(mbod, varargin)
            % ADD puts more bodies, joints, etc into the multibody system.
            % If the system has previously been solved, add assumes that
            % the new objects should be added at the most recent time in
            % the system state history. Use SETNOW or RESET before adding
            % if this is not the case. Syntax:
            %
            %   mbody = add(mbody, body, joint, force, ...)
            %   mbody = mbody.add(body, sensor, joint, ...)
            %
            % You can list any number of body, sensor, force, and joint
            % objects, in any order. If you add joints, forces, or sensors
            % that refer to the bodies you add, then the bodies should be
            % listed first.
            %
            % See also REMOVE, SETTIME, SETNOW, RESET.
            
            if isempty(varargin)
                return
            end
            
            warning('off', 'BODY:setParent:dontDoThat');
            
            % Check to see if there's a state history, and set to end time
            if numel(mbod.t) > 1
                mbod.setnow;
            end
            
            % Go through arguments and add to the mBody
            for i = 1:numel(varargin)
                switch class(varargin{i})
                    case 'body'
                        varargin{i}.clearParent;
                        varargin{i}.setParent(mbod);
                        mbod.bodies = [mbod.bodies {varargin{i}}];
                        mbod.numBodies = mbod.numBodies + 1;
                        mbod.numStates = mbod.numStates + 14;
                        name = inputname(i+1);
                        if isempty(name)
                            name = 'defaultBodyName';
                        end
                        mbod.bodyNames = [mbod.bodyNames {name}];
             %added cases for magnet and plate so mbody will treat them
             %like bodies
                    case 'magnet'
                        varargin{i}.clearParent;
                        varargin{i}.setParent(mbod);
                        mbod.bodies = [mbod.bodies {varargin{i}}];
                        mbod.numBodies = mbod.numBodies + 1;
                        mbod.numStates = mbod.numStates + 14;
                        name = inputname(i+1);
                        if isempty(name)
                            name = 'defaultBodyName';
                        end
                        mbod.bodyNames = [mbod.bodyNames {name}];
                        
                    case 'plate'
                        varargin{i}.clearParent;
                        varargin{i}.setParent(mbod);
                        mbod.bodies = [mbod.bodies {varargin{i}}];
                        mbod.numBodies = mbod.numBodies + 1;
                        mbod.numStates = mbod.numStates + 14;
                        name = inputname(i+1);
                        if isempty(name)
                            name = 'defaultBodyName';
                        end
                        mbod.bodyNames = [mbod.bodyNames {name}];
                        
                    case 'joint'
                        mbod.joints = [mbod.joints {varargin{i}}];
                        mbod.numJoints = mbod.numJoints + 1;
                        mbod.numConstraints = mbod.numConstraints ...
                            + varargin{i}.numConstraints;
                        name = inputname(i+1);
                        if isempty(name)
                            name = 'defaultJointName';
                        end
                        mbod.jointNames = [mbod.jointNames {name}];
                        
                        for j = 1:mbod.numBodies
                            if mbod.bodies{j} == mbod.joints{end}.bodies{1}
                                mbod.S(j, mbod.numJoints) = 1;
                            elseif mbod.bodies{j} == mbod.joints{end}.bodies{2}
                                mbod.S(j, mbod.numJoints) = -1;
                            else
                                mbod.S(j, mbod.numJoints) = 0;
                            end
                        end
                    case 'force'
                        mbod.forces = [mbod.forces {varargin{i}}];
                        mbod.numForces = mbod.numForces + 1;
                        name = inputname(i+1);
                        if isempty(name)
                            name = 'defaultForceName';
                        end
                        mbod.forceNames = [mbod.forceNames {name}];
                        
                        for j = 1:mbod.numBodies
                            if mbod.bodies{j} == mbod.forces{end}.body1
                                mbod.Sf(j, mbod.numForces) = 1;
                            elseif mbod.bodies{j} == mbod.forces{end}.body2
                                mbod.Sf(j, mbod.numForces) = -1;
                            else
                                mbod.Sf(j, mbod.numForces) = 0;
                            end
                        end
                    case 'sensor'
                        mbod.sensors = [mbod.sensors {varargin{i}}];
                        mbod.numSensors = mbod.numSensors + 1;
                        name = inputname(i+1);
                        if isempty(name)
                            name = 'defaultSensorName';
                        end
                        mbod.sensorNames = [mbod.sensorNames {name}];
                end
            end
            
            warning('off', 'MBODY:unSolve:dontDoThat');
            mbod.setnow;
            warning('on', 'MBODY:unSolve:dontDoThat');
            
            warning('on', 'BODY:setParent:dontDoThat');
        end
        
        %% ANIMATE
        function animate(mbod, varargin)
            % ANIMATE animates the motion of a solved multibody system
            % in the current axes. Syntax:
            %
            %   animate(mBody, 'property', value, ...)
            %   mBody.animate('property', value, ...)
            %
            % Required arguments
            %
            %   mBody:   multibody system to animate
            %
            % Optional properties
            %
            %   tspan   int     Time vector at which to start and stop the
            %                       animation: [start stop]
            %   speed   dbl     Playback speed of the animation, with 1
            %                       being real-time (default). Zero, Inf,
            %                       or NaN will result in the animation
            %                       playing at maximum speed (slower where
            %                       there are more integration points, etc)
            %   color   dbl/str RGB or Matlab string specifying color of
            %                       bodies (overrides body colors)
            %   alpha   dbl     Opacity of bodies, range 0-1
            %   axis    dbl/str Any input you would typically give to the
            %                       MATLAB axis command, applied to every
            %                       frame of the animation. E.g. 'normal',
            %                       'equal', or a 6-element axis limit
            %                       vector. Default is 'equal'.
            %   lit     T/F     Light bodies if true
            %   hold    T/F     Draw on top of current graphics if true
            %   bg      dbl/str Override background color of figure and
            %                       axes
            %   filename        If present, save movie with indicated file
            %                       name in working directory. Extention
            %                       defaults to avi (don't include '.avi'
            %                       in filename).
            %
            %   Any surfaceplot property will also work.
            %
            % See also DRAW, GHOST, TRACE, ERASE, BODY\DRAW.
            
            tspan = [0 mbod.t(end)];
            lit = false;
            bg = [];
            ax = 'equal';
            filename = [];
            speed = 1;
            
            if ~mbod.solved
                warning('MBODY:animate:unSolved', 'Call SOLVE to find the equations of motion before calling ANIMATE.')
                return
            end
            
            %% --Parse inputs
            if numel(varargin)/2 ~= round(numel(varargin)/2)
                error('MBODY:animate:input', 'All properties must be paired with values.')
            end
            plotargs = true(1, length(varargin));
            for i = 1:2:(length(varargin))
                if ~ischar(varargin{i})
                    error('MBODY:animate:input', 'Property designations must be strings.')
                elseif strncmpi(varargin{i}, 'ts', 2)
                    if ~isnumeric(varargin{i+1})
                        error('MBODY:animate:input', 'Tspan must be numeric.')
                    elseif varargin{i+1} ~= round(varargin{i+1})
                        error('MBODY:animate:input', 'Tspan must be integer indices into time array.')
                    elseif varargin{i+1} <= 0
                        error('MBODY:animate:input', 'Tspan indices must be >= 1.')
                    elseif numel(varargin{i+1}) ~= 2
                        error('MBODY:animate:input', 'Tspan vector must have 2 elements.')
                    else
                        tspan = varargin{i+1};
                        plotargs(i:(i+1)) = [false false];
                    end
                elseif strncmpi(varargin{i}, 'fi', 2)
                    if ~ischar(varargin{i+1})
                        error('MBODY:animate:input', 'Filename must be a string.')
                    else
                        filename = varargin{i+1};
                        plotargs(i:(i+1)) = [false false];
                    end
                elseif strncmpi(varargin{i}, 'li', 2)
                    lit = varargin{i+1}(1);
                    plotargs(i:(i+1)) = [false false];
                elseif strncmpi(varargin{i}, 'bg', 2)
                    bg = varargin{i+1}(1);
                    plotargs(i:(i+1)) = [false false];
                elseif strncmpi(varargin{i}, 'ax', 2)
                    ax = varargin{i+1};
                    plotargs(i:(i+1)) = [false false];
                elseif strncmpi(varargin{i}, 'sp', 2)
                    speed = varargin{i+1};
                    if isnan(speed) || isinf(speed) || speed < 0
                        speed = 0;
                    end
                    plotargs(i:(i+1)) = [false false];
                end
            end
            
            %% --Set initial body states
            idx = 1:length(mbod.t);
            start = idx(mbod.t >= tspan(1));
            start = start(1);
            stop = idx(mbod.t <= tspan(2));
            stop = stop(end);
            
            if numel(mbod.x) ~= mbod.numStates
                time = mbod.t(start);
                mbod.settime(start);
                title(['t = ' num2str(time, '%05.2f')])
            end
            
            %% --Draw bodies, joints, sensors
            for i = 1:mbod.numBodies
                draw(mbod.bodies{i}, varargin{plotargs});
                if i == 1
                    hold on
                end
            end
            hold off
            if lit
                light;
            end
            if ~isempty(bg)
                set(gcf, 'color', bg)
                set(gca, 'color', bg)
            end
            
            %% --Set up movie, if necessary
            if ~isempty(filename)
                if speed ~= 0
                    FPS = speed * numel(mbod.t(start:stop))/mbod.t(stop);
                else
                    FPS = 15;
                end
                ftime=datestr(now);
                fsp=find(ftime==' ');
                ftime=[ftime(1:(fsp-1)) '_' ftime((fsp+1):end)];
                fco=find(ftime==':');
                filename=[filename '_' ftime(1:(fco(1)-1)) '-' ftime((fco(1)+1):(fco(2)-1)) '-' ftime((fco(2)+1):end) '.avi'];
                mov = avifile(filename,'fps',FPS,'Compression','Cinepak','Quality',100);
                
                F = getframe(gcf);
                mov = addframe(mov,F);
            end
            
            %% --Update body states
            if numel(mbod.x) ~= mbod.numStates
                for time = start+1:stop
                    
                    mbod.settime(time);
                    
                    if ~(ischar(ax) && ax(1) == 'e')
                        axis(ax);
                    end
                    
                    for j = 1:mbod.numJoints
                        erase(mbod.joints{j});
                        draw(mbod.joints{j});
                    end
                    c = get(gca, 'colororder');
                    for k = 1:mbod.numSensors
                        erase(mbod.sensors{k});
                        draw(mbod.sensors{k}, ...
                            'color', c(mod(k-1,size(c,1))+1,:));
                    end
                    title(['t = ' num2str(mbod.t(time))])
                    
                    if speed ~= 0
                        pause((mbod.t(time) - mbod.t(time-1))/speed);
                    else
                        drawnow;
                    end
                    
                    if ~isempty(filename)
                        F = getframe(gcf);
                        mov = addframe(mov,F);
                    end
                end
            end
            
            if ~isempty(filename)
                mov = close(mov);
            end
        end
        
        %% CHECKCOLLISIONS
        function collided = checkCollisions(mbod)
            % CHECKCOLLISIONS performs a weak test for collisions on an 
            % mBody system. It tests the distance between each pair of 
            % bodies against the sum of the minimum sizes (sx, sy, and sz) 
            % of those two bodies. Syntax:
            %
            %   collided = checkCollisions(mBody)
            %   collided = mBody.checkCollisions
            
            collided = false;
            
            for i = 1:mbod.numBodies
                for j = 1:mbod.numBodies
                    if i ~= j
                        ri = min([mbod.bodies{i}.sx mbod.bodies{i}.sy mbod.bodies{i}.sz]);
                        rj = min([mbod.bodies{j}.sx mbod.bodies{j}.sy mbod.bodies{j}.sz]);
                        
                        if norm(mbod.bodies{i}.pos - mbod.bodies{j}.pos) < 0.485*(ri+rj)
                            collided = true;
                            return
                        end
                    end
                end
            end
        end
        
        %% CHECKORPHANS
        function checkOrphans(mbod)
            % CHECKORPHANS sees if there are any sensors or joints with a
            % body that is not a member of this multibody system, and
            % prints the names of those objects if they exist. Syntax:
            %
            %   checkOrphans(mbody)
            %   mbody.checkOrphans
            %
            % See also REMOVE, ADD.
            
            orph = sum(mbod.S) ~= 0;
            for i = 1:mbod.numJoints
                if orph(i) > 0 && strcmp(mbod.joints{i}.type, 'gnd')
                    orph(i) = 0;
                elseif orph(i) == 0 && strcmp(mbod.joints{i}.type, 'gnd')
                    orph(i) = 1;
                end
            end
            if any(orph)
                disp('The following joints refer to bodies that are not a member of this system:')
                disp(mbod.jointNames(orph))
            end
            
            orph = sum(mbod.Sf) ~= 0;
            if any(orph)
                disp('The following forces refer to bodies that are not a member of this system:')
                disp(mbod.forceNames(orph))
            end
            
            orph = false(1,mbod.numSensors);
            for i = 1:mbod.numSensors
                for j = 1:numel(mbod.sensors{i}.master)
                    orph(i) = ~any(logical(...
                        cellfun(@(b)(b == mbod.sensors{i}.master(j).body), ...
                        mbod.bodies) ));
                end
                for j = 1:numel(mbod.sensors{i}.slave)
                    orph(i) = orph(i) || ~any(logical(...
                        cellfun(@(b)(b == mbod.sensors{i}.slave(j).body), ...
                        mbod.bodies) ));
                end
            end
            if any(orph)
                disp('The following sensors refer to bodies that are not a member of this system:')
                disp(mbod.sensorNames(orph))
            end
        end
        
        %% CM
        function cm = cm(mbod)
            % CM returns the center-of-mass position of the mBody.
            % Syntax:
            %
            %   cmPos = mBody.cm
            %
            
            cm = [0 0 0]';
            m = 0;
            for i = 1:mbod.numBodies
                cm = cm + mbod.bodies{i}.mass * mbod.bodies{i}.pos;
                m = m + mbod.bodies{i}.mass;
            end
            cm = cm/m;
        end
        
        %% CONSTRAINTS
        function [bigA bigB] = constraints(mbod)
            % CONSTRAINTS is not for human consumption. But it will return
            % the [A b] matrix and vector for U-K equations of motion.
            %
            % See SOLVE.
            
            %% --Set up
            bigA = cell(mbod.numJoints + mbod.numBodies, mbod.numBodies);
            bigB = cell(mbod.numJoints + mbod.numBodies, 1);
            
            %% --Calculate big ol' constraint matrices
            % Query each joint for constraint matrices
            for i = 1:mbod.numJoints
                tempA = mbod.joints{i}.A;
                bigA{i, mbod.S(:,i) == 1} = tempA(:,1:7);
                bigB{i,1} = mbod.joints{i}.b;
                if ~strcmp( mbod.joints{i}.type, 'gnd' )
                    bigA{i, mbod.S(:,i) == -1} = tempA(:,8:end);
                end
                for j = 1:mbod.numBodies
                    if isempty(bigA{i,j})
                        bigA{i,j} = zeros(size(tempA(:,1:7)));
                    end
                end
            end
            
            % Add constraints that q'q = 1 for each body
            for i = (mbod.numJoints + 1):(mbod.numJoints + mbod.numBodies)
                k = i - mbod.numJoints;
                q = mbod.bodies{k}.q;
                qdot = mbod.bodies{k}.qdot;
                
                bigA{i, k} = ...
                    [0 0 0 2*q'];
                bigB{i, 1} = -2*(qdot'*qdot);
                for j = 1:mbod.numBodies
                    if isempty(bigA{i,j})
                        bigA{i,j} = zeros(1,7);
                    end
                end
            end
            
            bigA = cell2mat(bigA);
            bigB = cell2mat(bigB);
        end
        
        %% CONSTRUCTOR
        function mbod = mBody(varargin)
            % MBODY creates a multibody object for simulations. Syntax:
            %
            %   mbod = mBody(bodies, ..., joints, ..., sensors, ...,
            %                   forces, ..., 'property', value, ...)
            %
            % List bodies, joints, forces, and sensors, in any order, 
            % followed by property/value pairs.
            %
            % Optional properties
            %
            %   damping dbl     Damping coefficient to apply to all bodies.
            %   force   @fn     Force on ALL body centers of mass as a
            %                       function of position, attitude,
            %                       velocity, angular velocity, and time.
            %                       Adds to individual body forces. E.g.,
            %                       'force', @(x,q,v,w,t) ( -4*x - 2.5*v )
            %                       is a simple spring-damper. Force
            %                       matrices should be in INERTIAL frame.
            %   torque  @fn     Torque about ALL body centers of mass as a
            %                       function of position, attitude,
            %                       velocity, angular velocity, and time.
            %                       Adds to individual body forces. Torque
            %                       matrices should be in BODY frames.
            %   U       @fn     Potential energy function applied to the
            %                       entire system, which then experiences a
            %                       force -grad(U), as a function of either
            %                       (position, time) or a body object. Use
            %                       the body-argument form for potential
            %                       energies that depend on attitude, mass,
            %                       or other properties.
            %   thresh  dbl     Threshhold on equilibrium detection in
            %                       integration (defaults to 1e-2).
            %   data    any     User-defined data field
            %
            % See also BODY, JOINT, FORCE, SENSOR.
            
            %% --Set defaults
            warning('off', 'MBODY:unSolve:status');
            warning('off', 'BODY:setParent:dontDoThat');
            
            % Force/torque defaults
            mbod.force = @(pos,q,vel,om,time) ([0; 0; 0]);
            mbod.torque = @(pos,q,vel,om,time) ([0; 0; 0]);
            mbod.U = @(x,t) (0);
            
            
            %% --Parse inputs
            bodyList = false(size(varargin));
            jointList = bodyList;
            sensorList = bodyList;
            forceList = bodyList;
            if nargin == 1 && isa(varargin{1}, 'mBody')  % --If given just a mBody, return that body
                mbod = varargin{1};
            elseif nargin > 0   % --If given arguments, construct the specified body
                i = 1;
                while i <= numel(varargin) && ~ischar(varargin{i})
                    switch class(varargin{i})
                        case 'body'
                            bodyList(i) = true;
                            name = inputname(i);
                            if isempty(name)
                                name = 'defaultBodyName';
                            end
                            mbod.bodyNames = [mbod.bodyNames {name}];
                            %BENCHANGE 9/10 added cases for magnet and
                            %plate
                        case 'magnet'
                            bodyList(i) = true;
                            name = inputname(i);
                            if isempty(name)
                                name = 'defaultBodyName';
                            end
                            mbod.bodyNames = [mbod.bodyNames {name}];
                        
                        case 'plate'
                            bodyList(i) = true;
                            name = inputname(i);
                            if isempty(name)
                                name = 'defaultBodyName';
                            end
                            mbod.bodyNames = [mbod.bodyNames {name}];
                        case 'joint'
                            jointList(i) = true;
                            name = inputname(i);
                            if isempty(name)
                                name = 'defaultJointName';
                            end
                            mbod.jointNames = [mbod.jointNames {name}];
                        case 'sensor'
                            sensorList(i) = true;
                            name = inputname(i);
                            if isempty(name)
                                name = 'defaultSensorName';
                            end
                            mbod.sensorNames = [mbod.sensorNames {name}];
                        case 'force'
                            forceList(i) = true;
                            name = inputname(i);
                            if isempty(name)
                                name = 'defaultForceName';
                            end
                            mbod.forceNames = [mbod.forceNames {name}];
                        otherwise
                            error('MBODY:mbody:input', 'List only bodies, joints, forces, and sensors before property/value pairs.')
                    end
                    i = i + 1;
                end
                
                % Options
                if numel(varargin(i:end))/2 ~= round(numel(varargin(i:end))/2)
                    error('MBODY:mbody:input', 'All properties must be paired with values.')
                end
                j = i;
                for i = j:2:length(varargin)
                    if ~ischar(varargin{i})
                        error('BODY:body:input', 'Property designations must be strings.')
                    end
                    switch lower(varargin{i})
                        case {'force' 'f'}
                            if ~isa(varargin{i+1}, 'function_handle')
                                error('MBODY:mbody:input', 'Force must be a function handle.')
                            elseif nargin(varargin{i+1}) ~= 5 && nargin(varargin{i+1}) ~= 1
                                error('MBODY:mbody:input', 'Force must be a function of (position, attitude, vel, ang vel, time).')
                            else
                                mbod.force = varargin{i+1};
                            end
                        case {'torque' 'tor' 'tq'}
                            if ~isa(varargin{i+1}, 'function_handle')
                                error('MBODY:mbody:input', 'Torque must be a function handle.')
                            elseif nargin(varargin{i+1}) ~= 5 && nargin(varargin{i+1}) ~= 1
                                error('MBODY:mbody:input', 'Torque must be a function of (position, attitude, vel, ang vel, time) or a body object.')
                            else
                                mbod.torque = varargin{i+1};
                            end
                        case {'potential' 'pot' 'u'}
                            if ~isa(varargin{i+1}, 'function_handle')
                                error('MBODY:mbody:input', 'Potential energy must be a function handle.')
                            elseif (nargin(varargin{i+1}) == 2 && ...
                                    all(size(varargin{i+1}(zeros(3,1),0)) ~= [1 1])) ...
                                    || (nargin(varargin{i+1}) == 1 && ...
                                    all(size(varargin{i+1}(body)) ~= [1 1]))
                                error('MBODY:mbody:input', 'Potential energy function must return a scalar.')
                            else
                                mbod.U = varargin{i+1};
                            end
                        case {'eqthresh' 'forcethresh' 'minforce' 'minf' 'fthresh' 'thresh' 'threshhold'}
                            if ~isnumeric(varargin{i+1})
                                error('MBODY:mbody:input', 'Threshhold must be numeric.')
                            end
                            mbod.thresh = varargin{i+1}(1);
                        case {'damping', 'damp', 'd', 'c'}
                            if ~isnumeric(varargin{i+1})
                                error('MBODY:mbody:input', 'Damping must be numeric.')
                            end
                            mbod.damping = varargin{i+1}(1);
                        case {'data' 'dat'}
                            mbod.data = varargin{i+1};
                        otherwise
                            warning('MBODY:mbody:input', ...
                                'Discarding unknown property "%s"', varargin{i})
                    end
                end
                
            else    % --Construct object with default properties
                
            end
            
            %% --Add bodies, joints, etc to system
            % Adding them all AFTER they have been extracted from the arg
            % list is important because joints/forces/sensors depend on the
            % bodies being added first
            bodyList = varargin(bodyList);
            for i = 1:length(bodyList)
                bodyList{i}.setParent(mbod);
                mbod.bodies = [mbod.bodies {bodyList{i}}];
                mbod.numBodies = mbod.numBodies + 1;
                mbod.numStates = mbod.numStates + 14;
            end
            
            jointList = varargin(jointList);
            for i = 1:length(jointList)
                mbod.joints = [mbod.joints {jointList{i}}];
                mbod.numJoints = mbod.numJoints + 1;
                mbod.numConstraints = mbod.numConstraints ...
                    + jointList{i}.numConstraints;
                
                for j = 1:mbod.numBodies
                    if mbod.bodies{j} == mbod.joints{end}.bodies{1}
                        mbod.S(j, mbod.numJoints) = 1;
                    elseif mbod.bodies{j} == mbod.joints{end}.bodies{2}
                        mbod.S(j, mbod.numJoints) = -1;
                    else
                        mbod.S(j, mbod.numJoints) = 0;
                    end
                end
            end
            
            sensorList = varargin(sensorList);
            for i = 1:length(sensorList)
                mbod.sensors = [mbod.sensors {sensorList{i}}];
                mbod.numSensors = mbod.numSensors + 1;
            end
            
            forceList = varargin(forceList);
            for i = 1:length(forceList)
                mbod.forces = [mbod.forces {forceList{i}}];
                mbod.numForces = mbod.numForces + 1;
                
                for j = 1:mbod.numBodies
                    if mbod.bodies{j} == mbod.forces{end}.body1
                        mbod.Sf(j, mbod.numForces) = 1;
                    elseif mbod.bodies{j} == mbod.forces{end}.body2
                        mbod.Sf(j, mbod.numForces) = -1;
                    else
                        mbod.Sf(j, mbod.numForces) = 0;
                    end
                end
            end
            %% --Set up initial state
            mbod.x = zeros(mbod.numStates, 1);
            idx = 1;
            for i = 1:mbod.numBodies
                mbod.x(idx:(idx+2)) = mbod.bodies{i}.pos;
                mbod.x((mbod.numStates/2+idx):(mbod.numStates/2+idx+2)) ...
                    = mbod.bodies{i}.vel;
                idx = idx + 3;
                mbod.x(idx:(idx+3)) = mbod.bodies{i}.att;
                mbod.x((mbod.numStates/2+idx):(mbod.numStates/2+idx+3)) ...
                    = mbod.bodies{i}.qdot;
                idx = idx + 4;
            end
            
            warning('on', 'BODY:setParent:dontDoThat');
            warning('on', 'MBODY:unSolve:status');
            
        end
        
        %% DELETE
        function delete(mbod)
            % DELETE performs clean-up actions before clearing an mBody
            % from memory.
            
            warning('off', 'BODY:setParent:dontDoThat');
            
            for i = 1:mbod.numBodies
                try
                    mbod.bodies{i}.clearParent;
                catch
                    % Just ignore errors
                end
            end
            
            warning('on', 'BODY:setParent:dontDoThat');
        end
        
        %% DISPLAY
        function display(mbod)
            % DISPLAY handles the printout of an mBody object when someone
            % types its name without ending the line in a semicolon.
            disp([inputname(1) ' = '])
            disp( '     _____             QuIRK: mBody object')
            disp(['    |\    \   /\\\\\\   ' num2str(mbod.numBodies) ' bodies (' num2str(mbod.numStates) ' states)'])
            disp(['    | \____\o/  \\\\\\  ' num2str(mbod.numJoints) ' joints (' num2str(mbod.numConstraints) ' constraints)'])
            disp(['    \ |    | \  /####/  ' num2str(mbod.numSensors) ' sensors'])
            if mbod.solved
                disp(['     \|____|  \/####/   Equation of motion solved for t = ' num2str(mbod.t(1)) ':' num2str(mbod.t(end)) '.'])
            else
                disp( '     \|____|  \/####/   Equation of motion not yet solved.')
            end
            disp(' ');
        end
        
        %% DRAW
        function h = draw(mbod, varargin)
            % DRAW adds a graphical representation of a multibody system to
            % the current axes. The graphics will update as this mBody's
            % properties change. Syntax:
            %
            %   h = draw(mBody, 'property', value, ...)
            %   h = mBody.draw('property', value, ...)
            %
            % Required arguments
            %
            %   mBody:   multibody system to draw
            %
            % Optional properties
            %
            %   time    int     Index in time vector at which to draw
            %                       the system. If not specified, draws
            %                       each body at last specified time.
            %   state   dbl     State vector at which to draw the mBody.
            %                       This should be a vector with either
            %                       7*numBodies or 14*numBodies elements.
            %                       (Velocity states will be ignored.)
            %   color   dbl/str RGB or Matlab string specifying color of
            %                       bodies.
            %   alpha   dbl     Opacity of bodies, range 0-1.
            %   hold    T/F     Draw on top of current graphics if true.
            %   lit     T/F     Light the mBody graphic.
            %   snap    T/F     Graphic will not update as mBody changes if
            %                       this flag is true. Good for making many
            %                       plots at intermediate steps.
            %   bodies  T/F     If false, don't draw bodies.
            %   joints  T/F     If false, don't draw joints.
            %   sensors T/F     If false, don't draw sensors.
            %   filename        If present, uses print2im to save image 
            %                       with indicated file name in working 
            %                       directory. If no extension, defaults to
            %                       png. Requires your favorite function 
			%                       to turn a figure window into an image 
			%                       be named print2im (e.g. Matlab file 
			%                       exchange #22093).
            %
            %   Any surface property will also work.
            %
            % See also GHOST, ERASE, ANIMATE, TRACE, BODY\DRAW.
            
            time = numel(mbod.t);
            filename = [];
            lit = false;
            state = [];
            show = [];
            drawBodies = true;
            drawJoints = true;
            drawSensors = true;
            
            %% --Parse inputs
            if numel(varargin)/2 ~= round(numel(varargin)/2)
                error('MBODY:draw:input', 'All properties must be paired with values.')
            end
            plotargs = true(size(varargin));
            for i = 1:2:(length(varargin))
                if ~ischar(varargin{i})
                    error('MBODY:draw:input', 'Property designations must be strings.')
                end
                switch lower(varargin{i})
                    case 'time'
                        if ~isnumeric(varargin{i+1})
                            error('MBODY:draw:input', 'Time must be numeric.')
                        elseif varargin{i+1} ~= round(varargin{i+1})
                            error('MBODY:draw:input', 'Time must be an integer index into time array.')
                        elseif varargin{i+1} <= 0
                            error('MBODY:draw:input', 'Time index must be >= 1.')
                        else
                            time = varargin{i+1}(1);
                            plotargs(i:(i+1)) = [false false];
                        end
                    case 'filename'
                        if ~ischar(varargin{i+1})
                            error('MBODY:draw:input', 'Filename must be a string.')
                        else
                            filename = varargin{i+1}(1);
                            plotargs(i:(i+1)) = [false false];
                        end
                    case 'lit'
                        lit = varargin{i+1}(1);
                        plotargs(i:(i+1)) = [false false];
                    case {'bodies' 'bod' 'b'}
                        drawBodies = varargin{i+1}(1);
                        plotargs(i:(i+1)) = [false false];
                    case {'joints' 'jnt' 'j'}
                        drawJoints = varargin{i+1}(1);
                        plotargs(i:(i+1)) = [false false];
                    case {'sensors' 'sen'}
                        drawSensors = varargin{i+1}(1);
                        plotargs(i:(i+1)) = [false false];
                    case 'state'
                        if ~isnumeric(varargin{i+1})
                            error('MBODY:draw:input', 'State must be numeric.')
                        elseif numel(varargin{i+1}) ~= mbod.numStates ...
                                && numel(varargin{i+1}) ~= mbod.numStates/2
                            error('MBODY:draw:input', ['State vector must have ' ...
                                num2str(mbod.numStates/2) ' or ' num2str(mbod.numStates)' elements.'])
                        end
                        state = varargin{i+1};
                        plotargs(i:(i+1)) = [false false];
                    %{
                    case 'show'
                        switch lower(varargin{i+1})
                            case {'velocity' 'vel' 'v'}
                                show(1) = 'v';
                            case {'angvelocity' 'omega' 'om' 'o' 'w'}
                                show(2) = 'o';
                            case {'momentum' 'mom' 'p'}
                                show(3) = 'p';                                
                            case {'angmomentum' 'angmom' 'h' 'l'}
                                show(4) = 'h';
                            case {'force' 'for' 'fc' 'f'}
                                show(5) = 'f';
                            case {'torque' 'tor' 'tq' 't' 'tau'}
                                show(6) = 't';
                        end
                        plotargs(i:(i+1)) = [false false];
                	%}
                end
            end
            
            %% --Set body states
            if ~isempty(state)
                idx = 1;
                for i = 1:mbod.numBodies
                    mbod.bodies{i}.pos = state(idx:(idx+2));
                    idx = idx + 3;
                    mbod.bodies{i}.att = state(idx:(idx+3));
                    idx = idx + 4;
                end
            elseif numel(mbod.x) ~= mbod.numStates
                mbod.settime(time);
            end
            
            %% --Draw bodies, joints, sensors
            h = zeros(mbod.numBodies + mbod.numJoints + ...
                mbod.numSensors + mbod.numBodies*sum(show ~= 0), 1);
            
            if drawBodies
                for i = 1:mbod.numBodies
                    h(i) = draw(mbod.bodies{i}, varargin{plotargs});
                    if i == 1
                        hold on
                    end
                    
                    %{
                    for m = find(show ~= 0)
                        switch show(m)
                            case 'v'
                                quiver3(mbod.bodies{i}.pos(1), ...
                                    mbod.bodies{i}.pos(2), ...
                                    mbod.bodies{i}.pos(3), ...
                                    mbod.bodies{i}.vel(1), ...
                                    mbod.bodies{i}.vel(2), ...
                                    mbod.bodies{i}.vel(3), ...
                                    'Color', [0 0.5 0]);
                            case 'o'
                                quiver3(mbod.bodies{i}.pos(1), ...
                                    mbod.bodies{i}.pos(2), ...
                                    mbod.bodies{i}.pos(3), ...
                                    mbod.bodies{i}.om(1), ...
                                    mbod.bodies{i}.om(2), ...
                                    mbod.bodies{i}.om(3), ...
                                    'Color', [0 0.5 0], 'LineStyle', ':');
                            case 'p'
                                [p,junk] = mbod.bodies{i}.momentum;
                                quiver3(mbod.bodies{i}.pos(1), ...
                                    mbod.bodies{i}.pos(2), ...
                                    mbod.bodies{i}.pos(3), ...
                                    p(1), ...
                                    p(2), ...
                                    p(3), ...
                                    'Color', 'b', 'LineWidth', 2);
                            case 'h'
                                [junk,H] = mbod.bodies{i}.momentum;
                                quiver3(mbod.bodies{i}.pos(1), ...
                                    mbod.bodies{i}.pos(2), ...
                                    mbod.bodies{i}.pos(3), ...
                                    H(1), ...
                                    H(2), ...
                                    H(3), ...
                                    'Color', 'b', 'LineWidth', 2, ...
                                    'LineStyle', ':');
                            case 'f'
                                f = getForce(mbod.bodies{i});
                                quiver3(mbod.bodies{i}.pos(1), ...
                                    mbod.bodies{i}.pos(2), ...
                                    mbod.bodies{i}.pos(3), ...
                                    f(1), ...
                                    f(2), ...
                                    f(3), ...
                                    'Color', 'r');
                            case 't'
                                T = getTorque(mbod.bodies{i});
                                quiver3(mbod.bodies{i}.pos(1), ...
                                    mbod.bodies{i}.pos(2), ...
                                    mbod.bodies{i}.pos(3), ...
                                    T(1), ...
                                    T(2), ...
                                    T(3), ...
                                    'Color', 'r', 'LineStyle', ':');
                        end
                    end
                    %}
                end
            end
            
            if drawJoints
                for j = 1:mbod.numJoints
                    h(i + j) = ...
                        draw(mbod.joints{j}, varargin{plotargs});
                end
            end
            
            if drawSensors
                c = get(gca, 'colororder');
                for k = 1:mbod.numSensors
                    h(i + j + k) = ...
                        draw(mbod.sensors{k}, ...
                        'color', c(mod(k-1,size(c,1))+1,:));
                end
            end
            
            hold off
            xlabel('x')
            ylabel('y')
            zlabel('z')
            title(['t = ' num2str(mbod.t(time)) ' s'])
            
            if lit
                light
            end
            
            if ~isempty(filename)
                light
                print2im(filename);
            end
        end
        
        %% ENERGY
        function [Ktot Utot] = energy(mbod)
            % ENERGY returns the current kinetic energy of a multibody
            % system and potential energy if the mBody object has a
            % potential energy function. Syntax:
            %
            %   [K U] = energy(mBody)
            %   [K U] = mBody.energy
            %
            % See also PLOTENERGY, MOMENTUM, PLOTMOMENTUM, BODY\ENERGY.
            
            Ktot = 0;
            Utot = 0;
            for i = 1:mbod.numBodies
                Ktot = Ktot + mbod.bodies{i}.energy;
                if nargin(mbod.U) == 2
                    Utot = Utot + mbod.U(mbod.bodies{i}.pos, mbod.bodies{i}.time);
                elseif nargin(mbod.U) == 1
                    Utot = Utot + mbod.U(mbod.bodies{i});
                end
            end
        end
        
        %% EOM
        function xdot = eom(mbod, time, state, tidy)
            % EOM is not for human use. They use SOLVE.
            %
            % See SOLVE.
            
            %% --Set current state of each body
            mbod.overwriteState(time, state);
            
            %% --Get matrices
            [A b] = mbod.constraints;
            [M F] = mbod.massforce;
            
            %% --U-K's Fundamental Equation with singular mass matrix
            % Udwadia, F. and Phohomsiri, P. "Explicit equations of motion
            %   for constrained mechanical systems with singular mass
            %   matrices and applications to multi-body dynamics."
            %   Proc. R. Soc. A, vol. 462, 2006, p. 2097-2117
            Mbar = [ ( eye(mbod.numStates/2) - pinv(A)*A ) * M;
                A];
            xdd = pinv(Mbar) * [F; b];
            
            %% --Tidy actions
            if tidy
                % Subtract off any velocity that is not in the virtual
                % displacement direction (not compatible with constraints)
                % before returning velocity states
                Rsp = orth(A');
                % ^ Basis for row space of A: not-allowed directions
                %   (virtual displacements are w = null(A))
                for i = 1:size(Rsp,2)
                    state((end/2 + 1):end) = state((end/2 + 1):end) - ...
                        ( Rsp(:,i)'*state((end/2 + 1):end) ) * Rsp(:,i);
                end
                
                % If there aren't any body forces or force objects acting
                % on the bodies in the system, then reduce the magnitude of
                % the system velocity vector if current total energy
                % exceeds initial total energy
                %{
                if mbod.numForces == 0 && ...
                        norm(mbod.force(state(1:3), state(4:7), [0;0;0], [0;0;0], time)) == 0 && ...
                        norm(mbod.torque(state(1:3), state(4:7), [0;0;0], [0;0;0], time)) == 0
                    ft = 0;
                    for i = 1:mbod.numBodies
                        bod = mbod.bodies{i};
                        ft = ft + norm(bod.force(bod.pos, bod.att, bod.vel, bod.om, bod.time)) + ...
                            norm(bod.torque(bod.pos, bod.att, bod.vel, bod.om, bod.time));
                    end

                    % Current system energy
                    [K U] = mbod.energy;
                    
                    if ft == 0 && (K+U) > tidy;
                        vhat = state((end/2 + 1):end)/norm(state((end/2 + 1):end));
                        vmag = sqrt( abs(2*(tidy - U)/(vhat'*M*vhat)) );
                        state((end/2 + 1):end) = vmag*vhat;
                        %K = vmag^2 * vhat'*M*vhat/2;
                    end
                    
                    %mbod.currentE = K + U;
                end
                %}
            end
            
            %% --Return state derivative
            xdot = [state((mbod.numStates/2 + 1):end);
                xdd];
        end
        
        %% ERASE
        function erase(mbod)
            % ERASE clears the graphical representation of a mBody from all
            % axes, if one exists. Syntax:
            %
            %   erase(mBody)
            %   mBody.erase
            %
            % See also DRAW.
            
            for i = 1:mbod.numBodies
                erase(mbod.bodies{i});
            end
            for j = 1:mbod.numJoints
                erase(mbod.joints{j});
            end
            for k = 1:mbod.numSensors
                erase(mbod.sensors{k});
            end
        end
        
        %% FINDEQ
        function x0 = findeq(mbod, varargin)
            % FINDEQ locates the equilibrium state of a multibody system
            % to the precision of mBody.thresh. It proceeds from the
            % current state of the mBody and simulates until the system
            % reaches equilibrium. Then it restores the previous state
            % information of the mBody. Syntax:
            %
            %   x0 = mbod.findeq('property', value, ...)
            %   x0 = findeq(mbod, 'property', value, ...)
            %
            % x0 is the full system state at equilibrium.
            %
            % Optional arguments:
            %
            %   mode    'v'/'f' Choose 'v' to have findeq stop searching
            %                       when the system has zero velocity.
            %                       Choose 'f' to stop when the system
            %                       experiences zero force along virtual
            %                       displacements. 'f' by default.
            %   chunk   dbl     Length of time each sub-simulation should
            %                       go for. Smaller chunks are slower
            %                       overall, but more robust. Defaults to 5
            %                       seconds.
            %   tLim    dbl     Limit on how far findeq should simulate
            %                       before giving up. Defaults to Inf.
            %   debug   T/F     If true, produces a plot of system energy
            %                       after each attempt to find equilibrium.
            %
            % See also SOLVE.
            
            %% -- Parse inputs
            debug = false;
            chunk = 5;
            tLim = inf;
            mode = 'f';
            arglist = true(size(varargin));
            for i = 1:2:(length(varargin))
                if ~ischar(varargin{i})
                    error('MBODY:findeq:input', 'Property designations must be strings.')
                end
                switch lower(varargin{i})
                    case 'mode'
                        mode = varargin{i+1};
                        if ~ischar(mode) || (mode(1) ~= 'v' && mode(1) ~= 'f')
                            error('MBODY:findeq:input', 'Mode should be either ''v'' or ''f''.')
                        end
                        arglist(i:i+1) = [false false];
                    case 'chunk'
                        chunk = varargin{i+1}(1);
                        arglist(i:i+1) = [false false];
                    case 'tlim'
                        tLim = varargin{i+1}(1);
                        arglist(i:i+1) = [false false];
                    case 'debug'
                        debug = varargin{i+1}(1);
                        arglist(i:i+1) = [false false];
                end
            end
            
            time = mbod.t;
            state = mbod.x;
            if debug
                h = figure;
            end
            
            disp('-- QuIRK: Finding equilibrium...')
            
            % Check to see if we're already at equilibrium
            mbod.settime;
            if mbod.virtualWork < mbod.thresh
                x0 = mbod.x(:,end);
                disp('     System started at equilibrium.');
                return
            end
            
            fprintf('    [-');
            cnt = 1;
            mbod.setnow;
            mbod.solve([0 chunk], 'stop', mode(1), 'msg', 'none', varargin{arglist});
            %evalc(['mbod.solve([0 chunk], ''stop'', ''' mode(1) ''')']);
            % Note: using 'evalc' here suppresses mBody.solve's
            % status messages
            while mbod.t(end) == chunk && chunk*cnt < tLim
                if debug
                    mbod.plotEnergy;
                    drawnow;
                end
                mbod.setnow;
                mbod.solve([0 chunk], 'stop', mode(1), 'msg', 'none', varargin{arglist});
                %evalc(['mbod.solve([0 chunk], ''stop'', ''' mode(1) ''')']);
                cnt = cnt + 1;
                if rem(cnt,50) == 0
                    fprintf('+\n     ');
                elseif rem(cnt,5) == 0
                    fprintf('+');
                else
                    fprintf('-');
                end
            end
            fprintf(']\n     %u calls to mBody.solve.\n', cnt);
            if chunk*cnt >= tLim
                disp('              Time limit reached.')
            end
            if debug
                close(h);
            end
            
            x0 = mbod.x(:,end);
            %x0((end/2+1):end) = zeros(size(x0((end/2+1):end)));
            
            mbod.overwriteState(time, state);
        end
        
        %% GHOST
        function h = ghost(mbod, varargin)
            % GHOST adds a graphical representation of a multibody system
            % to the current axes with default opacity of 10%. This is good
            % for seeing through systems of many bodies. Refer to DRAW for
            % a list of full options. Syntax:
            %
            %   h = ghost(mBody, 'property', value, ...)
            %   h = mBody.ghost('property', value, ...)
            %
            % See also DRAW, ERASE, ANIMATE, TRACE.
            
            %% --Call draw()
            h = draw(mbod, 'alpha', 0.1, varargin{:});
        end
        
        %% MASSFORCE
        function [Meff Feff] = massforce(mbod)
            % MASSFORCE is not for general use. Put it down! But if you
            % attempt to use it, it will spit out the effective mass and
            % force matrices for the mBody. Sometimes this is useful for
            % debugging and such. 
            %
            % See SOLVE.
            
            %% --Set up
            t = mbod.t(end);
            Meff = zeros(7*mbod.numBodies, 7*mbod.numBodies);
            Feff = zeros(7*mbod.numBodies, 1);
            
            %% --Calculate big ol' matrices
            % Query each body for force/torque
            row = 1;
            col = 1;
            for i = 1:mbod.numBodies
                m = mbod.bodies{i}.mass;
                I = mbod.bodies{i}.inertia;
                r0 = mbod.bodies{i}.pos;
                v = mbod.bodies{i}.vel;
                q0 = mbod.bodies{i}.q;
                qdot = mbod.bodies{i}.qdot;
                s = mbod.bodies{i}.diameter/100;
                
                % Calculate intermediate quantities for rotational EoM
                MT = 2*[(-crs(q0(1:3)) + q0(4)*eye(3)) -q0(1:3)];
                MTdot = 2*[(-crs(qdot(1:3)) + qdot(4)*eye(3)) -qdot(1:3)];
                omega = MT * qdot;
                Mtilde = MT' * I * MT;
                
                % Potential gradient
                gradU = [0;0;0];
                gradUt = [0;0;0];
                if nargin(mbod.U) == 2
                    gradU = ([mbod.U(r0 + [s 0 0]', t);
                        mbod.U(r0 + [0 s 0]', t);
                        mbod.U(r0 + [0 0 s]', t)] - mbod.U(r0, t)*ones(3,1))/s;
                elseif nargin(mbod.U) == 1
                    
                    % forces
                    U1 = mbod.U(mbod.bodies{i});
                    mbod.bodies{i}.pos = r0 + [s 0 0]';
                    U2 = mbod.U(mbod.bodies{i});
                    gradU(1) = (U2 - U1)/s;
                    
                    mbod.bodies{i}.pos = r0 + [0 s 0]';
                    U2 = mbod.U(mbod.bodies{i});
                    gradU(2) = (U2 - U1)/s;
                    
                    mbod.bodies{i}.pos = r0 + [0 0 s]';
                    U2 = mbod.U(mbod.bodies{i});
                    gradU(3) = (U2 - U1)/s;
                    
                    mbod.bodies{i}.pos = r0;
                    
                    % torques
                    dq = [sin(0.01/2) 0 0 cos(0.01/2)]';
                    mbod.bodies{i}.att = ...
                        [(dq(4)*eye(3)+crs(dq(1:3))) dq(1:3);
                        -dq(1:3)' dq(4)] * q0;
                    U2 = mbod.U(mbod.bodies{i});
                    gradUt(1) = (U2 - U1)/0.01;
                    
                    dq = [0 sin(0.01/2) 0 cos(0.01/2)]';
                    mbod.bodies{i}.att = ...
                        [(dq(4)*eye(3)+crs(dq(1:3))) dq(1:3);
                        -dq(1:3)' dq(4)] * q0;
                    U2 = mbod.U(mbod.bodies{i});
                    gradUt(2) = (U2 - U1)/0.01;
                    
                    dq = [0 0 sin(0.01/2) cos(0.01/2)]';
                    mbod.bodies{i}.att = ...
                        [(dq(4)*eye(3)+crs(dq(1:3))) dq(1:3);
                        -dq(1:3)' dq(4)] * q0;
                    U2 = mbod.U(mbod.bodies{i});
                    gradUt(3) = (U2 - U1)/0.01;
                    
                    mbod.bodies{i}.att = q0;
                end
                
                % Force objects
                F_obj = [0 0 0]';
                if ~isempty(mbod.Sf) && any(mbod.Sf(i,:)) ~= 0
                    for j = find(mbod.Sf(i,:) ~= 0)
                        F_obj = F_obj + ...
                            mbod.Sf(i,j)*mbod.forces{j}.getForce;
                    end
                end
                
                T_obj = [0 0 0]';
                if ~isempty(mbod.Sf) && any(mbod.Sf(i,:)) ~= 0
                    for j = find(mbod.Sf(i,:) ~= 0)
                        T_obj = T_obj + ...
                            mbod.Sf(i,j)*mbod.forces{j}.getTorque;
                    end
                end
                
                % Body forces
                if nargin(mbod.force) == 5
                    bodyFc = mbod.force(r0, q0, v, omega, t);
                else
                    bodyFc = mbod.force(mbod.bodies{i});
                end
                
                F_ext = mbod.bodies{i}.getForce ...
                    + bodyFc ...
                    - gradU + F_obj ...
                    - mbod.damping*v;
                
                if nargin(mbod.torque) == 5
                    bodyTq = mbod.torque(r0, q0, v, omega, t);
                else
                    bodyTq = mbod.torque(mbod.bodies{i});
                end
                
                tau_ext = mbod.bodies{i}.getTorque ...
                    + bodyTq ...
                    - q2d(q0)'*gradUt + q2d(q0)'*T_obj ...
                    - mbod.damping*mbod.bodies{i}.om / (mbod.bodies{i}.diameter/2);
                
                Ftilde = MT'*tau_ext - MT'*crs(omega)*I*omega - MT'*I*MTdot*qdot;
                
                Feff(row:(row+6), 1) = [F_ext; Ftilde];
                Meff(row:(row+6), col:(col+6)) = ...
                    [m*eye(3)	zeros(3, 4);
                    zeros(4, 3)	Mtilde];
                
                row = row + 7;
                col = col + 7;
            end
        end
        
        %% MOMENTUM
        function [ptot Htot] = momentum(mbod)
            % MOMENTUM returns the current linear and angular momentum of a
            % multibody system in the inertial frame. Syntax:
            %
            %   [p H] = momentum(mBody)
            %   [p H] = mBody.momentum
            %
            % p:    total linear momentum vector of mBody
            % H:    total angular momentum vector of mBody
            %
            % See also ENERGY, PLOTMOMENTUM, BODY\MOMENTUM.
            
            ptot = [0 0 0]';
            Htot = ptot;
            for i = 1:mbod.numBodies
                [p H] = mbod.bodies{i}.momentum;
                ptot = ptot + p;
                Htot = Htot + H;
            end
        end
        
        %% OVERWRITESTATE
        function mbod = overwriteState(mbod, t, state)
            % OVERWRITESTATE copies a new matrix into the state history of
            % the multibody system. To be safe, you should only do this
            % with a state extracted from a multibody system with the same
            % number of bodies (or, preferably, the same bodies).
            %
            %   mBody = overwriteState(mBody, t, x)
            %   mBody = mBody.overwriteState(t, x)
            %
            % Required arguments
            %
            %   t:  Time vector.
            %   x:  State vector history. This should be a matrix with
            %           14*numBodies rows in the form
            %               [body1Pos; body1Quat; ...;
            %                   body1Vel; body1QDeriv; ...],
            %           and length(t) columns.
            %
            % If you just want to display the multibody system at a
            % particular state, but don't want to overwrite the state
            % history, try using mBody.draw('state', x).
            %
            % See also RESET, SETTIME, SETNOW, SOLVE.
            
            if size(t,2) == 1
                t = t';
            end
            if size(t,1) ~= 1
                error('MBODY:overwriteState:input', ...
                    'Time must be a vector.')
            end
            if size(state,1) ~= mbod.numStates
                error('MBODY:overwriteState:input', ...
                    ['State history must have ' num2str(mbod.numStates) ' rows.'])
            end
            
            mbod.t = t;
            mbod.x = state;
            mbod.settime;
            mbod.solved = true;
        end
        
        %% PLOTENERGY
        function [Ktot Utot] = plotEnergy(mbod)
            % PLOTENERGY displays a plot of the total kinetic energy of a
            % multibody system. It is most useful after the system has been
            % solved. If the mBody has a potential energy function, the
            % value of that function is also plotted. Syntax:
            %
            %   [K U] = plotEnergy(mBody)
            %   [K U] = mBody.plotEnergy
            %
            % K and U are vectors with one entry per element of the time
            % vector kept in mBody.t.
            %
            % See also PLOTPOWER, PLOTMOMENTUM, ENERGY, BODY\ENERGY.
            
            if ~mbod.solved
                [Ktot Utot] = energy(mbod);
                return
            end
            
            Ktot = zeros(size(mbod.t));
            Utot = Ktot;
            
            for time = 1:length(mbod.t)
                % One way to do this is to query each body object at each
                % time, like this:
                %   mbod.settime(time);
                %   [Ktot(time) Utot(time)] = mbod.energy;
                % ...however, that is less computationally efficient than
                % using the known state values, as below.
                
                r = mbod.x(1:(mbod.numStates/2), time);
                v = mbod.x((mbod.numStates/2+1):end, time);
                
                if nargin(mbod.U) == 1
                    mbod.settime(time);
                end
                    
                for i = 1:mbod.numBodies
                    bod = mbod.bodies{i};
                    
                    q = r( (7*i-3):(7*i) );
                    T = 2*[(q(4)*eye(3) - crs(q(1:3))) -q(1:3)];
                    
                    K = bod.mass/2 * norm(v( (7*i-6):(7*i-4) ))^2 + ...
                        v( (7*i-3):(7*i) )' * T'*bod.inertia*T * v( (7*i-3):(7*i) )/2;
                    if nargin(mbod.U) == 2
                        U = mbod.U( r( (7*i-6):(7*i-4) ), mbod.t(time) );
                    elseif nargin(mbod.U) == 1
                        U = mbod.U( bod );
                    end
                    
                    Ktot(time) = Ktot(time) + K;
                    Utot(time) = Utot(time) + U;
                end
            end
            
            plot(mbod.t, Ktot, '-r', mbod.t, Utot, '-b')
            hold on
            plot(mbod.t, Ktot+Utot, '--k', 'LineWidth', 2)
            hold off
            title('System Energy')
            xlabel('Time, s')
            ylabel('Energy, J')
            legend('Kinetic', 'Potential', 'Total')
        end
        
        %% PLOTMOMENTUM
        function [ptot Htot] = plotMomentum(mbod)
            % PLOTMOMENTUM displays a plot of the linear and angular
            % momentum magnitudes of a multibody system. It is most useful
            % after the system has been solved. Syntax:
            %
            %   [p H] = plotMomentum(mBody)
            %   [p H] = mBody.plotMomentum
            %
            % p and H are vectors with one entry per element of the time
            % vector kept in mBody.t.
            %
            % See also PLOTENERGY, MOMENTUM, BODY\MOMENTUM.
            
            ptot = zeros(size(mbod.t));
            Htot = zeros(size(mbod.t));
            
            for time = 1:length(mbod.t);
                mbod.settime(time);
                
                [p H] = mbod.momentum;
                ptot(time) = norm(p);
                Htot(time) = norm(H);
            end
            
            [AX,junk1,junk2] = plotyy(mbod.t, ptot, mbod.t, Htot);
            set(get(AX(1),'Ylabel'),'String','Linear momentum, kg m/s')
            set(get(AX(2),'Ylabel'),'String','Angular momentum, kg m^2/s')
            title('System Momentum Magnitudes')
            xlabel('Time, s')
        end
        
        %% PLOTPOWER
        function [Ptot] = plotPower(mbod)
            % PLOTPOWER displays a plot of the total power applied to a
            % multibody system. It is most useful after the system has been
            % solved. Syntax:
            %
            %   P = plotPower(mBody)
            %   P = mBody.plotPower
            %
            % P is a vector with one entry per element of the time vector
            % kept in mBody.t.
            %
            % See also PLOTENERGY, PLOTMOMENTUM, ENERGY, BODY\ENERGY.
            
            if ~mbod.solved
                return
            end
            
            [Ktot Utot] = mbod.plotEnergy;
            clf
            Ptot = [0 diff(Ktot + Utot)./diff(mbod.t)];
            
            plot(mbod.t, Ptot, '-r')
            title('Power Applied to System')
            xlabel('Time, s')
            ylabel('Power, W')
        end
        
        %% RECOMPUTEINERTIA
        function mbod = recomputeInertia(mbod)
            % RECOMPUTEINERTIA recalculates the inertia matrix of all
            % bodies in their body frames. Call this method if you have
            % manually changed any of the body size parameters (bod.mass,
            % bod.size, bod.sx, etc) and want to derive the inertia
            % matrices from the new parameters. Syntax:
            %
            %   mbody = recomputeInertia(mbody)
            %   mbody = mbody.recomputeInertia
            %
            % See also BODY\RECOMPUTEINERTIA, BODY\RESIZE.
            
            for i = 1:mbod.numBodies
                mbod.bodies{i} = recomputeInertia(mbod.bodies{i});
            end
        end
        
        %% REMOVE
        function mbod = remove(mbod, varargin)
            % REMOVE takes bodies, joints, etc. out of the multibody
            % system. If the system has previously been solved, it assumes
            % that the objects should be removed at the most recent time
            % in the system state history. Use SETNOW or RESET before
            % removing if this is not the case. Syntax:
            %
            %   mbody = remove(mbody, body, joint, sensor, ...)
            %   mbody = mbody.remove(body, sensor, joint, ...)
            %
            % You can list any number of body, sensor, and joint objects,
            % in any order. Be careful when removing objects with
            % dependencies that you do not leave any orphans (for example,
            % removing one body connected to a joint without removing the
            % joint). After removing objects, remove will automatically
            % perform an orphan check.
            %
            % See also ADD, SETTIME, SETNOW, RESET, CHECKORPHANS.
            
            if isempty(varargin)
                return
            end
            
            warning('off', 'BODY:setParent:dontDoThat');
            
            % Check to see if there's a state history, and set to end time
            if numel(mbod.t) > 1
                mbod.setnow;
            end
            
            % Go through arguments and construct logical array of objects
            keepB = true(1, mbod.numBodies);
            keepJ = true(1, mbod.numJoints);
            keepF = true(1, mbod.numForces);
            keepS = true(1, mbod.numSensors);
            for i = 1:numel(varargin)
                switch class(varargin{i})
                    case 'body'
                        for j = 1:mbod.numBodies
                            if mbod.bodies{j} == varargin{i}
                                keepB(j) = false;
                                varargin{i}.clearParent;
                            end
                        end
                    case 'joint'
                        for j = 1:mbod.numJoints
                            if mbod.joints{j} == varargin{i}
                                keepJ(j) = false;
                            end
                        end
                    case 'force'
                        for j = 1:mbod.numForces
                            if mbod.forces{j} == varargin{i}
                                keepF(j) = false;
                            end
                        end
                    case 'sensor'
                        for j = 1:mbod.numSensors
                            if mbod.sensors{j} == varargin{i}
                                keepS(j) = false;
                            end
                        end
                end
                
            end
            
            % Pull out discarded objects
            mbod.bodies = { mbod.bodies{keepB} };
            mbod.numBodies = sum(keepB);
            mbod.numStates = 14 * sum(keepB);
            mbod.bodyNames = { mbod.bodyNames{keepB} };
            
            mbod.joints = { mbod.joints{keepJ} };
            mbod.numJoints = sum(keepJ);
            mbod.jointNames = { mbod.jointNames{keepJ} };
            mbod.numConstraints = 0;
            for i = 1:mbod.numJoints
                mbod.numConstraints = mbod.numConstraints ...
                    + mbod.joints{i}.numConstraints;
            end
            
            mbod.forces = { mbod.forces{keepF} };
            mbod.numForces = sum(keepF);
            mbod.forceNames = { mbod.forceNames{keepF} };
            
            mbod.sensors = { mbod.sensors{keepS} };
            mbod.numSensors = sum(keepS);
            mbod.sensorNames = { mbod.sensorNames{keepS} };
            
            % Pull out incidence matrix
            if mbod.numJoints > 0
                mbod.S = mbod.S(keepB, keepJ);
            else
                mbod.S = [];
            end
            if mbod.numForces > 0
                mbod.Sf = mbod.Sf(keepB, keepF);
            else
                mbod.Sf = [];
            end
            
            % Check for orphans
            mbod.checkOrphans;
            
            warning('off', 'MBODY:unSolve:dontDoThat');
            mbod.setnow;
            warning('on', 'MBODY:unSolve:dontDoThat');
            
            warning('on', 'BODY:setParent:dontDoThat');
        end
        
        %% RESET
        function mbod = reset(mbod)
            % RESET brings a multibody system back to its initial state.
            % Use before adding/removing objects and re-solving. Syntax:
            %
            %   mbody = reset(mbody)
            %   mbody = mbody.reset
            %
            % Both commands produce the same result as mbody.settime(1)
            % followed by mbody.setnow.
            %
            % See also SETTIME, SETNOW, SOLVE.
            
            mbod = mbod.settime(1);
            mbod = mbod.setnow;
        end
        
        %% SET.FORCE
        function mbod = set.force(mbod, f)
            % Parse inputs
            if ~isa(f, 'function_handle')
                error('MBODY:set:input', 'Force must be a function handle.')
            elseif nargin(f) ~= 5 && nargin(f) ~= 1
                error('MBODY:set:input', 'Force must be a function of (position, attitude, vel, ang vel, time) or a body object.')
            end
            
            % Set
            mbod.force = f;
            
            warning('off', 'MBODY:unSolve:dontDoThat')
            unSolve(mbod);
            warning('on', 'MBODY:unSolve:dontDoThat')
        end
        
        %% SET.TORQUE
        function mbod = set.torque(mbod, T)
            % Parse inputs
            if ~isa(T, 'function_handle')
                error('MBODY:set:input', 'Torque must be a function handle.')
            elseif nargin(T) ~= 5 && nargin(T) ~= 1
                error('MBODY:set:input', 'Torque must be a function of (position, attitude, vel, ang vel, time) or a body object.')
            end
            
            % Set
            mbod.torque = T;
            
            warning('off', 'MBODY:unSolve:dontDoThat')
            unSolve(mbod);
            warning('on', 'MBODY:unSolve:dontDoThat')
        end
        
        %% SETNOW
        function mbod = setnow(mbod)
            % SETNOW clears the state history of a multibody system and
            % tells it that the current state of each object is the current
            % state of the system. Use after adding/removing/editing
            % objects and before re-solving. Syntax:
            %
            %   mbody = setnow(mbody)
            %   mbody = mbody.setnow
            %
            % Be careful not to confuse this command with SETTIME without
            % arguments.
            %
            % See also SOLVE, RESET, SETTIME.
            
            %% --Set current state
            state = zeros(mbod.numStates, 1);
            idx = 1;
            for i = 1:mbod.numBodies
                state(idx:(idx+2)) = mbod.bodies{i}.pos;
                state((mbod.numStates/2+idx):(mbod.numStates/2+idx+2)) = ...
                    mbod.bodies{i}.vel;
                idx = idx + 3;
                state(idx:(idx+3)) = mbod.bodies{i}.att;
                state((mbod.numStates/2+idx):(mbod.numStates/2+idx+3)) = ...
                    mbod.bodies{i}.qdot;
                idx = idx + 4;
                
                mbod.bodies{i}.time = 0;
            end
            
            mbod.x = state;
            mbod.t = 0;
            mbod.solved = false;
        end
        
        %% SETTIME
        function mbod = settime(mbod, time)
            % SETTIME brings a multibody system to its state at a
            % particular time value. Syntax:
            %
            %   mbody = settime(mbody, time)
            %   mbody = mbody.settime(time)
            %
            % Optional arguments:
            %
            %   time:	Index into time vector. Defaults to end.
            %
            % See also RESET, SETNOW.
            
            if nargin < 2
                time = length(mbod.t);
            elseif ~isnumeric(time)
                error('MBODY:setnow:input', 'Time must be numeric.')
            else
                time = round(time(1));
            end
            
            if time > length(mbod.t)
                warning('MBODY:setnow:time', ...
                    'Specified time exceeds mBody time vector; defaulting to (end).')
                time = length(mbod.t);
            elseif time <= 0
                warning('MBODY:setnow:time', ...
                    'Specified time index is less than 1; defaulting to 1.')
                time = 1;
            end
            
            %% --Set body states
            state = mbod.x(:,time);
            idx = 1;
            for i = 1:mbod.numBodies
                mbod.bodies{i}.pos = state(idx:(idx+2));
                mbod.bodies{i}.vel = ...
                    state((mbod.numStates/2+idx):(mbod.numStates/2+idx+2));
                idx = idx + 3;
                mbod.bodies{i}.att = state(idx:(idx+3));
                q = state(idx:(idx+3));
                mbod.bodies{i}.om = ...
                    2*[(-crs(q(1:3))+q(4)*eye(3)) -q(1:3)] * ...
                    state((mbod.numStates/2+idx):(mbod.numStates/2+idx+3));
                idx = idx + 4;
                mbod.bodies{i}.time = mbod.t(time);
            end
        end
        
        %% SOLVE
        function mbod = solve(mbod, tspan, varargin)
            % SOLVE calculates the state time history of a multibody system
            % using the Udwadia-Kalaba equations of motion. You must call
            % solve on a multibody system before animating it. Syntax:
            %
            %   mbody = solve(mbody, tspan, property, value, odeOpts)
            %   mbody = mbody.solve(tspan, property, value, odeOpts)
            %
            % Once solved, the mBody object will retain its state history
            % for further plotting commands, interrogation of its energy
            % and momentum, and further manipulation. However, some
            % manipulations of the mBody object or its constituent
            % components will require re-solving the mBody.
            %
            % Required arguments
            %
            %   tspan:	ODE45 timespan vector, [start end] or [start:end]
            %
            % Optional properties
            %
            %   sense   T/F,int Set this property to true to have the
            %                       simulation terminate if any sensors
            %                       trigger. Specify an integer to have the
            %                       simulation terminate if that number of
            %                       sensors trigger.
            %   stop    str     State conditions on which to terminate
            %                       solution. You may supply the 'stop'
            %                       keyword multiple times to specify more
            %                       than one condition. Available options:
            %                       'velocity'  Stop solution when the 
            %                                   bodies have near-zero
            %                                   velocity.
            %                       'force'     Stop solution when the 
            %                                   bodies have near-zero force
            %                                   applied in the direction of
            %                                   virtual displacements.
            %                       'collision' Stop when checkCollisions 
            %                                   returns true.
            %                       numeric     Stop integration after the 
            %                                   specified number of seconds
            %                                   (real time).
            %   solver  str     Follow this property with a string
            %                       containing the name of any ode solver,
            %                       for example 'ode45' or 'ode15s'.
            %   tidy    T/F     Set this property to true to instruct
            %                       solve to perform extra tidying-up
            %                       operations on the system state as it
            %                       integrates (slower, but produces
            %                       cleaner, more robust integration). Good
            %                       for long solutions.
            %   msg     str    Set to 'none' to prevent solve from
            %                       printing output messages. Defaults to
            %                       'full'.
            %   odeOpts	struct  ode45 options structure from odeset
            %                       (e.g. odeset('OutputFcn', @odeplot) )
            %
            % See also RESET, SETTIME, SETNOW, ANIMATE, PLOTENERGY,
            % PLOTMOMENTUM, UNSOLVE.
            
            %% --Parse inputs
            odeOpts = odeset;
            solver = 'ode45';
            stopType = [0 0 0 0 0];
            tidy = false;
            msg = 'full';
            for i = 1:2:length(varargin)
                switch lower(varargin{i})
                    case {'sense' 'sensor' 'sen' 'se'}
                        stopType(1) = double(varargin{i+1}(1));
                    case 'stop'
                        if ~isempty(strfind(varargin{i+1},'v'))
                            stopType(2) = 1;
                        elseif ~isempty(strfind(varargin{i+1},'f'))
                            stopType(3) = 1;
                        elseif ~isempty(strfind(varargin{i+1},'c'))
                            stopType(4) = 1;
                        elseif isnumeric(varargin{i+1})
                            stopType(5) = cputime + varargin{i+1}(1);
                        end
                    case 'tidy'
                        tidy = logical(varargin{i+1}(1));
                    case {'solver' 'solve' 'sol' 'so'}
                        solver = char(varargin{i+1});
                    case {'odeopts' 'ode' 'opts'}
                        odeOpts = odeset(odeOpts, varargin{i+1});
                    case {'messages' 'message' 'msg' 'm'}
                        msg = char(varargin{i+1});
                end
            end
            
            if ~strcmpi(msg, 'none')
                disp('-- QuIRK: Solving equations of motion...')
                solveTime = tic;
            end
            
            % Beef up RelTol to catch sensor events
            if any(stopType)
                odeOpts = odeset(odeOpts, 'Events', ...
                        @(t,y)solverEvents(mbod, t, y, stopType, tspan));
            end
            if isempty(odeOpts.RelTol)
                odeOpts = odeset(odeOpts, 'RelTol', 1e-8);
            else
                odeOpts = odeset(odeOpts, 'RelTol', 1e-5);
            end
            
            %% --Set up initial state
            x0 = mbod.x(:,1);
            
            %% --Subtract off any incompatible initial velocity 
            % (not in the virtual displacement direction)
            [A, junk] = mbod.constraints;
            Rsp = orth(A');
            % ^ Basis for row space of A: not-allowed directions
            % (virtual displacements are w = null(A))
            for i = 1:size(Rsp,2)
                x0((end/2 + 1):end) = x0((end/2 + 1):end) - ...
                    ( Rsp(:,i)'*x0((end/2 + 1):end) ) * Rsp(:,i);
            end
            
            %% --Check to see if we're already at equilibrium
            if stopType(3)
                if mbod.virtualWork < mbod.thresh
                    if strcmpi(msg, 'full')
                        disp('              System started at equilibrium.')
                    end
                    mbod.solved = true;
                    return
                end
            end
            
            %% --Set up "tidy" option with initial energy
            if tidy
                [K U] = mbod.energy;
                tidy = K + U;
            end
            
            %% --Solve
            eqn = @(t, x)eom(mbod, t, x, tidy);
            
            if any(stopType)
                [time, state, junk1, junk2, idx] = eval([solver, ...
                    '(eqn, tspan, x0, odeOpts);']);
                if any(idx == 5)
                    state(end, (mbod.numStates/2 + 1):end) = ...
                        zeros(1, mbod.numStates/2);
                end
                if strcmpi(msg, 'full')
                    if any(idx == 1)
                        disp('              Solution stopped: sensor triggered.')
                    elseif any(idx == 2)
                        disp('              Solution stopped: zero velocity.')
                    elseif any(idx == 3)
                        disp('              Solution stopped: zero force.')
                    elseif any(idx == 4)
                        disp('              Solution stopped: body collision.')
                    elseif any(idx == 5)
                        disp('              Solution stopped: time elapsed.')
                    end
                end
            else
                [time, state] = eval([solver, ...
                    '(eqn, tspan, x0, odeOpts);']);
            end
            
            mbod.t = time';
            mbod.x = state';
            mbod.solved = true;
            
            % Note: settime is really important here. Since the equation of
            % motion involves querying and setting the body states, ode45's
            % adaptive step-size algorithm may leave the mBody at some
            % intermediate state between the values output in mbod.x.
            % Subsequent operations on the mBody would then proceed from
            % that fictitious intermediate state. Settime makes sure that
            % the mBody ends up at the actual last state stored in mbod.x.
            mbod.settime(length(time));
            
            if ~strcmpi(msg, 'none')
                disp(['              Solution complete in ' ...
                    num2str(toc(solveTime)) ' seconds.'])
            end
        end
        
        %% SOLVEREVENTS
        function [value, isterminal, dir] = solverEvents(mbod, t, y, type, tspan)
            % SOLVEREVENTS is not for human use. It is for ode45 to
            % determine if any ode-shattering events occur.
            
            value = [1 1 1 1 1]';
            isterminal = [1 1 1 1 1]';
            dir = [1 -1 0 0 0]';
            
            %% Sensor trigger detection
            if type(1)                
                sensorRegister = 0;
                i = 1;
                range = Inf;
                while i <= mbod.numSensors %&& sensorRegister < type(1)
                    [switched, num, mR] = trigger(mbod.sensors{i});
                    if switched
                        sensorRegister = sensorRegister + ...
                            num;
                        range = min([range mR]);
                    end
                    i = i + 1;
                end
                %[type(1) sensorRegister num switched]
                %value(1) = switched;
                %value(1) = range;
                value(1) = type(1) - sensorRegister - 0.5;
            end
            
            %% Zero-velocity detection
            if type(2) && t > (tspan(1) + eps)
                value(2) = norm(y((mbod.numStates/2+1):end)) > mbod.thresh;
            elseif t <= (tspan(1) + eps)
                value(2) = 1;
            end
            
            %% Zero-force detection
            if type(3)
                value(3) = mbod.virtualWork > mbod.thresh;
            end
            
            %% collision detection
            if type(4) && t > (tspan(1) + eps)
                value(4) = ~mbod.checkCollisions;
            elseif t <= (tspan(1) + eps)
                value(4) = 1;
            end
            
            %% crash-time detection
            if type(5)
                value(5) = type(5) - cputime;
            end
        end
        
        %% TRACE
        function trace(mbod, varargin)
            % TRACE ghosts a graphical representation of a multibody system
            % through its motion with default opacity of 10%, and opacity
            % of 50% in its final configuration. Syntax:
            %
            %   trace(mBody, 'property', value, ...)
            %   mBody.trace('property', value, ...)
            %
            %   mBody:   multibody system to trace
            %
            % Optional properties
            %
            %   tspan   int     Indices in time vector at which to start
            %                       and stop the animation: [start stop]
            %   steps   int     Number of intermediate steps to illustrate
            %   color   dbl/str RGB or Matlab string specifying color of body
            %   alpha   dbl     Override opacity of body
            %
            % See also DRAW, ERASE, ANIMATE, TRACE.
            
            tspan = [1 length(mbod.t)];
            steps = 10;
            
            %% --Parse inputs
            plotargs = true(size(varargin));
            if numel(varargin)/2 ~= round(numel(varargin)/2)
                error('MBODY:draw:input', 'All properties must be paired with values.')
            end
            for i = 1:2:length(varargin)
                if ~ischar(varargin{i})
                    error('MBODY:draw:input', 'Property designations must be strings.')
                elseif strncmpi(varargin{i}, 'ts', 2)
                    if ~isnumeric(varargin{i+1})
                        error('MBODY:draw:input', 'Tspan must be numeric.')
                    elseif varargin{i+1} ~= round(varargin{i+1})
                        error('MBODY:draw:input', 'Tspan must be integer indices into time array.')
                    elseif varargin{i+1} <= 0
                        error('MBODY:draw:input', 'Tspan indices must be >= 1.')
                    elseif numel(varargin{i+1}) ~= 2
                        error('MBODY:draw:input', 'Tspan vector must have 2 elements.')
                    else
                        tspan = varargin{i+1}(1:2);
                        plotargs(i:(i+1)) = [false false];
                    end
                elseif strncmpi(varargin{i}, 'st', 2)
                    if ~isnumeric(varargin{i+1})
                        error('MBODY:draw:input', 'Steps must be numeric.')
                    elseif varargin{i+1} ~= round(varargin{i+1})
                        error('MBODY:draw:input', 'Steps must be integer indices into time array.')
                    else
                        steps = varargin{i+1}(1);
                        plotargs(i:(i+1)) = [false false];
                    end
                end
            end
            
            %% --Draw ghost snapshots
            for f = tspan(1):((tspan(2)-1 - tspan(1))/steps):(tspan(2)-1)
                ghost(mbod, 'time', round(f), 'snap', true, ...
                    'joints', false, 'sensors', false, ...
                    'LineStyle', 'none', 'Marker', 'none', ...
                    varargin{plotargs});
                hold on
            end
            draw(mbod, 'time', tspan(2), 'snap', true, 'alpha', 0.5, ...
                varargin{plotargs});
            hold off
            
            title('System Trajectory')
        end
        
        %% UNSOLVE
        function mbod = unSolve(mbod)
            % UNSOLVE clears the mBody's "solved" flag. Don't do this
            % manually; it will update as you manipulate the mBody and its
            % constituent bodies using other commands. Syntax:
            %
            %   mbody = unSolve(mbody)
            %   mbody = mbody.unSolve
            %
            % See also SOLVE.
            
            if mbod.solved
                warning('MBODY:unSolve:dontDoThat', ...
                    'Manually unsolving an mBody is usually not a good idea.')
                warning('MBODY:unSolve:status', ...
                    'mBody object UNSOLVED.')
                
                mbod.solved = false;
            end
        end
        
        %% VIRTUALDISP
        function w = virtualDisp(mbod)
            % VIRTUALDISP returns the virtual displacement vectors of the
            % mBody at its current state. These are the directions in which
            % the system can move (at the current time). Syntax:
            %
            %   w = mbod.virtualDisp
            %
            
            [A, junk] = mbod.constraints;
            w = null(A);
        end
        
        %% VIRTUALWORK
        function dW = virtualWork(mbod)
            % VIRTUALWORK returns the total virtual work on the mBody at
            % its current state. It is a measure of how much force is
            % acting on the mBody in directions that are compatible with
            % the system constraints. Syntax:
            %
            %   W = mbod.virtualWork
            %
            
            [junk, F] = mbod.massforce;
            w = mbod.virtualDisp;
            dW = norm(w'*F);
        end
        
        
    end %methods
    
    %     events
    %
    %     end %events
end

%   /\\\\\\         % QuIRK :: Quaternion-state Interface
%  /  \\\\\\        %                               for Rigid-body Kinetics
%  \  /####/        %
%   \/####/         %         A multibody dynamics package for Matlab 2009+
%         o_____    %
%         |\    \   % Author: Joseph Shoer
%         | \____\  %         Space Systems Design Studio
%         \ |    |  %         Cornell University
%          \|____|  %         jps87 (at) cornell (dot) edu
%
% QuIRK is an interactive Matlab command line interface for constructing
% systems of rigid bodies and simple joint constraints, solving the
% equations of motion of those systems when subject to various force
% expressions, displaying and animating solved systems, and extracting
% information about the state history and energetics of those systems.
%
% QuIRK uses the Udwadia-Kalaba pseudoinverse method for constructing
% equations of motion for constrained systems, augmented for singular mass
% matrices. For details, see:
%
%   Udwadia, F. and Kalaba, R. Analytical Dynamics. Cambridge University
%       Press, 1996.
%
%   Udwadia, F. and Phohomsiri, P. "Explicit equations of motion for
%       constrained mechanical systems with singular mass matrices and
%       applications to multi-body dynamics." Proc. R. Soc. A, vol. 462,
%       2006, p. 2097-2117.
%
% mBody class
%   Container class for bodies/joints/forces/sensors, for executing the
%   Udwadia-Kalaba solution of multibody systems and animating results.


% M-Lint messages to ignore
%#ok<*CTCH>
%#ok<*PROP>