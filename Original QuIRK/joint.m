classdef joint < hgsetget
    properties
        type = 'sph';
        axis = [0 0 1]';
        bodies;
        hardpts = [0 0; 0 0; 0 0];
        custA = [];
        custb = [];
        data
    end
    
    properties (SetAccess = 'private')
        tri = [];
    end
    
    properties (Access = 'private')
        h = [];
    end %properties
    
    methods

        %% A
        function Amat = A(jnt)
            % A returns the instantaneous joint constraint matrix 
            % for the Udwadia-Kalaba equations of motion. It determines the 
            % matrix numerically, based on the states of the joined bodies. 
            % Syntax:
            %
            %   A = joint.A
            %   A = A(joint)
            %
            % See also B.
            
            % Form intermediate matrices
            qi = jnt.bodies{1}.att; qj = jnt.bodies{2}.att;
            ci = jnt.hardpts(:,1); cj = jnt.hardpts(:,2);
            Ti = 2*[(qi(4)*eye(3) - crs(qi(1:3))) -qi(1:3)];
            Tj = 2*[(qj(4)*eye(3) - crs(qj(1:3))) -qj(1:3)];
            NQi = q2d(qi); NQj = q2d(qj);
            
            switch jnt.type
                case 'rev'
                    A_NQi1 = -crs( NQi * (ci + jnt.axis) ) * NQi * Ti;   
                    A_NQj1 = crs( NQj * (cj + NQj'*NQi*jnt.axis) ) * NQj * Tj;
                    A_NQi2 = -crs( NQi * (ci - jnt.axis) ) * NQi * Ti;   
                    A_NQj2 = crs( NQj * (cj - NQj'*NQi*jnt.axis) ) * NQj * Tj;
                    
                    % Create constraint matrix
                    Amat = [eye(3)      A_NQi1       -eye(3)     A_NQj1;
                        eye(3)      A_NQi2       -eye(3)     A_NQj2;];
                    
                case 'trn'
                    a1 = jnt.tri(:,2); a2 = jnt.tri(:,3);
                    ri = jnt.bodies{1}.pos; rj = jnt.bodies{2}.pos;
                    
%                     Amat = [...
%                         (NQi*a1)'  (NQi*a1)'*crs(NQi*ci)*NQi*Ti	-(NQi*a1)'  -(NQi*a1)'*crs(NQj*cj)*NQj*Tj;
%                         (NQi*a2)'  (NQi*a2)'*crs(NQi*ci)*NQi*Ti	-(NQi*a2)'  -(NQi*a2)'*crs(NQj*cj)*NQj*Tj;
%                         zeros(3)    NQi*Ti                  zeros(3)    -NQj*Tj ...
%                         ];
                    Amat = [...
                        ...(NQi*a1)'   (NQi*a1)'*(-crs(NQi*ci)*NQi*Ti) -(NQi*a1)'   (NQi*a1)'*crs(NQj*cj)*NQj*Tj;
                        ...(NQi*a2)'   (NQi*a2)'*(-crs(NQi*ci)*NQi*Ti) -(NQi*a2)'   (NQi*a2)'*crs(NQj*cj)*NQj*Tj;
                        ...
                        ...(NQi*a1)'   (-(ri+NQi*ci-rj-NQj*cj)'*crs(NQi*a1)*NQi*Ti - (NQi*a1)'*crs(NQi*ci)*NQi*Ti) -(NQi*a1)'  (NQi*a1)'*crs(NQj*cj)*NQj*Tj;
                        ...(NQi*a2)'   (-(ri+NQi*ci-rj-NQj*cj)'*crs(NQi*a2)*NQi*Ti - (NQi*a2)'*crs(NQi*ci)*NQi*Ti) -(NQi*a2)'  (NQi*a2)'*crs(NQj*cj)*NQj*Tj;
                        ...
                        crs(NQi*jnt.axis)   -(crs(NQi*jnt.axis)*crs(NQi*ci)-crs(ri-rj+NQi*ci-NQj*cj)*crs(NQi*jnt.axis))*NQi*Ti  -crs(NQi*jnt.axis)  crs(NQi*jnt.axis)*crs(NQj*cj)*NQj*Tj;
                        zeros(3)    NQi*Ti                  zeros(3)    -NQj*Tj ...
                        ];
%                 case 'cyl'
%                     a1 = jnt.tri(:,2); a2 = jnt.tri(:,3);
%                     
%                     % Portions of A matrix freezing out rotations
%                     % other than those about the joint axis
%                     A_axQi = NQj' * crs( NQi * jnt.axis ) * Ti;
%                     A_axQj = NQj' * crs( NQi * jnt.axis ) * Tj;
%                     
%                     Amat = [...
%                         (NQi*a1)'   (NQi*a1)'*crs(NQi*ci)*Ti    -(NQi*a1)'  -(NQi*a1)'*crs(NQj*cj)*Tj;
%                         (NQi*a2)'   (NQi*a2)'*crs(NQi*ci)*Ti    -(NQi*a2)'  -(NQi*a2)'*crs(NQj*cj)*Tj;
%                         zeros(3)    A_axQi                      zeros(3)    -A_axQj ...
%                         ];
                    
                case 'sph'
                    % Portion of A matrix associated with rotation matrix N_Q_i
                    %A_NQi = crs( NQi * ci ) * Ti;
                    A_NQi = -crs( NQi * (ci) ) * NQi * Ti;    
                    % Portion of A matrix associated with rotation matrix N_Q_j
                    %A_NQj = crs( NQj * cj ) * Tj;
                    A_NQj = crs( NQj * (cj) ) * NQj * Tj;
                    
                    % Create constraint matrix
                    Amat = [eye(3)      A_NQi       -eye(3)     A_NQj];
                case 'fix'
                    % Portion of A matrix associated with rotation matrix N_Q_i
                    A_NQi = -crs( NQi * ci ) * NQi * Ti;
                    
                    % Portion of A matrix associated with rotation matrix N_Q_j
                    A_NQj = crs( NQj * cj ) * NQj * Tj;
                    
                    
                    % Create constraint matrix
                    Amat = [eye(3)	A_NQi	-eye(3)     A_NQj;
                        zeros(3)	NQi*Ti	zeros(3)    -NQj*Tj];
                case 'gnd'
                    Amat = [eye(3) zeros(3,4) -eye(3) zeros(3,4);
                        zeros(4,3) eye(4) zeros(4,3) -eye(4)];
                case 'fre'
                    Amat = zeros(1, 14);
                otherwise % Custom joint
                    if isnumeric(jnt.custA)
                        Amat = jnt.custA;
                    else
                        if nargin(jnt.custA) == 10
                            ri = jnt.bodies{1}.pos; rj = jnt.bodies{2}.pos;
                            rid = jnt.bodies{1}.vel; rjd = jnt.bodies{2}.vel;
                            qid = jnt.bodies{1}.qdot; qjd = jnt.bodies{2}.qdot;
                            
                            Amat = jnt.custA(ri, rj, qi, qj, ...
                                rid, rjd, qid, qjd, ci, cj);
                        elseif nargin(jnt.custA) == 3
                            Amat = jnt.custA(jnt, jnt.bodies{:});
                        end
                    end
            end
        end

        %% B
        function bvec = b(jnt)
            % B returns the instantaneous joint constraint vector 
            % for the Udwadia-Kalaba equations of motion. It determines the 
            % vector numerically, based on the states of the joined bodies. 
            % Syntax:
            %
            %   b = joint.b
            %   b = b(joint)
            %
            % See also A.
            
            % Form intermediate matrices
            qi = jnt.bodies{1}.att; qj = jnt.bodies{2}.att;
            qid = jnt.bodies{1}.qdot; qjd = jnt.bodies{2}.qdot;
            ci = jnt.hardpts(:,1); cj = jnt.hardpts(:,2);
            Ti = 2*[(qi(4)*eye(3) - crs(qi(1:3))) -qi(1:3)];
            Tj = 2*[(qj(4)*eye(3) - crs(qj(1:3))) -qj(1:3)];
            Tid = 2*[(qid(4)*eye(3) - crs(qid(1:3))) -qid(1:3)];
            Tjd = 2*[(qjd(4)*eye(3) - crs(qjd(1:3))) -qjd(1:3)];
            NQi = q2d(qi); NQj = q2d(qj);
            omi = Ti*qid;   omj = Tj*qjd;
            
            switch jnt.type
                case 'rev'
                    % These should be like two ball joints
                    % Portion of b matrix associated with rotation matrix N_Q_i
                    b_NQi1 = -( crs(NQi*omi) )^2 * NQi*(ci+jnt.axis) + crs(NQi*(ci+jnt.axis))*NQi*Tid*qid;
                    b_NQi2 = -( crs(NQi*omi) )^2 * NQi*(ci-jnt.axis) + crs(NQi*(ci-jnt.axis))*NQi*Tid*qid;
                    
                    % Portion of b matrix associated with rotation matrix N_Q_j
                    b_NQj1 = -( crs(NQj*omj) )^2 * NQj*(cj+NQj'*NQi*jnt.axis) + crs(NQj*(cj+NQj'*NQi*jnt.axis))*NQj*Tjd*qjd;
                    b_NQj2 = -( crs(NQj*omj) )^2 * NQj*(cj-NQj'*NQi*jnt.axis) + crs(NQj*(cj-NQj'*NQi*jnt.axis))*NQj*Tjd*qjd;
            
                    % Create constraint matrix
                    bvec = [b_NQi1 - b_NQj1;
                        b_NQi2 - b_NQj2];
                case 'trn'
                    a1 = jnt.tri(:,2); a2 = jnt.tri(:,3);
                    ri = jnt.bodies{1}.pos; rj = jnt.bodies{2}.pos;
%                     
%                     bvec = [...
%                         -(NQi*a1)'*( (crs(NQi*Ti*qid))^2 * NQi*ci + crs(NQi*ci)*NQi*Tid*qid - (crs(NQj*Tj*qjd))^2 * NQj*cj - crs(NQj*cj)*NQj*Tjd*qjd );
%                         -(NQi*a2)'*( (crs(NQi*Ti*qid))^2 * NQi*ci + crs(NQi*ci)*NQi*Tid*qid - (crs(NQj*Tj*qjd))^2 * NQj*cj - crs(NQj*cj)*NQj*Tjd*qjd );
%                         zeros(3,1) ...
%                         ];
                    rid = jnt.bodies{1}.vel; rjd = jnt.bodies{2}.vel;
                    bvec = [...
                        ...-(crs(NQi*omi)*NQi*a1)'*(rid-rjd+crs(NQi*omi)*NQi*ci-crs(NQj*omj)*NQj*cj) - (NQi*a1)'*(-crs(NQi*ci)*NQi*Tid*qid + crs(NQi*omi)^2*NQi*ci + crs(NQj*cj)*NQj*Tjd*qjd - crs(NQj*omj)^2*NQj*cj);
                        ...-(crs(NQi*omi)*NQi*a2)'*(rid-rjd+crs(NQi*omi)*NQi*ci-crs(NQj*omj)*NQj*cj) - (NQi*a2)'*(-crs(NQi*ci)*NQi*Tid*qid + crs(NQi*omi)^2*NQi*ci + crs(NQj*cj)*NQj*Tjd*qjd - crs(NQj*omj)^2*NQj*cj);
                        ...
                        ...-(ri+NQi*ci-rj-NQj*cj)'*(-crs(NQi*a1)*NQi*Tid*qid + crs(NQi*omi)^2*NQi*a1) - 2*(crs(NQi*omi)*NQi*a1)'*(rid-rjd+crs(NQi*omi)*(NQi*ci-NQj*cj)) - (NQi*a1)'*(-crs(NQi*ci)*NQi*Tid*qid + crs(NQj*cj)*NQj*Tjd*qjd + crs(NQj*omj)^2*(NQi*ci-NQj*cj));
                        ...-(ri+NQi*ci-rj-NQj*cj)'*(-crs(NQi*a2)*NQi*Tid*qid + crs(NQi*omi)^2*NQi*a2) - 2*(crs(NQi*omi)*NQi*a2)'*(rid-rjd+crs(NQi*omi)*(NQi*ci-NQj*cj)) - (NQi*a2)'*(-crs(NQi*ci)*NQi*Tid*qid + crs(NQj*cj)*NQj*Tjd*qjd + crs(NQj*omj)^2*(NQi*ci-NQj*cj));
                        -crs(ri-rj+NQi*ci-NQj*cj)*(crs(NQi*omi)^2*NQi*jnt.axis - crs(NQi*jnt.axis)*NQi*Tid*qid) + 2*crs(crs(NQi*omi)*NQi*jnt.axis)*(crs(NQi*omi)*NQi*ci-crs(NQj*omj)*NQj*cj+rid-rjd) + crs(NQi*jnt.axis)*(crs(NQi*omi)^2*NQi*ci-crs(NQj*omj)^2*NQj*cj-crs(NQi*ci)*NQi*Tid*qid+crs(NQj*cj)*NQj*Tjd*qjd);
                        ...
                        NQj*Tjd*qjd - NQi*Tid*qid...zeros(3,1) ...
                        ];
%                 case 'cyl'
%                     a1 = jnt.tri(:,2); a2 = jnt.tri(:,3);
%                     
%                     % Portion of b matrix to freeze out rotations other
%                     % than about the joint axis
%                     b_ax = NQj' * ( ...
%                         (crs(Tj*qjd))^2 - (crs(Ti*qid))^2 - ...
%                         2*crs(Tj*qjd)*crs(Ti*qid) - crs(Tid*qid - Tjd*qjd) ...
%                         ) * NQi * jnt.axis;
%                     
%                     bvec = [...
%                         -(NQi*a1)'*( (crs(Ti*qid))^2 * NQi*ci + crs(NQi*ci)*Tid*qid - (crs(Tj*qjd))^2 * NQj*cj - crs(NQj*cj)*Tjd*qjd );
%                         -(NQi*a2)'*( (crs(Ti*qid))^2 * NQi*ci + crs(NQi*ci)*Tid*qid - (crs(Tj*qjd))^2 * NQj*cj - crs(NQj*cj)*Tjd*qjd );
%                         b_ax ...
%                         ];
                case 'sph'
                    % Portion of b matrix associated with rotation matrix N_Q_i
                    %b_NQi = -( crs(Ti*qid) )^2 * NQi*ci - crs(NQi*ci)*Tid*qid;
                    b_NQi = -( crs(NQi*omi) )^2 * NQi*(ci) + crs(NQi*ci)*NQi*Tid*qid;
                    
                    % Portion of b matrix associated with rotation matrix N_Q_j
                    %b_NQj = -( crs(Tj*qjd) )^2 * NQj*cj - crs(NQj*cj)*Tjd*qjd;
                    b_NQj = -( crs(NQj*omj) )^2 * NQj*(cj) + crs(NQj*cj)*NQj*Tjd*qjd;
                    
                    % Create constraint matrix
                    bvec = b_NQi - b_NQj;
                case 'fix'
                    % Portion of b matrix associated with rotation matrix N_Q_i
                    b_NQi = -( crs(NQi*omi) )^2 * NQi*(ci) + crs(NQi*ci)*NQi*Tid*qid;
                    
                    % Portion of b matrix associated with rotation matrix N_Q_j
                    b_NQj = -( crs(NQj*omj) )^2 * NQj*(cj) + crs(NQj*cj)*NQj*Tjd*qjd;
                    
                    b_rot = NQj*Tjd*qjd - NQi*Tid*qid;
                    
                    % Create constraint matrix
                    bvec = [b_NQi - b_NQj;
                        b_rot];
                case 'gnd'
                    bvec = zeros(7,1);
                case 'fre'
                    bvec = 0;
                otherwise % Custom joint
                    if isnumeric(jnt.custb)
                        bvec = jnt.custb;
                    else
                        if nargin(jnt.custb) == 10
                            ri = jnt.bodies{1}.pos; rj = jnt.bodies{2}.pos;
                            rid = jnt.bodies{1}.vel; rjd = jnt.bodies{2}.vel;
                            
                            bvec = jnt.custb(ri, rj, qi, qj, ...
                                rid, rjd, qid, qjd, ci, cj);
                        elseif nargin(jnt.custb) == 3
                            bvec = jnt.custb(jnt, jnt.bodies{:});
                        end
                    end
            end
        end
        
        %% CONSTRUCTOR
        function jnt = joint(body1, body2, varargin)
            % JOINT creates a joint object between bodies for multibody
            % simulations. Syntax:
            %
            %   jnt = joint(body1, body2, 'property', value, ...)
            %
            % Required arguments
            %
            %   body1:  First body to join
            %   body2:  Second body to join or 'ground' (overwrites all
            %               other joint options)
            %
            % Optional properties
            %
            %   type    str     [ {sphere} | sph | ball
            %                       | revolute | rev | hinge
            %                       | translation | trn | prismatic | slide
            %                       | fixed | fix
            %                       | free | fre | none
            %                       | custom ]
            %   axis    dbl     Joint axis in BODY 1 body coordinates
            %                       (defaults to [0 0 1])
            %   pt1     dbl     Joint attachment point on body 1 in body 1
            %                       coordinates
            %   pt2     dbl     Joint attachment point on body 2 in body 2
            %                       coordinates
            %   A       dbl     Nx14 constraint matrix or function handle, 
            %                       required for custom joint type. 
            %                       Overwrites joint type with 'custom'.
            %                       Function handles take arguments 
            %                       (ri,rj,qi,qj,rid,rjd,qid,qjd,ci,cj) or
            %                       (thisJoint, body1, body2).
            %   b       dbl     Nx1 constraint vector or function handle,
            %                       required for custom joint type.
            %                       Overwrites joint type with 'custom'.
            %                       Function handles take arguments 
            %                       (ri,rj,qi,qj,rid,rjd,qid,qjd,ci,cj) or
            %                       (thisJoint, body1, body2).
            %   data    any     User-defined data field
            %
            % See also BODY, FORCE, SENSOR, MBODY.
                
            %% --Parse inputs
            if nargin == 1 && isa(body1, 'joint')  % --If given just a joint, return that joint
                jnt = body1;
            elseif nargin < 2 && ~isa(body1, 'joint')
                error('JOINT:joint:input', 'Joint constructor requires two bodies.')
            elseif nargin >= 2   % --If given arguments, construct the specified joint
                % Bodies
                if ischar(body2) && strncmpi(body2, 'g', 1)
                    body2 = body;
                    jnt.type = 'gnd';
                end
                if ~isa(body1, 'body')
                    error('JOINT:joint:input', 'First argument must be a body.')
                elseif ~isa(body2, 'body')
                    error('JOINT:joint:input', 'Second argument must be a body.')
                end
                jnt.bodies = {body1 body2};
                
                if strcmp( jnt.type, 'gnd' )
                    return
                end

                % Options
                if numel(varargin)/2 ~= round(numel(varargin)/2)
                    error('JOINT:joint:input', 'All properties must be paired with values.')
                end
                for i = 1:2:(length(varargin))
                    if ~ischar(varargin{i})
                        error('JOINT:joint:input', 'Property designations must be strings.')
                    end
                    switch lower(varargin{i})
                        case {'type' 't'}
                            if ~ischar(varargin{i+1})
                                error('JOINT:joint:input', 'Type must be a string.')
                            else
                                switch lower(varargin{i+1})
                                    case {'sphere' 'sph' 'ball'}
                                        jnt.type = 'sph';
                                    case {'revolute' 'rev' 'hinge'}
                                        jnt.type = 'rev';
                                    case {'translation' 'translational' ...
                                            'trans' 'prismatic' 'prism' ...
                                            'slider' 'slide' 'trn'} 
                                        jnt.type = 'trn';
%                                     case {'cylindrical' 'cylinder' 'cyl'}
%                                         jnt.type = 'rev';
                                    case {'custom' 'cust' ...
                                            'specify' 'specified' 'spec'}
                                        jnt.type = 'cst';
                                    case {'fixed' 'fix'}
                                        jnt.type = 'fix';
                                    case {'free' 'fre' 'none' 'non'}
                                        jnt.type = 'fre';
                                    otherwise
                                        error('JOINT:joint:input', 'Unknown joint type')
                                end
                            end
                        case {'axis' 'ax'}
                            if ~isnumeric(varargin{i+1})
                                error('JOINT:joint:input', 'Axis must be numeric.')
                            elseif all(size(varargin{i+1}) == [1 3])
                                varargin{i+1} = varargin{i+1}';
                            elseif numel(varargin{i+1}) ~= 3
                                error('JOINT:joint:input', 'Axis must be a 3-element vector.')
                            elseif all(varargin{i+1} == 0)
                                error('JOINT:joint:input', 'Axis must be nonzero.')
                            end
                            jnt.axis = varargin{i+1}/norm(varargin{i+1});
                        case {'pt1' 'point1' 'hardpt1' 'hardpoint1'}
                            if ~isnumeric(varargin{i+1})
                                error('JOINT:joint:input', 'Point1 must be numeric.')
                            elseif all(size(varargin{i+1}) == [1 3])
                                varargin{i+1} = varargin{i+1}';
                            elseif numel(varargin{i+1}) ~= 3
                                error('JOINT:joint:input', 'Point1 must be a 3-element vector.')
                            end
                            jnt.hardpts(:,1) = varargin{i+1};
                        case {'pt2' 'point2' 'hardpt2' 'hardpoint2'}
                            if ~isnumeric(varargin{i+1})
                                error('JOINT:joint:input', 'Point2 must be numeric.')
                            elseif all(size(varargin{i+1}) == [1 3])
                                varargin{i+1} = varargin{i+1}';
                            elseif numel(varargin{i+1}) ~= 3
                                error('JOINT:joint:input', 'Point2 must be a 3-element vector.')
                            end
                            jnt.hardpts(:,2) = varargin{i+1};
                        case 'a'
                            jnt.custA = varargin{i+1};
                        case 'b'
                            jnt.custb = varargin{i+1};
                        case {'data' 'dat' 'd'}
                            jnt.data = varargin{i+1};
                        otherwise
                            warning('JOINT:joint:input', ...
                                'Discarding unknown property "%s"', varargin{i})
                    end
                end
                
                %if all(jnt.type == 'trn') || all(jnt.type == 'cyl')
                    [a b c] = triad(jnt.axis);
                    jnt.tri = [a b c];
                %end
                
            else    % --Construct object with default properties

            end

            %% --Check constraints for consistency
            if ~isempty(jnt.custA) || ~isempty(jnt.custb)
                jnt.type = 'cst';
            end
            if strcmp(jnt.type, 'cst') && (isempty(jnt.custA) || isempty(jnt.custb))
                error('JOINT:joint:input', 'For a custom joint type, you must specify A and b.')
            end
            if (isnumeric(jnt.custA) && isnumeric(jnt.custb)) && ...
                    (size(jnt.custA, 1) ~= size(jnt.custb, 1))
                error('JOINT:joint:input', 'A and b matrices must have the same number of rows.')
            end
            
            if any((body1.pos + q2d(body1.att)*jnt.hardpts(:,1) ...
                    - body2.pos - q2d(body2.att)*jnt.hardpts(:,2)) > eps)
                warning('JOINT:joint:alignment', 'Hardpoints on bodies do not line up.')
            end
        end
        
        %% DELETE
        function delete(jnt)
            % DELETE performs clean-up actions before clearing a joint from 
            % memory.
            erase(jnt);
        end
        
        %% DISPLAY
        function display(jnt)
            % DISPLAY handles the printout of a joint object when someone
            % types its name without ending the line in a semicolon.
            disp([inputname(1) ' = '])
            disp(['    QuIRK: ' jnt.type 'joint'])
            disp(' ');
        end

        %% DRAW
        function h = draw(jnt, varargin)
            % DRAW adds a graphic of a joint to the current axes.
            % Syntax:
            %
            %   h = draw(joint, 'property', value, ...)
            %   h = joint.draw('property', value, ...)
            %
            %   joint:   joint to draw
            %
            % Optional properties
            %
            %   color   dbl/str RGB or Matlab string specifying color of joint
            %   snap    lgc     Graphic will not update as joint changes if
            %                       this flag is true
            % 
            %   Any line property will also work.
            %
            % See also ERASE, MBODY\DRAW.
            
            %% --Set defaults
            color = 'm';
            over = false;
            snap = false;
            
            %% --Parse inputs
            plotargs = true(size(varargin));
            if numel(varargin)/2 ~= round(numel(varargin)/2)
                error('JOINT:draw:input', 'All properties must be paired with values.')
            end
            for i = 1:2:(length(varargin))
                if ~ischar(varargin{i})
                    error('JOINT:draw:input', 'Property designations must be strings.')
                end
                switch lower(varargin{i})
                    case 'snap'
                        snap = varargin{i+1}(1);
                        plotargs(i:(i+1)) = [false false];
                    case 'alpha'
                        plotargs(i:(i+1)) = [false false];
                end
            end
            
            %% --Create line points
            Q = q2d(jnt.bodies{1}.att);
            switch jnt.type
                case 'rev'
                    style = '-.';
                    s = max([jnt.bodies{1}.sx jnt.bodies{1}.sy jnt.bodies{1}.sz]);
                    r1 = jnt.bodies{1}.pos + Q*jnt.hardpts(:,1) - s*Q*jnt.axis;
                    r2 = jnt.bodies{1}.pos + Q*jnt.hardpts(:,1) + s*Q*jnt.axis;
                    X = [r1(1); r2(1)]; Y = [r1(2); r2(2)]; Z = [r1(3); r2(3)];
                case 'trn'
                    style = '-';
                    s = max([jnt.bodies{1}.sx jnt.bodies{1}.sy jnt.bodies{1}.sz]);
                    r1 = jnt.bodies{1}.pos + Q*jnt.hardpts(:,1) - 2*s*Q*jnt.axis;
                    r2 = jnt.bodies{1}.pos + Q*jnt.hardpts(:,1) + 2*s*Q*jnt.axis;
                    X = [r1(1)+0.05; r2(1)-0.05]; 
                    Y = [r1(2)+0.05; r2(2)-0.05]; 
                    Z = [r1(3)+0.05; r2(3)-0.05];
                case 'cyl'
                    style = '-';
                    s = max([jnt.bodies{1}.sx jnt.bodies{1}.sy jnt.bodies{1}.sz]);
                    r1 = jnt.bodies{1}.pos + Q*jnt.hardpts(:,1) - s*Q*jnt.axis;
                    r2 = jnt.bodies{1}.pos + Q*jnt.hardpts(:,1) + s*Q*jnt.axis;
                    X = [r1(1); r2(1)]; Y = [r1(2); r2(2)]; Z = [r1(3); r2(3)];
                case {'cst' 'fre'}
                    style = 's';
                    r = jnt.bodies{1}.pos + Q*jnt.hardpts(:,1);
                    X = r(1); Y = r(2); Z = r(3);
                case {'fix' 'gnd'}
                    style = 'x';
                    r = jnt.bodies{1}.pos + Q*jnt.hardpts(:,1);
                    X = r(1); Y = r(2); Z = r(3);
                otherwise %sphere
                    style = 'o';
                    Q1 = q2d(jnt.bodies{1}.att);
                    Q2 = q2d(jnt.bodies{2}.att);
                    r = [(jnt.bodies{1}.pos + Q1*jnt.hardpts(:,1)), ...
                        (jnt.bodies{2}.pos + Q2*jnt.hardpts(:,2))];
                    X = r(1,:)'; Y = r(2,:)'; Z = r(3,:)';
            end
            
            %% --Plot line
            wasHeld = ishold;
            if over && ~wasHeld
                hold on
            end
            
            warning('off', 'MATLAB:hg:set_chk:MadeMarkerAndLineStyleSeparateProperties')
            h = line(X, Y, Z, 'color', color, 'LineStyle', style, ...
                'LineWidth', 2, 'MarkerSize', 14);%, varargin{plotargs});
            warning('on', 'MATLAB:hg:set_chk:MadeMarkerAndLineStyleSeparateProperties')
            if ~snap
                jnt.h = h;
            end
            
            if ~wasHeld
                hold off
            end
            
        end
        
        %% ERASE
        function erase(jnt)
            % ERASE clears the graphical representation of a joint from all
            % axes, if one exists. Syntax:
            %
            %   erase(joint)
            %   joint.erase
            %
            % See also DRAW.

            if ~isempty(jnt.h)
                try
                    delete(jnt.h);
                catch
                    % ignore errors
                end
                jnt.h = [];
            end
        end
        
        %% NUMCONSTRAINTS
        function num = numConstraints(jnt)
            % NUMCONSTRAINTS returns the number of constraint equations
            % represented by the joint object. Note that not all of these
            % constraints may be linearly independent, depending on how the
            % constraint matrices are expressed and the current body 
            % states. Syntax:
            %
            %   num = numConstraints(jnt)
            %   num = jnt.numContraints
            %
            % See also A, B.
            
            num = numel(jnt.b);
        end

        %% SET.TYPE
        function jnt = set.type(jnt, newType)
            if ~ischar(newType)
                error('JOINT:joint:input', 'Type must be a string.')
            else
                switch lower(newType)
                    case {'sphere' 'sph' 'ball'}
                        jnt.type = 'sph';
                    case {'revolute' 'rev' 'hinge'}
                        jnt.type = 'rev';
                    case {'translation' 'translational' ...
                            'trans' 'prismatic' 'prism' ...
                            'slider' 'slide' 'trn'}
                        jnt.type = 'trn';
                        [a b c] = triad(jnt.axis);
                        jnt.tri = [a b c];
%                     case {'cylindrical' 'cylinder' 'cyl'}
%                         jnt.type = 'cyl';
                    case {'custom' 'cust' ...
                            'specify' 'specified' 'spec'}
                        jnt.type = 'cst';
                        warning('JOINT:set:needCustom', ...
                            'Custom joint will require explicit A and b matrices.');
                    case {'fixed' 'fix'}
                        jnt.type = 'fix';
                    case {'free' 'fre' 'none' 'non'}
                        jnt.type = 'fre';
                    case {'ground' 'grd' 'gnd' 'gr' 'grnd' 'gd' 'g'}
                        jnt.type = 'gnd';
                    otherwise
                        error('JOINT:set:input', 'Unknown joint type')
                end
            end
            
            % Redraw
            if ~isempty(jnt.h)
                erase(jnt);
                draw(jnt, 'hold', true);
            end
        end

        %% SET.AXIS
        function jnt = set.axis(jnt, newAx)
            if ~isnumeric(newAx)
                error('JOINT:set:input', 'Axis must be numeric.')
            elseif all(size(newAx) == [1 3])
                newAx = newAx';
            elseif numel(newAx) ~= 3
                error('JOINT:set:input', 'Axis must be a 3-element vector.')
            elseif all(newAx == 0)
                error('JOINT:set:input', 'Axis must be nonzero.')
            end
            jnt.axis = newAx/norm(newAx);
            [a b c] = triad(jnt.axis);
            jnt.tri = [a b c];
        end

        %% SET.HARDPTS
        function jnt = set.hardpts(jnt, newPts)
            if ~isnumeric(newPts)
                error('JOINT:set:input', 'Hardpoints must be numeric.')
            elseif all(size(newPts) == [2 3])
                newPts = newPts';
            elseif all(size(newPts) ~= [3 2])
                error('JOINT:set:input', 'Hardpoints must be a in a 3x2 matrix.')
            end
            jnt.hardpts = newPts;
            
            if all((jnt.bodies{1}.pos + jnt.hardpts(:,1) ...
                    - jnt.bodies{2}.pos - jnt.hardpts(:,2)) > eps)
                warning('JOINT:set:alignment', 'Hardpoints on bodies do not line up.')
            end
        end

        %% SET.CUSTA
        function jnt = set.custA(jnt, newA)
            if ~isnumeric(newA) && ~isa(newA, 'function_handle')
                error('JOINT:set:input', 'A must be numeric or a function handle.')
            end
            if isnumeric(newA)
                if all(size(newA, 2) ~= 14)
                    error('JOINT:set:inputSize', 'Matrix A must have 14 columns.')
                end
            elseif isa(newA, 'function_handle') ...
                    && nargin(newA) ~= 10 && nargin(newA) ~= 3
                error('JOINT:set:inputArgs', ...
                    'Function A must have inputs (ri, rj, qi, qj, ri_dot, rj_dot, qi_dot, qj_dot, cij, cji)\nor (thisJoint, body1, body2).')
            end
            
            jnt.custA = newA;
        end

        %% SET.CUSTB
        function jnt = set.custb(jnt, newb)
            if ~isnumeric(newb) && ~isa(newb, 'function_handle')
                error('JOINT:set:input', 'b must be numeric or a function handle.')
            end
            if isnumeric(newb)
                if all(size(newb, 1) ~= 1)
                    error('JOINT:set:inputSize', 'Matrix b must have 1 column.')
                end
            elseif isa(newb, 'function_handle') ...
                    && nargin(newb) ~= 10 && nargin(newb) ~= 3
                error('JOINT:set:inputArgs', ...
                    'Function b must have inputs (ri, rj, qi, qj, ri_dot, rj_dot, qi_dot, qj_dot, cij, cji)\nor (thisJoint, body1, body2).')
            end
            
            jnt.custb = newb;
        end
        
    end %methods
    
%     events
%         
%     end %events
end

%  /\\\\\\        % QuIRK :: Quaternion-state Interface
% /  \\\\\\       %                                 for Rigid-body Kinetics
% \  /####/       %
%  \/####/        %         A multibody dynamics package for Matlab 2009+
%        o_____   %
%        |\    \  % Author: Joseph Shoer
%        | \____\ %         Space Systems Design Studio
%        \ |    | %         Cornell University 
%         \|____| %         jps87 (at) cornell (dot) edu
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
% Joint class
%   Objects describing constraints between rigid bodies.


% M-Lint messages to ignore
%#ok<*CTCH>