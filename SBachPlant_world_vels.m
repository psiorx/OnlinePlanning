classdef SBachPlant_world_vels < DrakeSystem
    % Defines the dynamics for the SBach rc plane
    % Jan 5, 2015.
    
    properties
    end
    
    methods
        function obj = SBachPlant_world_vels()
            obj = obj@DrakeSystem(12,0,4,12,0,1);
            obj = obj.setOutputFrame(obj.getStateFrame);
        end
        
        function xdot = dynamics(obj,t,x,u)
            %             % min/max protection here
            %             u(1) = min(850, max(150, u(1)));
            %             u(2) = min(850, max(155, u(2)));
            %             u(3) = min(850, max(180, u(3)));
            %             u(4) = min(860, max(160, u(4)));
            % @param t time
            % @param x state: x =
            %  Plane's X coordinate faces forward, Y to the right, and Z down.
            %  x(1):x    (Forward Position, Earth frame)
            %  x(2):y    (East or y position, Earth frame)
            %  x(3):z     (z position (down), Earth frame)
            %  x(4):phi   (roll angle)
            %  x(5):theta (pitch angle)
            %  x(6):psi   (yaw angle)
            %  x(7):xdot  (X-velocity, world frame)
            %  x(8):ydot  (Y-velocity, world frame)
            %  x(9):zdot  (Z-velocity, world frame)
            %  x(10):P    (Angular velocity in X direction, body frame)
            %  x(11):Q    (Angular velocity in Y direction, body frame)
            %  x(12):R    (Angular velocity in Z direction, body frame)
            %
            %  u(1):thr   (Throttle command)
            %  u(2):ail   (Aileron Command)
            %  u(3):elev  (Elevator Command)
            %  u(4):rud   (Rudder command)
            
            % Convert vels to model frame
            x = ConvertVelsToModelFrame(x);
            
            %% Parameters fit from data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Rotational inertias
            Jx_fac = 1.7837;
            Jy_fac =  4.2679;
            Jz_fac =  6.9248;
            
            
            % Throttle/propwash
            thr_to_sp_ail = 2.3940;
            thr_vel_fac_ail = 1;
            thr_to_sp_elev = 1.2296;
            thr_vel_fac_elev = 1;
            
            thr_fac = 1;
            
            % Elevator
            elev_lift_fac = 1;
            
            % Stabilizer
            stab_force_fac = 0.1;
            
            % Body drag
            body_x_drag_fac = 0;
            body_y_drag_fac = 0;
            body_z_drag_fac = 0;
            
            % Rate dependent force
            F_Q_fac_x = 0; %
            F_Q_fac_z = 0; %
            
            % Throttle aerodynamic drag
            thr_drag_fac = 0; % thr_drag_fac*1e-6;
            
            % Rate dependent moments
            M_P_fac = 0;
            M_Q_fac = 0;
            M_R_fac = 0;
            
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Rotational inertias
            Jx = Jx_fac*.0005; % The numbers are just guesses to get things in the right ball park
            Jy = Jy_fac*.0009;
            Jz = Jz_fac*.0004;
            
            J = diag([Jx,Jy,Jz]);
            invJ = diag([1/Jx,1/Jy,1/Jz]);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Measured parameters (MKS units):
            wing_area_out = 7848/(1000*1000);% m^2
            wing_area_in = (2.7885*10^3)/(1000*1000); % m^2
            wing_total_area = 2*wing_area_out + 2*wing_area_in;
            out_dist = 132.8/1000; % Moment arm of outer wing section
            in_dist = 46.4/1000; % Moment arm of inner wing section
            ail_out_dist_x = 41/1000; % These are behind the COM
            ail_in_dist_x = 65/1000; % These are behind the COM
            elev_area = 5406/(1000*1000); % m^2
            elev_arm = 240/1000;
            rudder_area = (3.80152*1000)/(1000*1000);
            rudder_arm = 258.36/1000; % m
            stab_area = 923.175/(1000*1000); % m^2
            stab_arm = 222/1000; % m
            m =  76.6/1000; % kg (with battery in)
            g = 9.81; % m/s^2
            thr_to_thrust = 0.1861; % grams per unit throttle command
            
            ail_comm_to_rad = 8.430084159809215e-04; % Multiply raw command by these to get deflection in rads
            elev_comm_to_rad = -0.001473692553866; % Sign is correct
            rud_comm_to_rad = -0.001846558348610; % Sign is correct
            
            ail_area_out = (4.5718848*10^3)/(1000*1000);
            ail_area_in = (1.51275*10^3)/(1000*1000);
            
            thr_min = 270; % If it's less than 270, prop doesn't spin
            
            rho = 1.1839; % kg/m^3 (density of air)
            
            throttle_trim = 250; % At 250, we have 0 propwash speed (according to fit from anemometer readings)
            ail_trim = 512;
            rud_trim = 512;
            elev_trim = 512;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Get states
            rpy = x(4:6);
            U = x(7);
            V = x(8);
            W = x(9);
            P = x(10);
            Q = x(11);
            R = x(12);
            
            R_body_to_world = rpy2rotmat(rpy);
            R_world_to_body = R_body_to_world';
            
            % COM velocity in world coordinates
            xdots_world = R_body_to_world*[U;V;W];
            
            % Angular velocity in world coordinate frame
            omega_world = R_body_to_world*[P;Q;R];
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Get control inputs
            
            %Throttle signal is 150-850
            thr = u(1) - throttle_trim; % Shift it
            if thr < (thr_min - throttle_trim)
                thr = 0;
            end
            % thr = thr_fac*max(thr,0); % Scale it and don't let it go negative
            
            % positive AilL is negative lift (front of aileron tips downwards)
            ail = (u(2)-ail_trim)*ail_comm_to_rad; % input in radians of deflection
            
            ailL = ail; % This is correct sign
            % Positive ailR is positive lift (front of aileron tips downwards)
            ailR = ail; % This is correct sign
            
            %positive elevator is deflection up (negative lift - makes plane
            %pitch up)
            elev = (u(3)-elev_trim)*elev_comm_to_rad; % input in radians of deflection
            
            %positive rudder is deflection to the right
            rud = (u(4)-rud_trim)*rud_comm_to_rad;% input in radians of deflection
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Now, the hard stuff
            
            % Do translataional stuff first
            
            % Speed of plane
            % vel = sqrt(U^2 + V^2 + W^2);
            velXZ = sqrt(U^2 + W^2);
            velXY = sqrt(U^2 + V^2);
            
            
            % Angle of attack
            alpha = atan2(W,U);
            
            
            %%Forces due to wings and ailerons%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Propwash over ailerons (0.2932 was fit from anemometer experiments)
            upa = sqrt((velXZ^2)/4 + thr_to_sp_ail*0.2932*thr) - velXZ/2; %
            
            uwa = thr_vel_fac_ail*sqrt(velXZ^2 + upa^2 + 2*velXZ*upa*cos(alpha));
            alpha_wa = atan2(W,U + upa);
            
            %Lift force generated by wing components. (flat plate)
            %lift = dynamic pressure * area * Coefficient of lift.
            left_wing_out_lift = pressure(velXZ) * wing_area_out * Cl_fp_fit(alpha);
            left_wing_in_lift = pressure(uwa) * wing_area_in * Cl_fp_fit(alpha_wa);
            right_wing_out_lift = pressure(velXZ) * wing_area_out * Cl_fp_fit(alpha);
            right_wing_in_lift = pressure(uwa) * wing_area_in * Cl_fp_fit(alpha_wa);
            
            %Lift force generated by ailerons. (flat plate)
            %lift = dynamic pressure * area * Coefficient of lift.
            left_ail_out_lift = pressure(velXZ) * ail_area_out * Cl_fp(alpha-ailL);
            left_ail_in_lift = pressure(uwa) * ail_area_in * Cl_fp(alpha_wa-ailL);
            right_ail_out_lift = pressure(velXZ) * ail_area_out * Cl_fp(alpha+ailR);
            right_ail_in_lift = pressure(uwa) * ail_area_in * Cl_fp(alpha_wa+ailR);
            
            %Drag force generated by wing components.
            left_wing_out_drag = pressure(velXZ) * wing_area_out * Cd_fp_fit(alpha);
            left_wing_in_drag = pressure(uwa) * wing_area_in * Cd_fp_fit(alpha_wa);
            right_wing_out_drag = pressure(velXZ) * wing_area_out * Cd_fp_fit(alpha);
            right_wing_in_drag = pressure(uwa) * wing_area_in * Cd_fp_fit(alpha_wa);
            
            %Drag force generated by ailerons.
            left_ail_out_drag = pressure(velXZ) * ail_area_out * Cd_fp(alpha-ailL);
            left_ail_in_drag = pressure(uwa) * ail_area_in * Cd_fp(alpha_wa-ailL);
            right_ail_out_drag = pressure(velXZ) * ail_area_out * Cd_fp(alpha+ailR);
            right_ail_in_drag = pressure(uwa) * ail_area_in * Cd_fp(alpha_wa+ailR);
            
            % Collect these forces and represent them in correct frame
            F_left_wing_out = rotAlpha([left_wing_out_drag;0;left_wing_out_lift],alpha);
            F_left_wing_in = rotAlpha([left_wing_in_drag;0;left_wing_in_lift],alpha_wa);
            F_left_ail_out = rotAlpha([left_ail_out_drag;0;left_ail_out_lift],alpha);
            F_left_ail_in = rotAlpha([left_ail_in_drag;0;left_ail_in_lift],alpha_wa);
            
            F_right_wing_out = rotAlpha([right_wing_out_drag;0;right_wing_out_lift],alpha);
            F_right_wing_in = rotAlpha([right_wing_in_drag;0;right_wing_in_lift],alpha_wa);
            F_right_ail_out = rotAlpha([right_ail_out_drag;0;right_ail_out_lift],alpha);
            F_right_ail_in = rotAlpha([right_ail_in_drag;0;right_ail_in_lift],alpha_wa);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Forces due to elevator
            
            % Propwash over rudder/elevator (0.1444 was fit from anemometer
            % experiments)
            upe = sqrt((velXZ^2)/4 + thr_to_sp_elev*0.1444*thr) - velXZ/2;
            
            % Elevator position and velocity
            % Velocity of elevator in world coordinate frame
            xdot_elev = xdots_world + cross(omega_world,R_body_to_world*[-elev_arm;0;0]);
            xdot_elev_body = R_world_to_body*xdot_elev;
            
            alpha_elev = atan2(xdot_elev_body(3),xdot_elev_body(1));
            
            uwe = thr_vel_fac_elev*sqrt(velXZ^2 + upe^2 + 2*velXZ*upe*cos(alpha_elev));
            alpha_we = atan2(xdot_elev_body(3),xdot_elev_body(1) + upe);
            
            %include lift terms from flat plate theory of elevator
            elev_lift = elev_lift_fac*pressure(uwe) * elev_area * ... likely a small term
                Cl_fp(alpha_we-elev); %angle of deflection of Elevator
            
            % NOTE: Do we need separate inner/outer terms for elevator too? (I decided
            % not to do this - I think that's reasonable)
            
            %Compute drag on elevator using flat plate theory
            elev_drag = pressure(uwa) * elev_area * Cd_fp(alpha_we-elev);
            
            % Rotate to correct frame
            F_elev = rotAlpha([elev_drag;0;elev_lift],alpha_we); % elevator
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Rudder and stabilizer
            
            %Stabilizing force from aircraft yawing (this is a crude approximation)
            F_stabilizer = [0;-stab_force_fac*sign(R) * pressure(R*stab_arm) * (stab_area+rudder_area);0];
            
            % Rudder position and velocity
            % Velocity of rudder in world coordinate frame
            xdot_rudder = xdots_world + cross(omega_world,R_body_to_world*[-rudder_arm;0;-9/1000]);
            xdot_rudder_body = R_world_to_body*xdot_rudder;
            
            velXY_rud = sqrt(xdot_rudder_body(1)^2 + xdot_rudder_body(2)^2);
            V_rud = xdot_rudder_body(2);
            
            % Sideslip angle for rudder
            beta_rud = atan2(V,sqrt(velXY_rud^2-V_rud^2) + upe); % Assuming propwash over rudder is same as elevator
            
            % Rudder force flat plate
            rudder_force_l = pressure(uwe) * rudder_area * Cl_fp(beta_rud + rud);
            rudder_force_d = pressure(uwe) * rudder_area * Cd_fp(beta_rud + rud);
            
            rudder_force = [rudder_force_d;rudder_force_l;0];
            
            % Rotate to correct frame
            F_rudder = [-cos(beta_rud), sin(beta_rud), 0; -sin(beta_rud), -cos(beta_rud), 0;0 0 0]*rudder_force;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Gravity, thrust, and body drag
            F_gravity = R_world_to_body*[0;0;m*g];
            
            F_thrust = [thr_fac*thr_to_thrust*thr*g/1000;0;0]; % thr_to_thrust was estimated from digital scale experiments
            
            % Drag due to body (in x-direction)
            body_drag_x =  body_x_drag_fac*pressure(U);
            
            % Drag due to body in y-direction
            body_drag_y = body_y_drag_fac * pressure(V) * wing_total_area; % Just a rough initial estimate
            
            % Drag due to wing in z-direction
            body_drag_z = body_z_drag_fac * pressure(W) * wing_total_area;
            
            F_body_drag = [body_drag_x;-sign(V)*body_drag_y;-sign(W)*body_drag_z];
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Additional angular rate dependent forces
            F_rate_dependent = [F_Q_fac_x*Q;0;F_Q_fac_z*Q]; % These should be small effects hopefully
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Now moments/rotational stuff
            
            % Moment from wings and ailerons
            d_left_wing_out = [0;-out_dist;0];
            d_left_wing_in = [0;-in_dist;0];
            d_left_ail_out = [-ail_out_dist_x;-out_dist;0];
            d_left_ail_in = [-ail_in_dist_x;-in_dist;0];
            
            d_right_wing_out = [0;out_dist;0];
            d_right_wing_in = [0;in_dist;0];
            d_right_ail_out = [-ail_out_dist_x;out_dist;0];
            d_right_ail_in = [-ail_in_dist_x;in_dist;0];
            
            M_left_wing_out = cross(d_left_wing_out,F_left_wing_out);
            M_left_wing_in = cross(d_left_wing_in,F_left_wing_in);
            M_left_ail_out = cross(d_left_ail_out,F_left_ail_out);
            M_left_ail_in = cross(d_left_ail_in,F_left_ail_in);
            
            M_right_wing_out = cross(d_right_wing_out,F_right_wing_out);
            M_right_wing_in = cross(d_right_wing_in,F_right_wing_in);
            M_right_ail_out = cross(d_right_ail_out,F_right_ail_out);
            M_right_ail_in = cross(d_right_ail_in,F_right_ail_in);
            
            % Moment from elevator
            d_elev = [-elev_arm;0;0];
            M_elev = cross(d_elev,F_elev);
            
            % Moment from stabilizer
            d_stabilizer = [-stab_arm;0;-35/1000]; % -35 mm in z approximately
            M_stabilizer = cross(d_stabilizer,F_stabilizer);
            
            % Moment from rudder
            d_rudder = [-rudder_arm;0;-9/1000]; % -9 mm in z approximately
            M_rudder = cross(d_rudder,F_rudder);
            
            % Moment from throttle aerodynamic drag
            M_throttle = [thr_drag_fac*thr;0;0];
            
            % Additional rate dependent moments
            M_rate_dependent = [sign(P)*M_P_fac*P^2;sign(Q)*M_Q_fac*Q^2;sign(R)*M_R_fac*R^2];
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Put equations together
            
            % Kinematics
            xyzdot = R_body_to_world*[U;V;W];
            Phi = angularvel2rpydotMatrix(rpy);
            rpydot = Phi*omega_world;
            
            % Dynamics
            % Translational equations
            F_total = F_left_wing_out + F_left_wing_in + F_left_ail_out + F_left_ail_in + ... % left wing
                F_right_wing_out + F_right_wing_in + F_right_ail_out + F_right_ail_in + ... % right wing
                F_elev + F_stabilizer + F_rudder + ... % elevator, stabilizer, rudder
                F_gravity + F_thrust + F_body_drag + F_rate_dependent; % gravity, thrust, body drag, rate dependent force
            
            Sw = [0 -R Q; R 0 -P;-Q P 0];
            UVW_dot = -Sw*[U;V;W] + F_total/m;
            
            % Rotational stuff
            M_total = M_left_wing_out + M_left_wing_in + M_left_ail_out + M_left_ail_in + ...
                M_right_wing_out + M_right_wing_in + M_right_ail_out + M_right_ail_in + ...
                M_elev + M_stabilizer + M_rudder + M_throttle + M_rate_dependent;
            PQR_dot = invJ*(M_total - cross([P;Q;R],J*[P;Q;R]));
            
            
            xdot = [xyzdot;rpydot;UVW_dot;PQR_dot];
            
            % Convert xdot velocities to world coordinate frame
            xdot = ConvertVelsToWorldCoords(xdot, R_body_to_world, rpy, [U;V;W]);
            
            %% Helper functions
            function cl = Cl_fp(a) % Flat plate model
                if a > pi/2
                    a = pi/2;
                elseif a<-pi/2
                    a = -pi/2;
                end
                
                cl = 2*sin(a)*cos(a);
                
            end
            
            function cd = Cd_fp(a) % Flat plate
                if a > pi/2
                    a = pi/2;
                elseif a<-pi/2
                    a = -pi/2;
                end
                
                cd = 2*(sin(a)^2);
                
            end
            
            function cl = Cl_fp_fit(a) % Flat plate model with correction terms
                if a > pi/2
                    a = pi/2;
                elseif a<-pi/2
                    a = -pi/2;
                end
                
                % These numbers were fit from no-throttle experiments
                cl = 2*sin(a)*cos(a) + 0.6602*sin(2.8717*a);
                
            end
            
            function cd = Cd_fp_fit(a) % Flat plate with correction terms
                if a > pi/2
                    a = pi/2;
                elseif a<-pi/2
                    a = -pi/2;
                end
                
                % These numbers were fit from no-throttle experiments
                cd = 2*(sin(a)^2) + 0.8296*(sin(a)^2) + 0.1372;
                
            end
            
            function f = rotAlpha(f,a)
                % Rotation matrix
                Ra = [-cos(a) 0  sin(a); ...
                    0      0  0     ; ...
                    -sin(a) 0 -cos(a)];
                
                % Rotate f using Ra
                f = Ra*f;
            end
            
            function pre = pressure(vel) %Dynamic Pressure = .5 rho * v^2
                pre = .5 * 1.1839 * vel^2; % N/m^2
            end
            
            function xdot_world = ConvertVelsToWorldCoords(xdot, R_body_to_world, rpy, uvw)
                xyzdot0 = xdot(1:3);
                rpydot0 = xdot(4:6);
                UVW_dot0 = xdot(7:9);
                pqr_dot = xdot(10:12);
                phi = rpy(1);
                theta = rpy(2);
                psi = rpy(3);
                phidot = rpydot0(1);
                thetadot = rpydot0(2);
                psidot = rpydot0(3);

                % Convert UVW dot to xyz double dot
                Rdot = [ 0, sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta), cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta); ...
                    0, cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi), - cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta); ...
                    0, cos(phi)*cos(theta), -cos(theta)*sin(phi)]*phidot + ...
                    [ -cos(psi)*sin(theta), cos(psi)*cos(theta)*sin(phi), cos(phi)*cos(psi)*cos(theta); ...
                    -sin(psi)*sin(theta), cos(theta)*sin(phi)*sin(psi), cos(phi)*cos(theta)*sin(psi); ...
                    -cos(theta), -sin(phi)*sin(theta), -cos(phi)*sin(theta)]*thetadot + ...
                    [ -cos(theta)*sin(psi), - cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta), cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta); ...
                    cos(psi)*cos(theta), cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta); ...
                    0, 0, 0]*psidot;
%                 rpy_ddot = Phi0*R0*pqr_dot + reshape((dPhi*[phidot;thetadot;psidot]),3,3)*R0*pqr + ...
%                     Phi0*Rdot*pqr;
                xyz_ddot_world = R_body_to_world*UVW_dot0 + Rdot * uvw;
                xdot_world = [xyzdot0; rpydot0; xyz_ddot_world; pqr_dot];
            end
            
            
            function x_body = ConvertVelsToModelFrame(x0_drake)
                x_body = 0*x0_drake;
                x_body([1:6,10:12],:) = x0_drake([1:6,10:12],:);
                rpy0 = [ x0_drake(4); x0_drake(5); x0_drake(6)];
                R_body_to_world0 = rpy2rotmat(rpy0);
                R_world_to_body0 = R_body_to_world0';
                x_body(7:9,:) = R_world_to_body0*x0_drake(7:9,:);
            end
            
            
        end
        
        function y = output(obj,t,x,u)
            y = x;
        end
        
        function x = getInitialState(obj)
            x = zeros(12,1);
        end
        
        
    end
    
end



