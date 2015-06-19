classdef PlanePlantAni < DrakeSystem
% Defines the dynamics for the powered plane 
% March 14 2013. Trying to fit parameters from experiments.
  
  properties
    g = 9.81; % m/s^2
    Cmbreaks = [-120 -14.8 -4.5 8.4 19 120];
    Cmcoeffs =[      0    0.0746
               -0.0084    0.0733
                0.0012   -0.0134
               -0.0061    0.0024
                     0   -0.0653];
                 
    Cdcoeffs = [-0.0086    1.6993
                -0.0333    1.3982
                -0.0160    0.4169
                 0.0013    0.0081
                 0.0217    0.0205
                 0.0338    0.4200
                 0.0100    1.5000];
    Cdbreaks = [-90 -55 -28 -2.45 6.9 28 60 90];
    
    Clbreaks = [-90 -40 -20 -15 -6.5 8.9 11.85 45 110];
    Clcoeffs =[-0.0213    0.0643
                0.0190   -1.0000
               -0.0280   -0.6140
                0.0184   -0.7600
                0.1069   -0.6149
               -0.1420    1.0262
                0.0143    0.5905
               -0.0210    1.0500];
    Cmpp;
    Clpp;
    Cdpp;
  end
  
  methods
    function obj = PlanePlantAni()
      obj = obj@DrakeSystem(12,0,5,12,0,1);
      obj.Cmpp = mkpp(obj.Cmbreaks, obj.Cmcoeffs);   
      obj.Cdpp = mkpp(obj.Cdbreaks,obj.Cdcoeffs);
      obj.Clpp = mkpp(obj.Clbreaks,obj.Clcoeffs);
      obj = obj.setOutputFrame(obj.getStateFrame);
    end
    
    function xdot = dynamics(obj,t,x,u)
    % function [xdot, df, d2f, d3f] = dynamics(obj,t,x,u)
        % @param t time
        % @param x state: x = 
        %  Plane's X coordinate faces forward, Y to the right, and Z down.
        %  x(1):Pn    (North or forward Position, Earth frame)
        %  x(2):Pe    (East or y position, Earth frame)
        %  x(3):Pd     (z position (down), Earth frame)
        %  x(4):phi   (roll Euler angle)
        %  x(5):theta (pitch Euler angle)
        %  x(6):psi   (yaw Euler angle)
        %  x(7):U     (X-velocity, body frame)
        %  x(8):V     (Y-velocity, body frame)
        %  x(9):W     (Z-velocity, body frame)
        %  x(10):P    (Angular Rate Vector of roll, body frame)
        %  x(11):Q    (Angular Rate Vector of pitch, body frame)
        %  x(12):R    (Angular Rate Vector of yaw, body frame)
        %  
        %  u(1):thr   (Thrust command)
        %  u(2):elev  (Elevator Command)
        %  u(3):rud   (Rudder Command)
        %  u(4):ailL  (Aileron left command)
        %  u(5):ailR  (Aileron right command) 
        
         % Fit params
          Jx_fac = 1.4475;
          Jy_fac = 1.2500;
          Jz_fac = 1.2500;
          body_drag_fac = 3.0000;
          body_z_drag_fac = 3.0000;
          roll_torque_fac = -2.0000;
          pitch_torque_fac = 2.0000;
          elev_lift_fac = -0.1896;
          thr_fac = 0.7500;
          thr_vel_fac_elev = 0.7500;
          thr_vel_fac_ail = 0.7889;
          stab_force_fac = 0.5000;
          yaw_torque_fac = 1.5000;
        
        
        % Paramaters in model (MKS units):
        wing_area_out = 0.208*0.225;% m^2
        wing_area_in = 0.037*0.208; % m^2
        wing_total_area = 2*wing_area_out + 2*wing_area_in;
        out_dist = .239; % Moment arm of outer wing section
        in_dist = .058; % Moment arm of inner wing section
        elev_area = .022725; % m^2
        elev_arm = .468;
        rudder_area = .007084; 
        rudder_arm = .471; % m
        stab_area = .0092; % m^2
        stab_arm = .381; % m 
        m = 0.575; % kg
        obj.g = 9.81; % m/s^2
        %Moments of Inertia in kg/m^2 (cross terms 100x smaller)
        % Jx_fac = 1;
        Jx = Jx_fac*.00515; % 7*0.00515
        Jy = Jy_fac*(.00881 + 0.0017); % 0.0017 comes from the optotrak mount
        Jz = Jz_fac*(.00390 + 0.0017);
        
        rho = 1.1839; % kg/m^3
        
       
        
        
        % Dynamics computations after this
        Pn = x(1);
        Pe = x(2);
        Pd = x(3);
        phi = x(4);
        theta = x(5);
        psi = x(6);
        U = x(7);
        V = x(8);
        W = x(9);
        P = x(10);
        Q = x(11);
        R = x(12);
        
        %Throttle signal is 0-255
        thr = thr_fac*max(u(1),0); % Don't let it go negative
        % thr = thr_fac*u(1); % sym
        %positive elevator is deflection up (negative lift - makes plane
        %pitch up)
        elev = u(2)*pi/180;% input in radians of deflection--check sign
        %positive rudder is deflection to the right
        rud = u(3)*pi/180;% input in radians of deflection 
        % positive AilL is negative lift (front of aileron tips downwards)
        ailL = u(4)*pi/180; % input in radians of deflection 
        % positive AilR is positive lift (front of aileron tips upwards)
        ailR = u(5)*pi/180; % input in radians of deflection
        
        % Now, the hard stuff
        
        % Do translataional stuff first
         
        % Velocity of plane
        vel = sqrt(U^2 + V^2 + W^2);
        
        % Angle of attack
        alpha = atan2(W,U);
        % alpha = atan(W,U); % sym
        
        %Sideslip angle
        beta = atan2(V,sqrt(vel^2-V^2));
        % beta = atan(V,sqrt(vel^2 - V^2)); % sym
        
        % Propwash over aileron
        upa = sqrt((vel^2)/4 + 1.4220*thr) - vel/2; % 1.4220 was fit from experiments with hot-wire anemometer
        uwa = thr_vel_fac_ail*sqrt(vel^2 + upa^2 + 2*vel*upa*cos(alpha));
        alpha_wa = atan2(W,U + upa);
        % alpha_wa = atan(W,U + upa);
        
        % Propwash over rudder/elevator
        upe = sqrt((vel^2)/4 + 0.7262*thr) - vel/2; % 0.7262 was fit from anemometer data
        uwe = thr_vel_fac_elev*sqrt(vel^2 + upe^2 + 2*vel*upe*cos(alpha));
        alpha_we = atan2(W,U + upe);
        % alpha_we = atan(W,U + upe); % sym
        
        %Lift force generated by wing components. (flat plate)
        %lift = dynamic pressure * area * Coefficient of lift.
        left_out_lift = pressure(vel) * wing_area_out * Cl_fp(alpha-ailL); 
        left_in_lift = pressure(uwa) * wing_area_in * Cl_fp(alpha_wa-ailL); 
        right_out_lift = pressure(vel) * wing_area_out * Cl_fp(alpha+ailR); 
        right_in_lift = pressure(uwa) * wing_area_in * Cl_fp(alpha_wa+ailR);
        
%         % Lift force generated by wing components. (xfoil)
%         %lift = dynamic pressure * area * Coefficient of lift.
%         left_out_lift = pressure(vel) * wing_area_out * Cl(alpha-ailL,Clpp); 
%         left_in_lift = pressure(uwa) * wing_area_in * Cl(alpha_wa-ailL,Clpp); 
%         right_out_lift = pressure(vel) * wing_area_out * Cl(alpha+ailR,Clpp); 
%         right_in_lift = pressure(uwa) * wing_area_in * Cl(alpha_wa+ailR,Clpp);
        
        %include lift terms from flat plate theory of elevator
        lift = left_out_lift + left_in_lift + right_out_lift + right_in_lift;
        elev_lift = elev_lift_fac*pressure(uwe) * elev_area * ... likely a small term
            Cl_fp(alpha_we-elev); %angle of deflection of Elevator
                
        %Drag force generated by wing components.
        left_out_drag = pressure(vel) * wing_area_out * Cd_fp(alpha-ailL);
        left_in_drag = pressure(uwa) * wing_area_in * Cd_fp(alpha_wa-ailL);
        right_out_drag = pressure(vel) * wing_area_out * Cd_fp(alpha+ailR);
        right_in_drag = pressure(uwa) * wing_area_in * Cd_fp(alpha_wa+ailR);        
        
        %Compute drag on elevator using flat plate theory
        elev_drag = pressure(uwa) * wing_area_in * Cd_fp(alpha_we-elev);
       
        %The body drag has no strict linkage to the actual airplane.  I
        %pulled the numbers from my head as what seemed reasonable.
        %Excellent candidates for fits from experimental data. The body
        %drag term takes all the non-wing frontal areas of the plane, and
        %treats them as a flat plate perpendicular to the airspeed, and
        %calculates a drag for that thin flat plate.
        %estimated frontal area of cameras + body. 1.2~cd for flat plate
        % body_drag_fac = 1;
        body_drag =  body_drag_fac*1.2 * pressure(U) * .004;
        
        % body_z_drag_fac = 0;
        body_z_drag = body_z_drag_fac*1.2 * pressure(W) * wing_total_area;

        % Estimated from experiments with digital scale
        thrust = thr_fac*(4.9417*min(thr,140))*9.81/1000; % Saturates after 140
        % thrust = thr_fac*(4.9417*thr)*9.81/1000; % Saturates after 140

        
        % Assemble fv
        % First gravity
        fv = [-m*obj.g*sin(theta); m*obj.g*sin(phi)*cos(theta); m*obj.g*cos(phi)*cos(theta)];
        
        % Then lift/drag
        R_alpha = [-cos(alpha) 0  sin(alpha); ...
                    0          1  0         ; ...
                    sin(alpha) 0 -cos(alpha)];
                
        R_alpha_in = [-cos(alpha_wa) 0 sin(alpha_wa); ...
                       0             1 0            ; ...
                       sin(alpha_wa) 0 -cos(alpha_wa)];
                   
        R_alpha_elev = [-cos(alpha_we) 0 sin(alpha_we); ...
                         0             1 0            ; ...
                         sin(alpha_we)  0 -cos(alpha_we)];
        
                        
        fv = fv + R_alpha*[left_out_drag;0;left_out_lift] ...
                + R_alpha_in*[left_in_drag;0;left_in_lift] ...
                + R_alpha*[right_out_drag;0;right_out_lift] ...
                + R_alpha_in*[right_in_drag;0;right_in_lift] ...
                + R_alpha_elev*[elev_drag;0;elev_lift];
            
       % Finally, add body drag terms
       fv = fv - [body_drag;0;sign(W)*body_z_drag];
       % fv = fv - [body_drag;0;(W/abs(W))*body_z_drag]; % sym
       
       
        % Then rotational stuff
        
        %Roll torque neglects the rolling torque generated by the rudder
        roll_torque = (left_out_lift*out_dist + left_in_lift*in_dist)...
            - (right_in_lift*in_dist + right_out_lift*out_dist);
        
        % roll_torque_fac = 1;
        roll_torque = roll_torque + roll_torque_fac*0.5*rho*out_dist*in_dist*wing_total_area*upa*P;
        
         %(Cm*dynamic pressure*wing reference area*average chord)=pitching moment
        % .204 is the average chord of the wing
        pitch_torque = -elev_lift*elev_arm + ...
             (pressure(U) * Cm(obj,alpha) * wing_total_area * .204);
        
        % pitch_torque_fac = 1;
        pitch_torque = pitch_torque + pitch_torque_fac*0.5*rho*(elev_arm^2)*elev_area*upe*Q; 
        
        %Stabilizing force from aircraft yawing
        stab_force = -stab_force_fac*sign(R) * pressure(R*stab_arm) * (stab_area+rudder_area);
        % stab_force = -stab_force_fac*(R/abs(R)) * pressure(R*stab_arm) *
        % (stab_area+rudder_area); % sym
        
        %include rudder area in the V term of the stabilizing force?  The
        %rudder isn't always perpendicular to V, but it is close most of
        %the time.
        %stabilizing force from sideslip
        stab_force = stab_force - sign(V)*pressure(V)*stab_area*1.2;
        % stab_force = stab_force - (V/abs(V))*pressure(V)*stab_area*1.2;
        % sym
        
        % Rudder force flat plate
        rudder_force = -pressure(uwe) * rudder_area * Cl_fp(beta + rud);
        
        yaw_torque = stab_force * stab_arm + rudder_force * -rudder_arm;
        
        yaw_torque = yaw_torque + yaw_torque_fac*0.5*rho*(rudder_arm^2)*rudder_area*upe*R;
        
        % Put equations together
        % Kinematics
        Pn_dot = U * cos(theta)*cos(psi) + ...
            V*(-cos(phi)*sin(psi) + sin(phi)*sin(theta)*cos(psi)) +...
            W*(sin(phi)*sin(psi) + cos(phi)*sin(theta)*cos(psi));
        Pe_dot = U * cos(theta)*sin(psi) + ...
            V*(cos(phi)*cos(psi) + sin(phi)*sin(theta)*sin(psi)) +...
            W*(-sin(phi)*cos(psi) + cos(phi)*sin(theta)*sin(psi));
        h_dot = -U*sin(theta) + V*sin(phi)*cos(theta) + W*cos(phi)*cos(theta);
        phi_dot = P + tan(theta)*(Q*sin(phi) + R*cos(phi));
        theta_dot = Q*cos(phi) - R*sin(phi);
        psi_dot = (Q*sin(phi) + R*cos(phi))/cos(theta);
        % Dynamics
        Sw = [0 -R Q; R 0 -P;-Q P 0];
        UVW_dot = -Sw*[U;V;W] + fv/m + [1;0;0]*thrust/m;
        U_dot = UVW_dot(1);
        V_dot = UVW_dot(2);
        W_dot = UVW_dot(3);
        
        
        P_dot = (Jy-Jz)*R*Q/Jx + roll_torque/Jx;
        Q_dot = (Jz-Jx)*P*R/Jy + pitch_torque/Jy;
        R_dot =  yaw_torque/Jz+(Jx-Jy)*P*Q/Jz;
        
        xdot = [Pn_dot Pe_dot h_dot phi_dot theta_dot psi_dot U_dot V_dot...
            W_dot P_dot Q_dot R_dot]';
        
%         if (nargout>1)
%             [df,d2f,d3f]= dynamicsGradients(obj,t,x,u,nargout-1);
%         end
    
    
    function cl = Cl(obj,a) %polars from xfoil, then with fit lines/curves
        if a > pi/2
            a = pi/2;
        elseif a<-pi/2
            a = -pi/2;
        end
        
        cl = ppval(obj.Clpp,a*180/pi); 
 
    end
    
    function cl = Cl_fp(a) % Flat plate model
        if a > pi/2
            a = pi/2;
        elseif a<-pi/2
            a = -pi/2;
        end
        
        cl = 2*sin(a)*cos(a);
        
    end
    
    function cd = Cd(obj,a) % from xfoil, then fit with a curve
        if a > pi/2
            a = pi/2;
        elseif a<-pi/2
            a = -pi/2;
        end

        cd = ppval(obj.Cdpp, a*180/pi);
       
    end
    
    function cd = Cd_fp(a) % Flat plate
        if a > pi/2
            a = pi/2;
        elseif a<-pi/2
            a = -pi/2;
        end

        cd = 2*(sin(a)^2);
       
    end
        
     function cm = Cm(obj,a) % xfoil
         if a > pi/2
            a = pi/2;
        elseif a<-pi/2
            a = -pi/2;
         end

         cm = ppval(obj.Cmpp, a*180/pi);
     end
     
     function pre = pressure(vel) %Dynamic Pressure = .5 rho * v^2
        pre = .5 * 1.1839 * vel^2; % N/m^2
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