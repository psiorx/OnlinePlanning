classdef PlanePlantIntegrator < DrakeSystem
    % Defines the dynamics for a trivial integrator (just for debugging)
    
    properties
    end
    
    methods
        function obj = PlanePlantIntegrator()
            obj = obj@DrakeSystem(12,0,4,12,0,1);
            obj = obj.setOutputFrame(obj.getStateFrame);
        end
        
        function xdot = dynamics(obj,t,x,u)
            
            xdot = 0*x;
            
            xdot(1:6) = x(7:12);
            
            xdot(7) = u(1);
            xdot(8) = u(2);
            xdot(9) = 0;
            xdot(10) = u(3);
            xdot(11) = 0;
            xdot(12) = u(4);
            
%             xdot(7) = u(1);
%             xdot(8) = u(2);
%             xdot(9) = u(3);
%             xdot(10) = u(4);
%             xdot(11) = u(5);
%             xdot(12) = u(6);
                       
        end
        
        function y = output(obj,t,x,u)
            y = x;
        end
        
        function x = getInitialState(obj)
            x = zeros(12,1);
        end
        
    end
    
end



