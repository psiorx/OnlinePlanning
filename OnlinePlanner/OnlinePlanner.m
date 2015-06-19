classdef OnlinePlanner < DrakeSystem
    
    properties
        % Constant properties
        funnelLibrary;
        t_replan;
        cyclicIdx;
        
        % Mutable properties
        currentFunnel;
        forestAhead;
        t_start;
        x_start;
        
    end
    
    methods
        
        % construction
        function obj = OnlinePlanner(funnelLibrary,forestAhead,currentFunnel,cyclicIdx,t_replan,t_start,x_start,num_inputs)
            
            obj = obj@DrakeSystem(0,0,length(x_start),num_inputs,0,0);
            
            obj.funnelLibrary = funnelLibrary;
            
            obj.forestAhead = forestAhead;
            obj.currentFunnel = currentFunnel;
            
            obj.t_replan = t_replan;
            obj.t_start = t_start;
            obj.x_start = x_start;
            obj.cyclicIdx = cyclicIdx;
            
            obj.forestAhead = SharedDataHandleMutable(obj.forestAhead);
            obj.currentFunnel = SharedDataHandleMutable(obj.currentFunnel);
            obj.t_start = SharedDataHandleMutable(obj.t_start);
            obj.x_start = SharedDataHandleMutable(obj.x_start);
            
        end
        
        % control decisions
        function y = output(obj,t,~,x)
            
            % Get index of current funnel
            currentIdx = obj.currentFunnel.getData();    
            
            % Get t_current
            t_current = t - obj.t_start.getData();
            
            % Get x_current
            x_current = x;
            x_s = obj.x_start.getData();
            x0 = obj.funnelLibrary{currentIdx}.xtraj.fasteval(0);
            
            % Shift stuff along cyclic dimensions and compute relative
            % state
            x_current(obj.cyclicIdx) = x(obj.cyclicIdx) - x_s(obj.cyclicIdx) + x0(obj.cyclicIdx);
            
            % Check if it is time to replan (t_current > t_replan)
            if t_current > obj.t_replan
                replan = true;
            else % If it is not yet time...
                % Check if current state is inside current funnel
                Snow = obj.funnelLibrary{currentIdx}.V.S.fasteval(t_current);
                s1now = obj.funnelLibrary{currentIdx}.V.s1.fasteval(t_current);
                s2now = obj.funnelLibrary{currentIdx}.V.s2.fasteval(t_current);
                insideCurrent = (x_current'*Snow*x_current + s1now'*x_current + s2now < 1);
                % insideCurrent = (obj.funnelLibrary{currentIdx}.V.eval(t_current,x_current) < 1);
                     
                % If so, check if current funnel is still collision free
                if insideCurrent
                    collisionFree = obj.isCollisionFree(currentIdx,x_s);
                    
                    % If it is collision free, no need to replan
                    if collisionFree
                        replan = false;
                    else % If it is not collision free anymore, replan
                        replan = true;
                    end
                    
                else % If not inside current funnel anymore, replan
                    replan = true;
                end
            end
                
            % If we have to replan, do so
            if replan
                
                % Get next funnel from replanFunnels
                nextFunnel = obj.replanFunnels(t,x);
                % Get t_current (This should really always be 0!)
                t_current = t - obj.t_start.getData();
                % Get x_current
                x_current = x;
                x_s = obj.x_start.getData();
                x0 = obj.funnelLibrary{nextFunnel}.xtraj.fasteval(0);
                % Shift stuff along cyclic dimensions and compute relative
                % state
                x_current(obj.cyclicIdx) = x(obj.cyclicIdx) - x_s(obj.cyclicIdx) + x0(obj.cyclicIdx);
                
                % Compute correct control input
                y = obj.funnelLibrary{nextFunnel}.controller.output(t_current,[],x_current);
            else % If no replanning necessary, follow current plan
                
                % Compute correct control input
                y = obj.funnelLibrary{currentIdx}.controller.output(t_current,[],x_current);
            end
            
        end
        
        function nextFunnel = replanFunnels(obj,t,x)
            
            % Go through funnels and find the first collision free one
            
            % If none is found, pick one with smallest objective value
            % (TODO)
            nextFunnel = 1;
            
            N = length(obj.funnelLibrary);

            for k = 1:N
                collFree = obj.isCollisionFree(k,x);

                if collFree
                    nextFunnel = k;
                    break;
                end
            end
            
            % Set start time for new funnels
            obj.t_start.setData(t);
            
            % Set start state for new funnels
            obj.x_start.setData(x);
            
            % Set current funnel to new planned funnel
            obj.currentFunnel.setData(nextFunnel);
            
        end
        
        
        % Checks if a given funnel is collision free if executed beginning
        % at x (assuming polytopic obstacles)
        function collisionFree = isCollisionFree(obj,funnelIdx,x)
            
            % Get latest reported obstacle positions
            forest = obj.forestAhead.getData();
            
            % Get funnel
            funnel = obj.funnelLibrary{funnelIdx};
            
            % Get time samples
            ts = funnel.ts;
            
            % For each time sample, check if collision free
            collisions = zeros(length(ts),size(forest,2));
            
            x0 = funnel.xtraj.fasteval(0);
            
            for k = 1:length(ts)
                % Check if this particular ellipsoid intersects any
                % polytope in forestAhead
                
                S = funnel.Sp{k};
                s1 = funnel.s1p{k};
                s2 = funnel.s2p{k};
                
                for j = 1:size(forest,2)
                    Aineq = forest{j}.Aineq;
                    bineq = forest{j}.bineq + Aineq*(x0(obj.cyclicIdx) - x(obj.cyclicIdx)); % ASSUMES OBSTACLES ARE IN CYCLIC DIMENSIONS!
                   
                    
                    collisions(k,j) = obj.checkIntersectionCVX(Aineq,bineq,S,s1,s2);
                end
            end
            
            collisionFree = full(~any(any(collisions)));
                
        end
        
        % Checks if a given ellipse intersects a polytope with bullet
        function [collision, optval] = checkIntersection(obj,Aineq,bineq,S,s1,s2)
            
            % Cvxgen setup
            settings.verbose = 0;
            params.A = Aineq;
            params.b = bineq;
            params.S = S; 
            params.s1 = s1;
            params.s2 = s2;

            % Solve with cvxgen
            [~, status] = csolve(params,settings);
            
            % Get optimal value
            optval = status.optval;
            
            % Check if there is a collision
            collision = (optval < 1);
            
        end
        
        
        % Checks if a given ellipse intersects a polytope with cvxgen
        function [collision, optval] = checkIntersectionCVX(obj,Aineq,bineq,S,s1,s2)
            
            % Cvxgen setup
            settings.verbose = 0;
            params.A = Aineq;
            params.b = bineq;
            params.S = S; 
            params.s1 = s1;
            params.s2 = s2;

            % Solve with cvxgen
            [~, status] = csolve(params,settings);
            
            % Get optimal value
            optval = status.optval;
            
            % Check if there is a collision
            collision = (optval < 1);
            
        end
        
    end
    
end
            
            
            
            
            
            




