classdef generalizedoptimizeBernstein

    properties
        %starting and final way points of trajectory
        x0 
        y0
        xf
        yf
        %lane constraints min and max
        xmin
        ymin
        xmax
        ymax
        %velocity constraints entire duration of the trajectory
        xdotmin
        ydotmin
        xdotmax
        ydotmax
        %accleration constraints entire duration
        xddotmin
        yddotmin
        xddotmax
        yddotmax 
        
        bPoly %object to bernstein
        xdoteq %xdot at t0 
        ydoteq %ydot at t0 
        
        % do we need them at all, isn't it restrictive?
        xc %xc cordinates apart from the x0 and xf in form of an array [tc1, xc1; tc2 xc2..]
        yc %yc cordinates apart from the y0 and yf [tc1, yc1; tc2 yc2; .. ]
        tc
        
    end
    
    methods
        % exactly as in formulation
        function sum1 = cost(obj, x)
            t0 = obj.bPoly.t0;
            tf = obj.bPoly.tf;
            sum1=0;
            xmul = x(1:obj.bPoly.degree+1); 
            ymul = x(obj.bPoly.degree+2:end);
            for t = t0:obj.bPoly.dt:tf
                [ poly_d ] = obj.bPoly.get_coefficients(t, 1); %xdot
                [ poly_dd ] = obj.bPoly.get_coefficients(t, 2); %xddot
                [ poly_ddd ] = obj.bPoly.get_coefficients(t, 3); % xdddot
                xdot = dot( xmul, poly_d );
                ydot = dot( ymul, poly_d ); 
                xddot = dot( xmul, poly_dd );
                yddot =  dot( ymul, poly_dd );
                xdddot = dot( xmul, poly_ddd );
                ydddot = dot( ymul, poly_ddd );
                sum1 = sum1+(xdddot+ydddot) + 1*(((yddot*xdot-xddot*ydot)^2)/((xdot^2+ydot^2)^3)); 
            end
        end
        
        function [A, b, Aeq, beq] = get_constraints(obj)
            
            t0 = obj.bPoly.t0;
            tf = obj.bPoly.tf;
            t1 = obj.bPoly.tc;
            
            Aeq = [];
            beq = [];
            A = [];
            b = [];
            
            [ poly ] = obj.bPoly.get_coefficients(t0, 0);
            Aeq = [Aeq; [poly', zeros(1,obj.bPoly.degree+1)]];
            beq = [beq; obj.x0];
            Aeq = [Aeq; [zeros(1,obj.bPoly.degree+1), poly']];
            beq = [beq; obj.y0];
            
            [ poly ] = obj.bPoly.get_coefficients(tf, 0);
            Aeq = [Aeq; [poly', zeros(1,obj.bPoly.degree+1)]];
            beq = [beq; obj.xf];
            Aeq = [Aeq; [zeros(1,obj.bPoly.degree+1), poly']];
            beq = [beq; obj.yf];
            
            
            if ~isempty(obj.xmin) && ~isempty(obj.xmax) && ~isempty(obj.ymin)  && ~isempty(obj.ymax)
                counter = 1;
                for t = t1
                    [ poly ] = obj.bPoly.get_coefficients(t, 0);
                    
                    A = [A; [poly', zeros(1,obj.bPoly.degree+1)]];
                    b = [b; obj.xmax(counter)];
                    A = [A; [-1*poly', zeros(1,obj.bPoly.degree+1)]];
                    b = [b; -obj.xmin(counter)];
                    
                    A = [A; [zeros(1,obj.bPoly.degree+1),poly']];
                    b = [b; obj.ymax(counter)];
                    A = [A; [zeros(1,obj.bPoly.degree+1),-1*poly']];
                    b = [b; -obj.ymin(counter)];
                    
                    counter = counter + 1;
                end
            end
            
             if ~isempty(obj.tc)
                 for i=1:size(obj.tc,2)
                    [ poly ] = obj.bPoly.get_coefficients(obj.tc(1,i), 0);
                    
                    Aeq = [Aeq; [poly', zeros(1,obj.bPoly.degree+1)]];
                    beq = [beq; obj.xc(1,i)];
                    
                    Aeq = [Aeq; [zeros(1,obj.bPoly.degree+1),poly']];
                    beq = [beq; obj.yc(1,i)];

                end
            end

            if ~isempty(obj.xdoteq) && ~isempty(obj.ydoteq)
                [ poly ] = obj.bPoly.get_coefficients(t0, 1);
                
                Aeq = [Aeq; [poly', zeros(1,obj.bPoly.degree+1)]];
                beq = [beq; obj.xdoteq(1)];
                Aeq = [Aeq; [zeros(1,obj.bPoly.degree+1), poly']];
                beq = [beq; obj.ydoteq(1)];
                
                [ poly ] = obj.bPoly.get_coefficients(tf, 1);
                
                Aeq = [Aeq; [poly', zeros(1,obj.bPoly.degree+1)]];
                beq = [beq; obj.xdoteq(2)];
                Aeq = [Aeq; [zeros(1,obj.bPoly.degree+1), poly']];
                beq = [beq; obj.ydoteq(2)];
            end
            
            % velocity constraints xdotmin, ydotmin
            if ~isempty(obj.xdotmin) && ~isempty(obj.ydotmin)
                for t = t1
                    
                    A = [A; [-1*poly', zeros(1,obj.bPoly.degree+1)]];
                    b = [b; -obj.xdotmin];
                    
                    
                    A = [A; [zeros(1,obj.bPoly.degree+1),-1*poly']];
                    b = [b; -obj.ydotmin];
                end
            end
            
            if ~isempty(obj.xdotmax)  && ~isempty(obj.ydotmax)
                for t = t1
                    [ poly ] = obj.bPoly.get_coefficients(t, 1);
                    A = [A; [poly', zeros(1,obj.bPoly.degree+1)]];
                    b = [b; obj.xdotmax];
                    A = [A; [zeros(1,obj.bPoly.degree+1),poly']];
                    b = [b; obj.ydotmax];
                end
            end
                    
        end   
        
        function [c,ceq] = get_nlconstraints(obj, x)
            c = [];
            ceq = [];
        end
    end
end