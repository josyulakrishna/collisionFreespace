%iterative optimization of bernstein weights
function[bPoly]= optimise_weights(bPoly, bounds)

    import bernsteinPoly.*;
    import optimizeBernstein.*;
    optbPoly = generalizedoptimizeBernstein;
    optbPoly.bPoly = bPoly;
    
    optbPoly.xc = bPoly.X(2:end-1);
    optbPoly.yc = bPoly.Y(2:end-1);
    optbPoly.tc = bPoly.tc(2:end-1);
    optbPoly.x0 = bPoly.X(1);
    optbPoly.y0 = bPoly.Y(1);
    optbPoly.xf = bPoly.X(end);
    optbPoly.yf = bPoly.Y(end);
    
    if isfield(bounds,'xdoteq')
        optbPoly.xdoteq = bounds.xdoteq ;
        optbPoly.ydoteq = bounds.ydoteq ;
    end
    
    if isfield(bounds,'xdotmax')
        optbPoly.xdotmax = bounds.xdotmax;
        optbPoly.ydotmax = bounds.ydotmax;
    end

    if isfield(bounds,'xmin')
        optbPoly.xmin = bounds.xmin; 
        optbPoly.ymin = bounds.ymin; 
    end
    
    if isfield(bounds,'xmax')
        optbPoly.xmax = bounds.xmax;
        optbPoly.ymax = bounds.ymax;
    end
    if isfield(bounds,'winit')
        x_guess = bounds.winit;
    end
    
    [A, b, Aeq, beq] = optbPoly.get_constraints();
    
    % 'Algorithm','sqp' for accuracy
    % else use default interior point solver
    % options = optimoptions('fmincon','Display','iter');
    
    wts = fmincon(@(x)optbPoly.cost(x), x_guess, A, b, Aeq, beq, [], [], ...
                   @(x)optbPoly.get_nlconstraints);
    bPoly.wts = wts;
    
end 