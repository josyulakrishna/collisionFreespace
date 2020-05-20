classdef Agent < handle
    
    % A non-holonomic obstacle moving on road
    
    properties 
        length
        width       
        cur_x       % x-coordinate
        cur_y       % y-cooridnate
        cur_vel     % forward velcity
        cur_head    % costant heading angle
    end
    
    methods 
        function state_transition(obj, delta_t, nsteps)
            % fowrad simulate agent for nsteps using non-holonomic motion
            % model
            for i = 1:nsteps
                obj.cur_x = obj.cur_x + obj.cur_vel*cos(obj.cur_head)*delta_t;
                obj.cur_y = obj.cur_y + obj.cur_vel*sin(obj.cur_head)*delta_t;
            end
        end
    end
    
end 