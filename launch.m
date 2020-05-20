% Free space planning problem
close all;
clear;
clc;

% load('clothoidcomponents.mat','x11','y11', 'x2', 'y2');
% 
% waypoints{1} = [-1 -1 x11(1:50:1000) x11(end); 2 6 y11(1:50:1000) y11(end)];
% time{1} = [0;10];
% waypoints{2} = [ 1  x2(1:50:1000) x2(end) -1 -1  -1; 20 y2(1:50:1000) y2(end) 35 38 40];
% time{2} = [10;20];

delta_t = 0.1;
road_width = 3.7;
car_width = 1.8;
car_length = 3.6;
clearance = (road_width - car_width)/2;

waypoints{1} = [-road_width/2 road_width/2; 2  20];
time{1} = [0;10];
waypoints{2} = [road_width/2 -road_width/2; 20  40];
time{2} = [10;20];
% waypoints{3} = [-road_width/2 road_width/2; 40  60];
% time{3} = [20;30];


deg = 3;

robot.car_length = car_length;
robot.car_width = car_width;
robot.x = [waypoints{1}(1,1)];
robot.y = [waypoints{1}(2,1)];
robot.vx = [];
robot.vy = [];

agents{1} = Agent;
agents{1}.cur_x = -road_width/2;
agents{1}.cur_y = 10;
agents{1}.cur_vel = 0.5;
agents{1}.cur_head = pi/2;
agents{1}.length = car_length;
agents{1}.width = car_width;

agents{2} = Agent;
agents{2}.cur_x = road_width/2;
agents{2}.cur_y = 3;
agents{2}.cur_vel = 1.0;
agents{2}.cur_head = pi/2;
agents{2}.length = car_length;
agents{2}.width = car_width;

world.rl = -road_width;
world.rr = 0;
world.w = 2*road_width;
world.l = 50;
world.road_width = road_width;
world.road_length = world.l;

prev.tf = 0;
prev.xdoteq = [];
prev.ydoteq = [];
prev.wts = [];
prev.lane = 0;
prev.deg = 3;
prev.dt = 0.1;

tf_prev = 0;

sim_t = 0;
cur_t = 0;
end_t = time{end}(2);

freespace = [];

p_seg = 1;
new_plan = true;

ts.run = false;
ts.scale = 1;
ts.prevscale = 1;

figure(1);
axis equal;
hold on;
grid on;

v = VideoWriter('scaled.avi');
open(v);

while(sim_t <= end_t)
    
    % bernstein path planning
    if new_plan
        
        % intermediate points time estimation
        X =  waypoints{p_seg}(1,:);
        Y =  waypoints{p_seg}(2,:);
        t0 = time{p_seg}(1);
        tf = time{p_seg}(2);
        dist = cumsum(sqrt((X(2:end)-X(1:end-1)).^2 + ...
                           (Y(2:end)-Y(1:end-1)).^2));
        tc = interp1([0;dist(end)],[t0; tf],[0,dist],'Spline');
        waypoints{p_seg}(3,:) = tc;
        
        % bounds re estimatation
        [bounds] = update_bounds(waypoints{p_seg}, time{p_seg}, prev, ...
                                 freespace, world, robot);
        [bPoly, prev] = plan_bernstein(waypoints{p_seg}, time{p_seg}, ...
                                       prev, bounds);
        
        seg_end_t = time{p_seg}(2)-time{p_seg}(1);
        cur_t = bPoly.t0;
        ts.prevscale = 1;
        new_plan = false;
    end
    
    if (sim_t >= 3) && (sim_t < 6)
        ts.run = true;
        ts.scale = 0.8;
    elseif (sim_t >= 6)
        ts.run = true;
        ts.scale = 1.2;
    end

    if (sim_t >= 1.6)
        ts.run = true;
        ts.scale = 2;
    end    

%     if abs(sim_t -3) <= 0.00001
%         t_win = [14, 17];
%         t = seg_end_t - sim_t;
%         vel.vx = bPoly.getBernsteinVel(delta_t);
%         vel.vy = bPoly.getBernsteinVel(delta_t);
%         vel.vmin = 0;
%         vel.vmax = 4;
%         [scale] = optimise_scale_t(t_win, vel, t);        
%         ts.run = true;
%         ts.scale = scale;
%     end
    if sim_t > 8
        ts.run = true;
    end
    
    [robot, agents, seg_end_t, cur_t, ts] = simulate_scene(cur_t, sim_t, delta_t, ...
                                                       bPoly, agents, robot, world, ...
                                                       waypoints{p_seg}, ts, seg_end_t);
    frame = getframe(gcf);
    writeVideo(v,frame);
    pause(0.01);
    
    time{p_seg}(2) = time{p_seg}(1) + seg_end_t;
    if p_seg < size(waypoints,2)
        for m = p_seg+1:size(waypoints,2)
            del = time{m}(2)-time{m}(1);
            time{m} = time{m-1}(2) + [0; del];
        end
    end
    
    
    if sim_t >= time{p_seg}(2)-delta_t-0.00001
        new_plan = true;
        t_prev = time{p_seg}(2);
        prev.tf = time{p_seg}(2);
        if p_seg < size(waypoints,2)
            p_seg = p_seg + 1;
        else
            break;
        end
    end
    
    sim_t = sim_t + delta_t;
    end_t = time{end}(2);
    
end

close(v);