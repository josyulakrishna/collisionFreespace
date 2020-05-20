function [robot, agents, seg_end_t, t, ts] = simulate_scene(t, sim_t, delta_t, bPoly, agents, robot, world, waypoints_i, ts, seg_end_t)
%PLOT_SCENE Summary of this function goes here
%   Detailed explanation goes here

cla();

%Plot road
yroad = 0:world.l;
xroad = repmat(world.rl+world.w/2,world.l+1);
rectangle('Position',[world.rl, world.rr, world.w,world.l],'FaceColor',[0 0 0 0.9]);
plot(xroad, yroad, 'w--','LineWidth',1.5);

%time annotation
str = strcat('t =',string(sim_t+delta_t));
text(15,40,str,'Color','k')

if ts.run
    scale = ts.scale;
    prev_scale = ts.prevscale;
    ts_dt = delta_t*scale;
    seg_end_t = t + ((seg_end_t-t)/(scale/prev_scale));
    t = t + ts_dt;
else
    t = t + delta_t;
end

Wx = bPoly.wts(1:bPoly.degree+1);
Wy = bPoly.wts(bPoly.degree+2:end);

[ poly ] = bPoly.get_coefficients(t,1);
vx = dot(poly,Wx);
vy = dot(poly,Wy);

if ts.run
    x = robot.x(end) + scale*vx*delta_t;
    y = robot.y(end) + scale*vy*delta_t;
    robot.x = [robot.x; x];
    robot.y = [robot.y; y];
    robot.vx = [robot.vx; scale*vx];
    robot.vy = [robot.vy; scale*vy];
else
    x = robot.x(end) + vx*delta_t;
    y = robot.y(end) + vy*delta_t;
    robot.x = [robot.x; x];
    robot.y = [robot.y; y];
    robot.vx = [robot.vx; vx];
    robot.vy = [robot.vy; vy];
end

phi = atan2(vy, vx);
[x_rot, y_rot]=get_rectangle(phi, x, y, robot.car_length, robot.car_width);
fill(x_rot, y_rot,'g');
plot(robot.x, robot.y,'w');

%
for m = 1:size(agents,2)
    agents{m}.state_transition(delta_t, 1);
    phi = agents{m}.cur_head;
    [ax_rect, ay_rect]=get_rectangle(phi, agents{m}.cur_x, agents{m}.cur_y, ...
        agents{m}.length, agents{m}.width);
    fill(ax_rect, ay_rect,'r')
end

plot(waypoints_i(1,:),waypoints_i(2,:),'y+');

if ts.run
    ts.run = false;
    ts.prevscale = ts.scale;
end

    function[xa,ya] = get_rectangle(phi,xc,yc,len, wid)
        
        x_rect = [-len, len, len, -len]/2;
        y_rect = [-wid, -wid, wid, wid]/2;
        
        % Calculate rotated rectangle
        x_rot_f = x_rect*cos(phi) - y_rect*sin(phi);
        y_rot_f = x_rect*sin(phi) + y_rect*cos(phi);
        
        % Calculate offset
        xa = x_rot_f + xc;
        ya = y_rot_f + yc;
    end
end

