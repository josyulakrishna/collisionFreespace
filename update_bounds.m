function [bounds] = update_bounds(waypoints_i, time_i, prev, freespace, world, robot)
%UPDATE_BOUNDS Summary of this function goes here
%   Detailed explanation goes here

X =  waypoints_i(1,:);
Y =  waypoints_i(2,:);
t0 = time_i(1);
tf = time_i(2);

% must be done from freespace rather than x and heuristic
road_width = world.road_width;
car_width = robot.car_width;

clearance = (road_width - car_width)/2;

min_x = min(X) - clearance;
min_y = min(Y) - clearance;
max_x = max(X) + clearance;
max_y = max(Y) + clearance;

tpts = size(t0:prev.dt:tf,2);
bounds.xmin = min_x*ones(tpts,1);
bounds.ymin = min_y*ones(tpts,1);
bounds.xmax = max_x*ones(tpts,1);
bounds.ymax = max_y*ones(tpts,1);

lane_n = 1;
lane_p = 2;

if (prev.lane == 0)
    bounds.xdoteq = [0, 0];
    bounds.ydoteq = [lane_n, lane_p];
elseif(prev.lane == 1)
    bounds.xdoteq = [prev.xdoteq, 0];
    bounds.ydoteq = [prev.ydoteq, lane_n];
else
    bounds.xdoteq = [prev.xdoteq, 0];
    bounds.ydoteq = [prev.ydoteq, lane_p];
end

if isempty(prev.wts)
    bounds.winit = rand(2*(prev.deg+1),1);
else
    % not good because both paths might be of different lenghts
    bounds.winit = prev.wts;
end

end

