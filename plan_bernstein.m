function [bPoly, prev] = plan_bernstein(waypoints_i, time_i, prev, bounds)
%BERNSTEIN_PLANNER Summary of this function goes here
%   Detailed explanation goes here

X =  waypoints_i(1,:);
Y =  waypoints_i(2,:);
t0 = time_i(1)-prev.tf;
tf = time_i(2)-prev.tf;
tc = waypoints_i(3,:)-prev.tf;

bPoly = bernsteinPoly;
bPoly.X = X;
bPoly.Y = Y;
bPoly.t0 = t0;
bPoly.tf = tf;
bPoly.tc = tc;
bPoly.dt = prev.dt;
bPoly.degree = prev.deg;

bPoly = optimise_weights(bPoly, bounds);

[ poly ] = bPoly.get_coefficients(bPoly.tf, 1);
vx_end =dot(bPoly.wts(1:bPoly.degree+1), poly);
vy_end =dot(bPoly.wts(bPoly.degree+2:end), poly);

prev.tf = time_i(2);
prev.xdoteq = vx_end;
prev.ydoteq = vy_end;
prev.wts = bPoly.wts;

if X(end) > 0
    prev.lane = 1;
else
    prev.lane = -1;
end

end

